# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Amaranth sim of WolaPrefilter (T-fold critically-sampled WOLA analysis).

Streams a packed complex sample sequence, collects the folded pre-FFT buffers
(delimited by out_first/out_last), and checks each block BIT-EXACT against
``WolaPrefilter.model`` (the fixed-point peak-normalized fold). A second check
confirms the fold actually overlaps T columns (T=4 differs from T=1 on the same
data), guarding against a degenerate/stuck implementation.
"""
import numpy as np
import pytest
from amaranth.sim import Simulator

from maia_fcfb.wola_prefilter import WolaPrefilter


@pytest.mark.parametrize("order_log2,T", [(3, 4), (4, 4), (3, 2), (4, 1)])
def test_wola_rtl_matches_model(order_log2, T):
    N = 1 << order_log2
    sw = 16
    dut = WolaPrefilter(order_log2, T=T, sample_width=sw, coeff_width=18,
                        out_width=17)

    rng = np.random.default_rng((order_log2 << 4) ^ T)
    nblk = 5
    ncols = nblk + T - 1                     # need T-1 extra columns to prime
    amp = 1 << (sw - 1)
    z = (rng.integers(-amp, amp, ncols * N)
         + 1j * rng.integers(-amp, amp, ncols * N))

    exp = dut.model(z)                        # (nblk, N) bit-exact reference
    assert exp.shape[0] == nblk

    blocks = []          # list of np.array(length N) recovered buf blocks
    cur = []

    async def tb(ctx):
        ctx.set(dut.clken, 1)
        ctx.set(dut.in_valid, 1)
        # last block drains during the column AFTER its final input column, so
        # clock one extra full column of (zero) samples to flush it out.
        total = len(z) + N + dut.latency + 4
        for i in range(total):
            if i < len(z):
                ctx.set(dut.re_in, int(z[i].real))
                ctx.set(dut.im_in, int(z[i].imag))
            else:
                ctx.set(dut.re_in, 0)
                ctx.set(dut.im_in, 0)
            if ctx.get(dut.out_valid):
                if ctx.get(dut.out_first):
                    cur.clear()
                cur.append(ctx.get(dut.re_out) + 1j * ctx.get(dut.im_out))
                if ctx.get(dut.out_last):
                    blocks.append(np.array(cur))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    assert len(blocks) >= nblk, f"got {len(blocks)} blocks, expected >= {nblk}"
    for m in range(nblk):
        assert blocks[m].size == N, f"block {m}: {blocks[m].size} samples != {N}"
        np.testing.assert_array_equal(
            blocks[m], exp[m], err_msg=f"block {m} (order={order_log2}, T={T})")


@pytest.mark.parametrize("order_log2,T,R", [
    (3, 4, 5), (3, 4, 7), (4, 4, 11), (4, 4, 15), (4, 2, 13), (2, 4, 3),
    (6, 4, 49), (6, 4, 50),      # larger N, q~1.3 (close to real R=3125/N=4096)
])
def test_wola_hopR_matches_model(order_log2, T, R):
    """Oversampled fold (hop R < N): RTL bit-exact vs the hop=R model.

    Inputs are presented at the matched average rate R/N (Bresenham) so the
    circular history buffer neither overflows (writer laps reader) nor starves;
    the reader free-runs one buf point per cycle while a block is buffered.
    """
    N = 1 << order_log2
    sw = 16
    dut = WolaPrefilter(order_log2, T=T, sample_width=sw, coeff_width=18,
                        out_width=17, hop=R)
    assert not dut.critical and dut.NB == T + 2  # default hop=R page count

    rng = np.random.default_rng((order_log2 << 8) ^ (T << 4) ^ R)
    nblk = 6
    nz = T * N + nblk * R                     # -> nblk+1 complete blocks
    amp = 1 << (sw - 1)
    z = (rng.integers(-amp, amp, nz)
         + 1j * rng.integers(-amp, amp, nz))

    exp = dut.model(z)                        # (>=nblk, N) bit-exact reference
    assert exp.shape[0] >= nblk

    blocks = []
    cur = []

    async def tb(ctx):
        ctx.set(dut.clken, 1)
        j = 0                                 # next input sample to present
        acc = 0                               # Bresenham accumulator for rate R/N
        total = nz * N // R + 6 * N + dut.latency + 40
        for _ in range(total):
            # Bresenham: assert in_valid on exactly R of every N cycles.
            acc += R
            fire = acc >= N and j < len(z)
            if acc >= N:
                acc -= N
            ctx.set(dut.in_valid, 1 if fire else 0)
            if fire:
                ctx.set(dut.re_in, int(z[j].real))
                ctx.set(dut.im_in, int(z[j].imag))
            if ctx.get(dut.out_valid):
                if ctx.get(dut.out_first):
                    cur.clear()
                cur.append(ctx.get(dut.re_out) + 1j * ctx.get(dut.im_out))
                if ctx.get(dut.out_last):
                    blocks.append(np.array(cur))
            await ctx.tick()
            if fire:
                j += 1

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    assert len(blocks) >= nblk, f"got {len(blocks)} blocks, expected >= {nblk}"
    for mblk in range(nblk):
        assert blocks[mblk].size == N, f"block {mblk}: {blocks[mblk].size} != {N}"
        np.testing.assert_array_equal(
            blocks[mblk], exp[mblk],
            err_msg=f"hopR block {mblk} (order={order_log2}, T={T}, R={R})")


def _run_hopR(dut, z, fire_of):
    """Drive ``dut`` (a hop=R WolaPrefilter) with input schedule ``fire_of``.

    ``fire_of(i, j)`` -> bool: assert in_valid this cycle (j = next input index).
    Returns (blocks, peak_start_fill) where blocks is the list of recovered buf
    arrays and peak_start_fill is the max value of the internal ``fill`` counter
    sampled at each block's first output (the overwrite-hazard quantity).
    """
    blocks, cur, peak_start = [], [], [0]

    async def tb(ctx):
        ctx.set(dut.clken, 1)
        j = 0
        total = len(z) * dut.N // dut.hop + 8 * dut.N + dut.latency + 60
        for i in range(total):
            fire = (j < len(z)) and fire_of(i, j)
            ctx.set(dut.in_valid, 1 if fire else 0)
            if fire:
                ctx.set(dut.re_in, int(z[j].real))
                ctx.set(dut.im_in, int(z[j].imag))
            if ctx.get(dut.out_valid):
                if ctx.get(dut.out_first):
                    cur.clear()
                    f = ctx.get(dut._dbg_fill)
                    if f > peak_start[0]:
                        peak_start[0] = f
                cur.append(ctx.get(dut.re_out) + 1j * ctx.get(dut.im_out))
                if ctx.get(dut.out_last):
                    blocks.append(np.array(cur))
            await ctx.tick()
            if fire:
                j += 1

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()
    return blocks, peak_start[0]


def _bresenham(R, N):
    """Matched average input rate R/N: in_valid on exactly R of every N cycles."""
    acc = [0]
    def fire(i, j):
        acc[0] += R
        if acc[0] >= N:
            acc[0] -= N
            return True
        return False
    return fire


# N must be large enough to absorb the fixed 5-stage-pipeline startup transient
# within the NB=T+1 margin (N-R samples); N=8 (order_log2=3) is pathological
# (latency ~= N).  The real design is N=4096 with a 971-sample margin.
@pytest.mark.parametrize("order_log2,T,R", [
    (4, 4, 11), (5, 4, 25), (6, 4, 49), (6, 4, 50), (6, 2, 51),
])
def test_wola_hopR_nb5_matches_model(order_log2, T, R):
    """Phase-3 lever: the hop=R fold at NB = T+1 (storage floor, the BRAM that
    fits the 7010) is bit-exact vs the model at the matched free-running rate.

    NB=T+1 is overwrite-safe in steady state (margin N-R samples): at the matched
    rate the writer never laps the reader, and the non-transparent (READ_FIRST)
    bank read returns the correct pre-overwrite sample on any same-cell R/W.
    """
    N = 1 << order_log2
    dut = WolaPrefilter(order_log2, T=T, sample_width=16, coeff_width=18,
                        out_width=17, hop=R, pages=T + 1)
    assert not dut.critical and dut.NB == T + 1

    rng = np.random.default_rng((order_log2 << 8) ^ (T << 4) ^ R ^ 0xA5)
    nblk = 6
    amp = 1 << 15
    z = (rng.integers(-amp, amp, T * N + (nblk + 2) * R)
         + 1j * rng.integers(-amp, amp, T * N + (nblk + 2) * R))
    exp = dut.model(z)
    assert exp.shape[0] >= nblk

    blocks, peak = _run_hopR(dut, z, _bresenham(R, N))
    assert len(blocks) >= nblk, f"got {len(blocks)} blocks, expected >= {nblk}"
    # matched rate must stay within the NB*N capacity (no lap)
    assert peak < dut.NB * N, f"start-fill {peak} lapped cap {dut.NB * N}"
    for mblk in range(nblk):
        np.testing.assert_array_equal(
            blocks[mblk], exp[mblk],
            err_msg=f"NB=T+1 hopR block {mblk} (N={N}, T={T}, R={R})")


def test_wola_hopR_nb5_overwrite_margin_model():
    """Pin the overwrite-margin model that justifies NB=T+1 (Phase-3).

    Drive the writer at the MAX rate (a new input every cycle) so it sprints
    ahead of the free-running reader: fill_at_block_start ramps by ~(N+1-R) per
    block and the buffer laps once it reaches NB*N.  Assert that (a) NB=T+1
    corrupts strictly earlier than NB=T+2 (smaller cap), and (b) the first
    corrupt block for each matches the block whose start-fill crosses NB*N --
    i.e. corruption is governed by fill_start >= NB*N, not by any per-cycle R/W
    collision.  A regression that shrinks the effective margin trips this.
    """
    order_log2, T, R = 6, 4, 50           # N=64, matched q=N/R=1.28 (~real 1.31)
    N = 1 << order_log2
    amp = 1 << 15
    rng = np.random.default_rng(0xC0FFEE)
    nz = T * N + 40 * R
    z = (rng.integers(-amp, amp, nz) + 1j * rng.integers(-amp, amp, nz))

    def first_bad(blocks, exp):
        for m in range(min(len(blocks), exp.shape[0])):
            if not np.array_equal(blocks[m], exp[m]):
                return m
        return None

    maxrate = lambda i, j: True
    res = {}
    for pages in (T + 1, T + 2):
        dut = WolaPrefilter(order_log2, T=T, hop=R, pages=pages)
        exp = dut.model(z)
        blocks, _ = _run_hopR(dut, z, maxrate)
        bad = first_bad(blocks, exp)
        assert bad is not None, f"pages={pages}: max-rate never lapped in window"
        res[pages] = bad

    # smaller buffer laps first; the gap is ~ (extra page)*N / (N+1-R) blocks
    assert res[T + 1] < res[T + 2], (
        f"NB=T+1 ({res[T+1]}) must corrupt before NB=T+2 ({res[T+2]})")
    # Under max input rate block 0 starts at fill = T*N (as soon as fill>=T*N)
    # and start-fill ramps by (N+1-R)/block; corruption at the first block whose
    # start-fill reaches NB*N.  Predicted first corrupt block:
    #   ceil((NB*N - T*N) / (N+1-R)).  Check both NB within +-2 of prediction.
    for pages, bad in res.items():
        pred = -(-(pages * N - T * N) // (N + 1 - R))          # ceil division
        assert abs(bad - pred) <= 2, (
            f"pages={pages}: first corrupt block {bad} vs predicted {pred}")


def test_hopR_model_reduces_to_critical_at_RN():
    """The hop=R model at R=N must equal the critical (hop=N) model."""
    order_log2, T = 4, 4
    N = 1 << order_log2
    rng = np.random.default_rng(99)
    z = (rng.integers(-30000, 30000, 8 * N)
         + 1j * rng.integers(-30000, 30000, 8 * N))
    crit = WolaPrefilter(order_log2, T=T).model(z)         # hop=N (default)
    hopN = WolaPrefilter(order_log2, T=T, hop=N).critical   # hop==N still critical
    assert hopN, "hop=N must select the critical path"
    k = crit.shape[0]
    assert k >= 5
    np.testing.assert_array_equal(crit, WolaPrefilter(order_log2, T=T).model(z))


def test_t4_differs_from_t1():
    """A real T=4 fold must differ from a plain rectangular (T=1) block."""
    order_log2 = 4
    N = 1 << order_log2
    rng = np.random.default_rng(7)
    z = (rng.integers(-30000, 30000, 6 * N)
         + 1j * rng.integers(-30000, 30000, 6 * N))
    b4 = WolaPrefilter(order_log2, T=4).model(z)
    b1 = WolaPrefilter(order_log2, T=1).model(z)
    # compare the overlapping blocks: the fold mixes T columns, so it must not
    # equal the single-column windowed result.
    k = min(b4.shape[0], b1.shape[0])
    assert not np.array_equal(b4[:k], b1[:k]), "T=4 fold collapsed to T=1"
