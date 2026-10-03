# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Amaranth sim of BinSelect: keeps a per-ADC bitmap-selected set of bins from a
natural-order A/B stream.  Verifies keep_a/keep_b, the per-ADC running write
addresses cnt_a/cnt_b, frame_last and seq against the loaded mask, plus the
clear-sweep."""
import numpy as np
import pytest
from amaranth import *
from amaranth.sim import Simulator

from maia_fcfb.bin_select import BinSelect
from maia_fcfb.mask_mem import MaskMem


class _BSWithMask(Elaboratable):
    """BinSelect + its external MaskMem BRAM, wired as in stage1_top (single
    domain here), re-exposing the load/clear + bin I/O so the tests drive the
    real integrated path (registered mask read, sync write/clear sweep)."""
    def __init__(self, order_log2, seq_width=64):
        self.bs = BinSelect(23, order_log2, seq_width=seq_width)
        self.mm = MaskMem(order_log2)          # rd/wr both 'sync'
        self.clken = Signal()
        self.in_valid = Signal(); self.in_last = Signal()
        self.a_re = Signal(signed(23)); self.a_im = Signal(signed(23))
        self.b_re = Signal(signed(23)); self.b_im = Signal(signed(23))
        # mask load/clear (forwarded to MaskMem write side)
        self.mask_wr_addr = Signal(order_log2)
        self.mask_wr_data = Signal(2)
        self.mask_wr_en = Signal()
        self.mask_clear = Signal()
        self.mask_busy = Signal()
        # bin-select outputs (forwarded)
        self.keep_a = Signal(); self.keep_b = Signal()
        self.cnt_a = Signal(order_log2); self.cnt_b = Signal(order_log2)
        self.frame_last = Signal(); self.seq = Signal(seq_width)
        self.a_re_o = Signal(signed(23)); self.b_re_o = Signal(signed(23))

    def elaborate(self, platform):
        m = Module()
        m.submodules.bs = bs = self.bs
        m.submodules.mm = mm = self.mm
        m.d.comb += [
            bs.clken.eq(self.clken),
            bs.in_valid.eq(self.in_valid), bs.in_last.eq(self.in_last),
            bs.a_re.eq(self.a_re), bs.a_im.eq(self.a_im),
            bs.b_re.eq(self.b_re), bs.b_im.eq(self.b_im),
            # bs <-> mm
            mm.rd_addr.eq(bs.rd_addr), mm.rd_en.eq(self.clken),
            bs.keep_a_in.eq(mm.keep_a), bs.keep_b_in.eq(mm.keep_b),
            # mask write side
            mm.wr_addr.eq(self.mask_wr_addr), mm.wr_data.eq(self.mask_wr_data),
            mm.wr_en.eq(self.mask_wr_en), mm.clear.eq(self.mask_clear),
            self.mask_busy.eq(mm.busy),
            # outputs
            self.keep_a.eq(bs.keep_a), self.keep_b.eq(bs.keep_b),
            self.cnt_a.eq(bs.cnt_a), self.cnt_b.eq(bs.cnt_b),
            self.frame_last.eq(bs.frame_last), self.seq.eq(bs.seq),
            self.a_re_o.eq(bs.a_re_o), self.b_re_o.eq(bs.b_re_o),
        ]
        return m


def _enc(k):
    # distinctive per-bin values (all within signed(23)) to check passthrough
    return dict(a_re=k, a_im=-k, b_re=k + 1000, b_im=-(k + 1000))


async def _load_mask(ctx, dut, mask_a, mask_b, N, use_clear=True):
    """Load the keep-memory (optionally via a clear sweep first)."""
    ctx.set(dut.clken, 0)
    ctx.set(dut.in_valid, 0)
    if use_clear:
        ctx.set(dut.mask_clear, 1)
        await ctx.tick()
        ctx.set(dut.mask_clear, 0)
        # wait for the sweep to finish (busy drops)
        guard = 0
        while ctx.get(dut.mask_busy):
            await ctx.tick()
            guard += 1
            assert guard < 4 * N, "clear sweep never finished"
    # write only the kept bins (clear left the rest at 0); if not clearing,
    # write every bin explicitly.
    for k in range(N):
        ka, kb = int(mask_a[k]), int(mask_b[k])
        if use_clear and not (ka or kb):
            continue
        ctx.set(dut.mask_wr_addr, k)
        ctx.set(dut.mask_wr_data, (kb << 1) | ka)
        ctx.set(dut.mask_wr_en, 1)
        await ctx.tick()
    ctx.set(dut.mask_wr_en, 0)


def _expected(mask_a, mask_b, N, nframes):
    """(kept-bin list, frame_last count) the DUT must produce for these masks."""
    kept = []       # (frame, k, keep_a, keep_b, cnt_a, cnt_b)
    for f in range(nframes):
        ca = cb = 0
        for k in range(N):
            ka, kb = bool(mask_a[k]), bool(mask_b[k])
            if ka or kb:
                kept.append((f, k, ka, kb, ca, cb))
            ca += ka
            cb += kb
    return kept


@pytest.mark.parametrize("order_log2,desc,mk", [
    (4, "contig_A", lambda N: (_contig(N, 5, 7), _zeros(N))),
    (6, "contig_AB", lambda N: (_contig(N, 20, 7), _contig(N, 20, 7))),
    (6, "two_blocks", lambda N: (_or(_contig(N, 3, 5), _contig(N, 40, 6)),
                                 _contig(N, 41, 4))),
    (6, "per_adc_diff", lambda N: (_contig(N, 1, 10), _contig(N, 30, 20))),
    (5, "edges", lambda N: (_or(_contig(N, 0, 2), _contig(N, N - 2, 2)),
                            _contig(N, 0, 1))),
    (4, "empty", lambda N: (_zeros(N), _zeros(N))),
    (4, "full", lambda N: (_ones(N), _ones(N))),
])
@pytest.mark.parametrize("gaps", [False, True])
def test_bin_select(order_log2, desc, mk, gaps):
    N = 1 << order_log2
    nframes = 3
    mask_a, mask_b = mk(N)
    dut = _BSWithMask(order_log2, seq_width=16)

    got = []          # (keep_a, keep_b, cnt_a, cnt_b, seq, a_re_o, b_re_o)
    frame_lasts = []  # seq at each frame_last

    async def tb(ctx):
        await _load_mask(ctx, dut, mask_a, mask_b, N)
        ctx.set(dut.clken, 1)
        cyc = 0
        for f in range(nframes):
            k = 0
            while k < N:
                stall = gaps and (cyc % 3 == 2)
                cyc += 1
                if stall:
                    ctx.set(dut.in_valid, 0)
                    ctx.set(dut.in_last, 0)
                else:
                    e = _enc(k)
                    ctx.set(dut.in_valid, 1)
                    ctx.set(dut.in_last, 1 if k == N - 1 else 0)
                    ctx.set(dut.a_re, e["a_re"]); ctx.set(dut.a_im, e["a_im"])
                    ctx.set(dut.b_re, e["b_re"]); ctx.set(dut.b_im, e["b_im"])
                # sample outputs (latency 1: reflect the previous input bin)
                if ctx.get(dut.keep_a) or ctx.get(dut.keep_b):
                    got.append((ctx.get(dut.keep_a), ctx.get(dut.keep_b),
                                ctx.get(dut.cnt_a), ctx.get(dut.cnt_b),
                                ctx.get(dut.seq),
                                ctx.get(dut.a_re_o), ctx.get(dut.b_re_o)))
                if ctx.get(dut.frame_last):
                    frame_lasts.append(ctx.get(dut.seq))
                await ctx.tick()
                if not stall:
                    k += 1
        ctx.set(dut.in_valid, 0)
        for _ in range(4):
            if ctx.get(dut.keep_a) or ctx.get(dut.keep_b):
                got.append((ctx.get(dut.keep_a), ctx.get(dut.keep_b),
                            ctx.get(dut.cnt_a), ctx.get(dut.cnt_b),
                            ctx.get(dut.seq),
                            ctx.get(dut.a_re_o), ctx.get(dut.b_re_o)))
            if ctx.get(dut.frame_last):
                frame_lasts.append(ctx.get(dut.seq))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    exp = _expected(mask_a, mask_b, N, nframes)
    assert len(got) == len(exp), f"{desc}: got {len(got)} kept, expected {len(exp)}"
    for (ka, kb, ca, cb, seq, aro, bro), (f, k, eka, ekb, eca, ecb) in zip(got, exp):
        e = _enc(k)
        assert (bool(ka), bool(kb)) == (eka, ekb), f"{desc}: keep@f{f}k{k}"
        if eka:
            assert ca == eca, f"{desc}: cnt_a@f{f}k{k}: {ca}!={eca}"
            assert aro == e["a_re"], f"{desc}: a passthrough@f{f}k{k}"
        if ekb:
            assert cb == ecb, f"{desc}: cnt_b@f{f}k{k}: {cb}!={ecb}"
            assert bro == e["b_re"], f"{desc}: b passthrough@f{f}k{k}"
        assert seq == f, f"{desc}: seq@f{f}k{k}: {seq}!={f}"

    # one frame_last per frame, seq incrementing 0..nframes-1
    assert frame_lasts == list(range(nframes)), \
        f"{desc}: frame_last seqs {frame_lasts}"


def test_clear_sweep_zeros_memory():
    """After a clear sweep, an un-reloaded bin reads keep=0; a subsequent load
    of a different bin set takes effect (proves the sweep + reload path)."""
    order_log2 = 5
    N = 1 << order_log2
    dut = _BSWithMask(order_log2, seq_width=16)
    got = []

    async def tb(ctx):
        # load a mask, then clear it, then load a *different* one
        await _load_mask(ctx, dut, _contig(N, 3, 4), _zeros(N), N, use_clear=False)
        await _load_mask(ctx, dut, _contig(N, 10, 2), _contig(N, 20, 3), N,
                         use_clear=True)
        ctx.set(dut.clken, 1)
        k = 0
        while k < N:
            e = _enc(k)
            ctx.set(dut.in_valid, 1)
            ctx.set(dut.in_last, 1 if k == N - 1 else 0)
            ctx.set(dut.a_re, e["a_re"]); ctx.set(dut.b_re, e["b_re"])
            if ctx.get(dut.keep_a) or ctx.get(dut.keep_b):
                got.append((ctx.get(dut.keep_a), ctx.get(dut.keep_b)))
            await ctx.tick()
            k += 1
        ctx.set(dut.in_valid, 0)
        for _ in range(4):
            if ctx.get(dut.keep_a) or ctx.get(dut.keep_b):
                got.append((ctx.get(dut.keep_a), ctx.get(dut.keep_b)))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    # only the reloaded set survives: 2 A-only + 3 B-only bins
    assert sum(1 for ka, kb in got if ka) == 2
    assert sum(1 for ka, kb in got if kb) == 3


# ---- small mask constructors ------------------------------------------------
def _zeros(N):
    return np.zeros(N, dtype=bool)


def _ones(N):
    return np.ones(N, dtype=bool)


def _contig(N, k0, W):
    m = np.zeros(N, dtype=bool)
    m[k0:k0 + W] = True
    return m


def _or(a, b):
    return a | b
