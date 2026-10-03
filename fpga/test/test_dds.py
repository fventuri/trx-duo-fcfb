# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""DDS tone generator (input-scene injection), linear-interpolated NCO.

Checks the phase accumulator advances only on (enable & strobe), the pipelined
interpolated re/im outputs match the shared ``nco_samples`` model twin to the LSB
(up to the constant pipeline sample lag), the exact-bin phase-increment property
(phase_inc = k<<12 => k whole turns per N=4096 block), and that an arbitrary
OFF-grid phase_inc yields a clean tone whose intrinsic SFDR sits well below the
-79 dBc wideband bank floor the DDS is used to measure.
"""
import numpy as np
from amaranth.sim import Simulator

from maia_fcfb.dds import (Dds, nco_samples, cos_lut,
                           PHASE_BITS, LUT_BITS, FRAC_BITS)


def _run(dut, inc, nsamp, strobe_period=1):
    """Drive enable=1 and pulse strobe every `strobe_period` clocks; return the
    (re,im) sampled ON each strobe as complex (one entry per consumed sample).

    strobe_period>1 exercises the SPARSE-strobe cadence of the real datapath
    (the consumed-sample tick is not every clock); the DDS pipeline must present
    the correct sample at each strobe regardless."""
    res = []

    async def tb(ctx):
        ctx.set(dut.phase_inc, inc)
        ctx.set(dut.enable, 1)
        t = 0
        while len(res) < nsamp:
            stb = 1 if (t % strobe_period == 0) else 0
            ctx.set(dut.strobe, stb)
            await ctx.tick()
            if stb:
                res.append(ctx.get(dut.re) + 1j * ctx.get(dut.im))
            t += 1

    sim = Simulator(dut)
    sim.add_clock(8e-9)                       # datapath (sync) clock 125 MHz
    sim.add_testbench(tb)
    sim.run()
    return np.array(res)


def _match_lag(hw, model, maxlag=6):
    """Smallest sample lag with the fewest bit mismatches (constant pipeline
    delay on a single tone is a per-sample constant, absorbed downstream)."""
    best = None
    for lag in range(maxlag):
        a = hw[lag:]
        n = min(len(a), len(model))
        mism = int(np.sum(a[:n] != model[:n]))
        if best is None or mism < best[1]:
            best = (lag, mism, n)
    return best


def test_dds_matches_model_offgrid():
    amp = 8192
    inc = (700 << (PHASE_BITS - 12)) + 123        # OFF-grid (fractional bin)
    dut = Dds(out_width=16, amplitude=amp, domain='sync')
    hw = _run(dut, inc, 400)
    model = nco_samples(len(hw), inc, amp)
    lag, mism, n = _match_lag(hw, model)
    assert mism == 0, f"RTL != model twin: {mism}/{n} mismatches at lag {lag}"


def test_dds_matches_model_onbin():
    amp = 8192
    inc = 697 << (PHASE_BITS - 12)                 # exact bin 697
    dut = Dds(out_width=16, amplitude=amp, domain='sync')
    hw = _run(dut, inc, 400)
    model = nco_samples(len(hw), inc, amp)
    lag, mism, n = _match_lag(hw, model)
    assert mism == 0, f"RTL != model twin (on-bin): {mism}/{n} at lag {lag}"


def test_dds_matches_model_sparse_strobe():
    """The consumed-sample strobe is NOT every clock in the real datapath; the
    pipeline must still present the right sample per strobe (regression for the
    HW failure where a fragile un-pipelined interp mis-sampled)."""
    amp = 8192
    inc = (700 << (PHASE_BITS - 12)) + 321         # off-grid
    for period in (2, 3, 5):
        dut = Dds(out_width=16, amplitude=amp, domain='sync')
        hw = _run(dut, inc, 300, strobe_period=period)
        model = nco_samples(len(hw), inc, amp)
        lag, mism, n = _match_lag(hw, model)
        assert mism == 0, (f"sparse strobe /{period}: {mism}/{n} mismatches "
                           f"at lag {lag}")


def test_dds_gating():
    """enable=0 and strobe=0 both freeze the DDS: the per-sample pipeline only
    advances on (enable & strobe), so the output holds when either is low."""
    amp = 8192
    dut = Dds(out_width=16, amplitude=amp, domain='sync')

    async def tb(ctx):
        ctx.set(dut.phase_inc, 12345)
        # run a while to fill the pipeline with a live tone
        ctx.set(dut.enable, 1)
        ctx.set(dut.strobe, 1)
        for _ in range(40):
            await ctx.tick()
        # freeze via strobe=0 -> output must not change
        ctx.set(dut.strobe, 0)
        await ctx.tick()
        held = ctx.get(dut.re) + 1j * ctx.get(dut.im)
        for _ in range(6):
            await ctx.tick()
            assert ctx.get(dut.re) + 1j * ctx.get(dut.im) == held, "strobe=0 moved"
        # freeze via enable=0 (strobe high) -> also must not change
        ctx.set(dut.strobe, 1)
        ctx.set(dut.enable, 0)
        await ctx.tick()
        held2 = ctx.get(dut.re) + 1j * ctx.get(dut.im)
        for _ in range(6):
            await ctx.tick()
            assert ctx.get(dut.re) + 1j * ctx.get(dut.im) == held2, "enable=0 moved"

    sim = Simulator(dut)
    sim.add_clock(8e-9)
    sim.add_testbench(tb)
    sim.run()


def test_dds_exact_bin_wraps():
    """phase_inc = k<<12 advances exactly k whole turns over N=4096 samples, so
    the phase returns to 0 -> the output equals the phase-0 output again."""
    amp = 4096
    inc = 3 << (PHASE_BITS - 12)                    # k=3
    dut = Dds(out_width=16, amplitude=amp, domain='sync')
    hw = _run(dut, inc, 4096 + 8)
    # settle the pipeline; the phase-0 sample recurs every 4096 samples
    lag, mism, _ = _match_lag(hw, nco_samples(len(hw), inc, amp))
    s0 = hw[lag]
    assert hw[lag + 4096] == s0, (hw[lag + 4096], s0)


def test_dds_offgrid_sfdr_below_bank_floor():
    """The model twin's intrinsic off-grid SFDR must sit comfortably below the
    -79 dBc wideband bank floor so it does not mask the bank's images."""
    amp, N, L = 8192, 4096, 1 << 17
    worst = -999.0
    for base in (200, 1500, 3500):
        for j in range(0, 17):                      # 1/16-bin steps (coherent)
            kfrac = base + j / 16.0
            inc = int(round(kfrac * (1 << (PHASE_BITS - 12))))
            z = nco_samples(L, inc, amp)
            Z = np.abs(np.fft.fft(z)) / L
            ci = int(round(kfrac / N * L)) % L
            b = Z.copy()
            for d in (-2, -1, 0, 1, 2):
                b[(ci + d) % L] = 0.0
            worst = max(worst, 20 * np.log10(b.max() / Z[ci]))
    assert worst < -88.0, f"DDS off-grid SFDR only {worst:.1f} dBc (need < -88)"
