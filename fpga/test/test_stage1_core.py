# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Verify Stage1Core (packed dual-ADC windowed FFT + A/B split).

A. RTL (multi-clock: sync + clk2x + clk3x) of the *windowed*, cmult3x FFT feeding
   ABSplit -- the shipping config -- checked against the core model for BOTH the
   ADC0 (A) and ADC1 (B) spectra.
B. Model: an ADC0-only tone recovers in A and is absent from B (and symmetrically
   for ADC1) -- the dual-ADC separation that the PLAN §3 packing buys.
"""
import numpy as np
import pytest
from amaranth import *
from amaranth.sim import Simulator

from maia_fcfb.stage1_core import Stage1Core


# Inline copy of upstream test/common_edge.py (MIT) to drive common_edge_2x/3x.
class _CommonEdgeTb(Elaboratable):
    def __init__(self, dut, domains):
        self.dut = dut
        self.domains = domains

    def elaborate(self, platform):
        m = Module()
        m.submodules.dut = self.dut
        for domain, nx, name in self.domains:
            if hasattr(self.dut, name):
                d = Signal(nx, init=1, name=f'ce_del_{domain}')
                m.d[domain] += d.eq(Cat(d[-1], d))
                m.d.comb += getattr(self.dut, name).eq(d[1])
        return m


def relerr(a, b):
    return np.sqrt(np.sum(np.abs(a - b) ** 2) / max(np.sum(np.abs(b) ** 2), 1e-30))


def test_core_rtl_vs_model():
    order_log2 = 6
    N = 1 << order_log2
    width = 17
    core = Stage1Core(width, order_log2, window='blackmanharris', cmult3x=True,
                      domain_2x='clk2x', domain_3x='clk3x')
    dut = _CommonEdgeTb(core, [('clk2x', 2, 'common_edge_2x'),
                               ('clk3x', 3, 'common_edge_3x')])

    rng = np.random.default_rng(0)
    amp = 1 << (width - 3)          # keep |z| within the FFT input rule
    zeros = np.zeros(N, complex)
    dist = [(rng.integers(-amp, amp, N) + 1j * rng.integers(-amp, amp, N))
            for _ in range(4)]
    zseq = np.concatenate([zeros] + dist)

    collected = []

    async def tb(ctx):
        ctx.set(core.clken, 1)
        total = len(zseq) + 3 * N + core.latency + 8
        for i in range(total):
            if i < len(zseq):
                ctx.set(core.re_in, int(zseq[i].real))
                ctx.set(core.im_in, int(zseq[i].imag))
            else:
                ctx.set(core.re_in, 0); ctx.set(core.im_in, 0)
            if ctx.get(core.out_valid):
                collected.append((ctx.get(core.out_last),
                                  ctx.get(core.a_re), ctx.get(core.a_im),
                                  ctx.get(core.b_re), ctx.get(core.b_im)))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(12e-9)
    sim.add_clock(6e-9, domain='clk2x', phase=6e-9)
    sim.add_clock(4e-9, domain='clk3x', phase=6e-9)
    sim.add_testbench(tb)
    sim.run()

    outA, outB, cA, cB = [], [], [], []
    for last, ar, ai, br, bi in collected:
        cA.append(ar + 1j * ai); cB.append(br + 1j * bi)
        if last:
            if len(cA) == N:
                outA.append(np.array(cA)); outB.append(np.array(cB))
            cA, cB = [], []
    assert len(outA) >= 3, f"only {len(outA)} full frames"

    models = [core.model(f.real.astype(int), f.imag.astype(int)) for f in dist]
    matched = 0
    for Aof, Bof in zip(outA, outB):
        errs = [max(relerr(Aof, mA), relerr(Bof, mB)) for (mA, mB) in models]
        if min(errs) < 2e-2:
            matched += 1
    assert matched >= 3, f"only {matched} frames matched a model"
    # distinct inputs -> distinct A spectra (guard against stuck output)
    assert relerr(models[0][0], models[1][0]) > 0.2


@pytest.mark.parametrize("kbin", [11, 50, 100])
def test_dual_adc_separation(kbin):
    order_log2 = 8
    N = 1 << order_log2
    width = 17
    core = Stage1Core(width, order_log2, window=None, cmult3x=False)
    amp = 1 << (width - 3)
    n = np.arange(N)
    tone = np.round(amp * np.cos(2 * np.pi * kbin * n / N)).astype(int)
    z0 = np.zeros(N, int)

    # ADC0-only tone -> recovered in A, ~absent from B
    A, B = core.model(tone, z0)
    assert int(np.argmax(np.abs(A))) in (kbin, N - kbin), "A peak bin"
    sep_a = np.abs(A).max() / max(np.abs(B).max(), 1e-9)
    assert sep_a > 1000, f"A/B separation only {20*np.log10(sep_a):.0f} dB"

    # ADC1-only tone -> recovered in B, ~absent from A
    A2, B2 = core.model(z0, tone)
    assert int(np.argmax(np.abs(B2))) in (kbin, N - kbin), "B peak bin"
    sep_b = np.abs(B2).max() / max(np.abs(A2).max(), 1e-9)
    assert sep_b > 1000, f"B/A separation only {20*np.log10(sep_b):.0f} dB"
