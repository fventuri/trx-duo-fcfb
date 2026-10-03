# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Amaranth simulation of the ABSplit RTL vs. its model + a bit-exact check.

Streams several random complex spectra ``C`` through the module in the FFT's
bit-reversed bin order (frames back-to-back), collects the recovered A/B output
frames (delimited by ``out_last``), and checks each against:
  * ``module.model(C)`` (float A/B, loose relerr), and
  * the BIT-EXACT fixed-point expectation ``(C[k] +/- conj C[N-k]) >> 1`` (the
    block is pure add/sub + arithmetic >>1, so it must match exactly).
"""
import numpy as np
import pytest
from amaranth.sim import Simulator
from maia_hdl.util import bit_invert

from maia_fcfb.ab_split import ABSplit


def relerr(a, b):
    return np.sqrt(np.sum(np.abs(a - b) ** 2) / max(np.sum(np.abs(b) ** 2), 1e-30))


def exact_ab(C):
    """Bit-exact fixed-point A/B: matches the RTL's add/sub + arithmetic >>1.

    A_re=(Cr+Mr)>>1  A_im=(Ci-Mi)>>1   B_re=(Ci+Mi)>>1  B_im=(Mr-Cr)>>1
    where M = C[N-k]; Python >> floors toward -inf, as Amaranth's signed >>.
    """
    N = C.size
    k = np.arange(N)
    M = C[(N - k) % N]
    Cr, Ci = C.real.astype(int), C.imag.astype(int)
    Mr, Mi = M.real.astype(int), M.imag.astype(int)
    a_re = (Cr + Mr) >> 1
    a_im = (Ci - Mi) >> 1
    b_re = (Ci + Mi) >> 1
    b_im = (Mr - Cr) >> 1
    return a_re + 1j * a_im, b_re + 1j * b_im


@pytest.mark.parametrize("extra_pipe", [False, True])
@pytest.mark.parametrize("order_log2", [4, 6])
def test_rtl_matches_model(order_log2, extra_pipe):
    width = 23                                 # fcfb Stage-1 FFT output width
    N = 1 << order_log2
    dut = ABSplit(width, order_log2, extra_pipe=extra_pipe)

    rng = np.random.default_rng(order_log2)
    nframes = 5
    amp = 1 << (width - 2)
    frames = [(rng.integers(-amp, amp, N) + 1j * rng.integers(-amp, amp, N))
              for _ in range(nframes)]
    inv = [bit_invert(p, order_log2, 1) for p in range(N)]   # stream order

    collected = []           # (out_last, a_re, a_im, b_re, b_im) while out_valid

    async def tb(ctx):
        ctx.set(dut.clken, 1)
        total_cycles = (nframes + 2) * N + dut.latency + 4
        f = 0
        p = 0
        for _ in range(total_cycles):
            if f < nframes:
                val = frames[f][inv[p]]
                ctx.set(dut.re_in, int(val.real))
                ctx.set(dut.im_in, int(val.imag))
            ctx.set(dut.input_last, 1 if p == N - 1 else 0)
            if ctx.get(dut.out_valid):
                collected.append((ctx.get(dut.out_last),
                                  ctx.get(dut.a_re), ctx.get(dut.a_im),
                                  ctx.get(dut.b_re), ctx.get(dut.b_im)))
            await ctx.tick()
            p += 1
            if p == N:
                p = 0
                f += 1

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    # Split collected stream into frames on out_last.
    out_A, out_B, curA, curB = [], [], [], []
    for last, ar, ai, br, bi in collected:
        curA.append(ar + 1j * ai)
        curB.append(br + 1j * bi)
        if last:
            out_A.append(np.array(curA)); out_B.append(np.array(curB))
            curA, curB = [], []

    assert len(out_A) >= nframes - 1, f"got {len(out_A)} output frames"
    for j in range(min(len(out_A), nframes)):
        assert len(out_A[j]) == N, f"frame {j}: {len(out_A[j])} bins (expected {N})"
        A_flt, B_flt = dut.model(frames[j])
        A_ext, B_ext = exact_ab(frames[j])
        # loose float check
        assert relerr(out_A[j], A_flt) < 1e-3, f"A frame {j} relerr"
        assert relerr(out_B[j], B_flt) < 1e-3, f"B frame {j} relerr"
        # bit-exact check (the real acceptance)
        np.testing.assert_array_equal(out_A[j], A_ext, err_msg=f"A frame {j}")
        np.testing.assert_array_equal(out_B[j], B_ext, err_msg=f"B frame {j}")
