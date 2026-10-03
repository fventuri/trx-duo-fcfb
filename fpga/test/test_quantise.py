# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Amaranth sim of Quantise vs a bit-exact round-half-up + saturate model."""
import numpy as np
import pytest
from amaranth.sim import Simulator

from maia_fcfb.quantise import Quantise, I16_MAX, I16_MIN


def q_ref(x, shift, omin, omax):
    """Bit-exact: round-half-up by `shift`, then saturate to [omin, omax]."""
    y = x if shift == 0 else (x + (1 << (shift - 1))) >> shift   # floor after +half
    return int(np.clip(y, omin, omax))


@pytest.mark.parametrize("out_width", [16, 24])
@pytest.mark.parametrize("shift", [0, 1, 6, 7, 8, 12])
def test_quantise_matches_ref(shift, out_width):
    w = 23
    dut = Quantise(w, shift_width=5, out_width=out_width)
    omax = 2 ** (out_width - 1) - 1
    omin = -(2 ** (out_width - 1))
    if out_width == 16:
        assert (omin, omax) == (I16_MIN, I16_MAX)
    rng = np.random.default_rng(shift ^ (out_width << 4))
    lim = 1 << (w - 1)                         # signed(w) range: [-lim, lim-1]
    # random values (already exercise saturation for small shift) + in-range
    # edge cases that probe rounding at +/- half-LSB and the clip points.
    vals = list(rng.integers(-lim, lim, 400))
    half = (1 << shift) if shift else 1
    vals += [0, 1, -1, lim - 1, -lim,
             half // 2, half // 2 - 1, -(half // 2), -(half // 2) - 1,
             omax * half if omax * half < lim else lim - 1,
             omin * half if -omin * half <= lim else -lim]
    re = [int(np.clip(v, -lim, lim - 1)) for v in vals]
    im = [int(np.clip(-v, -lim, lim - 1)) for v in vals]
    n = len(re)

    got = []

    async def tb(ctx):
        ctx.set(dut.clken, 1)
        ctx.set(dut.shift, shift)
        for i in range(n):
            ctx.set(dut.re_in, re[i])
            ctx.set(dut.im_in, im[i])
            ctx.set(dut.in_valid, 1)
            if ctx.get(dut.out_valid):
                got.append((ctx.get(dut.re_out), ctx.get(dut.im_out)))
            await ctx.tick()
        # flush last registered output
        await ctx.tick()
        if ctx.get(dut.out_valid):
            got.append((ctx.get(dut.re_out), ctx.get(dut.im_out)))

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    assert len(got) == n, f"{len(got)} outputs vs {n} inputs"
    for i in range(n):
        assert got[i][0] == q_ref(re[i], shift, omin, omax), \
            f"re[{i}]={re[i]} shift={shift} ow={out_width}: " \
            f"{got[i][0]} != {q_ref(re[i], shift, omin, omax)}"
        assert got[i][1] == q_ref(im[i], shift, omin, omax), \
            f"im[{i}]={im[i]} shift={shift} ow={out_width}: " \
            f"{got[i][1]} != {q_ref(im[i], shift, omin, omax)}"
