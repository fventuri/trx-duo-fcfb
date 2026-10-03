# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Exact ring-lap counter (fcfb ``dma_laps`` register).

The board server used to detect ring wraps in software by polling the modulo
``next_address`` pointer over AXI-Lite and testing for a large backward step.
A glitchy/aliased poll near the ring end could miss or fake a wrap, mis-tracking
``produced`` by a full lap and de-framing the UDP egress.

The fix moves lap counting into the PL, where ``next_address`` is visible on
EVERY clock edge: a wrap is exactly "``next_address`` decreased since last
cycle", counted once per wrap with no possibility of aliasing.  This test drives
the real maia-hdl ``DmaStreamWrite`` in circular mode and checks that the counter
logic copied verbatim from ``stage1_top`` matches the true wrap count from the
AXI ``awaddr`` sequence, counts each wrap exactly once (including the momentary
end-address cycle), and resets to 0 on ``dma_start``.
"""
from amaranth import *
from amaranth.sim import Simulator

from maia_hdl.dma import DmaStreamWrite

START = 0x0001_0000
END = 0x0001_0400          # 0x400 = 1024 B = 16 x 64-B bursts
SHIFT = 6
BURST = 1 << SHIFT
NBURST = (END - START) // BURST


class _DmaWithLaps(Elaboratable):
    """DmaStreamWrite plus the exact stage1_top lap counter around it."""
    def __init__(self):
        self.dma = DmaStreamWrite(START, END, width=32, circular=True)
        self.laps = Signal(32)

    def elaborate(self, platform):
        m = Module()
        m.submodules.dma = dma = self.dma
        # --- verbatim from stage1_top.py ---
        dma_next_prev = Signal(len(dma.next_address))
        m.d.sync += dma_next_prev.eq(dma.next_address)
        with m.If(dma.start):
            m.d.sync += [self.laps.eq(0), dma_next_prev.eq(0)]
        with m.Elif(dma.next_address < dma_next_prev):
            m.d.sync += self.laps.eq(self.laps + 1)
        return m


def _run(ncycles, restart_at=None):
    top = _DmaWithLaps()
    dut = top.dma
    addrs = []
    laps_samples = []

    async def tb(ctx):
        ctx.set(dut.axi.awready, 1)
        ctx.set(dut.axi.wready, 1)
        ctx.set(dut.stream_valid, 1)
        ctx.set(dut.stream_data, 0xABCD)
        ctx.set(dut.start, 1)
        await ctx.tick()
        ctx.set(dut.start, 0)
        pending_b = 0
        prev_pending = False
        for c in range(ncycles):
            if restart_at is not None and c == restart_at:
                ctx.set(dut.start, 1)
            elif restart_at is not None and c == restart_at + 1:
                ctx.set(dut.start, 0)
            if ctx.get(dut.axi.awvalid) and ctx.get(dut.axi.awready):
                addrs.append(ctx.get(dut.axi.awaddr))
            if (ctx.get(dut.axi.wvalid) and ctx.get(dut.axi.wready)
                    and ctx.get(dut.axi.wlast)):
                pending_b += 1
            ctx.set(dut.axi.bvalid, 1 if prev_pending else 0)
            if prev_pending and ctx.get(dut.axi.bready):
                pending_b -= 1
            prev_pending = pending_b > 0
            laps_samples.append(ctx.get(top.laps))
            await ctx.tick()

    sim = Simulator(top)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()
    return addrs, laps_samples


def _true_wraps(addrs):
    return sum(1 for i in range(1, len(addrs))
               if addrs[i - 1] == END - BURST and addrs[i] == START)


def test_laps_matches_true_wrap_count():
    addrs, laps = _run(ncycles=4000)
    wraps = _true_wraps(addrs)
    assert wraps >= 3, f'test needs several wraps, got {wraps}'
    # Final lap count equals the number of AXI awaddr wraps exactly (no miss,
    # no double-count on the momentary end-address cycle).
    assert laps[-1] == wraps, f'laps={laps[-1]} but true wraps={wraps}'
    # Monotone non-decreasing and only ever steps by 1.
    for a, b in zip(laps, laps[1:]):
        assert b - a in (0, 1), f'laps stepped by {b - a}'


def test_laps_resets_on_start():
    # Re-pulse start partway through: the counter must drop back to 0 and resume
    # counting wraps of the fresh run (a retune re-arms the DMA -> lap 0 again).
    addrs, laps = _run(ncycles=4000, restart_at=1500)
    assert min(laps[1502:]) == 0, 'laps must return to 0 after a restart pulse'
    assert laps[-1] >= 1, 'laps must resume counting after the restart'
