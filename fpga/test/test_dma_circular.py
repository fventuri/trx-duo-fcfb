# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Circular-mode check for maia-hdl ``DmaStreamWrite`` (fcfb 3c).

fcfb runs the streaming DMA in ``circular=True`` mode so it never stops at the
ring end -- the write address wraps back to the start and the transfer keeps
running, eliminating the re-arm gap where the tiny egress FIFO could overflow
(the intermittent multi-record wrap loss at high W).  maia-hdl's own DMA tests
are cocotb/Verilog-simulator based; this is a pure-Amaranth ``pysim`` check (no
external simulator needed) driving a trivial always-ready AXI write slave, so it
runs in the fcfb suite alongside the datapath block tests.

It asserts, for a small ring:
  * circular mode issues bursts across many wraps, never writes the end address,
    wraps ``end-BURST -> start``, and never pulses ``finished`` (never stops);
  * ``circular=False`` is unchanged: exactly one lap then a single ``finished``.
"""
from amaranth.sim import Simulator

from maia_hdl.dma import DmaStreamWrite

START = 0x0001_0000
END = 0x0001_0400          # 0x400 = 1024 B = 16 x 64-B bursts
SHIFT = 6                  # burst_len_log2(4) + bytes_per_word_log2(2), width=32
BURST = 1 << SHIFT
NBURST = (END - START) // BURST


def _run(circular, ncycles):
    """Drive the DMA against an always-ready slave; return (awaddrs, n_finished).

    The slave asserts AWREADY/WREADY continuously and answers each burst's write
    response one cycle after WLAST (a realistic >=1-cycle B latency, needed for
    the one-shot ``finished`` edge to form).
    """
    dut = DmaStreamWrite(START, END, width=32, circular=circular)
    addrs = []
    n_finished = 0

    async def tb(ctx):
        nonlocal n_finished
        ctx.set(dut.axi.awready, 1)
        ctx.set(dut.axi.wready, 1)
        ctx.set(dut.stream_valid, 1)
        ctx.set(dut.stream_data, 0xABCD)
        ctx.set(dut.start, 1)
        await ctx.tick()
        ctx.set(dut.start, 0)
        pending_b = 0
        prev_pending = False
        for _ in range(ncycles):
            if ctx.get(dut.axi.awvalid) and ctx.get(dut.axi.awready):
                addrs.append(ctx.get(dut.axi.awaddr))
            if (ctx.get(dut.axi.wvalid) and ctx.get(dut.axi.wready)
                    and ctx.get(dut.axi.wlast)):
                pending_b += 1
            ctx.set(dut.axi.bvalid, 1 if prev_pending else 0)
            if prev_pending and ctx.get(dut.axi.bready):
                pending_b -= 1
            prev_pending = pending_b > 0
            if ctx.get(dut.finished):
                n_finished += 1
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()
    return addrs, n_finished


def test_circular_wraps_and_never_finishes():
    addrs, n_finished = _run(circular=True, ncycles=4000)
    assert addrs, 'no bursts issued'
    assert END not in addrs, 'end address must never be written'
    assert min(addrs) == START and max(addrs) == END - BURST
    wraps = sum(1 for i in range(1, len(addrs))
                if addrs[i - 1] == END - BURST and addrs[i] == START)
    assert wraps >= 3, f'expected multiple wraps, got {wraps}'
    assert n_finished == 0, 'circular mode must never pulse finished'
    assert len(addrs) > 3 * NBURST, 'addresses must keep flowing (no stall)'


def test_oneshot_unchanged():
    addrs, n_finished = _run(circular=False, ncycles=4000)
    assert END not in addrs
    assert len(addrs) == NBURST, f'one-shot must issue exactly one lap ({NBURST})'
    assert min(addrs) == START and max(addrs) == END - BURST
    assert n_finished == 1, 'one-shot must pulse finished exactly once'
