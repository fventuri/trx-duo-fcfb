# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Amaranth sim of StreamFormat: emitted words must equal the fcfb_stream.h
BlockRecord byte layout, the same one host/fcfb_stream.h reads.  Covers both the
shipping int16 (one word per bin {Q,I}) and the Option-B int24 packing (48-bit
bins bit-packed into a word stream, payload zero-padded to a word boundary), now
with per-ADC selection (W_a != W_b) and the ping-pong record buffers."""
import struct
import pytest
from amaranth.sim import Simulator

from maia_fcfb.stream_format import StreamFormat


def _av(j, bin_width):
    span = 1 << (bin_width - 2)
    return ((j * 7 - 3) % span, (j * 5 + 5) % span)


def _bv(j, bin_width):
    span = 1 << (bin_width - 2)
    return (-((j * 3 + 1) % span), (100 - j) % span)


def _enc(v, bin_width):
    return int(v).to_bytes(bin_width // 8, "little", signed=True)


def _expected_record(Wa, Wb, seq, bin_width):
    """The byte-exact record for a frame with Wa A-bins and Wb B-bins."""
    payload = b""
    for j in range(Wa):
        ai, aq = _av(j, bin_width)
        payload += _enc(ai, bin_width) + _enc(aq, bin_width)
    for j in range(Wb):
        bi, bq = _bv(j, bin_width)
        payload += _enc(bi, bin_width) + _enc(bq, bin_width)
    payload += b"\x00" * ((-len(payload)) % 4)
    return struct.pack("<Q", seq) + payload


def _frame_cycles(Wa, Wb, seq, bin_width, tail_idle=0):
    """Per-cycle drive ops for one frame: interleave A/B writes, frame_last last.

    Each op is a dict of signal values.  A bins are written at addr 0..Wa-1 (in
    order), B at 0..Wb-1; the two run on independent counters, one write of each
    where present.  ``frame_last`` lands on the final op (optionally on an idle
    tail cycle, exercising the "bin N-1 not kept" case with addr held).
    """
    ncyc = max(Wa, Wb, 1)
    ops = []
    for c in range(ncyc):
        op = dict(we_a=0, we_b=0, addr_a=min(c, Wa), addr_b=min(c, Wb),
                  a_i=0, a_q=0, b_i=0, b_q=0, seq=seq, frame_last=0)
        if c < Wa:
            ai, aq = _av(c, bin_width)
            op.update(we_a=1, addr_a=c, a_i=ai, a_q=aq)
        if c < Wb:
            bi, bq = _bv(c, bin_width)
            op.update(we_b=1, addr_b=c, b_i=bi, b_q=bq)
        ops.append(op)
    # optional idle tail (no writes); addr_* held at the totals.
    for _ in range(tail_idle):
        ops.append(dict(we_a=0, we_b=0, addr_a=Wa, addr_b=Wb,
                        a_i=0, a_q=0, b_i=0, b_q=0, seq=seq, frame_last=0))
    ops[-1]["frame_last"] = 1
    return ops


def _run(dut, ops, drain):
    """Drive a flat op stream, collect (out_last, out_data) on out_valid."""
    words = []

    async def tb(ctx):
        ctx.set(dut.clken, 1)
        for op in ops:
            for k, val in op.items():
                ctx.set(getattr(dut, k), val)
            if ctx.get(dut.out_valid):
                words.append((ctx.get(dut.out_last), ctx.get(dut.out_data)))
            await ctx.tick()
        # idle + drain
        for s in ("we_a", "we_b", "frame_last"):
            ctx.set(getattr(dut, s), 0)
        for _ in range(drain):
            if ctx.get(dut.out_valid):
                words.append((ctx.get(dut.out_last), ctx.get(dut.out_data)))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()
    return words


def _split_records(words):
    """Split the (out_last, out_data) stream into record byte strings."""
    recs, cur = [], []
    for last, data in words:
        cur.append(data)
        if last:
            recs.append(b"".join(struct.pack("<I", d) for d in cur))
            cur = []
    assert cur == [], "trailing words with no out_last"
    return recs


@pytest.mark.parametrize("bin_width", [16, 24])
@pytest.mark.parametrize("Wa,Wb,seq", [
    (7, 7, 0),             # A + B equal
    (7, 0, 42),            # A only
    (0, 5, 7),             # B only
    (1, 1, 1),             # single bin, both ADCs
    (1, 0, 3),             # single bin, A only (int24: 6B payload -> 2B pad)
    (7, 3, 11),            # W_a != W_b
    (3, 9, 12),            # W_a != W_b (more B)
    (0, 0, 99),            # empty frame -> seq-only record
    (33, 5, 123456789),    # wide, big seq
    (300, 300, 5),         # the W that broke the M4 wide-window HW capture
    (511, 0, 4),           # odd total, single ADC (int24 tail pad)
    (512, 512, 2),         # full wmax, both ADCs
])
def test_stream_format(bin_width, Wa, Wb, seq):
    dut = StreamFormat(12, wmax=512, seq_width=64, bin_width=bin_width)
    ops = _frame_cycles(Wa, Wb, seq, bin_width, tail_idle=(1 if Wa != Wb else 0))
    words = _run(dut, ops, drain=3 * (Wa + Wb) + 16)
    recs = _split_records(words)
    assert len(recs) == 1, f"expected 1 record, got {len(recs)}"
    assert recs[0] == _expected_record(Wa, Wb, seq, bin_width), \
        f"bw={bin_width} Wa={Wa} Wb={Wb}: record mismatch"


@pytest.mark.parametrize("bin_width", [16, 24])
@pytest.mark.parametrize("Wa,Wb,seq", [
    (513, 0, 7),           # just past the old 512 ceiling, single ADC (aw 9->10)
    (600, 600, 3),         # both ADCs well past 512
    (1024, 0, 8),          # full wmax=1024, single ADC (int24 tail pad path)
    (1024, 1024, 9),       # full wmax=1024, both ADCs
    (513, 511, 1),         # W_a != W_b straddling the old boundary
])
def test_stream_format_wmax1024(bin_width, Wa, Wb, seq):
    """wmax=1024 crosses the StreamFormat bank address 2^9->2^10 (aw 9->10,
    bankdepth 512->1024).  Confirm records stay bit-exact past the old 512 cap."""
    dut = StreamFormat(12, wmax=1024, seq_width=64, bin_width=bin_width)
    ops = _frame_cycles(Wa, Wb, seq, bin_width, tail_idle=(1 if Wa != Wb else 0))
    words = _run(dut, ops, drain=3 * (Wa + Wb) + 16)
    recs = _split_records(words)
    assert len(recs) == 1, f"expected 1 record, got {len(recs)}"
    assert recs[0] == _expected_record(Wa, Wb, seq, bin_width), \
        f"wmax1024 bw={bin_width} Wa={Wa} Wb={Wb}: record mismatch"


@pytest.mark.parametrize("bin_width", [16, 24])
def test_stream_format_pingpong(bin_width):
    """Two frames where frame 2's early writes land WHILE frame 1 is still
    draining: a single buffer would corrupt frame 1.  Double-buffering must keep
    both records exact.  Frame 2's writes are driven right after frame 1's
    frame_last (no gap), then idle cycles let frame 1 finish before frame 2's own
    frame_last fires (kept short so the whole thing is < the emit budget)."""
    Wa1, Wb1, seq1 = 6, 4, 100
    Wa2, Wb2, seq2 = 5, 7, 101
    dut = StreamFormat(12, wmax=512, seq_width=64, bin_width=bin_width)

    # Frame 1, then frame 2's write bins immediately (during frame 1's emit),
    # then idle cycles so frame 1 finishes, then frame 2's frame_last on idle.
    ops = _frame_cycles(Wa1, Wb1, seq1, bin_width)
    f2_writes = _frame_cycles(Wa2, Wb2, seq2, bin_width)
    for op in f2_writes:               # strip frame 2's own frame_last for now
        op["frame_last"] = 0
    ops += f2_writes
    # long idle so frame 1 fully drains, then assert frame 2 frame_last.
    for i in range(3 * (Wa1 + Wb1) + 16):
        last = (i == 3 * (Wa1 + Wb1) + 15)
        ops.append(dict(we_a=0, we_b=0, addr_a=Wa2, addr_b=Wb2,
                        a_i=0, a_q=0, b_i=0, b_q=0, seq=seq2,
                        frame_last=1 if last else 0))

    words = _run(dut, ops, drain=3 * (Wa2 + Wb2) + 16)
    recs = _split_records(words)
    assert len(recs) == 2, f"expected 2 records, got {len(recs)}"
    assert recs[0] == _expected_record(Wa1, Wb1, seq1, bin_width), "frame 1 corrupt"
    assert recs[1] == _expected_record(Wa2, Wb2, seq2, bin_width), "frame 2 corrupt"
