# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Multi-clock CDC gate for the wideband (Phase-3b) two-rate datapath.

This is the real correctness check for the sync->fft->sync clock-domain crossings
that ``Stage1Top(hop=R)`` introduces (a green timing report does NOT prove a CDC
is sound).  A small harness wires the wideband pieces EXACTLY as ``Stage1Top``
does -- the LUTRAM ``InCDC`` (sync->fft AsyncFIFO), the hop=R / ab_extra_pipe
``Stage1`` renamed into the fast ``fft`` domain, the async ``OutCDC`` (fft->sync
AsyncFIFO), and the per-field control FFSynchronizers -- then:

  * drives FS-rate packed samples into the sync side (one input per sync cycle),
  * runs ``fft`` faster than ``sync`` at the real 4:3 ratio (166.67 / 125),
  * collects the record word stream on the sync egress side,
  * asserts it is BYTE-EXACT vs ``Stage1.model`` (the domain-agnostic golden
    fold + FFT + backend), so the CDC neither drops, duplicates, nor reorders.

f_fft/f_sync = 4/3 satisfies the throughput floor R*(f_fft/f_sync) >= N+1 for the
matched q = N/R here (small-N proxy N=64, mirroring the Phase-3 margin test).
"""
import struct

import numpy as np
import pytest
from amaranth import *
from amaranth.lib.cdc import FFSynchronizer, PulseSynchronizer
from amaranth.lib.fifo import AsyncFIFO
from amaranth.sim import Simulator

from maia_fcfb.stage1 import Stage1
from maia_fcfb.mask_mem import MaskMem


class _WidebandHarness(Elaboratable):
    """The wideband datapath in isolation: InCDC + fft-domain Stage1 + OutCDC.

    Mirrors ``Stage1Top``'s wideband wiring; the ADC / DMA / AXI structural
    blocks are omitted so the whole thing is Amaranth-simulatable.  Control
    inputs are sync-domain and crossed to fft by FFSynchronizer, just like the
    top (they are quasi-static in the test, set before ``enable``).
    """
    def __init__(self, order_log2, T, hop, sample_width=16, shift_width=5,
                 incdc_depth=16, egress_depth=512, bin_width=16):
        self.order_log2 = order_log2
        self.sample_width = sample_width
        self.shift_width = shift_width
        self.stage1 = Stage1(
            width_in=17, order_log2=order_log2, window=None, cmult3x=False,
            shift_width=shift_width, T=T, sample_width=sample_width,
            hop=hop, ab_extra_pipe=True, quantise_extra_pipe=True,
            bin_width=bin_width)
        self.incdc = AsyncFIFO(width=2 * sample_width, depth=incdc_depth,
                               w_domain='sync', r_domain='fft')
        self.egress = AsyncFIFO(width=32, depth=egress_depth,
                                w_domain='fft', r_domain='sync')
        # sync-domain producer / control inputs
        self.enable = Signal()
        self.in_valid = Signal()
        self.re_in = Signal(signed(sample_width))
        self.im_in = Signal(signed(sample_width))
        self.mask_wr_addr = Signal(order_log2)
        self.mask_wr_data = Signal(2)
        self.mask_wr_en = Signal()
        self.mask_clear = Signal()
        self.shift = Signal(shift_width)
        # sync-domain egress
        self.out_data = Signal(32)
        self.out_valid = Signal()
        self.out_ready = Signal()

    def elaborate(self, platform):
        m = Module()
        m.domains += [ClockDomain('sync'), ClockDomain('fft')]
        m.submodules.stage1 = DomainRenamer({'sync': 'fft'})(self.stage1)
        m.submodules.incdc = incdc = self.incdc
        m.submodules.egress = egress = self.egress
        sw = self.sample_width
        s = self.stage1

        # producer (sync): push packed samples into the InCDC on valid.
        m.d.comb += [
            incdc.w_data.eq(Cat(self.re_in, self.im_in)),
            incdc.w_en.eq(self.enable & self.in_valid),
        ]

        # control registers crossed sync->fft (per-field FFSynchronizer).  The
        # keep-bitmap is a dual-clock BRAM (MaskMem): written directly in sync,
        # read in fft -- no control CDC, exactly as Stage1Top does it.
        enable_fft = Signal()
        shift_fft = Signal(self.shift_width)
        m.submodules.enable_cdc = FFSynchronizer(self.enable, enable_fft,
                                                 o_domain='fft')
        m.submodules.shift_cdc = FFSynchronizer(self.shift, shift_fft,
                                                o_domain='fft')
        m.submodules.maskmem = mm = MaskMem(
            self.order_log2, rd_domain='fft', wr_domain='sync')
        m.d.comb += [
            mm.wr_addr.eq(self.mask_wr_addr), mm.wr_data.eq(self.mask_wr_data),
            mm.wr_en.eq(self.mask_wr_en), mm.clear.eq(self.mask_clear),
            mm.rd_addr.eq(s.mask_rd_addr), mm.rd_en.eq(enable_fft),
            s.keep_a_in.eq(mm.keep_a), s.keep_b_in.eq(mm.keep_b),
        ]

        # fft consumer -> Stage1 (free-running reader; in_valid = InCDC r_rdy).
        m.d.comb += [
            s.clken.eq(enable_fft),
            s.in_valid.eq(incdc.r_rdy),
            s.re_in.eq(incdc.r_data[:sw].as_signed()),
            s.im_in.eq(incdc.r_data[sw:].as_signed()),
            incdc.r_en.eq(enable_fft),
            s.shift.eq(shift_fft),
        ]

        # egress: fft-domain record stream -> OutCDC -> sync consumer.
        m.d.comb += [
            egress.w_data.eq(s.out_data),
            egress.w_en.eq(s.out_valid),
            self.out_data.eq(egress.r_data),
            self.out_valid.eq(egress.r_rdy),
            egress.r_en.eq(self.out_ready),
        ]
        return m


def _split_records(byte_stream, Wa, Wb, bin_width=16):
    """Split a contiguous record byte stream into (payload, seq) tuples."""
    payload = (Wa + Wb) * (bin_width // 8) * 2
    rec_len = 8 + payload + ((-payload) % 4)      # + word-boundary pad
    assert len(byte_stream) % rec_len == 0, \
        f"{len(byte_stream)} not a multiple of record len {rec_len}"
    out = []
    for off in range(0, len(byte_stream), rec_len):
        rec = byte_stream[off:off + rec_len]
        out.append((rec[8:], struct.unpack_from("<Q", rec, 0)[0]))
    return out


def _masks(dut, spec):
    N = 1 << dut.order_log2
    if spec[0] == "run":
        return dut.stage1.run_masks(spec[1], spec[2], spec[3])
    _, ba, bb = spec
    a = np.zeros(N, dtype=bool); b = np.zeros(N, dtype=bool)
    for k0, W in ba:
        a[k0:k0 + W] = True
    for k0, W in bb:
        b[k0:k0 + W] = True
    return a, b


@pytest.mark.parametrize("order_log2,T,hop,spec,shift,bin_width", [
    (6, 4, 50, ("run", 20, 7, 0b01), 6, 16),      # N=64, q=1.28; A only
    (6, 4, 50, ("run", 5, 5, 0b11), 5, 16),       # A + B, different run
    (6, 4, 49, ("run", 10, 6, 0b10), 6, 16),      # B only, different hop
    (6, 4, 50, ("run", 20, 7, 0b01), 6, 24),      # int24: A only, odd W (tail pad)
    (6, 4, 50, ("run", 5, 5, 0b11), 5, 24),       # int24: A + B
    (6, 4, 50, ("custom", [(3, 4), (40, 6)], [(41, 5)]), 5, 24),  # 2 windows/ADC
])
def test_wideband_cdc_byte_exact(order_log2, T, hop, spec, shift, bin_width):
    N = 1 << order_log2
    sw = 16
    dut = _WidebandHarness(order_log2, T=T, hop=hop, sample_width=sw,
                           bin_width=bin_width)
    mask_a, mask_b = _masks(dut, spec)
    Wa, Wb = int(mask_a.sum()), int(mask_b.sum())

    rng = np.random.default_rng(abs(hash(repr((order_log2, hop, spec, shift))))
                                & 0xFFFFFFFF)
    nblk = 6
    amp = 1 << (sw - 1)
    nz = T * N + (nblk + 2) * hop
    z = (rng.integers(-amp, amp, nz) + 1j * rng.integers(-amp, amp, nz))

    exp_stream = dut.stage1.model(z, mask_a, mask_b, shift)
    exp = _split_records(exp_stream, Wa, Wb, bin_width)
    exp_payloads = [p for (p, _) in exp if p != b""]
    assert len(exp_payloads) >= nblk

    words = []

    async def tb(ctx):
        ctx.set(dut.out_ready, 1)
        ctx.set(dut.shift, shift)
        # Load the keep-bitmap first (datapath disabled), one bin per write with
        # enough sync cycles between pulses for the PulseSynchronizer to cross.
        for k in range(N):
            ka, kb = int(mask_a[k]), int(mask_b[k])
            if not (ka or kb):
                continue
            ctx.set(dut.mask_wr_addr, k)
            ctx.set(dut.mask_wr_data, (kb << 1) | ka)
            ctx.set(dut.mask_wr_en, 1)
            await ctx.tick("sync")
            ctx.set(dut.mask_wr_en, 0)
            for _ in range(6):
                await ctx.tick("sync")
        ctx.set(dut.enable, 1)
        # One input per sync cycle while feeding z; then flush.  fft runs 4/3
        # faster, so the free-running reader keeps up (R*4/3 >= N+1).
        total = len(z) + 12 * N
        for i in range(total):
            feed = i < len(z)
            ctx.set(dut.in_valid, 1 if feed else 0)
            if feed:
                ctx.set(dut.re_in, int(z[i].real))
                ctx.set(dut.im_in, int(z[i].imag))
            if ctx.get(dut.out_valid):
                words.append(ctx.get(dut.out_data))
            await ctx.tick("sync")

    sim = Simulator(dut)
    sim.add_clock(1e-6, domain='sync')
    sim.add_clock(0.75e-6, domain='fft')      # 4/3 faster than sync
    sim.add_testbench(tb)
    sim.run()

    # The egress word stream is a back-to-back concatenation of full records.
    byte_stream = b"".join(struct.pack("<I", w) for w in words)
    got = _split_records(byte_stream, Wa, Wb, bin_width)

    # Warm-up / flush blocks (all-zero payload) drop out; the matched records
    # must be a contiguous, in-order run of the model's blocks with consecutive
    # seqs -- proving the CDC preserved order and count with no drops/dups.
    matched = []
    for ei, (payload, seq) in enumerate(got):
        hits = [i for i, e in enumerate(exp_payloads) if e == payload]
        if hits:
            assert len(hits) == 1, f"record {ei} payload ambiguous: {hits}"
            matched.append((ei, hits[0], seq))

    assert len(matched) >= nblk, \
        f"only {len(matched)} records matched a WOLA block (need >= {nblk})"
    bi = [b for _, b, _ in matched]
    assert bi == list(range(bi[0], bi[0] + len(bi))), \
        f"matched blocks not contiguous/in-order: {bi}"
    ei = [e for e, _, _ in matched]
    assert ei == list(range(ei[0], ei[0] + len(ei))), \
        f"matched records not in consecutive emission slots: {ei}"
    sq = [s for _, _, s in matched]
    assert sq == list(range(sq[0], sq[0] + len(sq))), \
        f"seq not consecutive across matched blocks: {sq}"
