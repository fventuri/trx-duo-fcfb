# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Full Stage-1 sim (DESIGN step 4b): FFT -> A/B split -> bin-select -> quantise
-> stream-format, run end to end and compared BYTE-EXACT to ``Stage1.model``.

Feeds several random integer frames z = adc0 + j*adc1 back to back, collects the
emitted 32-bit BlockRecord words, reconstructs the record byte stream, and checks
each emitted record equals the model's record for the corresponding input frame
(matched by payload; the FFT pipeline offsets the output frames by a fixed whole
number of frames, and ``seq`` is verified to increment by 1 per emitted record).

Selection is now a per-ADC keep-bitmap loaded into the DUT before streaming; a
contiguous run (k0, W, adc_mask) is the regression special case (== the old wire
bytes).  window=None / cmult3x=False here (single sync domain, T=1 first-light);
the WOLA T=4 prefilter path is exercised too.
"""
import struct
import numpy as np
import pytest
from amaranth import *
from amaranth.sim import Simulator

from maia_fcfb.stage1 import Stage1
from maia_fcfb.mask_mem import MaskMem


class _Stage1WithMask(Elaboratable):
    """Stage1 + its external MaskMem BRAM, wired as in stage1_top (single domain
    here).  Re-exposes Stage1's I/O + the mask load/clear so the full-chain test
    drives the real integrated path; delegates .model/.run_masks/.core."""
    def __init__(self, *args, **kwargs):
        self.s1 = Stage1(*args, **kwargs)
        ol2 = self.s1.order_log2
        self.mm = MaskMem(ol2)
        # expose Stage1 I/O (same signal objects)
        self.clken = self.s1.clken; self.in_valid = self.s1.in_valid
        self.re_in = self.s1.re_in; self.im_in = self.s1.im_in
        self.shift = self.s1.shift
        self.out_valid = self.s1.out_valid
        self.out_data = self.s1.out_data; self.out_last = self.s1.out_last
        # mask write side (MaskMem)
        self.mask_wr_addr = self.mm.wr_addr; self.mask_wr_data = self.mm.wr_data
        self.mask_wr_en = self.mm.wr_en; self.mask_clear = self.mm.clear
        self.mask_busy = self.mm.busy
        # delegates
        self.order_log2 = ol2; self.core = self.s1.core
        self.model = self.s1.model; self.run_masks = self.s1.run_masks

    def elaborate(self, platform):
        m = Module()
        m.submodules.s1 = self.s1
        m.submodules.mm = self.mm
        m.d.comb += [
            self.mm.rd_addr.eq(self.s1.mask_rd_addr),
            self.mm.rd_en.eq(self.s1.clken),
            self.s1.keep_a_in.eq(self.mm.keep_a),
            self.s1.keep_b_in.eq(self.mm.keep_b),
        ]
        return m


def _records(byte_stream, Wa, Wb, bin_width=16):
    """Split a header-less record byte stream into (payload-bytes, seq) list."""
    payload = (Wa + Wb) * (bin_width // 8) * 2
    rec_len = 8 + payload + ((-payload) % 4)      # + word-boundary pad
    assert len(byte_stream) % rec_len == 0, \
        f"{len(byte_stream)} not a multiple of record len {rec_len}"
    out = []
    for off in range(0, len(byte_stream), rec_len):
        rec = byte_stream[off:off + rec_len]
        seq = struct.unpack_from("<Q", rec, 0)[0]
        out.append((rec[8:], seq))
    return out


async def _load_mask(ctx, dut, mask_a, mask_b, N):
    """Load the per-ADC keep-bitmap while the datapath is disabled."""
    ctx.set(dut.clken, 0)
    for k in range(N):
        ka, kb = int(mask_a[k]), int(mask_b[k])
        if not (ka or kb):
            continue
        ctx.set(dut.mask_wr_addr, k)
        ctx.set(dut.mask_wr_data, (kb << 1) | ka)
        ctx.set(dut.mask_wr_en, 1)
        await ctx.tick()
    ctx.set(dut.mask_wr_en, 0)


def _build_masks(dut, spec):
    """spec -> (mask_a, mask_b).  ('run', k0, W, adc_mask) or ('custom', a, b)."""
    N = 1 << dut.order_log2
    if spec[0] == "run":
        return dut.run_masks(spec[1], spec[2], spec[3])
    _, blocks_a, blocks_b = spec
    a = np.zeros(N, dtype=bool)
    b = np.zeros(N, dtype=bool)
    for k0, W in blocks_a:
        a[k0:k0 + W] = True
    for k0, W in blocks_b:
        b[k0:k0 + W] = True
    return a, b


# (spec, shift, bin_width)
_SCENES = [
    (("run", 20, 7, 0b01), 6, 16),      # regression: A only
    (("run", 20, 7, 0b11), 6, 16),      # regression: A + B
    (("run", 5, 5, 0b10), 4, 16),       # regression: B only
    (("run", 1, 12, 0b11), 8, 16),      # regression: wide run near DC
    (("run", 20, 7, 0b01), 6, 24),      # regression int24: A only, odd W
    (("run", 20, 7, 0b11), 6, 24),      # regression int24: A + B
    (("run", 1, 12, 0b11), 4, 24),      # regression int24: wide run
    (("custom", [(3, 4), (40, 6)], [(41, 5)]), 6, 16),   # 2 windows, per-ADC diff
    (("custom", [(3, 4), (40, 6)], [(41, 5)]), 6, 24),   # ... int24
    (("custom", [(10, 3)], [(30, 8)]), 5, 24),           # W_a != W_b
]


@pytest.mark.parametrize("spec,shift,bin_width", _SCENES)
def test_stage1_full_chain(spec, shift, bin_width):
    order_log2 = 6
    width_in = 17
    N = 1 << order_log2
    dut = _Stage1WithMask(width_in, order_log2, window=None, cmult3x=False,
                          seq_width=64, wmax=64, bin_width=bin_width)
    mask_a, mask_b = _build_masks(dut, spec)
    Wa, Wb = int(mask_a.sum()), int(mask_b.sum())

    rng = np.random.default_rng(abs(hash(repr((spec, shift, bin_width)))) & 0xFFFFFFFF)
    amp = 1 << (width_in - 3)
    P = 6
    frames = [(rng.integers(-amp, amp, N) + 1j * rng.integers(-amp, amp, N))
              for _ in range(P)]

    words = []

    async def tb(ctx):
        await _load_mask(ctx, dut, mask_a, mask_b, N)
        ctx.set(dut.clken, 1)
        ctx.set(dut.shift, shift)
        total = (P + 3) * N + dut.core.latency + 64
        for i in range(total):
            f, p = divmod(i, N)
            if f < P:
                ctx.set(dut.re_in, int(frames[f][p].real))
                ctx.set(dut.im_in, int(frames[f][p].imag))
            else:
                ctx.set(dut.re_in, 0)
                ctx.set(dut.im_in, 0)
            if ctx.get(dut.out_valid):
                words.append((ctx.get(dut.out_last), ctx.get(dut.out_data)))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    byte_stream, cur = b"", []
    for last, data in words:
        cur.append(data)
        if last:
            byte_stream += b"".join(struct.pack("<I", w) for w in cur)
            cur = []
    assert cur == [], "trailing words with no out_last (incomplete record)"

    got = _records(byte_stream, Wa, Wb, bin_width)
    exp = _records(dut.model(frames, mask_a, mask_b, shift), Wa, Wb, bin_width)
    exp_payloads = [p for (p, _) in exp]

    matched = []
    for ei, (payload, seq) in enumerate(got):
        hits = [i for i, e in enumerate(exp_payloads) if e == payload]
        if hits:
            assert len(hits) == 1, f"record {ei} payload is ambiguous: {hits}"
            matched.append((ei, hits[0], seq))

    in_frames = [mf for _, mf, _ in matched]
    assert in_frames == list(range(P)), \
        f"matched input frames {in_frames}, expected {list(range(P))}"
    emit_idx = [ei for ei, _, _ in matched]
    assert emit_idx == list(range(emit_idx[0], emit_idx[0] + P)), \
        f"matched records not in consecutive emission slots: {emit_idx}"
    seqs = [s for _, _, s in matched]
    assert seqs == list(range(seqs[0], seqs[0] + P)), \
        f"seq not consecutive across the P frames: {seqs}"


# (spec, shift, T, bin_width)
_SCENES_WOLA = [
    (("run", 20, 7, 0b01), 6, 4, 16),                 # regression: A only, WOLA
    (("run", 5, 5, 0b11), 5, 4, 16),                  # regression: A + B, WOLA
    (("run", 20, 7, 0b01), 6, 4, 24),                 # regression int24: A only
    (("run", 5, 5, 0b11), 5, 4, 24),                  # regression int24: A + B
    (("custom", [(3, 4), (40, 6)], [(41, 5)]), 5, 4, 24),  # 2 windows, per-ADC
]


@pytest.mark.parametrize("spec,shift,T,bin_width", _SCENES_WOLA)
def test_stage1_full_chain_wola(spec, shift, T, bin_width):
    """Full T=4 chain: WOLA prefilter -> FFT -> A/B -> select -> quantise ->
    stream, byte-exact vs the composed model (WOLA fold + FFT + back end)."""
    order_log2 = 6
    sw = 16
    N = 1 << order_log2
    dut = _Stage1WithMask(width_in=17, order_log2=order_log2, window=None,
                          cmult3x=False, T=T, sample_width=sw, bin_width=bin_width)
    mask_a, mask_b = _build_masks(dut, spec)
    Wa, Wb = int(mask_a.sum()), int(mask_b.sum())

    rng = np.random.default_rng(abs(hash(repr((spec, shift, T, bin_width)))) & 0xFFFFFFFF)
    amp = 1 << (sw - 1)
    ncols = 10
    z = (rng.integers(-amp, amp, ncols * N)
         + 1j * rng.integers(-amp, amp, ncols * N))

    words = []

    async def tb(ctx):
        await _load_mask(ctx, dut, mask_a, mask_b, N)
        ctx.set(dut.clken, 1)
        ctx.set(dut.in_valid, 1)
        ctx.set(dut.shift, shift)
        total = (ncols + 5) * N + dut.core.latency + 64
        for i in range(total):
            if i < len(z):
                ctx.set(dut.re_in, int(z[i].real))
                ctx.set(dut.im_in, int(z[i].imag))
            else:
                ctx.set(dut.re_in, 0)
                ctx.set(dut.im_in, 0)
            if ctx.get(dut.out_valid):
                words.append((ctx.get(dut.out_last), ctx.get(dut.out_data)))
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(tb)
    sim.run()

    byte_stream, cur = b"", []
    for last, data in words:
        cur.append(data)
        if last:
            byte_stream += b"".join(struct.pack("<I", w) for w in cur)
            cur = []
    assert cur == [], "trailing words with no out_last (incomplete record)"

    got = _records(byte_stream, Wa, Wb, bin_width)
    exp = _records(dut.model(z, mask_a, mask_b, shift), Wa, Wb, bin_width)
    exp_payloads = [p for (p, _) in exp]

    matched = []
    for ei, (payload, seq) in enumerate(got):
        hits = [i for i, e in enumerate(exp_payloads) if e == payload and e != b""]
        if hits:
            assert len(hits) == 1, f"record {ei} payload ambiguous: {hits}"
            matched.append((ei, hits[0], seq))

    assert len(matched) >= 3, f"only {len(matched)} records matched a WOLA block"
    fi = [f for _, f, _ in matched]
    assert fi == list(range(fi[0], fi[0] + len(fi))), \
        f"matched WOLA blocks not contiguous/in-order: {fi}"
    ei = [e for e, _, _ in matched]
    assert ei == list(range(ei[0], ei[0] + len(ei))), \
        f"matched records not in consecutive emission slots: {ei}"
    sq = [s for _, _, s in matched]
    assert sq == list(range(sq[0], sq[0] + len(sq))), \
        f"seq not consecutive across matched blocks: {sq}"
