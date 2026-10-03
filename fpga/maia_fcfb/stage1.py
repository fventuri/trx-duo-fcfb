# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Reuses the maia-hdl FFT (© Daniel Estevez, MIT) via Stage1Core, and this
# project's ABSplit / BinSelect / Quantise / StreamFormat blocks.
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``stage1`` — the full fcfb Stage-1 datapath (sync domain), FFT input to wire.

Chains the blocks built in steps 3/4a into one Elaboratable::

    re_in/im_in (ADC0/ADC1)
        -> Stage1Core   (windowed FFT + A/B split, width_out=23, natural order)
        -> BinSelect    (keep a per-ADC bitmap set of bins; tag cnt_a/cnt_b/
                         frame_last/seq)
        -> Quantise x2  (A and B independently: shift -> round-half-up -> int16/24)
        -> StreamFormat (fcfb_stream.h BlockRecords as a 32-bit LE AXI-stream)

The one alignment subtlety (DESIGN step 4b): BinSelect emits the per-ADC keep
valids (``keep_a``/``keep_b``), the per-ADC write addresses (``cnt_a``/``cnt_b``)
and the ``frame_last``/``seq`` tags at its output stage, but Quantise adds register
stages to the a/b data. The keep valids ride the quantisers' own ``out_valid``
(same latency); ``cnt_a``/``cnt_b``/``frame_last``/``seq`` are delayed by
``Quantise.latency`` (1 shipping, 2 wideband) so they line up with the quantised
I/Q. Two Quantise instances share one ``shift``.

The two ADCs are selected independently via the keep-bitmap (``mask_a``/``mask_b``);
both spectra are always computed by the split — the bitmap only gates the wire.
Control inputs: ``shift`` plus the keep-bitmap load/clear (``mask_wr_*``,
``mask_clear``, status ``mask_busy``), all runtime registers set by the PS
board-server (wired to AXI-Lite at the top; CDC'd into the fft domain when
wideband).

The stream header (magic/version/N/fs/masks/nblocks/bin_scale/bin_width) is written
once by PS software; the PL streams only the per-block records here.

For M3 first-light use ``window=None`` / ``cmult3x=False`` (single sync domain,
T=1); the shipping config adds the window + 2×/3× domains. Numerics are identical
either way for the parts below the FFT.
"""

from amaranth import *
import numpy as np

from .stage1_core import Stage1Core
from .bin_select import BinSelect
from .quantise import Quantise
from .stream_format import StreamFormat
from .wola_prefilter import WolaPrefilter


class Stage1(Elaboratable):
    """Full Stage-1: packed dual-ADC windowed FFT + A/B split + bin-select run +
    int16 quantise + fcfb_stream.h record framing.

    Parameters
    ----------
    width_in : int
        FFT input sample width (fcfb Stage-1 = 17).
    order_log2 : int
        log2 of the complex FFT size N.
    twiddle_width : int
        Twiddle width for the FFT.
    window : str or None
        FFT window name, or None for rectangular (T=1 first-light).
    cmult3x : bool
        Time-share the twiddle multiply at 3× (needs ``domain_3x``).
    domain_2x, domain_3x : str or None
        2×/3× clock domains (required when window / cmult3x are used).
    shift_width : int
        Bit width of the quantise ``shift`` control.
    seq_width : int
        Frame-counter / wire ``seq`` width (wire uses u64).
    wmax : int
        Max run width W buffered by StreamFormat.
    hop : int or None
        WOLA block hop R. ``None`` (default) = critical sampling (hop=N), the
        shipping path. ``R < N`` selects the wideband oversampled fold (per-bin
        rate FS/R, throughput q=N/R>1): the WOLA runs at ``pages=T+1`` (the NB=5
        BRAM lever) and the whole datapath must run in the fast ``fft`` domain
        (wired at the top; see ``Stage1Top(hop=...)``). Requires a WOLA (T>1).
    ab_extra_pipe : bool
        Add one register stage inside the A/B split (Phase-1): needed to close
        the fast ``fft`` clock (166.67 MHz). Default off = shipping 125 MHz path.

    Attributes
    ----------
    clken : Signal(), in
    common_edge_2x, common_edge_3x : Signal(), in    (only when 2×/3× used)
    re_in : Signal(signed(width_in)), in    ADC0 sample (packed real part).
    im_in : Signal(signed(width_in)), in    ADC1 sample (packed imag part).
    mask_rd_addr : Signal(order_log2), out  Keep-bitmap read address (bin bk).
    keep_a_in, keep_b_in : Signal(), in     Registered keep bits from the external
                                            MaskMem BRAM (latency 1).
    shift : Signal(shift_width), in         Quantise right-shift (shared A/B).
    out_valid : Signal(), out               32-bit wire word valid (on clken).
    out_data : Signal(32), out              BlockRecord word, little-endian.
    out_last : Signal(), out                Final word of a block record.
    """
    def __init__(self, width_in=17, order_log2=12, twiddle_width=16,
                 window=None, cmult3x=False, domain_2x=None, domain_3x=None,
                 shift_width=5, seq_width=64, wmax=64,
                 T=1, sample_width=16, coeff_width=18,
                 hop=None, ab_extra_pipe=False, quantise_extra_pipe=False,
                 ab_share_write_port=False, bin_width=16):
        self.width_in = width_in
        self.order_log2 = order_log2
        self.T = T
        self.hop = hop
        # On-wire I/Q component width: 16 (shipping critical) or 24 (Option-B
        # wideband int24, which streams the full 23-bit internal bin so the
        # reconstructed channel reaches the ~-102 dBc fixed-point ceiling).
        self.bin_width = bin_width

        # Optional T-fold WOLA prefilter in front of the FFT (T>1). It takes the
        # raw packed ADC sample stream (sample_width) and emits the folded pre-FFT
        # buffer at the FFT input width; the FFT then runs with NO window (window
        # must be None when the WOLA supplies the analysis window). T=1 => no
        # prefilter, the external input feeds the FFT directly (width_in).
        # hop=R<N selects the wideband oversampled fold at pages=T+1 (NB=5 lever);
        # None keeps critical sampling (hop=N). It only makes sense with a WOLA.
        if hop is not None and T <= 1:
            raise ValueError("hop (oversampled fold) requires a WOLA, i.e. T > 1")
        self.wola = None
        if T > 1:
            if window is not None:
                raise ValueError("window must be None when a WOLA (T>1) is used")
            self.wola = WolaPrefilter(
                order_log2, T=T, sample_width=sample_width,
                coeff_width=coeff_width, out_width=width_in,
                hop=hop, pages=(T + 1 if hop is not None else None))
            self.in_width = sample_width
        else:
            self.in_width = width_in

        self.core = Stage1Core(width_in, order_log2, twiddle_width,
                               window=window, cmult3x=cmult3x,
                               domain_2x=domain_2x, domain_3x=domain_3x,
                               ab_extra_pipe=ab_extra_pipe,
                               ab_share_write_port=ab_share_write_port)
        wab = self.core.width_out
        self.width_ab = wab
        self.bs = BinSelect(wab, order_log2, seq_width=seq_width)
        self.qa = Quantise(wab, shift_width=shift_width,
                           extra_pipe=quantise_extra_pipe, out_width=bin_width)
        self.qb = Quantise(wab, shift_width=shift_width,
                           extra_pipe=quantise_extra_pipe, out_width=bin_width)
        self.sf = StreamFormat(order_log2, wmax=wmax, seq_width=seq_width,
                               bin_width=bin_width)

        self._use_2x = self.core._use_2x
        self._use_3x = self.core._use_3x

        self.clken = Signal()
        if self._use_2x:
            self.common_edge_2x = Signal()
        if self._use_3x:
            self.common_edge_3x = Signal()
        self.in_valid = Signal()                 # sample strobe (WOLA path; T>1)
        self.re_in = Signal(signed(self.in_width))
        self.im_in = Signal(signed(self.in_width))
        # External per-ADC keep-bitmap read interface (the MaskMem BRAM lives at
        # the top level; see stage1_top / mask_mem).  rd_addr is the bin counter;
        # keep_a_in/keep_b_in are the registered keep bits (latency 1).
        self.mask_rd_addr = Signal(order_log2)
        self.keep_a_in = Signal()
        self.keep_b_in = Signal()
        self.shift = Signal(shift_width)

        self.out_valid = Signal()
        self.out_data = Signal(32)
        self.out_last = Signal()

    def elaborate(self, platform):
        m = Module()
        m.submodules.core = core = self.core
        m.submodules.bs = bs = self.bs
        m.submodules.qa = qa = self.qa
        m.submodules.qb = qb = self.qb
        m.submodules.sf = sf = self.sf

        # Optional WOLA prefilter runs on the raw clken; the FFT + back end only
        # advance when the WOLA is delivering valid folded samples (run), so the
        # T-column fill latency is absorbed and FFT frames align to WOLA blocks.
        run = Signal()
        if self.wola is not None:
            m.submodules.wola = wola = self.wola
            m.d.comb += [
                wola.clken.eq(self.clken),
                wola.in_valid.eq(self.in_valid),
                wola.re_in.eq(self.re_in), wola.im_in.eq(self.im_in),
                run.eq(self.clken & wola.out_valid),
                core.re_in.eq(wola.re_out), core.im_in.eq(wola.im_out),
            ]
        else:
            m.d.comb += [run.eq(self.clken),
                         core.re_in.eq(self.re_in), core.im_in.eq(self.im_in)]

        # Fan the gated advance (and the multi-clock common edges) to every block.
        for blk in (core, bs, qa, qb, sf):
            m.d.comb += blk.clken.eq(run)
        if self._use_2x:
            m.d.comb += core.common_edge_2x.eq(self.common_edge_2x)
        if self._use_3x:
            m.d.comb += core.common_edge_3x.eq(self.common_edge_3x)

        # Core (natural-order A/B stream) -> bin-select; external keep-bitmap read.
        m.d.comb += [
            bs.in_valid.eq(core.out_valid),
            bs.in_last.eq(core.out_last),
            bs.a_re.eq(core.a_re), bs.a_im.eq(core.a_im),
            bs.b_re.eq(core.b_re), bs.b_im.eq(core.b_im),
            self.mask_rd_addr.eq(bs.rd_addr),
            bs.keep_a_in.eq(self.keep_a_in),
            bs.keep_b_in.eq(self.keep_b_in),
        ]

        # Bin-select -> two quantisers (A, B selected independently), one shift.
        m.d.comb += [
            qa.in_valid.eq(bs.keep_a),
            qa.re_in.eq(bs.a_re_o), qa.im_in.eq(bs.a_im_o), qa.shift.eq(self.shift),
            qb.in_valid.eq(bs.keep_b),
            qb.re_in.eq(bs.b_re_o), qb.im_in.eq(bs.b_im_o), qb.shift.eq(self.shift),
        ]

        # Align the per-ADC write addresses + frame tags to the quantised data:
        # Quantise adds `qa.latency` cycles (1 shipping, 2 wideband).  The per-ADC
        # keep valids are carried by qa/qb.out_valid (same latency), so only
        # cnt_a/cnt_b, frame_last and seq need the matching delay.  (bs.cnt_* are
        # combinational at the bin_select output stage; the delay registers them.)
        ca_p, cb_p, last_p, seq_p = bs.cnt_a, bs.cnt_b, bs.frame_last, bs.seq
        for i in range(qa.latency):
            ca_n = Signal.like(bs.cnt_a, name=f'cnt_a_d{i}')
            cb_n = Signal.like(bs.cnt_b, name=f'cnt_b_d{i}')
            last_n = Signal(name=f'frame_last_d{i}')
            seq_n = Signal.like(bs.seq, name=f'seq_d{i}')
            with m.If(run):
                m.d.sync += [ca_n.eq(ca_p), cb_n.eq(cb_p),
                             last_n.eq(last_p), seq_n.eq(seq_p)]
            ca_p, cb_p, last_p, seq_p = ca_n, cb_n, last_n, seq_n

        # Quantised I/Q + aligned tags -> record framing.
        m.d.comb += [
            sf.we_a.eq(qa.out_valid),          # == bs.keep_a delayed by latency
            sf.we_b.eq(qb.out_valid),          # == bs.keep_b delayed by latency
            sf.addr_a.eq(ca_p),
            sf.addr_b.eq(cb_p),
            sf.frame_last.eq(last_p),
            sf.seq.eq(seq_p),
            sf.a_i.eq(qa.re_out), sf.a_q.eq(qa.im_out),
            sf.b_i.eq(qb.re_out), sf.b_q.eq(qb.im_out),
        ]

        m.d.comb += [
            self.out_valid.eq(sf.out_valid),
            self.out_data.eq(sf.out_data),
            self.out_last.eq(sf.out_last),
        ]
        return m

    def run_masks(self, k0, W, adc_mask):
        """Contiguous-run (k0, W, adc_mask) -> (mask_a, mask_b) keep-bitmaps.

        Convenience for the regression path: the old single-run selection is just
        the special case where both masks are the same k0..k0+W-1 window (gated by
        adc_mask).  Returns two length-N boolean np arrays.
        """
        N = 1 << self.order_log2
        run = np.zeros(N, dtype=bool)
        run[k0:k0 + W] = True
        mask_a = run if (adc_mask & 1) else np.zeros(N, dtype=bool)
        mask_b = run if (adc_mask & 2) else np.zeros(N, dtype=bool)
        return mask_a, mask_b

    # ---- full-chain byte-exact reference (records only; no 48-B header) ----
    def model(self, inp, mask_a, mask_b, shift):
        """Byte-exact Stage-1 egress for the input scene.

        With a WOLA (T>1), ``inp`` is the 1-D packed complex sample STREAM
        ``z = adc0 + j*adc1`` (integer-valued); it is folded by ``wola.model``
        into per-block FFT input frames first. Without a WOLA (T=1), ``inp`` is a
        sequence of length-N complex frames fed straight to the FFT.

        ``mask_a`` / ``mask_b`` are length-N keep-bitmaps (bool-like) selecting
        which bins of spectrum A (ADC0) / B (ADC1) are streamed, independently.
        Returns the concatenated BlockRecord bytes exactly as the RTL streams them
        (no header): per output block ``->`` ``u64 seq`` then the kept A bins in
        ascending k, then the kept B bins in ascending k, each component
        ``bin_width/8`` little-endian signed bytes, and the payload zero-padded up
        to a 32-bit-word boundary (int24 with an odd total bin count only).
        Reuses the WOLA fold, the FFT fixed-point model, the exact A/B ``>>1``
        split, and ``Quantise.model``.
        """
        import struct
        from maia_hdl.util import bit_invert

        ol2 = self.order_log2
        N = 1 << ol2
        inv = np.array([bit_invert(k, ol2, 1) for k in range(N)])
        k = np.arange(N)
        mirror = (N - k) % N
        ka = np.flatnonzero(np.asarray(mask_a, dtype=bool))
        kb = np.flatnonzero(np.asarray(mask_b, dtype=bool))

        if self.wola is not None:
            bufs = self.wola.model(np.asarray(inp))
            frames = [bufs[i] for i in range(bufs.shape[0])]
        else:
            frames = inp

        out = b""
        for seq, z in enumerate(frames):
            z = np.asarray(z)
            Cre, Cim = self.core.fft.model(z.real.astype(int), z.imag.astype(int))
            C = (np.asarray(Cre) + 1j * np.asarray(Cim))[inv]   # natural order
            Cr, Ci = C.real.astype(np.int64), C.imag.astype(np.int64)
            Mr, Mi = Cr[mirror], Ci[mirror]
            # exact A/B split (arithmetic >>1, floor toward -inf)
            a_re = (Cr + Mr) >> 1
            a_im = (Ci - Mi) >> 1
            b_re = (Ci + Mi) >> 1
            b_im = (Mr - Cr) >> 1

            bin_bytes = self.bin_width // 8

            def enc(v):
                return int(v).to_bytes(bin_bytes, "little", signed=True)

            def pack_bins(ire, iim, kk):
                bb = b""
                for ki in kk:
                    qi = self.qa.model(int(ire[ki]), shift)
                    qq = self.qa.model(int(iim[ki]), shift)
                    bb += enc(qi) + enc(qq)
                return bb

            payload = pack_bins(a_re, a_im, ka) + pack_bins(b_re, b_im, kb)
            payload += b"\x00" * ((-len(payload)) % 4)   # pad to word boundary
            out += struct.pack("<Q", int(seq)) + payload
        return out
