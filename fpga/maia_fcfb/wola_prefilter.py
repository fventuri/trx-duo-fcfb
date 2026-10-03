# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``wola_prefilter`` — critically-sampled (hop=N) T-fold WOLA analysis prefilter.

This is the fcfb Stage-1 front end that turns the plain windowed FFT into a
polyphase filter bank: it implements the M1 golden ``analysis()`` fold
(`sim/chanfcfb_model.py`), per output block m and bin position n::

    buf_m[n] = sum_{t=0..T-1}  h[t*N + n] * z[m*N + t*N + n]      -> FFT

where ``z = adc0 + j*adc1`` (PLAN §3 pack) and ``h`` is the length-T*N Kaiser
(β=9)-windowed-sinc prototype, **PEAK-NORMALIZED** (max|h|=1) per the fixed-point
decision (`fpga/models/fixed_point_sim.py`): M1's unit-DC-gain prototype has
max|h|≈1/N which would collapse the rounded fold to ~5 bits; peak-normalizing
fills the datapath and the resulting ~N constant gain is absorbed into the
programmable ``bin_scale`` at egress. ``T=1`` degenerates to a plain rectangular
FFT (no fold); the shipping default is T=4 (M1 [C] isolation verdict).

Fixed point: coefficients are ``round((2^(cw-1)-1) * h_peak)`` (signed, cw bits);
the fold sum is rounded (half-up) and arithmetic-right-shifted by ``cw-1`` to the
FFT input width ``outw`` (17). That residual near-unity gain (``(2^(cw-1)-1) /
2^(cw-1)``) and the ~N window gain both fold into ``bin_scale`` downstream, so the
absolute shift is not critical — only that the output fits ``outw`` and the shape
is preserved (coefficient-quant noise well below the int16 ship floor).

Streaming structure (polyphase commutator). Input is one complex sample per
``clken`` cycle; output is one ``buf`` sample per cycle after a T-column fill,
framed by ``out_first``/``out_last`` for the FFT. The last T*N samples are held in
**T+1 banks** indexed by column-phase (column = sample-index // N): the T columns
that make up the block being drained are read together for the fold, while the
(T+1)-th bank receives the incoming column — so the written bank is never one of
the T being read (no read/write collision, unlike a T-bank ring). Per output
position n the coefficient ROM (depth N, one packed entry per n) supplies all T
real taps h[0*N+n..(T-1)*N+n]; the fold is T real-by-complex products summed.
"""

from amaranth import *
from amaranth.lib.memory import Memory
import numpy as np


class WolaPrefilter(Elaboratable):
    """T-fold critically-sampled WOLA analysis prefilter (pre-FFT).

    Parameters
    ----------
    order_log2 : int
        log2 of the FFT / hop size N.
    T : int
        Prototype overlap factor (columns folded per block). T=4 default.
    sample_width : int
        Input sample component width (packed ADC: 16). |z| ≤ √2·(2^15−1).
    coeff_width : int
        Prototype coefficient width (signed). 18 gives ~102 dB coeff-quant
        floor, at/below the FFT fixed-point floor.
    out_width : int
        Output (FFT input) width. 17 (see fixed_point_sim.py).

    Attributes
    ----------
    clken : Signal(), in
    in_valid : Signal(), in            A packed complex input sample is present.
    re_in : Signal(signed(sample_width)), in   ADC0 (real part of z).
    im_in : Signal(signed(sample_width)), in   ADC1 (imag part of z).
    re_out, im_out : Signal(signed(out_width)), out   Folded pre-FFT buffer buf.
    out_valid : Signal(), out          buf sample valid (registered).
    out_first : Signal(), out          High on n=0 of a block (buf[0]).
    out_last : Signal(), out           High on n=N-1 of a block (buf[N-1]).
    """
    def __init__(self, order_log2, T=4, sample_width=16, coeff_width=18,
                 out_width=17, hop=None, pages=None):
        if T < 1:
            raise ValueError("T must be >= 1")
        self.order_log2 = order_log2
        self.N = 1 << order_log2
        self.T = T
        # hop=None (default) => critical sampling (hop=N), the shipping path.
        # hop=R<N is the wideband oversampled fold (per-bin rate FS/R): blocks
        # advance by R input samples instead of N, so N buf points are produced
        # per R inputs (throughput q=N/R>1 -- run this block in the fast 'fft'
        # domain).  R<N defaults to T+2 page banks: the length-T*N read window is
        # not page-aligned (spans T+1 pages) + 1 write page.
        self.hop = self.N if hop is None else int(hop)
        if not (1 <= self.hop <= self.N):
            raise ValueError("hop R must be in 1..N")
        self.critical = (self.hop == self.N)
        # NB = page-bank count (the ONLY BRAM lever).
        # Default: T+1 (critical) / T+2 (hop=R).  ``pages`` overrides it: the
        # Phase-3 lever runs hop=R at pages=T+1 -- the storage floor, which OOC-
        # synthesizes to the SAME 28 RAMB36 as the critical WOLA (vs 32 at T+2),
        # so the wideband build fits the xc7z010 at 60/60.  T+1 relies on two
        # facts, both verified: (1) the bank BRAMs map READ_FIRST (Vivado OOC
        # report), so a same-cell same-cycle read returns the OLD sample the fold
        # wants (no per-cycle R/W hazard); (2) the writer cannot lap the reader
        # while fill_at_block_start < NB*N -- steady-state margin NB*N-(T*N+R) =
        # N-R = 971 samples at N=4096,R=3125.  In the free-running datapath the
        # reader (fft clock, q=N/R>1) is faster than the writer (FS-rate inputs),
        # so fill drains and start-fill stays ~T*N+R; the 971-sample slack covers
        # CDC/startup jitter.  See tests test_wola_hopR_nb5_* .
        floor = T + 1
        if pages is None:
            self.NB = floor if self.critical else (T + 2)
        else:
            pages = int(pages)
            if pages < floor:
                raise ValueError(
                    f"pages={pages} < storage floor T+1={floor} "
                    "(fold reads T pages + needs >=1 write page)")
            self.NB = pages
        self.sw = sample_width
        self.cw = coeff_width
        self.outw = out_width
        self.shift = coeff_width - 1        # sum -> output right shift

        self.clken = Signal()
        self.in_valid = Signal()
        self.re_in = Signal(signed(self.sw))
        self.im_in = Signal(signed(self.sw))
        self.re_out = Signal(signed(self.outw))
        self.im_out = Signal(signed(self.outw))
        self.out_valid = Signal()
        self.out_first = Signal()
        self.out_last = Signal()

    @property
    def latency(self):
        """Cycles from an input sample's accept to the buf output register.

        The fold is pipelined to close 125 MHz (the single-cycle T-tap
        multiply+adder-tree was a ~16 ns path): 1 bank/ROM sync read + 1 select
        register (the drained-bank mux + tap, latched at the DSP inputs) + 1
        product register (the T DSP multiplies) + 1 adder-tree register + 1
        round/shift output register.
        """
        return 1 + 1 + 1 + 1 + 1

    # ---- fixed-point prototype (peak-normalized, quantized) ----
    def prototype(self):
        """Length T*N unit-DC-gain Kaiser(β)-sinc prototype for this block's N.

        Identical formula to ``sim/chanfcfb_model.prototype`` (which fixes
        N=4096); parameterized by ``self.N`` so the block is testable at small N.
        At N=4096 it reproduces ``M.prototype(T)`` exactly (same β, cutoff=1).
        """
        from importlib import import_module
        import os
        import sys
        sim = os.path.join(os.path.dirname(__file__), "..", "..", "sim")
        if os.path.abspath(sim) not in sys.path:
            sys.path.insert(0, os.path.abspath(sim))
        beta = import_module("chanfcfb_model").KBETA_PROTO
        N, L = self.N, self.T * self.N
        nn = np.arange(L) - (L - 1) / 2.0
        h = np.sinc(nn / N) * np.kaiser(L, beta)
        return h / h.sum()                            # unit DC gain

    def coeffs(self):
        """(T, N) int array: quantized peak-normalized prototype h[t*N+n]."""
        h = self.prototype()                          # unit-DC-gain, length T*N
        h = h / np.abs(h).max()                       # peak-normalized (max|h|=1)
        k = (1 << (self.cw - 1)) - 1
        q = np.round(k * h).astype(np.int64)
        return q.reshape(self.T, self.N)              # row t = h[t*N : (t+1)*N]

    def _packed_rom_init(self):
        """Depth-N ROM: entry n packs the T signed coeffs h[0*N+n..(T-1)*N+n]."""
        c = self.coeffs()
        mask = (1 << self.cw) - 1
        init = []
        for n in range(self.N):
            word = 0
            for t in range(self.T):
                word |= (int(c[t, n]) & mask) << (t * self.cw)
            init.append(word)
        return init

    def _rshift_round(self, x):
        """Round-half-up then arithmetic right shift by ``self.shift`` (Python)."""
        s = self.shift
        return (x + (1 << (s - 1))) >> s if s >= 1 else x

    # ---- reference model (bit-exact to the RTL) ----
    def model(self, z):
        """Fixed-point fold of a packed complex sample stream ``z``.

        ``z`` is a length ``>= T*N`` complex integer array. Returns an (Nblk, N)
        complex-int array of folded pre-FFT buffers (``Nblk`` complete blocks),
        each row = one block's ``buf`` in natural time order, bit-exact to the
        RTL (round-half-up + arithmetic ``>>`` by ``cw-1``).
        """
        z = np.asarray(z)
        N, T, R = self.N, self.T, self.hop
        c = self.coeffs()
        # Block m folds z[m*R : m*R + T*N]; last complete block needs T*N samples.
        nblk = (len(z) - T * N) // R + 1 if len(z) >= T * N else 0
        out = np.empty((max(nblk, 0), N), np.complex128)
        for m in range(nblk):
            seg = z[m * R: m * R + T * N]
            acc = np.zeros(N, np.complex128)
            for t in range(T):
                acc = acc + c[t] * seg[t * N:(t + 1) * N]
            re = self._rshift_round(np.round(acc.real).astype(np.int64))
            im = self._rshift_round(np.round(acc.imag).astype(np.int64))
            out[m] = re + 1j * im
        return out

    def elaborate(self, platform):
        if self.critical:
            return self._elaborate_critical(platform)
        return self._elaborate_hopR(platform)

    def _elaborate_critical(self, platform):
        m = Module()
        N, T, NB = self.N, self.T, self.NB
        ol2 = self.order_log2

        # ---- coefficient ROM (packed T taps per position n) ----
        rom = Memory(shape=T * self.cw, depth=N, init=self._packed_rom_init())
        m.submodules.rom = rom
        rom_rd = rom.read_port()

        # ---- T+1 sample banks (Cat(re, im)) ----
        banks = [Memory(shape=2 * self.sw, depth=N, init=[]) for _ in range(NB)]
        wr, rd = [], []
        for b in range(NB):
            setattr(m.submodules, f"bank{b}", banks[b])
            wr.append(banks[b].write_port())
            rd.append(banks[b].read_port())

        # ---- counters: position n, write-bank (col mod NB), fill state ----
        n = Signal(ol2)
        wbank = Signal(range(NB))            # current column's bank = col mod NB
        col_fill = Signal(range(T + 1))      # saturates at T once fully primed
        n_last = Signal()
        m.d.comb += n_last.eq(n == (N - 1))
        with m.If(self.clken & self.in_valid):
            with m.If(n_last):
                m.d.sync += n.eq(0)
                with m.If(wbank == (NB - 1)):
                    m.d.sync += wbank.eq(0)
                with m.Else():
                    m.d.sync += wbank.eq(wbank + 1)
                with m.If(col_fill != T):
                    m.d.sync += col_fill.eq(col_fill + 1)
            with m.Else():
                m.d.sync += n.eq(n + 1)

        # base = oldest drained bank = (col - T) mod NB = (col + 1) mod NB
        # (since T = NB-1). Register it to align with the 1-cycle bank read.
        base = Signal(range(NB))
        with m.If(wbank == (NB - 1)):
            m.d.comb += base.eq(0)
        with m.Else():
            m.d.comb += base.eq(wbank + 1)

        # ---- write incoming sample to the current column's bank ----
        for b in range(NB):
            m.d.comb += [
                wr[b].addr.eq(n),
                wr[b].data.eq(Cat(self.re_in, self.im_in)),
                wr[b].en.eq(self.clken & self.in_valid & (wbank == b)),
                rd[b].addr.eq(n),
                rd[b].en.eq(self.clken),
            ]
        m.d.comb += [rom_rd.addr.eq(n), rom_rd.en.eq(self.clken)]

        # ---- pipeline the control alongside the 1-cycle bank/ROM read ----
        base_q = Signal(range(NB))
        valid_q = Signal()
        first_q = Signal()
        last_q = Signal()
        drain = Signal()
        m.d.comb += drain.eq(self.in_valid & (col_fill == T))
        with m.If(self.clken):
            m.d.sync += [
                base_q.eq(base),
                valid_q.eq(drain),
                first_q.eq(drain & (n == 0)),
                last_q.eq(drain & n_last),
            ]

        # ---- fold: pick the T drained banks (base .. base+T-1) mod NB ----
        rd_re = Array([Signal(signed(self.sw), name=f"rd_re{b}") for b in range(NB)])
        rd_im = Array([Signal(signed(self.sw), name=f"rd_im{b}") for b in range(NB)])
        for b in range(NB):
            m.d.comb += [
                rd_re[b].eq(rd[b].data[:self.sw]),
                rd_im[b].eq(rd[b].data[self.sw:]),
            ]

        # unpack ROM taps (each signed cw)
        taps = []
        for t in range(T):
            ct = Signal(signed(self.cw), name=f"tap{t}")
            m.d.comb += ct.eq(rom_rd.data[t * self.cw:(t + 1) * self.cw])
            taps.append(ct)

        # The fold is pipelined into four register stages so it closes 125 MHz:
        #   stage 1b: register the drained-bank-selected sample + tap (so the
        #             5:1 bank mux ends at a fabric FF and the DSP uses its input
        #             registers, not a combinational path from the bank BRAM)
        #   stage 2:  the T real-by-complex DSP products
        #   stage 3:  the adder-tree sum of the products
        #   stage 4:  round-half-up + arithmetic shift, into the output register
        # The control (valid/first/last) is delayed in lockstep so the tags stay
        # aligned with the data at the output.  All numerics are unchanged (the
        # .model() is the single-cycle fold), so bit-exactness is preserved.
        accw = self.cw + self.sw + (T - 1).bit_length()   # headroom for the sum

        # ---- stage 1b: select the drained bank (base_q..base_q+T-1) and register
        #      the selected sample + tap at the DSP inputs ----
        sel_re_q = [Signal(signed(self.sw), name=f"sel_re_q{t}") for t in range(T)]
        sel_im_q = [Signal(signed(self.sw), name=f"sel_im_q{t}") for t in range(T)]
        tap_q = [Signal(signed(self.cw), name=f"tap_q{t}") for t in range(T)]
        valid_b = Signal()
        first_b = Signal()
        last_b = Signal()
        with m.If(self.clken):
            for t in range(T):
                sel = Signal(range(NB), name=f"sel{t}")
                s = base_q + t
                with m.If(s >= NB):
                    m.d.comb += sel.eq(s - NB)
                with m.Else():
                    m.d.comb += sel.eq(s)
                m.d.sync += [sel_re_q[t].eq(rd_re[sel]),
                             sel_im_q[t].eq(rd_im[sel]),
                             tap_q[t].eq(taps[t])]
            m.d.sync += [valid_b.eq(valid_q), first_b.eq(first_q),
                         last_b.eq(last_q)]

        # ---- stage 2: register the T products (registered DSP inputs -> A/B/M) ----
        pr_q = [Signal(signed(self.cw + self.sw), name=f"pr_q{t}") for t in range(T)]
        pi_q = [Signal(signed(self.cw + self.sw), name=f"pi_q{t}") for t in range(T)]
        valid_2 = Signal()
        first_2 = Signal()
        last_2 = Signal()
        with m.If(self.clken):
            for t in range(T):
                m.d.sync += [pr_q[t].eq(tap_q[t] * sel_re_q[t]),
                             pi_q[t].eq(tap_q[t] * sel_im_q[t])]
            m.d.sync += [valid_2.eq(valid_b), first_2.eq(first_b),
                         last_2.eq(last_b)]

        # ---- stage 3: register the adder-tree sum ----
        sum_re_q = Signal(signed(accw))
        sum_im_q = Signal(signed(accw))
        valid_3 = Signal()
        first_3 = Signal()
        last_3 = Signal()
        with m.If(self.clken):
            m.d.sync += [
                sum_re_q.eq(sum(pr_q)), sum_im_q.eq(sum(pi_q)),
                valid_3.eq(valid_2), first_3.eq(first_2), last_3.eq(last_2),
            ]

        # ---- stage 4: round-half-up then arithmetic >> shift, to out_width ----
        s = self.shift
        off = (1 << (s - 1)) if s >= 1 else 0
        out_re = Signal(signed(accw))
        out_im = Signal(signed(accw))
        if s >= 1:
            m.d.comb += [out_re.eq((sum_re_q + off) >> s),
                         out_im.eq((sum_im_q + off) >> s)]
        else:
            m.d.comb += [out_re.eq(sum_re_q), out_im.eq(sum_im_q)]

        with m.If(self.clken):
            m.d.sync += [
                self.re_out.eq(out_re),
                self.im_out.eq(out_im),
                self.out_valid.eq(valid_3),
                self.out_first.eq(first_3),
                self.out_last.eq(last_3),
            ]
        return m

    def _elaborate_hopR(self, platform):
        """Oversampled fold, hop R < N: decoupled write (input rate) / read
        (block rate) over a page-banked circular history buffer.

        Block m folds z[m*R + t*N + n] for t=0..T-1 -> buf_m[n].  The T tap
        addresses for a given n share the SAME within-page offset
        ``off = (m*R + n) mod N`` (t*N is a whole number of pages), and differ
        only by page: page (page0 + t) mod NB where page0 = ((m*R + n)//N) mod NB.
        So every bank reads at ``off`` and the T taps are a page rotation --
        identical fold datapath to the critical path, only the addressing and a
        block-readiness FSM differ.  NB = T+2 banks: the length-T*N window is not
        page-aligned (spans T+1 pages) plus one page being written.

        Rates decouple: inputs advance the writer on ``in_valid``; the reader
        free-runs one buf point per ``clken`` while a block is buffered
        (``fill >= T*N``), producing N points per R inputs (q = N/R).  Drive this
        block in the fast 'fft' domain with in_valid marking the FS-rate inputs.
        """
        m = Module()
        N, T, NB = self.N, self.T, self.NB
        ol2 = self.order_log2
        R = self.hop

        # ---- coefficient ROM (packed T taps per position n) ----
        rom = Memory(shape=T * self.cw, depth=N, init=self._packed_rom_init())
        m.submodules.rom = rom
        rom_rd = rom.read_port()

        # ---- NB circular page banks (Cat(re, im)) ----
        banks = [Memory(shape=2 * self.sw, depth=N, init=[]) for _ in range(NB)]
        wr, rd = [], []
        for b in range(NB):
            setattr(m.submodules, f"bank{b}", banks[b])
            wr.append(banks[b].write_port())
            rd.append(banks[b].read_port())

        # ---- writer: input-rate, page-banked circular ----
        wr_off = Signal(ol2)
        wr_page = Signal(range(NB))
        with m.If(self.clken & self.in_valid):
            with m.If(wr_off == (N - 1)):
                m.d.sync += wr_off.eq(0)
                with m.If(wr_page == (NB - 1)):
                    m.d.sync += wr_page.eq(0)
                with m.Else():
                    m.d.sync += wr_page.eq(wr_page + 1)
            with m.Else():
                m.d.sync += wr_off.eq(wr_off + 1)

        # ---- reader FSM: block index base (off, page0), position n, busy ----
        busy = Signal()
        n = Signal(ol2)
        off = Signal(ol2)                    # = (m*R + n) mod N; every bank's addr
        page0 = Signal(range(NB))            # = ((m*R + n)//N) mod NB
        blk_off = Signal(ol2 + 1)            # next block start off (0..N-1)
        blk_page0 = Signal(range(NB))        # next block start page0
        fill = Signal(signed(ol2 + 4))       # buffered-sample lead over consumed
        self._dbg_fill = fill                # exposed for margin validation sims

        n_last = Signal()
        off_last = Signal()
        m.d.comb += [n_last.eq(n == (N - 1)), off_last.eq(off == (N - 1))]

        blk_done = Signal()                  # completing block m this cycle
        m.d.comb += blk_done.eq(self.clken & busy & n_last)

        # readiness lead: +1 per accepted input, -R per completed block.
        with m.If(self.clken):
            m.d.sync += fill.eq(
                fill + Mux(self.in_valid, 1, 0) - Mux(busy & n_last, R, 0))

        page0_inc = Mux(page0 == (NB - 1), 0, page0 + 1)
        bpage_inc = Mux(blk_page0 == (NB - 1), 0, blk_page0 + 1)
        sum_off = Signal(ol2 + 1)
        m.d.comb += sum_off.eq(blk_off + R)   # next block start = this + R
        with m.If(self.clken):
            with m.If(busy):
                with m.If(n_last):
                    # end of block m: idle; roll block base forward by R.
                    m.d.sync += busy.eq(0)
                    with m.If(sum_off >= N):
                        m.d.sync += [blk_off.eq(sum_off - N),
                                     blk_page0.eq(bpage_inc)]
                    with m.Else():
                        m.d.sync += blk_off.eq(sum_off)
                with m.Else():
                    m.d.sync += n.eq(n + 1)
                    m.d.sync += off.eq(off + 1)        # wraps mod N (ol2 bits)
                    with m.If(off_last):
                        m.d.sync += page0.eq(page0_inc)
            with m.Elif(fill >= (T * N)):
                # enough samples buffered for block m -> start draining it.
                m.d.sync += [busy.eq(1), n.eq(0),
                             off.eq(blk_off), page0.eq(blk_page0)]

        # ---- bank/ROM access: all banks read at off; write bank wr_page ----
        for b in range(NB):
            m.d.comb += [
                wr[b].addr.eq(wr_off),
                wr[b].data.eq(Cat(self.re_in, self.im_in)),
                wr[b].en.eq(self.clken & self.in_valid & (wr_page == b)),
                rd[b].addr.eq(off),
                rd[b].en.eq(self.clken),
            ]
        # ROM (window taps h[t*N+n]) is addressed by the OUTPUT position n, not
        # the circular-buffer offset off: the banks live at off = (m*R+n) mod N,
        # but the tap for output bin n is always h[t*N+n].  (In the critical path
        # off==n, so they coincide; for hop=R they differ.)
        m.d.comb += [rom_rd.addr.eq(n), rom_rd.en.eq(self.clken)]

        # ---- pipeline the control alongside the 1-cycle bank/ROM read ----
        page0_q = Signal(range(NB))
        valid_q = Signal()
        first_q = Signal()
        last_q = Signal()
        with m.If(self.clken):
            m.d.sync += [
                page0_q.eq(page0),
                valid_q.eq(busy),
                first_q.eq(busy & (n == 0)),
                last_q.eq(busy & n_last),
            ]

        # ---- unpack bank reads + ROM taps ----
        rd_re = Array([Signal(signed(self.sw), name=f"rd_re{b}") for b in range(NB)])
        rd_im = Array([Signal(signed(self.sw), name=f"rd_im{b}") for b in range(NB)])
        for b in range(NB):
            m.d.comb += [rd_re[b].eq(rd[b].data[:self.sw]),
                         rd_im[b].eq(rd[b].data[self.sw:])]
        taps = []
        for t in range(T):
            ct = Signal(signed(self.cw), name=f"tap{t}")
            m.d.comb += ct.eq(rom_rd.data[t * self.cw:(t + 1) * self.cw])
            taps.append(ct)

        # ---- fold pipeline (identical to the critical path; base = page0_q) ----
        accw = self.cw + self.sw + (T - 1).bit_length()

        # stage 1b: select drained page (page0_q + t) mod NB, register sample+tap
        sel_re_q = [Signal(signed(self.sw), name=f"sel_re_q{t}") for t in range(T)]
        sel_im_q = [Signal(signed(self.sw), name=f"sel_im_q{t}") for t in range(T)]
        tap_q = [Signal(signed(self.cw), name=f"tap_q{t}") for t in range(T)]
        valid_b = Signal(); first_b = Signal(); last_b = Signal()
        with m.If(self.clken):
            for t in range(T):
                sel = Signal(range(NB), name=f"sel{t}")
                s = page0_q + t
                with m.If(s >= NB):
                    m.d.comb += sel.eq(s - NB)
                with m.Else():
                    m.d.comb += sel.eq(s)
                m.d.sync += [sel_re_q[t].eq(rd_re[sel]),
                             sel_im_q[t].eq(rd_im[sel]),
                             tap_q[t].eq(taps[t])]
            m.d.sync += [valid_b.eq(valid_q), first_b.eq(first_q),
                         last_b.eq(last_q)]

        # stage 2: register the T products
        pr_q = [Signal(signed(self.cw + self.sw), name=f"pr_q{t}") for t in range(T)]
        pi_q = [Signal(signed(self.cw + self.sw), name=f"pi_q{t}") for t in range(T)]
        valid_2 = Signal(); first_2 = Signal(); last_2 = Signal()
        with m.If(self.clken):
            for t in range(T):
                m.d.sync += [pr_q[t].eq(tap_q[t] * sel_re_q[t]),
                             pi_q[t].eq(tap_q[t] * sel_im_q[t])]
            m.d.sync += [valid_2.eq(valid_b), first_2.eq(first_b),
                         last_2.eq(last_b)]

        # stage 3: register the adder-tree sum
        sum_re_q = Signal(signed(accw))
        sum_im_q = Signal(signed(accw))
        valid_3 = Signal(); first_3 = Signal(); last_3 = Signal()
        with m.If(self.clken):
            m.d.sync += [sum_re_q.eq(sum(pr_q)), sum_im_q.eq(sum(pi_q)),
                         valid_3.eq(valid_2), first_3.eq(first_2),
                         last_3.eq(last_2)]

        # stage 4: round-half-up then arithmetic >> shift, into the output reg
        s = self.shift
        roff = (1 << (s - 1)) if s >= 1 else 0
        out_re = Signal(signed(accw))
        out_im = Signal(signed(accw))
        if s >= 1:
            m.d.comb += [out_re.eq((sum_re_q + roff) >> s),
                         out_im.eq((sum_im_q + roff) >> s)]
        else:
            m.d.comb += [out_re.eq(sum_re_q), out_im.eq(sum_im_q)]
        with m.If(self.clken):
            m.d.sync += [
                self.re_out.eq(out_re), self.im_out.eq(out_im),
                self.out_valid.eq(valid_3), self.out_first.eq(first_3),
                self.out_last.eq(last_3),
            ]
        return m
