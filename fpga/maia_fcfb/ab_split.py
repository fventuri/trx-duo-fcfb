# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Derived from the MaiaSDR TRX-duo port's `real_recovery.py` (© 2026 MaiaSDR
# TRX-duo port, MIT), which in turn reuses maia-hdl (© Daniel Estevez, MIT): the
# ping-pong frame buffer and the (k, N-k) bit-reversed pairing are taken from
# there. The change for fcfb: RealRecovery goes on to combine Xe + W*Xo into a
# single channel's 2N real spectrum; fcfb instead OUTPUTS Xe and Xo directly as
# the two SEPARATE ADC spectra (the PLAN §3 A/B packing split), so the twiddle
# ROM, the Cmult, and the final combine are removed.
#
# CLEAN-ROOM note: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``ab_split`` — recover the two real-ADC spectra A, B from the packed complex
FFT of ``z[n] = adc0[n] + j*adc1[n]`` (PLAN §3).

Per output bin ``k = 0 .. N-1`` (Y = the FFT's complex spectrum, natural order)::

    A[k] = (Y[k] + conj(Y[N-k])) / 2          # ADC0 spectrum
    B[k] = (Y[k] - conj(Y[N-k])) / (2j)       # ADC1 spectrum

Both are pure add/sub + a 1-bit shift (no multipliers). With
``conj(Y[N-k]) = (Cr_re, -Cr_im)`` and Y[k] = (Ck_re, Ck_im)::

    A_re = (Ck_re + Cr_re) >> 1     A_im = (Ck_im - Cr_im) >> 1
    B_re = (Ck_im + Cr_im) >> 1     B_im = (Cr_re - Ck_re) >> 1

The SDF FFT streams its N bins **bit-reversed**, frames back-to-back. A ping-pong
frame buffer (fill buffer written at ``bitrev(cnt)`` -> natural order; drain
buffer read at ``k`` and ``N-k``) presents the (k, N-k) pair the split needs.
"""

from amaranth import *
from amaranth.lib.memory import Memory
import numpy as np


class ABSplit(Elaboratable):
    """Dual-ADC A/B packing split.

    Parameters
    ----------
    width_in : int
        Width of the FFT output samples (re/im). For fcfb Stage-1 = 23
        (see ``fpga/models/fixed_point_sim.py``: 17-bit FFT in + 6-bit growth).
    order_log2 : int
        log2 of the complex FFT size N.

    Attributes
    ----------
    clken : Signal(), in
        Clock enable (advances the whole pipeline; gate with the FFT strobe).
    re_in, im_in : Signal(signed(width_in)), in
        FFT output stream Y (bit-reversed bin order).
    input_last : Signal(), in
        Asserted on the last sample (position N-1) of the incoming FFT frame.
    a_re, a_im : Signal(signed(width_out)), out   ADC0 spectrum A[k], natural order.
    b_re, b_im : Signal(signed(width_out)), out   ADC1 spectrum B[k], natural order.
    out_valid : Signal(), out
        High while a_*/b_* carry valid recovered bins.
    out_last : Signal(), out
        Asserted on the k = N-1 output of each drained frame.
    """
    def __init__(self, width_in, order_log2, extra_pipe=False,
                 share_write_port=False):
        self.w = width_in
        self.order_log2 = order_log2
        self.N = 1 << order_log2
        # (Ck +/- Cr) is width_in+1; >>1 brings it back to width_in.
        self.width_out = width_in
        # extra_pipe inserts one register stage between the frame-buffer read
        # (a slow BRAM clk->out) and the (Ck +/- Cr) combine adder, so the fast
        # ('fft') clock can close: the BRAM read + drain mux is one stage, the
        # combine add + >>1 the next. Off by default (latency 2, the shipping
        # 125 MHz build + 33-test suite are unchanged); on => latency 3, control
        # pipes track it automatically so downstream alignment is transparent.
        self.extra_pipe = extra_pipe
        # share_write_port halves the BRAM: each ping-pong buffer needs the two
        # read addresses (k and N-k) only while it is *draining*, and the write
        # port only while it is *filling* -- never both in the same frame. So the
        # write port (A) can time-share with the k-read: during fill A writes
        # bitrev(cnt), during drain A reads k. Port B always serves the N-k read.
        # The replicated (default) form gives each buffer a dedicated write port
        # plus two read ports (3 -> Vivado duplicates the RAM for the 2nd read),
        # costing 2x the BRAM. Value-identical either way (only the physical port
        # mapping changes); off by default so the shipping build is untouched.
        self.share_write_port = share_write_port

        self.clken = Signal()
        self.re_in = Signal(signed(self.w))
        self.im_in = Signal(signed(self.w))
        self.input_last = Signal()
        self.a_re = Signal(signed(self.width_out))
        self.a_im = Signal(signed(self.width_out))
        self.b_re = Signal(signed(self.width_out))
        self.b_im = Signal(signed(self.width_out))
        self.out_valid = Signal()
        self.out_last = Signal()

    # ---- reference model (float; matches sim/chanfcfb_model.py ab_split) ----
    def model(self, C):
        """Recover (A, B) from a natural-order complex spectrum ``C``."""
        C = np.asarray(C)
        N = self.N
        k = np.arange(N)
        Cm = np.conj(C[(N - k) % N])
        A = (C + Cm) / 2
        B = (C - Cm) / 2j
        return A, B

    @property
    def latency(self):
        """Cycles from a bin's read-address cycle to its output cycle."""
        # 1 (mem read) + [1 combine-pipe, if extra_pipe] + 1 (A/B output reg)
        return 1 + (1 if self.extra_pipe else 0) + 1

    def elaborate(self, platform):
        m = Module()
        N, ol2, w = self.N, self.order_log2, self.w

        # Two ping-pong frame buffers storing Cat(re, im). fill toggles each
        # frame; the *other* buffer is drained.
        bufs = [Memory(shape=2 * w, depth=N, init=[]) for _ in range(2)]
        m.submodules.buf0 = bufs[0]
        m.submodules.buf1 = bufs[1]
        wr = [b.write_port() for b in bufs]
        rd_k = [b.read_port() for b in bufs]        # read at address k
        rd_r = [b.read_port() for b in bufs]        # read at address N-k

        # Free-running position counter and ping-pong / valid flags.
        cnt = Signal(ol2)
        fill = Signal()          # which buffer is being filled this frame
        seen_frame = Signal()    # a full frame captured (drain valid)
        with m.If(self.clken):
            m.d.sync += cnt.eq(cnt + 1)
            with m.If(self.input_last):
                m.d.sync += [cnt.eq(0), fill.eq(~fill), seen_frame.eq(1)]

        # Addresses: write bitrev(cnt) into fill buffer; read k=cnt and N-k from
        # the drain buffer.
        rev = Signal(ol2)
        m.d.comb += rev.eq((-cnt)[:ol2])          # (N - cnt) mod N
        for i in range(2):
            if self.share_write_port:
                # Port A time-shares write@bitrev(cnt) (this buffer is filling)
                # with the k-read@cnt (this buffer is draining). Driving the
                # write and k-read ports from ONE address net lets Vivado infer a
                # single true-dual-port RAM (A=RW, B=read) -- no duplicate copy
                # for the 2nd read port. A buffer is only written while filling
                # and only read while draining, so A never reads+writes the same
                # cycle; the fill buffer's k-read output is unused (drain mux
                # selects the *other* buffer), so the differing fill-frame read
                # address is harmless.
                addr_a = Signal(ol2, name=f'addr_a{i}')
                m.d.comb += addr_a.eq(Mux(fill == i, cnt[::-1], cnt))
                m.d.comb += [
                    wr[i].addr.eq(addr_a),
                    wr[i].data.eq(Cat(self.re_in, self.im_in)),
                    wr[i].en.eq(self.clken & (fill == i)),
                    rd_k[i].addr.eq(addr_a),      # same net as write -> port A
                    rd_r[i].addr.eq(rev),         # port B
                    rd_k[i].en.eq(self.clken),
                    rd_r[i].en.eq(self.clken),
                ]
            else:
                m.d.comb += [
                    wr[i].addr.eq(cnt[::-1]),     # bit-reversed write -> natural
                    wr[i].data.eq(Cat(self.re_in, self.im_in)),
                    wr[i].en.eq(self.clken & (fill == i)),
                    rd_k[i].addr.eq(cnt),
                    rd_r[i].addr.eq(rev),
                    rd_k[i].en.eq(self.clken),
                    rd_r[i].en.eq(self.clken),
                ]

        # Stage 1: memory outputs (drain buffer = ~fill). The read issued this
        # cycle lands next cycle, so pick based on the registered fill selector.
        drain = Signal()
        m.d.comb += drain.eq(~fill)
        drain_q = Signal()
        with m.If(self.clken):
            m.d.sync += drain_q.eq(drain)

        Ck_re = Signal(signed(w)); Ck_im = Signal(signed(w))
        Cr_re = Signal(signed(w)); Cr_im = Signal(signed(w))
        m.d.comb += [
            Ck_re.eq(Mux(drain_q, rd_k[1].data[:w], rd_k[0].data[:w])),
            Ck_im.eq(Mux(drain_q, rd_k[1].data[w:], rd_k[0].data[w:])),
            Cr_re.eq(Mux(drain_q, rd_r[1].data[:w], rd_r[0].data[:w])),
            Cr_im.eq(Mux(drain_q, rd_r[1].data[w:], rd_r[0].data[w:])),
        ]

        # Optional combine-pipe stage: register the muxed frame-buffer reads so
        # the slow BRAM clk->out + drain mux is isolated from the combine adder.
        if self.extra_pipe:
            Ck_re_q = Signal(signed(w)); Ck_im_q = Signal(signed(w))
            Cr_re_q = Signal(signed(w)); Cr_im_q = Signal(signed(w))
            with m.If(self.clken):
                m.d.sync += [
                    Ck_re_q.eq(Ck_re), Ck_im_q.eq(Ck_im),
                    Cr_re_q.eq(Cr_re), Cr_im_q.eq(Cr_im),
                ]
        else:
            Ck_re_q, Ck_im_q = Ck_re, Ck_im
            Cr_re_q, Cr_im_q = Cr_re, Cr_im

        # A/B butterfly.  conj(C[N-k]) = (Cr_re, -Cr_im).
        #   A = (C[k] + conj(C[N-k])) / 2
        #   B = (C[k] - conj(C[N-k])) / (2j)  ->  B_re = D_im/2, B_im = -D_re/2
        s_re = Signal(signed(w + 1)); s_im = Signal(signed(w + 1))
        d_re = Signal(signed(w + 1)); d_im = Signal(signed(w + 1))
        m.d.comb += [
            s_re.eq(Ck_re_q + Cr_re_q),
            s_im.eq(Ck_im_q - Cr_im_q),
            d_re.eq(Ck_re_q - Cr_re_q),
            d_im.eq(Ck_im_q + Cr_im_q),
        ]
        # Stage 2 output registers.
        with m.If(self.clken):
            m.d.sync += [
                self.a_re.eq(s_re >> 1),
                self.a_im.eq(s_im >> 1),
                self.b_re.eq(d_im >> 1),
                self.b_im.eq((-d_re) >> 1),
            ]

        # Control pipeline: valid + out_last delayed by `latency` to line up with
        # the datapath.  A bin read at cnt=p emits `latency` cycles later.
        lat = self.latency
        vpipe = Signal(lat)
        lastpipe = Signal(lat)
        with m.If(self.clken):
            m.d.sync += [
                vpipe.eq(Cat(seen_frame, vpipe[:-1])),
                lastpipe.eq(Cat(cnt == (N - 1), lastpipe[:-1])),
            ]
        m.d.comb += [
            self.out_valid.eq(vpipe[-1]),
            self.out_last.eq(vpipe[-1] & lastpipe[-1]),
        ]
        return m
