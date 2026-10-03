# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``bin_select`` — keep an arbitrary per-ADC set of bins from the A/B stream.

``ab_split`` emits A[k], B[k] for every k = 0..N-1 in natural order, one bin per
``in_valid`` cycle, with ``in_last`` on k = N-1. This block passes through only
the bins the host asked for, with the two ADCs selected **independently**: for the
current bin ``bk`` it asks an external keep-bitmap (:class:`~.mask_mem.MaskMem`)
whether to stream A[bk] (``keep_a_in``) and/or B[bk] (``keep_b_in``).  This replaces
the old single contiguous run (``k0``/``w_run``): several bands at once, and
diversity (ADC B) only on the bins that want it (host-application milestone).

The keep-bitmap lives OUTSIDE this block, as a dual-clock BRAM at the top level:
``rd_addr`` (= the free-running bin counter ``bk``) drives its registered read port,
and its result comes back as ``keep_a_in`` / ``keep_b_in`` one cycle later, aligned
with the a/b passthrough here (latency 1).  Keeping the memory external — written
straight from the sync-domain register with no CDC pulse — is what makes the load
reliable and immune to ``dp_reset`` (see ``mask_mem`` for the history).

Per kept bin the block emits a **per-ADC write address** ``cnt_a`` / ``cnt_b`` (the
running count of kept-A / kept-B bins in the frame, 0-based) that StreamFormat uses
as its buffer address; ``W_a`` and ``W_b`` may differ.  It also carries
``frame_last`` (= ``in_last`` through the pipeline, the per-frame emit trigger —
independent of whether bin N-1 is kept) and the per-frame counter ``seq``.
"""

from amaranth import *


class BinSelect(Elaboratable):
    """Keep a per-ADC bitmap-selected set of bins from a natural-order stream.

    Parameters
    ----------
    width_in : int
        A/B component width (fcfb Stage-1 = 23).
    order_log2 : int
        log2 of the FFT size N.
    seq_width : int
        Width of the frame counter ``seq`` (wire uses u64).

    Attributes
    ----------
    clken : Signal(), in
    in_valid, in_last : Signal(), in     From ab_split (out_valid / out_last).
    a_re, a_im, b_re, b_im : Signal(signed(width_in)), in
    rd_addr : Signal(order_log2), out    Keep-bitmap read address (= bin ``bk``).
    keep_a_in, keep_b_in : Signal(), in  Registered keep bits for the current bin
                                         (from the external MaskMem, latency 1).
    keep_a, keep_b : Signal(), out       This bin kept for A / B (aligned out).
    cnt_a, cnt_b : Signal(order_log2), out    Per-ADC write address for this bin.
    frame_last : Signal(), out           End of the 4096-bin frame (emit trig).
    seq : Signal(seq_width), out         Frame index of the bin currently out.
    a_re/a_im/b_re/b_im _o : Signal(signed(width_in)), out   Passed-through bins.
    """
    def __init__(self, width_in, order_log2, seq_width=64):
        self.w = width_in
        self.order_log2 = order_log2
        self.seqw = seq_width

        self.clken = Signal()
        self.in_valid = Signal()
        self.in_last = Signal()
        self.a_re = Signal(signed(self.w)); self.a_im = Signal(signed(self.w))
        self.b_re = Signal(signed(self.w)); self.b_im = Signal(signed(self.w))

        # External keep-bitmap read interface (see MaskMem).
        self.rd_addr = Signal(order_log2)
        self.keep_a_in = Signal()
        self.keep_b_in = Signal()

        self.keep_a = Signal()
        self.keep_b = Signal()
        self.cnt_a = Signal(order_log2)
        self.cnt_b = Signal(order_log2)
        self.frame_last = Signal()
        self.seq = Signal(seq_width)
        self.a_re_o = Signal(signed(self.w)); self.a_im_o = Signal(signed(self.w))
        self.b_re_o = Signal(signed(self.w)); self.b_im_o = Signal(signed(self.w))

    @property
    def latency(self):
        return 1

    def elaborate(self, platform):
        m = Module()
        ol2 = self.order_log2

        # ---- input-stage bin / frame counters ----
        bk = Signal(ol2)                 # bin index within the current frame
        frame = Signal(self.seqw)        # frame counter (seq source)

        # Drive the external keep-bitmap read at the input stage; its (registered)
        # result lands next cycle on keep_a_in/keep_b_in, aligned with the a/b
        # passthrough registers below.
        m.d.comb += self.rd_addr.eq(bk)

        # ---- output stage (latency 1): passthrough + per-bin tags ----
        v_o = Signal()                   # this output slot carries a valid bin
        first_o = Signal()               # ... and it is bin index 0 of the frame
        with m.If(self.clken):
            m.d.sync += [
                v_o.eq(self.in_valid),
                first_o.eq(self.in_valid & (bk == 0)),
                self.frame_last.eq(self.in_valid & self.in_last),
                self.seq.eq(frame),
                self.a_re_o.eq(self.a_re), self.a_im_o.eq(self.a_im),
                self.b_re_o.eq(self.b_re), self.b_im_o.eq(self.b_im),
            ]
            with m.If(self.in_valid):
                with m.If(self.in_last):
                    m.d.sync += [bk.eq(0), frame.eq(frame + 1)]
                with m.Else():
                    m.d.sync += bk.eq(bk + 1)

        # Per-ADC keep + running write address for the output-stage bin.  cnt_*
        # is the pre-increment kept count (the buffer address for this bin); it
        # resets to 0 on the first bin of each frame.
        cnt_a_r = Signal(ol2)
        cnt_b_r = Signal(ol2)
        m.d.comb += [
            self.keep_a.eq(v_o & self.keep_a_in),
            self.keep_b.eq(v_o & self.keep_b_in),
            self.cnt_a.eq(Mux(first_o, 0, cnt_a_r)),
            self.cnt_b.eq(Mux(first_o, 0, cnt_b_r)),
        ]
        with m.If(self.clken):
            m.d.sync += [
                cnt_a_r.eq(self.cnt_a + self.keep_a),
                cnt_b_r.eq(self.cnt_b + self.keep_b),
            ]
        return m
