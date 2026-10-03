# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``stream_format`` — serialize selected bins into fcfb_stream.h records.

Consumes the per-block stream of quantised A/B bins (from bin_select -> quantise)
and emits each block as a **BlockRecord** in the exact byte order the host reader
(`host/fcfb_stream.h`) expects, as a 32-bit little-endian AXI-stream for the DMA
path.

The two ADCs are selected independently now (per-ADC keep-bitmap), so each block
carries ``W_a`` A-bins then ``W_b`` B-bins with **W_a and W_b differing freely**
(``W_a = popcount(mask_a)``, ``W_b = popcount(mask_b)``; either may be 0).  The
bins arrive as a per-ADC write-enable (``we_a`` / ``we_b``) plus a per-ADC write
address (``addr_a`` / ``addr_b``, the running kept count from bin_select).

The on-wire component width is ``bin_width``:

  * ``bin_width == 16`` -- one bin is exactly one 32-bit word ``{Q[31:16],
    I[15:0]}``, DMA/AXI-word aligned::

        u64 seq                          -> 2 words: seq[31:0], seq[63:32]
        W_a x {i16 I, i16 Q}             -> W_a words: {Q in [31:16], I}
        W_b x {i16 I, i16 Q}             -> W_b words

  * ``bin_width == 24`` (Option-B wideband build) -- each bin is I(24)+Q(24) = 48
    bits, so the payload is a contiguous little-endian byte stream that a bit-packer
    regroups into 32-bit DMA words::

        u64 seq                          -> 2 words
        W_a x { i24 I, i24 Q }  then  W_b x { i24 I, i24 Q }   (6 bytes/bin, LE)
        then 0..2 zero pad bytes so the whole record is a multiple of 4 bytes

    (a pad occurs only when the total kept bin count W_a+W_b is odd).

``out_last`` marks the final word of a block (DMA/packet boundary).

The stream HEADER is written ONCE by the PS/board-server software into the DMA
buffer; the PL only streams the per-block records.

**Emit trigger + double buffering.**  With a bitmap, kept bins can sit anywhere in
the frame (including k near 0 and k near N-1), so a block is emitted on
``frame_last`` (end of the N-bin frame), not on a mid-frame "last kept bin" as the
old contiguous-run design did.  Emitting on frame_last means the *next* frame's
early kept bins would overwrite the record buffer while it is still draining, so
the buffers are **ping-ponged**: frame f writes bank ``f%2`` while the emit reads
bank ``(f-1)%2``.  This removes the race with no timing assumption; the record for
frame f drains long before frame f+2 reuses its bank (emit <= ~3*wmax words << the
>= N-cycle frame period).

No output backpressure: downstream (the DMA FIFO) is assumed always-ready on clken
cycles -- valid for the buffered maia DMA path.
"""

from amaranth import *
from amaranth.lib.memory import Memory


class StreamFormat(Elaboratable):
    """Serialise selected bins into fcfb_stream.h BlockRecords (32-bit words).

    Parameters
    ----------
    order_log2 : int   log2 N (sizes the write-address inputs).
    wmax : int         Max kept bins per ADC per block (buffer bank depth).
    seq_width : int    Frame-counter width (wire uses u64).
    bin_width : int    On-wire I/Q component width: 16 (shipping) or 24 (int24).

    Attributes
    ----------
    clken : Signal(), in
    we_a, we_b : Signal(), in          A / B kept bin present this cycle.
    addr_a, addr_b : Signal(order_log2), in   Its per-ADC buffer address.
    seq : Signal(seq_width), in        Block frame index (stable over the block).
    frame_last : Signal(), in          End of the frame -> latch W_a/W_b, emit.
    a_i, a_q, b_i, b_q : Signal(signed(bin_width)), in   Quantised I/Q for A, B.
    out_valid : Signal(), out          32-bit word valid (on clken).
    out_data : Signal(32), out         Record word, little-endian.
    out_last : Signal(), out           Final word of the block record.
    """
    def __init__(self, order_log2, wmax=64, seq_width=64, bin_width=16):
        if bin_width not in (16, 24):
            raise ValueError("bin_width must be 16 or 24")
        self.order_log2 = order_log2
        self.wmax = wmax
        self.seqw = seq_width
        self.bin_width = bin_width
        # Bank address width (>= wmax); the ping-pong bank bit sits on top.
        self.aw = max(1, (wmax - 1).bit_length())
        self.bankdepth = 1 << self.aw

        self.clken = Signal()
        self.we_a = Signal(); self.we_b = Signal()
        self.addr_a = Signal(order_log2); self.addr_b = Signal(order_log2)
        self.seq = Signal(seq_width)
        self.frame_last = Signal()
        bw = bin_width
        self.a_i = Signal(signed(bw)); self.a_q = Signal(signed(bw))
        self.b_i = Signal(signed(bw)); self.b_q = Signal(signed(bw))

        self.out_valid = Signal()
        self.out_data = Signal(32)
        self.out_last = Signal()

    def elaborate(self, platform):
        if self.bin_width == 16:
            return self._elab_word_per_bin()
        return self._elab_bitpack()

    # ---- shared: ping-pong record buffers + frame-end capture --------------
    def _buffers(self, m, shape):
        """Two banked A/B buffers (depth 2*bankdepth, bank = high addr bit).

        Returns (wr_a, wr_b, rd_a, rd_b, wr_bank, rd_bank) with the write ports
        already wired to the per-ADC we/addr/data inputs (bank = wr_bank) and the
        read ports as async (comb, LUTRAM) reads addressed by ``jcnt`` within
        ``rd_bank`` -- the caller drives the read index and rd_bank/wr_bank state.
        """
        aw = self.aw
        m.submodules.mem_a = mem_a = Memory(shape=shape, depth=2 * self.bankdepth,
                                            init=[])
        m.submodules.mem_b = mem_b = Memory(shape=shape, depth=2 * self.bankdepth,
                                            init=[])
        wr_a = mem_a.write_port(); wr_b = mem_b.write_port()
        rd_a = mem_a.read_port(domain="comb")   # async read -> LUTRAM
        rd_b = mem_b.read_port(domain="comb")
        wr_bank = Signal(); rd_bank = Signal()

        if shape == 32:
            a_data = Cat(self.a_i, self.a_q)
            b_data = Cat(self.b_i, self.b_q)
        else:
            a_data = Cat(self.a_i, self.a_q)    # 48-bit int24 bin (i then q)
            b_data = Cat(self.b_i, self.b_q)
        m.d.comb += [
            wr_a.addr.eq(Cat(self.addr_a[:aw], wr_bank)),
            wr_a.data.eq(a_data),
            wr_a.en.eq(self.clken & self.we_a),
            wr_b.addr.eq(Cat(self.addr_b[:aw], wr_bank)),
            wr_b.data.eq(b_data),
            wr_b.en.eq(self.clken & self.we_b),
        ]
        return wr_a, wr_b, rd_a, rd_b, wr_bank, rd_bank

    # ------------------------------------------------------------------ int16
    def _elab_word_per_bin(self):
        """One 32-bit word per bin ``{Q[31:16], I[15:0]}`` (shipping critical)."""
        m = Module()
        aw = self.aw

        wr_a, wr_b, rd_a, rd_b, wr_bank, rd_bank = self._buffers(m, 32)

        seq_lat = Signal(self.seqw)
        w_lat_a = Signal(aw + 1)          # W_a of the block being emitted
        w_lat_b = Signal(aw + 1)          # W_b
        jcnt = Signal(aw)                 # read index within a run

        a_last = Signal(); b_last = Signal()
        m.d.comb += [
            a_last.eq(jcnt == (w_lat_a - 1)),
            b_last.eq(jcnt == (w_lat_b - 1)),
            rd_a.addr.eq(Cat(jcnt, rd_bank)),
            rd_b.addr.eq(Cat(jcnt, rd_bank)),
        ]

        with m.FSM():
            with m.State("IDLE"):
                # Frame end: latch W_a/W_b + seq, flip the ping-pong banks, and
                # start emitting the just-completed bank.  W_a/W_b include the
                # final bin (addr + its own we).
                with m.If(self.clken & self.frame_last):
                    m.d.sync += [
                        seq_lat.eq(self.seq),
                        # aw+1 slice so W == 2**aw (full wmax) does not wrap to 0:
                        # at full W the kept count sits at 2**aw on the (unkept)
                        # frame_last bin, and addr_a[:aw] would slice it to 0.
                        # Writes still use addr_a[:aw] (max write addr = W-1).
                        w_lat_a.eq(self.addr_a[:aw + 1] + self.we_a),
                        w_lat_b.eq(self.addr_b[:aw + 1] + self.we_b),
                        rd_bank.eq(wr_bank),
                        wr_bank.eq(~wr_bank),
                    ]
                    m.next = "SEQ0"
            with m.State("SEQ0"):
                m.d.comb += [self.out_valid.eq(self.clken),
                             self.out_data.eq(seq_lat[:32])]
                with m.If(self.clken):
                    m.next = "SEQ1"
            with m.State("SEQ1"):
                m.d.comb += [self.out_valid.eq(self.clken),
                             self.out_data.eq(seq_lat[32:64]),
                             # empty block (W_a=W_b=0): this seq word is the last.
                             self.out_last.eq((w_lat_a == 0) & (w_lat_b == 0))]
                with m.If(self.clken):
                    m.d.sync += jcnt.eq(0)
                    with m.If(w_lat_a != 0):
                        m.next = "EMIT_A"
                    with m.Elif(w_lat_b != 0):
                        m.next = "EMIT_B"
                    with m.Else():
                        m.next = "IDLE"
            with m.State("EMIT_A"):
                m.d.comb += [
                    self.out_valid.eq(self.clken),
                    self.out_data.eq(rd_a.data),
                    self.out_last.eq(a_last & (w_lat_b == 0)),
                ]
                with m.If(self.clken):
                    with m.If(a_last):
                        m.d.sync += jcnt.eq(0)
                        with m.If(w_lat_b != 0):
                            m.next = "EMIT_B"
                        with m.Else():
                            m.next = "IDLE"
                    with m.Else():
                        m.d.sync += jcnt.eq(jcnt + 1)
            with m.State("EMIT_B"):
                m.d.comb += [
                    self.out_valid.eq(self.clken),
                    self.out_data.eq(rd_b.data),
                    self.out_last.eq(b_last),
                ]
                with m.If(self.clken):
                    with m.If(b_last):
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += jcnt.eq(jcnt + 1)
        return m

    # ------------------------------------------------------------------ int24
    def _elab_bitpack(self):
        """48-bit bins packed into a 32-bit-word byte stream (Option-B int24).

        Each stored bin is ``Cat(i24, q24)`` (48 bits) whose low..high bits are
        i[0..23] then q[0..23]; the byte stream is therefore ``i`` as 3 LE signed
        bytes then ``q`` as 3 LE signed bytes.  A little bit accumulator collects
        the A-run (W_a bins) then the B-run (W_b bins) as one continuous bit stream
        and drains it 32 bits at a time (LE) into ``out_data``; a partial final word
        (when W_a+W_b is odd) is zero-padded to a word boundary.

        The bit-packer machinery (acc/nbits/binq prefetch) is unchanged from the
        Option-B critical-path split; only the run bounds became per-ADC (W_a/W_b)
        and the buffers are banked (ping-pong).
        """
        m = Module()
        aw = self.aw
        BW = self.bin_width          # 24
        BINW = 2 * BW                # 48 bits per bin (I+Q)
        ACCW = BINW + 32             # accumulator: <=31 leftover + one 48-bit push

        wr_a, wr_b, rd_a, rd_b, wr_bank, rd_bank = self._buffers(m, BINW)

        seq_lat = Signal(self.seqw)
        w_lat_a = Signal(aw + 1)         # W_a of the block being emitted
        w_lat_b = Signal(aw + 1)         # W_b
        # Precomputed run-end indices (W-1), latched at frame_last alongside
        # w_lat_a/b so the hot ld_idx loop tests ``ld_idx == run_end`` (a latched
        # compare) instead of an in-loop ``run_len - 1`` subtract.  An empty run
        # gives m1 = -1 (all ones) but is never entered, so numerics are identical.
        w_lat_a_m1 = Signal(aw + 1)
        w_lat_b_m1 = Signal(aw + 1)

        # ---- emit state (bit-packer, with a REGISTERED-address RAM read) ----
        # The banked record buffers are async-read distributed RAM.  At wmax=1024
        # they are 2*1024 deep, and a comb read straight off ``ld_idx`` (high
        # fanout, also feeding the run-end compare) missed the 166.67 MHz fft clock
        # by ~60 ps (route-bound: ld_idx -> deep-RAM addr -> MUXF7/F8 -> binq).
        # Fix: register the read address in ``rd_addr`` (a dedicated reg the tool
        # can replicate next to the RAM), giving a 1-cycle-latency read, and add a
        # one-deep skid (``pre``) so the packer still gets one bin ready per step.
        acc = Signal(ACCW)               # bit accumulator, filled LE from bit 0
        nbits = Signal(range(ACCW + 1))  # valid bits currently in acc
        ld_idx = Signal(aw + 1)          # index of the NEXT bin to ISSUE in this run
        ld_run_b = Signal()             # 0 = issuing A run, 1 = issuing B run
        fetch_more = Signal()           # more bins remain to ISSUE
        rd_addr = Signal(aw)            # REGISTERED read address (placed near RAM)
        rd_run_b = Signal()            # run of the in-flight (registered) read
        rd_pending = Signal()          # an issued read's data is valid this cycle
        pre = Signal(BINW)             # skid: holds the read bin before binq
        pre_full = Signal()
        binq = Signal(BINW)             # REGISTERED prefetched bin (I+Q, 48 bits)
        binq_full = Signal()            # binq holds a not-yet-packed bin

        cur_bin = Signal(BINW)
        m.d.comb += [
            rd_a.addr.eq(Cat(rd_addr, rd_bank)),
            rd_b.addr.eq(Cat(rd_addr, rd_bank)),
            cur_bin.eq(Mux(rd_run_b, rd_b.data, rd_a.data)),
        ]

        # Combinational per-cycle emit / pack arithmetic (operates on ``binq``).
        # ``drained`` = nothing left upstream of acc (all issued, no in-flight
        # read, skid + binq empty) -> the tail flush and EMIT-exit gate on it.
        emit_full = Signal()
        emit_tail = Signal()
        do_emit = Signal()
        nb_ae = Signal(range(ACCW + 1))          # nbits after a possible emit
        acc_ae = Signal(ACCW)                    # acc after a possible emit
        can_load = Signal()
        drained = Signal()
        m.d.comb += [
            drained.eq((~fetch_more) & (~rd_pending) & (~pre_full) & (~binq_full)),
            emit_full.eq(nbits >= 32),
            emit_tail.eq(drained & (nbits != 0) & (nbits < 32)),
            do_emit.eq(emit_full | emit_tail),
            nb_ae.eq(Mux(emit_full, nbits - 32, Mux(emit_tail, 0, nbits))),
            acc_ae.eq(Mux(emit_full, acc >> 32, Mux(emit_tail, 0, acc))),
            can_load.eq(binq_full & (nb_ae <= (ACCW - BINW))),
        ]
        ld_pos = Signal(range(ACCW - BINW + 1))
        acc_next = Signal(ACCW)
        nb_next = Signal(range(ACCW + 1))
        m.d.comb += [
            ld_pos.eq(Mux(can_load, nb_ae, 0)),
            acc_next.eq(Mux(can_load, acc_ae | (binq << ld_pos), acc_ae)),
            nb_next.eq(Mux(can_load, nb_ae + BINW, nb_ae)),
        ]

        # Pipeline handshake: binq refills from the skid ``pre``, ``pre`` from the
        # in-flight registered read, and a new read is issued whenever the pipe can
        # advance (no unconsumed pending read blocking + skid room downstream).
        binq_refill = Signal()
        capture_pre = Signal()
        issue = Signal()
        m.d.comb += [
            binq_refill.eq(pre_full & (can_load | ~binq_full)),
            capture_pre.eq(rd_pending & (~pre_full | binq_refill)),
            issue.eq(fetch_more & (capture_pre | ~rd_pending)),
        ]

        # End index (W-1) of the run currently being ISSUED, from the latched
        # per-ADC m1 values -- keeps the subtract out of the ld_idx hot loop.
        run_end = Signal(aw + 1)
        m.d.comb += run_end.eq(Mux(ld_run_b, w_lat_b_m1, w_lat_a_m1))
        any_bins = Signal()
        m.d.comb += any_bins.eq((w_lat_a != 0) | (w_lat_b != 0))

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.clken & self.frame_last):
                    m.d.sync += [
                        seq_lat.eq(self.seq),
                        # aw+1 slice so W == 2**aw (full wmax) does not wrap to 0
                        # (see the int16 path); writes still use addr_a[:aw].
                        w_lat_a.eq(self.addr_a[:aw + 1] + self.we_a),
                        w_lat_b.eq(self.addr_b[:aw + 1] + self.we_b),
                        w_lat_a_m1.eq((self.addr_a[:aw + 1] + self.we_a) - 1),
                        w_lat_b_m1.eq((self.addr_b[:aw + 1] + self.we_b) - 1),
                        rd_bank.eq(wr_bank),
                        wr_bank.eq(~wr_bank),
                    ]
                    m.next = "SEQ0"
            with m.State("SEQ0"):
                m.d.comb += [self.out_valid.eq(self.clken),
                             self.out_data.eq(seq_lat[:32])]
                with m.If(self.clken):
                    m.next = "SEQ1"
            with m.State("SEQ1"):
                m.d.comb += [self.out_valid.eq(self.clken),
                             self.out_data.eq(seq_lat[32:64]),
                             # empty block (W_a=W_b=0): this seq word is the last.
                             self.out_last.eq(~any_bins)]
                with m.If(self.clken):
                    # Start the bit-packer: issue A run first if present, else B.
                    m.d.sync += [
                        acc.eq(0), nbits.eq(0), ld_idx.eq(0),
                        ld_run_b.eq(w_lat_a == 0), binq_full.eq(0),
                        fetch_more.eq(any_bins),
                        rd_pending.eq(0), pre_full.eq(0),
                    ]
                    with m.If(any_bins):
                        m.next = "EMIT"
                    with m.Else():
                        m.next = "IDLE"
            with m.State("EMIT"):
                with m.If(drained & (nbits == 0)):
                    m.next = "IDLE"
                with m.Else():
                    m.d.comb += [
                        self.out_valid.eq(self.clken & do_emit),
                        self.out_data.eq(acc[:32]),
                        self.out_last.eq(do_emit & drained & (nb_next == 0)),
                    ]
                    with m.If(self.clken):
                        m.d.sync += [acc.eq(acc_next), nbits.eq(nb_next)]
                        # binq <- pre skid
                        with m.If(binq_refill):
                            m.d.sync += [binq.eq(pre), binq_full.eq(1)]
                        with m.Elif(can_load):
                            m.d.sync += binq_full.eq(0)
                        # pre <- in-flight registered read
                        with m.If(capture_pre):
                            m.d.sync += [pre.eq(cur_bin), pre_full.eq(1)]
                        with m.Elif(binq_refill):
                            m.d.sync += pre_full.eq(0)
                        # issue next read + advance the issue pointer
                        with m.If(issue):
                            m.d.sync += [rd_addr.eq(ld_idx[:aw]),
                                         rd_run_b.eq(ld_run_b), rd_pending.eq(1)]
                            with m.If(ld_idx == run_end):
                                # end of the current run
                                with m.If((~ld_run_b) & (w_lat_b != 0)):
                                    m.d.sync += [ld_run_b.eq(1), ld_idx.eq(0)]
                                with m.Else():
                                    m.d.sync += fetch_more.eq(0)
                            with m.Else():
                                m.d.sync += ld_idx.eq(ld_idx + 1)
                        with m.Elif(capture_pre):
                            m.d.sync += rd_pending.eq(0)
        return m
