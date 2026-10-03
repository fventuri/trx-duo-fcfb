# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``mask_mem`` — the per-ADC keep-bitmap memory (dual-clock BRAM + clear sweep).

Holds two bits per FFT bin (``keep_a`` = stream A[k], ``keep_b`` = stream B[k]) for
the whole N-bin frame.  It is a **dual-clock BRAM**: the write/clear side runs in
the register (``s_axi_lite``-fed ``sync``) domain and the read side in the datapath
domain (``fft`` in the wideband build, ``sync`` in the critical build).  The mask is
**quasi-static** — loaded only while the datapath is disabled — so the sync-write /
fft-read crossing is a plain stable-content BRAM read with no per-write handshake.

This deliberately replaces an earlier design that kept the memory inside the
fft-domain ``BinSelect`` and crossed each write in as a ``PulseSynchronizer`` pulse.
That was unreliable on hardware: (a) the load pulses could be dropped, and (b) the
soft datapath reset ``dp_reset`` resets the fft-side of the pulse synchronisers but
not their sync-side toggle flags, so releasing ``dp_reset`` after any ``clear`` had
been issued produced a **phantom clear pulse** that wiped the just-loaded mask
(HW-observed: ~every capture after a retune came back empty).  Writing the BRAM
directly from the sync register — with the memory living at the top level, outside
the ``dp_reset`` scope — removes both failure modes: writes are same-domain and
reliable, and no reset-sensitive pulse crossing exists.

Loading protocol (board server, while ``enable=0``): pulse ``clear`` (a sweep zeroes
all N entries; ``busy`` high meanwhile), then one ``wr_en`` write per kept bin
(``wr_data`` bit0=keep_a, bit1=keep_b at ``wr_addr``).
"""

from amaranth import *
from amaranth.lib.memory import Memory


class MaskMem(Elaboratable):
    """Dual-clock 2-bit-per-bin keep-bitmap with a sync-side clear sweep.

    Parameters
    ----------
    order_log2 : int
        log2 of the FFT size N (memory depth).
    rd_domain : str
        Read-side clock domain (datapath): ``'fft'`` wideband, ``'sync'`` critical.
    wr_domain : str
        Write/clear-side clock domain (register): ``'sync'``.

    Attributes
    ----------
    rd_addr : Signal(order_log2), in    Read address (the bin counter ``bk``).
    rd_en : Signal(), in                Read clock-enable (= datapath clken).
    keep_a, keep_b : Signal(), out      Registered keep bits for ``rd_addr`` (lat 1).
    wr_addr : Signal(order_log2), in    Write address (a bin), wr_domain.
    wr_data : Signal(2), in             bit0 = keep_a, bit1 = keep_b.
    wr_en : Signal(), in                Write strobe (one bin), wr_domain.
    clear : Signal(), in                Pulse: sweep-zero the whole memory, wr_domain.
    busy : Signal(), out                High while a clear sweep runs, wr_domain.
    """
    def __init__(self, order_log2, rd_domain='sync', wr_domain='sync'):
        self.order_log2 = order_log2
        self.rd_domain = rd_domain
        self.wr_domain = wr_domain

        self.rd_addr = Signal(order_log2)
        self.rd_en = Signal()
        self.keep_a = Signal()
        self.keep_b = Signal()

        self.wr_addr = Signal(order_log2)
        self.wr_data = Signal(2)
        self.wr_en = Signal()
        self.clear = Signal()
        self.busy = Signal()

    def elaborate(self, platform):
        m = Module()
        N = 1 << self.order_log2
        wd = self.wr_domain

        m.submodules.mem = mem = Memory(shape=2, depth=N, init=[])
        wp = mem.write_port(domain=self.wr_domain)
        rp = mem.read_port(domain=self.rd_domain)     # registered read

        # Clear sweep (write-side domain): on ``clear`` walk addr 0..N-1 writing 0,
        # ``busy`` high meanwhile.  Runs on the write-domain clock regardless of any
        # datapath enable (loads happen while the datapath is disabled).
        sweep = Signal(self.order_log2)
        with m.If(self.clear & ~self.busy):
            m.d[wd] += [self.busy.eq(1), sweep.eq(0)]
        with m.Elif(self.busy):
            m.d[wd] += sweep.eq(sweep + 1)
            with m.If(sweep == (N - 1)):
                m.d[wd] += self.busy.eq(0)

        m.d.comb += [
            wp.addr.eq(Mux(self.busy, sweep, self.wr_addr)),
            wp.data.eq(Mux(self.busy, 0, self.wr_data)),
            wp.en.eq(self.busy | self.wr_en),
            rp.addr.eq(self.rd_addr),
            rp.en.eq(self.rd_en),
            self.keep_a.eq(rp.data[0]),
            self.keep_b.eq(rp.data[1]),
        ]
        return m
