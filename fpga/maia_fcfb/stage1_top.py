# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Modeled on the trx-duo MaiaTrxduo top level (© Daniel Estevez / MaiaSDR port,
# MIT), simplified for fcfb: no analytic converter, no DDC, no even/odd
# double-pack, no integrator -- a single 125 MHz datapath domain.
# Reuses maia-hdl (Axi4LiteRegisterBridge / RegisterCDC / DmaStreamWrite) and the
# trx-duo AdcCapture / IqCDC (all MIT).  CLEAN-ROOM: no ka9q-radio / npapi source.
"""``Stage1Top`` — synthesizable top-level IP core for the fcfb Stage-1 datapath.

Wires the real ADC front end, the AXI-Lite register file, the Stage-1 datapath,
and the streaming DMA egress, exposing the clock domains as IP ports (the MMCM
that generates them lives in the Vivado block design; see ``fcfb_platform``)::

    adc pins ─▶ AdcCapture ─▶ IqCDC ─▶ Stage1 ─▶ SyncFIFO ─▶ DmaStreamWrite ─▶ m_axi_dma
    (sampling)  (sampling)   (→sync)  (sync: WOLA+FFT+split+select+quantise+frame)

    s_axi_lite ─▶ Axi4LiteRegisterBridge ─▶ control_registers (s_axi_lite)
                                          └▶ RegisterCDC ─▶ fcfb_registers (sync) ┘

The single-125 clock plan (confirmed 2026-08-15): ``sampling`` and ``sync`` are
both 125 MHz (the FFT runs one complex sample per sample-clock; ``cmult3x`` is
infeasible, so the FFT uses ``cmult3x=False`` / ``window=None`` and the WOLA
supplies the window).  ``AdcCapture`` runs in ``sampling`` and ``IqCDC`` crosses
to ``sync`` for clock-domain / IOB hygiene even though the two run at the same
rate.  ``s_axi_lite`` is the PS AXI clock (~100 MHz).

Egress: ``Stage1`` free-runs on ``clken`` (no back-pressure input), so a
``SyncFIFOBuffered`` absorbs its 32-bit record stream and rate-matches it to the
``DmaStreamWrite`` stream port, which writes a contiguous byte stream of
``fcfb_stream.h`` BlockRecords to a fixed DDR ring.  The DMA runs a 32-bit AXI3
master; the block design's data-width converter adapts it to the 64-bit AXI-HP
slave.  PS software writes the 48-byte stream header once; the PL streams only
the records.

Like the trx-duo top, the structural / ``Instance``-based parts (the ``FIFO18E1``
in ``IqCDC``, the AXI3 master in ``DmaStreamWrite``) are not exercised by the
Amaranth simulator; this module is verified by elaborating and converting it to
Verilog on the fcfb platform (see ``test_stage1_top.py``), plus the per-block
simulations of its components (the 33-test Stage-1 suite + ``test_registers``).
"""

import argparse

from amaranth import *
from amaranth.lib.cdc import FFSynchronizer, PulseSynchronizer
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFO
import amaranth.back.verilog

from maia_hdl import axi
from maia_hdl.axi4_lite import Axi4LiteRegisterBridge
from maia_hdl.cdc import RegisterCDC
from maia_hdl.dma import DmaStreamWrite

from maia_trxduo.adc_capture import AdcCapture
from maia_trxduo.iq_cdc import IqCDC

from .stage1 import Stage1
from .mask_mem import MaskMem
from .dds import Dds
from . import registers as regs


class _BramAsyncFIFO(AsyncFIFO):
    """``amaranth.lib.fifo.AsyncFIFO`` with its storage forced into block RAM.

    Stock ``AsyncFIFO`` builds its storage as ``Memory(..., init=[])`` with no
    ``attrs`` hook, so it always maps to distributed RAM (LUTRAM). A deep egress
    FIFO in LUTRAM both congests the fabric and gives a wide, route-dominated
    read-pointer subtract that misses 125 MHz on the near-full 7010 -- which is
    why the LUTRAM egress had to be shrunk to depth 128 (capping W<=127). This
    subclass copies amaranth 0.5.9's ``elaborate`` verbatim and changes ONLY the
    storage line to ``attrs={"ram_style": "block"}``, so the storage lands in a
    hard BRAM (registered ports, no scattered write fanout) and the FIFO can be
    deep (512 in one tile) -- restoring the full W range and decongesting the
    fabric. Pinned to amaranth 0.5.9's stable ``elaborate`` body; the flag logic
    (Gray counters / CDC) is unchanged, only the memory primitive moves.
    """
    def elaborate(self, platform):
        from amaranth.lib.cdc import FFSynchronizer, AsyncFFSynchronizer
        from amaranth.lib.fifo import _gray_encode, _gray_decode
        from amaranth.lib.memory import Memory
        m = Module()
        if self.depth == 0:
            m.d.comb += [self.w_rdy.eq(0), self.r_rdy.eq(0)]
            return m

        do_write = self.w_rdy & self.w_en
        do_read = self.r_rdy & self.r_en

        produce_w_bin = Signal(self._ctr_bits)
        produce_w_nxt = Signal(self._ctr_bits)
        m.d.comb += produce_w_nxt.eq(produce_w_bin + do_write)
        m.d[self._w_domain] += produce_w_bin.eq(produce_w_nxt)

        consume_r_bin = Signal(self._ctr_bits, reset_less=True)
        consume_r_nxt = Signal(self._ctr_bits)
        m.d.comb += consume_r_nxt.eq(consume_r_bin + do_read)
        m.d[self._r_domain] += consume_r_bin.eq(consume_r_nxt)

        produce_w_gry = Signal(self._ctr_bits)
        produce_r_gry = Signal(self._ctr_bits)
        produce_cdc = m.submodules.produce_cdc = \
            FFSynchronizer(produce_w_gry, produce_r_gry, o_domain=self._r_domain)
        m.d[self._w_domain] += produce_w_gry.eq(_gray_encode(produce_w_nxt))

        consume_r_gry = Signal(self._ctr_bits, reset_less=True)
        consume_w_gry = Signal(self._ctr_bits)
        consume_cdc = m.submodules.consume_cdc = \
            FFSynchronizer(consume_r_gry, consume_w_gry, o_domain=self._w_domain)
        m.d[self._r_domain] += consume_r_gry.eq(_gray_encode(consume_r_nxt))

        consume_w_bin = Signal(self._ctr_bits)
        m.d[self._w_domain] += consume_w_bin.eq(_gray_decode(consume_w_gry))

        produce_r_bin = Signal(self._ctr_bits)
        m.d.comb += produce_r_bin.eq(_gray_decode(produce_r_gry))

        w_full = Signal()
        r_empty = Signal()
        m.d.comb += [
            w_full.eq((produce_w_gry[-1] != consume_w_gry[-1]) &
                      (produce_w_gry[-2] != consume_w_gry[-2]) &
                      (produce_w_gry[:-2] == consume_w_gry[:-2])),
            r_empty.eq(consume_r_gry == produce_r_gry),
        ]

        m.d[self._w_domain] += self.w_level.eq(produce_w_bin - consume_w_bin)
        m.d.comb += self.r_level.eq(produce_r_bin - consume_r_bin)

        # The one change vs. stock AsyncFIFO: force the storage into block RAM.
        storage = m.submodules.storage = Memory(
            shape=self.width, depth=self.depth, init=[],
            attrs={"ram_style": "block"})
        w_port = storage.write_port(domain=self._w_domain)
        r_port = storage.read_port(domain=self._r_domain)
        m.d.comb += [
            w_port.addr.eq(produce_w_bin[:-1]),
            w_port.data.eq(self.w_data),
            w_port.en.eq(do_write),
            self.w_rdy.eq(~w_full),
        ]
        m.d.comb += [
            r_port.addr.eq(consume_r_nxt[:-1]),
            self.r_data.eq(r_port.data),
            r_port.en.eq(1),
            self.r_rdy.eq(~r_empty),
        ]

        w_rst = ResetSignal(domain=self._w_domain, allow_reset_less=True)
        r_rst = Signal()
        rst_cdc = m.submodules.rst_cdc = \
            AsyncFFSynchronizer(w_rst, r_rst, o_domain=self._r_domain)

        with m.If(r_rst):
            m.d.comb += r_empty.eq(1)
            m.d[self._r_domain] += consume_r_gry.eq(produce_r_gry)
            m.d[self._r_domain] += consume_r_bin.eq(_gray_decode(produce_r_gry))
            m.d[self._r_domain] += self.r_rst.eq(1)
        with m.Else():
            m.d[self._r_domain] += self.r_rst.eq(0)

        return m


class Stage1Top(Elaboratable):
    """fcfb Stage-1 top-level IP core.

    Parameters
    ----------
    order_log2 : int
        log2 of the complex FFT size N (12 => 4096 for first light; small values
        keep the FFT tiny for fast Verilog-conversion smoke tests).
    width_in : int
        FFT input sample width (17).
    sample_width : int
        Raw ADC / packed sample component width (16).
    T : int
        WOLA overlap factor (4 = shipping default).
    twiddle_width, coeff_width : int
        FFT twiddle width / WOLA coefficient width.
    shift_width : int
        Quantise right-shift register width.
    wmax : int
        Max run width W buffered by StreamFormat.
    seq_width : int
        Frame-counter / wire ``seq`` width.
    egress_fifo_depth : int
        Depth of the record rate-matching FIFO.
    dma_start_address, dma_end_address : int
        DDR ring-buffer bounds for the streaming DMA (hardcoded at synthesis;
        must match the PS reserved-memory region; 64-byte aligned for width=32).
    dma_width : int
        DMA / stream data width (32 = one record word per beat).
    dma_awidth : int
        DMA AXI address width.
    dma_circular : bool
        If ``True`` (default, 3c), the streaming DMA wraps at the ring end and
        runs continuously instead of stopping — no re-arm gap, so the egress FIFO
        cannot overflow at the wrap. The server chases ``next_address`` modulo the
        ring. ``False`` restores the original one-shot DMA.
    hop : int or None
        WOLA block hop R. ``None`` (default) = critical sampling: the whole
        datapath is a single 125 MHz ``sync`` domain (the shipping path). ``R<N``
        selects the wideband oversampled fold (Phase-3b): the WOLA + FFT + backend
        move to a fast ``fft`` clock (166.67 MHz), FS-rate inputs cross in through
        the LUTRAM ``incdc`` AsyncFIFO and the record stream crosses back through
        the async ``egress_fifo``; the control registers cross ``sync``→``fft``
        per-field (FFSynchronizer). Default keeps the build bit-identical.
    incdc_depth : int
        Depth of the sync→fft input CDC FIFO (wideband only). Small (LUTRAM).
    """
    def __init__(self, order_log2=12, width_in=17, sample_width=16, T=4,
                 twiddle_width=16, coeff_width=18, shift_width=5, wmax=512,
                 seq_width=64, egress_fifo_depth=512,
                 dma_start_address=0x1000_0000, dma_end_address=0x1a00_0000,
                 dma_width=32, dma_awidth=32, dma_circular=True,
                 dds_amplitude=8192, hop=None, incdc_depth=16, n_dds=1,
                 bin_width=16, build_id=0):
        self.order_log2 = order_log2
        self.T = T
        self.wmax = wmax
        self.build_id = build_id
        self.sample_width = sample_width
        self.shift_width = shift_width
        self.hop = hop
        self.wideband = hop is not None
        # On-wire I/Q component width (Option B): 16 for the shipping critical
        # build, 24 for the wideband int24 build that streams the full 23-bit
        # internal bin (reconstructed-channel SFDR ceiling ~-102 dBc vs the
        # int16 ~-96 dBc -- sim/precision_study.py). The internal FFT/coeff/
        # twiddle datapath is UNCHANGED; only Quantise saturate + StreamFormat
        # packing widen, so the FFT timing is not disturbed.
        self.bin_width = bin_width
        # n_dds = number of on-PL DDS tone generators for the input-scene SELF-TEST
        # path (the SFDR / multi-window / two-tone injection).  It is verification
        # infrastructure, not the production ADC->channelise->DMA datapath, so a
        # production bitstream builds with n_dds=0 to drop the DDS(s) + input mux and
        # reclaim their fabric (~2 DSP + interp LUTRAM each).  n_dds>=2 sums two
        # tones (light two disjoint mask windows at once, or a two-tone IMD test).
        # The `dds` register stays mapped even at n_dds=0 (server-binary compat);
        # dds2/dds3 are mapped only when built with n_dds>=2/3 (3-bit bank cap = 3).
        if n_dds > 3:
            raise ValueError("n_dds > 3 needs more than the 3-bit fcfb bank")
        self.n_dds = n_dds
        self.with_dds = n_dds > 0
        self.dds_amplitude = dds_amplitude
        # AXI-Lite register space: 32-bit word addressing, 5 word-address bits
        # (control bank at words 0..3 / byte 0x00, fcfb bank at word 8 / 0x20,
        # read-only params bank at words 16..26 / byte 0x40).  address[4] selects
        # the params region; address[3] then splits control vs fcfb (unchanged).
        self.axi4_awidth = 5

        # Clock domains (exposed as IP ports; the MMCM is in the block design).
        # 'sync'/'sampling'(/'fft') resets are driven internally from the
        # sdr_reset register (via FFSynchronizer), so declare them explicitly.
        self.s_axi_lite = ClockDomain()
        self.sampling = ClockDomain()
        self.sync = ClockDomain()
        # Wideband: the fast FFT-side clock (166.67 MHz), from MMCM clk_out3.
        if self.wideband:
            self.fft = ClockDomain()

        # AXI-Lite register bridge + banks.
        self.axi4lite = Axi4LiteRegisterBridge(
            self.axi4_awidth, name='s_axi_lite')
        self.control_registers = regs.control_registers()
        self.fcfb_registers = regs.fcfb_registers(
            order_log2, shift_width, dma_awidth, n_dds=n_dds,
            dma_base_init=dma_start_address)
        # Read-only params bank: the host reads R (the analysis hop the kernel must
        # match: the wideband hop, else N), T, N and the other build constants to
        # validate its synthesis kernel before streaming.
        self.param_registers = regs.param_registers(
            r_hop=(hop if self.wideband else (1 << order_log2)),
            t_frames=T, n_fft=(1 << order_log2), wmax=wmax,
            bin_width=bin_width, n_dds=n_dds, build_id=build_id)
        self.register_map = regs.register_map(
            self.control_registers, self.fcfb_registers, self.param_registers)

        # Datapath.
        self.adc = AdcCapture(sample_width, sample_width, domain='sampling')
        self.cdc = IqCDC('sampling', 'sync', width=sample_width, tag_width=1)
        # DDS tone generators (sync domain) for input-scene injection.
        # Verification-only; empty on a production (n_dds=0) build.  The register
        # names line up with these in order: dds, dds2, dds3.
        self.ddss = [Dds(out_width=sample_width, amplitude=dds_amplitude,
                         domain='sync') for _ in range(n_dds)]
        self.stage1 = Stage1(
            width_in=width_in, order_log2=order_log2,
            twiddle_width=twiddle_width, window=None, cmult3x=False,
            shift_width=shift_width, seq_width=seq_width, wmax=wmax,
            T=T, sample_width=sample_width, coeff_width=coeff_width,
            hop=hop, ab_extra_pipe=self.wideband,
            quantise_extra_pipe=self.wideband,
            ab_share_write_port=self.wideband, bin_width=bin_width)

        # Per-ADC keep-bitmap: a top-level dual-clock BRAM written straight from
        # the sync-domain register (no CDC pulse) and read in the datapath domain
        # (fft wideband / sync critical).  It lives OUTSIDE stage1 so it is not in
        # the dp_reset scope -- a dp_reset must never disturb the loaded mask.
        self.maskmem = MaskMem(
            order_log2, rd_domain=('fft' if self.wideband else 'sync'),
            wr_domain='sync')

        # Egress: record rate-matching FIFO -> streaming DMA.  Critical: a
        # single-domain SyncFIFO (sync).  Wideband: an async FIFO crossing the
        # fast fft-domain record stream back to the sync-domain DMA (OutCDC).
        if self.wideband:
            # OutCDC: the fft(166.67)->sync(125) record-stream crossing.  It maps
            # to BRAM (``_BramAsyncFIFO`` forces ram_style=block) so it can be
            # deep -- the full 512 words -- WITHOUT the LUTRAM congestion + wide
            # route-dominated read-pointer subtract that forced the plain
            # ``AsyncFIFO`` down to depth 128 (capping W<=127).  A hard BRAM has
            # registered ports and no scattered distributed-RAM write fanout, and
            # freeing ~500-1000 LUTRAM cells decongests the fabric globally.  The
            # ab-split (wideband) runs single-copy (``ab_share_write_port``),
            # which frees the BRAM tiles this deeper FIFO needs.  Depth 512 lifts
            # the egress W cap to the ``wmax`` StreamFormat ceiling (it absorbs
            # ~0.25*(npres*W) words per record burst, then drains over the block).
            wb_egress_depth = min(egress_fifo_depth, 512)
            self.egress_fifo = _BramAsyncFIFO(
                width=32, depth=wb_egress_depth,
                w_domain='fft', r_domain='sync')
            # InCDC: FS-rate packed samples (Cat(re, im), sync) -> fft.  Small,
            # so it maps to distributed RAM (LUTRAM) and costs 0 BRAM (verify in
            # the util report); do NOT let it land in BRAM (breaks the 60/60).
            self.incdc = AsyncFIFO(
                width=2 * sample_width, depth=incdc_depth,
                w_domain='sync', r_domain='fft')
        else:
            self.egress_fifo = SyncFIFOBuffered(width=32, depth=egress_fifo_depth)
        # circular=True (3c): the DMA wraps at the ring end and never stops, so
        # there is no re-arm gap where the egress FIFO could overflow (the
        # intermittent multi-record wrap loss seen at high W). next_address then
        # advances continuously modulo the ring and the server chases it.
        # Option B (udmabuf zero-copy): the DMA's internal address counter is
        # 0-based over the ring SIZE, and stage1_top adds a runtime base offset
        # (the ``dma_base`` register, when mapped; else the compile-time
        # ``dma_start_address``) to the AXI ``awaddr`` on the way out.  This lets
        # the board server point the PL DMA at whatever physical address the
        # page-backed u-dma-buf ring landed on, without re-baking the bitstream.
        # Keeping SIZE compile-time preserves the DMA's counter width and its
        # constant-optimized end-compare (maia-hdl DmaStreamWrite is untouched).
        # The DMA's own AXI interface stays INTERNAL (name 'dma_i_axi'); the real
        # top-level ``m_axi_dma`` port is a separate interface that passes every
        # write-channel signal through, overriding only ``awaddr`` with the base.
        self.dma_ring_size = dma_end_address - dma_start_address
        self.dma_base_default = dma_start_address
        self.has_dma_base = n_dds < 2
        self.dma = DmaStreamWrite(
            0, self.dma_ring_size,
            width=dma_width, axi_awidth=dma_awidth, circular=dma_circular,
            name='dma_i')
        # Real external AXI3 write manager exposed as m_axi_dma (offset applied).
        self.m_axi = axi.AxiInterface(
            axi.AxiDevice.MANAGER,
            [axi.AxiChannel(axi.AxiDirection.WRITE, dma_awidth, dma_width)],
            axi.AxiVersion.AXI3, name='m_axi_dma')

        # ADC data pins and interrupt.
        self.adc_dat_a = Signal(sample_width)
        self.adc_dat_b = Signal(sample_width)
        self.adc_csn = Signal()
        self.interrupt_out = Signal()

    def ports(self):
        return (
            self.axi4lite.axi.ports()
            + self.m_axi.ports()
            + [
                self.adc_dat_a,
                self.adc_dat_b,
                self.adc_csn,
                self.interrupt_out,
                self.s_axi_lite.clk,
                self.s_axi_lite.rst,
                self.sampling.clk,
                self.sync.clk,
                self.sync.rst,
            ]
            # Wideband: the fast fft clock is an extra IP input (MMCM clk_out3).
            # Its reset is internal (sdr_reset), so it is not a port.
            + ([self.fft.clk] if self.wideband else [])
        )

    def svd(self):
        return self.register_map.svd()

    def elaborate(self, platform):
        m = Module()
        m.domains += [self.s_axi_lite, self.sampling, self.sync]
        if self.wideband:
            m.domains += [self.fft]
        s_axi_lite_renamer = DomainRenamer({'sync': 's_axi_lite'})

        m.submodules.axi4lite = s_axi_lite_renamer(self.axi4lite)
        m.submodules.control_registers = s_axi_lite_renamer(
            self.control_registers)
        # Read-only params bank lives in the s_axi_lite domain (pure constants, no
        # CDC needed).
        m.submodules.param_registers = s_axi_lite_renamer(self.param_registers)
        m.submodules.fcfb_registers = self.fcfb_registers
        m.submodules.fcfb_registers_cdc = fcfb_cdc = RegisterCDC(
            's_axi_lite', 'sync', self.fcfb_registers.aw)
        m.submodules.dma_interrupt = dma_interrupt = PulseSynchronizer(
            i_domain='sync', o_domain='s_axi_lite')

        # ---- soft DATAPATH reset (dp_reset) ------------------------------------
        # A second reset over ONLY the signal-processing datapath -- the free-
        # running two-rate WOLA/FFT fold, the InCDC, the DDS phase accumulator and
        # the egress FIFO.  It deliberately does NOT reach the RegisterCDC,
        # fcfb_registers or the DMA engine, so (a) the run/quant/dds programming
        # survives the pulse, (b) no mid-flight AXI RegisterCDC access is disturbed
        # (the sdr_reset AXI-wedge hazard) and (c) the AXI-HP0 DMA burst state is
        # untouched (the start-project HP0 hazard).  The server pulses it on every
        # (re)program to realign the fold to the fresh-boot condition -- without
        # it, only the first capture after start-project is valid (the free-running
        # fold keeps a stale block phase across a retune -> frequency-scaling error
        # + a static spur).
        #
        # HOW it is applied differs by domain, and this matters for timing:
        #  - fft domain: it holds ONLY datapath state, so dp_reset is ORed into the
        #    *source* of the fft ResetSignal, upstream of its synchroniser (see the
        #    internal-resets block near the end of elaborate()).  This keeps the
        #    single existing reset mux on every fft flop; adding a 2nd (D-path)
        #    reset via ResetInserter on the FFT butterflies cost ~0.16 ns -> a
        #    marginal +0.001 ns build that flaked RANDOMLY run-to-run.
        #  - sync domain: it must keep the RegisterCDC/fcfb_registers/DMA alive, so
        #    its datapath blocks (DDS, InCDC/egress sync sides) take a *targeted*
        #    ResetInserter.  The sync path has ample slack for the extra logic.
        dp_reset = self.control_registers['control']['dp_reset']
        dp_reset_sync = Signal()
        m.submodules.dp_reset_sync_cdc = FFSynchronizer(
            dp_reset, dp_reset_sync, o_domain='sync')

        # Each datapath block is added as its ResetInserter-wrapped form (the
        # wrapper is the submodule that elaborates the block); the bare local var
        # still refers to the original object for its ports/signals -- the same
        # pattern the existing DomainRenamer(stage1) wrap uses.
        m.submodules.adc = adc = self.adc
        # cdc (sampling->sync) is a continuous sample conduit; it is NOT dp_reset
        # (dropping a few in-flight samples would only add a warm-up transient,
        # and its two sides would need a matched reset).  The fold realignment
        # below is what fixes the retune corruption.
        m.submodules.cdc = cdc = self.cdc
        # DDS generators, each dp_reset-wrapped (phase realigns on reprogram).
        for i, d in enumerate(self.ddss):
            name = 'dds' if i == 0 else f'dds{i + 1}'
            setattr(m.submodules, name, ResetInserter({'sync': dp_reset_sync})(d))
        # Wideband: the WOLA + FFT + backend run in the fast fft domain (rename
        # Stage1's internal 'sync' to 'fft'); the FS-rate inputs reach it through
        # the InCDC.  Critical: Stage1 stays in the single sync domain.
        stage1 = self.stage1
        egress = self.egress_fifo
        incdc = None
        if self.wideband:
            # stage1 + the fft sides of the InCDC/egress are reset by the fft
            # ResetSignal (= sdr_reset | dp_reset, ORed at the source below) -- no
            # ResetInserter on the FFT critical path.  Only the sync sides of the
            # FIFOs need a targeted dp_reset (their fft sides reset with the fft
            # domain); both sides thus reset together on a dp_reset pulse.
            m.submodules.stage1 = DomainRenamer({'sync': 'fft'})(stage1)
            incdc = self.incdc
            m.submodules.incdc = ResetInserter({'sync': dp_reset_sync})(incdc)
            m.submodules.egress_fifo = ResetInserter(
                {'sync': dp_reset_sync})(egress)
        else:
            m.submodules.stage1 = ResetInserter({'sync': dp_reset_sync})(stage1)
            m.submodules.egress_fifo = ResetInserter(
                {'sync': dp_reset_sync})(egress)
        m.submodules.dma = dma = self.dma
        # ---- Runtime DMA base offset (Option B): splice dma.axi -> m_axi_dma ----
        # The DMA runs in the sync (125 MHz) domain; its internal AXI awaddr is a
        # 0-based ring offset.  Add the runtime base (dma_base register when
        # mapped, else the compile-time default) and drive the external
        # m_axi_dma port; pass every other AXI3 write-channel signal straight
        # through.  A single 32-bit add in sync -- nowhere near the fft path.
        if self.has_dma_base:
            dma_base = self.fcfb_registers['dma_base']['base']
        else:
            dma_base = Const(self.dma_base_default, self.dma.axi_awidth)
        _intn, _ext = dma.axi, self.m_axi
        # Manager-driven (dma -> external), every write-channel signal but awaddr.
        for _s in ('awid', 'awlen', 'awsize', 'awburst', 'awlock', 'awcache',
                   'awprot', 'awvalid', 'wid', 'wdata', 'wstrb', 'wlast',
                   'wvalid', 'bready'):
            m.d.comb += getattr(_ext, _s).eq(getattr(_intn, _s))
        m.d.comb += _ext.awaddr.eq(dma_base + _intn.awaddr)
        # Slave-driven (external -> dma).
        for _s in ('awready', 'wready', 'bid', 'bresp', 'bvalid'):
            m.d.comb += getattr(_intn, _s).eq(getattr(_ext, _s))
        # Keep-bitmap BRAM: plain top-level submodule (NOT domain-renamed, NOT
        # dp_reset-wrapped) so its storage + sync write/clear side are untouched by
        # dp_reset and its writes need no clock-domain crossing.
        m.submodules.maskmem = maskmem = self.maskmem

        sdr_reset = self.control_registers['control']['sdr_reset']
        run_reg = self.fcfb_registers['run']
        quant_reg = self.fcfb_registers['quant']
        dma_ctrl = self.fcfb_registers['dma_control']
        dds_regs = [self.fcfb_registers['dds' if i == 0 else f'dds{i + 1}']
                    for i in range(self.n_dds)]
        mask_reg = self.fcfb_registers['mask_load']

        # ---- sampling domain: ADC capture -> pack z = adc0 + j*adc1 -> CDC ----
        m.d.comb += [
            adc.adc_dat_a.eq(self.adc_dat_a),
            adc.adc_dat_b.eq(self.adc_dat_b),
            self.adc_csn.eq(adc.csn),

            cdc.strobe_in.eq(adc.strobe),
            cdc.re_in.eq(adc.re_a),        # ADC0 -> real part of z
            cdc.im_in.eq(adc.re_b),        # ADC1 -> imag part of z
            cdc.tag_in.eq(0),              # fcfb needs no side-band tag
            cdc.reset.eq(sdr_reset),
        ]

        # ---- input mux: summed DDS tone(s) or the CDC'd ADC sample (sync) ----
        # Input-scene injection: each DDS phase advances only on genuine consumed-
        # sample ticks (its enable & strobe), so the first WOLA input has phase 0.
        # The enabled generators' complex tones are SUMMED (and saturated to the
        # sample width) to build a one/two/three-tone scene; the mux swaps the ADC
        # for that sum whenever ANY generator is enabled.  In the wideband build the
        # mux sits BEFORE the InCDC, so the scene crosses into the fft domain like a
        # real ADC sample.  On a production (n_dds=0) build the whole injection path
        # is gone: the CDC'd ADC sample feeds the datapath directly.
        re_src = Signal(signed(self.sample_width))
        im_src = Signal(signed(self.sample_width))
        if self.n_dds > 0:
            smax = 2 ** (self.sample_width - 1) - 1
            smin = -(2 ** (self.sample_width - 1))
            # widen enough to hold the sum of n_dds tones before saturating
            sw_sum = self.sample_width + (self.n_dds - 1).bit_length() + 1
            sum_re = Signal(signed(sw_sum))
            sum_im = Signal(signed(sw_sum))
            any_en = Signal()
            re_terms, im_terms, en_terms = [], [], []
            for d, dr in zip(self.ddss, dds_regs):
                m.d.comb += [
                    d.enable.eq(dr['enable']),
                    d.phase_inc.eq(dr['phase_inc']),
                    d.strobe.eq(run_reg['enable'] & cdc.strobe_out),
                ]
                re_terms.append(Mux(dr['enable'], d.re, 0))
                im_terms.append(Mux(dr['enable'], d.im, 0))
                en_terms.append(dr['enable'])

            def _sat(v):
                return Mux(v > smax, smax, Mux(v < smin, smin, v))
            m.d.comb += [
                sum_re.eq(sum(re_terms)),
                sum_im.eq(sum(im_terms)),
                any_en.eq(Cat(*en_terms).any()),
                re_src.eq(Mux(any_en, _sat(sum_re), cdc.re_out)),
                im_src.eq(Mux(any_en, _sat(sum_im), cdc.im_out)),
            ]
        else:
            m.d.comb += [
                re_src.eq(cdc.re_out),
                im_src.eq(cdc.im_out),
            ]

        if not self.wideband:
            # ---- single 125 MHz sync datapath (critical, shipping) ----
            # Stage1 free-runs on clken (gated by the master enable); in_valid is
            # the per-sample strobe.  The WOLA advances its commutator only on
            # (clken & in_valid) and the FFT+backend only on genuine WOLA outputs
            # (run = clken & wola.out_valid), so gaps in strobe_out stall cleanly.
            m.d.comb += [
                stage1.clken.eq(run_reg['enable']),
                stage1.in_valid.eq(cdc.strobe_out),
                stage1.re_in.eq(re_src),
                stage1.im_in.eq(im_src),
                stage1.shift.eq(quant_reg['shift']),
            ]
        else:
            # ---- wideband two-rate datapath ----
            # FS-rate packed samples cross sync->fft through the LUTRAM InCDC.
            # In fft, Stage1 free-runs on clken (=enable_fft) and drains a
            # buffered hop=R WOLA block one point per cycle; in_valid just marks
            # a fresh input.  q=N/R>1 (reader faster than writer) keeps the InCDC
            # near-empty; a pop happens exactly when the WOLA accepts a sample.
            m.d.comb += [
                incdc.w_data.eq(Cat(re_src, im_src)),
                incdc.w_en.eq(run_reg['enable'] & cdc.strobe_out),
            ]
            # Control registers crossed sync->fft (quasi-static: set while the
            # datapath is disabled, so a per-field FFSynchronizer is sufficient).
            enable_fft = Signal()
            shift_fft = Signal(self.shift_width)
            m.submodules.enable_cdc = FFSynchronizer(
                run_reg['enable'], enable_fft, o_domain='fft')
            m.submodules.shift_cdc = FFSynchronizer(
                quant_reg['shift'], shift_fft, o_domain='fft')
            # The keep-bitmap needs NO control CDC: the MaskMem is a dual-clock
            # BRAM written directly in the sync domain and read in the fft domain
            # (wired below, common to both builds).
            sw = self.sample_width
            m.d.comb += [
                stage1.clken.eq(enable_fft),
                stage1.in_valid.eq(incdc.r_rdy),
                stage1.re_in.eq(incdc.r_data[:sw].as_signed()),
                stage1.im_in.eq(incdc.r_data[sw:].as_signed()),
                incdc.r_en.eq(enable_fft),
                stage1.shift.eq(shift_fft),
            ]

        # ---- keep-bitmap: sync-domain write/clear + datapath-domain read ----
        # Write/clear straight from the sync register (no CDC); read addressed by
        # the datapath bin counter, enabled with the datapath clken (fft or sync).
        # mask_busy reads back in the sync register directly.
        m.d.comb += [
            maskmem.wr_addr.eq(mask_reg['addr']),
            maskmem.wr_data.eq(Cat(mask_reg['keep_a'], mask_reg['keep_b'])),
            maskmem.wr_en.eq(mask_reg['load']),
            maskmem.clear.eq(mask_reg['clear']),
            run_reg['mask_busy'].eq(maskmem.busy),
            maskmem.rd_addr.eq(stage1.mask_rd_addr),
            maskmem.rd_en.eq(stage1.clken),
            stage1.keep_a_in.eq(maskmem.keep_a),
            stage1.keep_b_in.eq(maskmem.keep_b),
        ]

        # ---- egress: record FIFO (rate-match / OutCDC fft->sync) -> DMA ----
        if self.wideband:
            # Register the StreamFormat->FIFO write launch in the fast fft domain
            # so the placer can put the launch FF next to the BRAM write port,
            # breaking the route-heavy `sf -> FIFO write` path.  out_valid is
            # already gated, so no clken is needed; this is a pure one-cycle
            # pipeline stage on a contiguous byte stream (the DMA writes it
            # verbatim), so record bytes are unchanged.
            ew_data = Signal(32)
            ew_valid = Signal()
            m.d.fft += [
                ew_data.eq(stage1.out_data),
                ew_valid.eq(stage1.out_valid),
            ]
            m.d.comb += [
                egress.w_data.eq(ew_data),
                egress.w_en.eq(ew_valid),
            ]
        else:
            m.d.comb += [
                egress.w_data.eq(stage1.out_data),
                egress.w_en.eq(stage1.out_valid),
            ]
        m.d.comb += [
            dma.stream_data.eq(egress.r_data),
            dma.stream_valid.eq(egress.r_rdy),
            egress.r_en.eq(dma.stream_ready),

            dma.start.eq(dma_ctrl['dma_start']),
            dma.stop.eq(dma_ctrl['dma_stop']),
            # Keep the read-back ABSOLUTE: internal next_address is 0-based over
            # the ring, so add the same runtime base the awaddr splice applies.
            self.fcfb_registers['dma_next_address']['next_address'].eq(
                dma_base + dma.next_address),
            dma_interrupt.i.eq(dma.finished),
        ]

        # ---- exact ring-lap counter (dma_laps register, n_dds<2 builds) --------
        # The circular DMA's next_address is a 0-based ring offset that advances
        # monotonically and wraps to 0 at the ring end.  We see it on EVERY sync
        # edge, so a wrap is exactly `next_address decreased since last cycle`.
        # Count those wraps in the PL: this is alias-proof, unlike the board
        # server's old approach of polling the modulo pointer over AXI-Lite and
        # inferring wraps in software (a glitchy/aliased poll near the ring end
        # could miss or fake a wrap -> produced misregistered by a full lap ->
        # the UDP egress de-framed).  Reset on dma_start so every session starts
        # at lap 0; the server delta-tracks it, so the absolute value is moot.
        if self.has_dma_base:
            dma_laps = Signal(32)
            dma_next_prev = Signal(len(dma.next_address))
            m.d.sync += dma_next_prev.eq(dma.next_address)
            with m.If(dma_ctrl['dma_start']):
                m.d.sync += [dma_laps.eq(0), dma_next_prev.eq(0)]
            with m.Elif(dma.next_address < dma_next_prev):
                m.d.sync += dma_laps.eq(dma_laps + 1)
            m.d.comb += self.fcfb_registers['dma_laps']['laps'].eq(dma_laps)

        # ---- register access: s_axi_lite domain decode ----
        # control bank at words 0..3 (address[3]==0), fcfb bank at word 8
        # (address[3]==1, byte offset 0x20), crossed to the sync domain.
        address = Signal(self.axi4_awidth, reset_less=True)
        wdata = Signal(32, reset_less=True)
        # address[4] selects the read-only params region (byte 0x40..); when it is
        # 0, address[3] splits control (0x00) vs fcfb (0x20) exactly as before.
        params_select = self.axi4lite.address[4] == 1
        fcfb_select = (self.axi4lite.address[4] == 0) & (self.axi4lite.address[3] == 1)
        control_select = (self.axi4lite.address[4] == 0) & (self.axi4lite.address[3] == 0)
        m.d.s_axi_lite += [
            self.axi4lite.rdata.eq(self.control_registers.rdata
                                   | fcfb_cdc.i_rdata
                                   | self.param_registers.rdata),
            self.axi4lite.rdone.eq(self.control_registers.rdone
                                   | fcfb_cdc.i_rdone
                                   | self.param_registers.rdone),
            self.axi4lite.wdone.eq(self.control_registers.wdone
                                   | fcfb_cdc.i_wdone
                                   | self.param_registers.wdone),
            self.control_registers.ren.eq(
                self.axi4lite.ren & control_select),
            self.control_registers.wstrobe.eq(
                Mux(control_select, self.axi4lite.wstrobe, 0)),
            self.param_registers.ren.eq(self.axi4lite.ren & params_select),
            self.param_registers.wstrobe.eq(
                Mux(params_select, self.axi4lite.wstrobe, 0)),
            fcfb_cdc.i_ren.eq(self.axi4lite.ren & fcfb_select),
            fcfb_cdc.i_wstrobe.eq(
                Mux(fcfb_select, self.axi4lite.wstrobe, 0)),
            address.eq(self.axi4lite.address),
            wdata.eq(self.axi4lite.wdata),
        ]
        m.d.comb += [
            self.control_registers.address.eq(address),
            self.control_registers.wdata.eq(wdata),
            self.param_registers.address.eq(address),
            self.param_registers.wdata.eq(wdata),
            fcfb_cdc.i_address.eq(address),
            fcfb_cdc.i_wdata.eq(wdata),
        ]

        # ---- register access: sync domain ----
        m.d.comb += [
            self.fcfb_registers.ren.eq(fcfb_cdc.o_ren),
            self.fcfb_registers.wstrobe.eq(fcfb_cdc.o_wstrobe),
            self.fcfb_registers.address.eq(fcfb_cdc.o_address),
            self.fcfb_registers.wdata.eq(fcfb_cdc.o_wdata),
            fcfb_cdc.o_rdone.eq(self.fcfb_registers.rdone),
            fcfb_cdc.o_wdone.eq(self.fcfb_registers.wdone),
            fcfb_cdc.o_rdata.eq(self.fcfb_registers.rdata),
        ]

        # ---- internal resets (FFSynchronizer, per upstream note re. #721) ----
        # fft (wideband) is sequenced by sdr_reset just like sync/sampling; the
        # MMCM lock gates the whole PL upstream (block_design proc_sys_reset).
        # The fft domain ALSO folds in the soft datapath reset (sdr_reset |
        # dp_reset) at the source here -- the fft domain holds only datapath state
        # (WOLA/FFT/backend + the fft sides of the InCDC/egress FIFOs), so a full-
        # domain reset on dp_reset realigns the free-running fold with ZERO added
        # per-flop logic (the whole point -- ResetInserter on the FFT flops cost
        # the critical path ~0.16 ns).  sync/sampling stay sdr_reset-only so the
        # RegisterCDC/fcfb_registers/DMA survive a dp_reset pulse.
        reset_src = {'sync': sdr_reset, 'sampling': sdr_reset}
        internal_domains = ['sync', 'sampling']
        if self.wideband:
            internal_domains.append('fft')
            reset_src['fft'] = sdr_reset | dp_reset
        for internal in internal_domains:
            setattr(m.submodules, f'{internal}_rst', FFSynchronizer(
                reset_src[internal], ResetSignal(internal), o_domain=internal,
                init=1))

        # ---- interrupt (s_axi_lite domain) ----
        interrupts_reg = self.control_registers['interrupts']
        m.d.comb += [
            self.interrupt_out.eq(interrupts_reg.interrupt),
            interrupts_reg['dma'].eq(dma_interrupt.o),
        ]

        return m


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file', help='Output verilog file')
    parser.add_argument('--svd', help='Also write the SVD to this path')
    parser.add_argument('--hop', type=int, default=None,
                        help='WOLA block hop R (wideband two-rate build); omit '
                             'for the critical single-125 MHz shipping build')
    parser.add_argument('--no-dds', action='store_true',
                        help='PRODUCTION build: drop the DDS tone-injection '
                             'self-test path (ADC input only), reclaiming its '
                             'fabric (== --n-dds 0); omit for a verification build')
    parser.add_argument('--n-dds', type=int, default=1, choices=(0, 1, 2, 3),
                        help='number of DDS tone generators for the self-test '
                             'input scene: 1 (default single tone), 2 for a '
                             'two-tone / two-window scene, 0 = production (no DDS)')
    parser.add_argument('--bin-width', type=int, default=16, choices=(16, 24),
                        help='On-wire I/Q component width: 16 (shipping) or 24 '
                             '(Option-B int24 -- streams the full 23-bit internal '
                             'bin for the ~-102 dBc reconstructed-channel ceiling)')
    parser.add_argument('--wmax', type=int, default=512,
                        help='Max kept bins per ADC per block (StreamFormat buffer '
                             'depth). Smaller wmax shortens the int24 bit-packer '
                             'ld_idx loop (aw=ceil(log2(wmax))) -- use 128 for a '
                             'narrow verification build to ease fft-clock timing; '
                             'keep the server FCFB_WMAX guard consistent.')
    parser.add_argument('--build-id', default='0',
                        help='provenance stamp baked into the read-only params '
                             'bank (build_id_lo/hi); a hex git short hash (e.g. '
                             '0x9615567c) or decimal, 0 if unset')
    return parser.parse_args()


def main():
    from .fcfb_platform import FcfbPlatform
    args = parse_args()
    n_dds = 0 if args.no_dds else args.n_dds
    build_id = int(args.build_id, 0) & ((1 << 64) - 1)
    top = Stage1Top(hop=args.hop, n_dds=n_dds, bin_width=args.bin_width,
                    wmax=args.wmax, build_id=build_id)
    if args.svd:
        with open(args.svd, 'wb') as f:
            f.write(top.svd())
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, platform=FcfbPlatform(), ports=top.ports()))


if __name__ == '__main__':
    main()
