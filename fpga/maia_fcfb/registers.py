# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Reuses the maia-hdl register infrastructure (Access / Field / Register /
# Registers / RegisterMap, © Daniel Estevez, MIT).  New for fcfb.
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""AXI-Lite register map for the fcfb Stage-1 IP core.

Two banks, mirroring the trx-duo layout so the PS driver stays close:

* ``control`` at byte offset 0x00 (``s_axi_lite`` domain): product id, version,
  ``sdr_reset``, and a DMA-finished interrupt.
* ``fcfb`` at byte offset 0x20 (``sync`` = 125 MHz domain, crossed via
  ``RegisterCDC``): the client-selected run + quantise controls the PS
  board-server sets from the client's bin selection, plus streaming-DMA
  start/stop/next-address.

Relative to trx-duo this drops the whole spectrometer/DDC/integrator register
set (fcfb has no integrator and no DDC) and adds the fcfb Stage-1 controls
(``enable``, ``shift``, the per-ADC keep-bitmap ``mask_load``) and the
streaming-DMA handshake registers.
"""

from maia_hdl.register import Access, Field, Registers, Register, RegisterMap

# IP core version (major.minor.bugfix).
VERSION = '0.1.0'

# Product id: 'fcfb' little-endian (bytes f,c,f,b in memory).  Distinct from the
# maia 'maia' id so the PS can tell the two cores apart.
PRODUCT_ID = 0x62666366

# Platform byte in the version register's top octet (distinguishes a fcfb build
# from the maia trx-duo build, which uses 2).
PLATFORM = 3

# Params block magic 'FCFP' (bytes F,C,F,P little-endian -> read back as this
# u32).  Distinguishes a bitstream that carries the read-only params bank from an
# older one (whose 0x40 region aliases 0x00 -> reads PRODUCT_ID, not this), so the
# host can detect params support and fall back to its interim source otherwise.
PARAM_BLOCK_MAGIC = 0x50464346
# Params-block format version (bump when the field layout changes).
PARAM_BLOCK_VER = 1


def control_registers():
    """Control bank (s_axi_lite domain): product id, version, reset, interrupt."""
    major, minor, bugfix = (int(x) for x in VERSION.split('.'))
    return Registers(
        'control',
        {
            0b00: Register('product_id', [
                Field('product_id', Access.R, 32, PRODUCT_ID),
            ]),
            0b01: Register('version', [
                Field('bugfix', Access.R, 8, bugfix),
                Field('minor', Access.R, 8, minor),
                Field('major', Access.R, 8, major),
                Field('platform', Access.R, 8, PLATFORM),
            ]),
            0b10: Register('control', [
                Field('sdr_reset', Access.RW, 1, 1),
                # dp_reset: soft DATAPATH reset (bit 1).  Pulsed by the server on
                # every (re)program to realign the free-running two-rate fold +
                # InCDC + DDS phase + egress FIFO to the fresh-boot condition,
                # WITHOUT the AXI-wedge risk of sdr_reset (it deliberately does
                # NOT reset the RegisterCDC / fcfb_registers / DMA, so programmed
                # values survive and no mid-flight AXI/HP0 access is disturbed).
                # Fixes the wideband live-retune corruption (only the 1st capture
                # per boot was valid otherwise).  Init 0 (deasserted).
                Field('dp_reset', Access.RW, 1, 0),
            ]),
            0b11: Register('interrupts', [
                # Pulsed by the streaming DMA's ``finished`` (crossed to
                # s_axi_lite via a PulseSynchronizer at the top).
                Field('dma', Access.Rsticky, 1, 0),
            ], interrupt=True),
        },
        2)


def fcfb_registers(order_log2=12, shift_width=5, axi_awidth=32, dds_phase_bits=24,
                   n_dds=1, dma_base_init=0x1000_0000):
    """fcfb Stage-1 control bank (sync = 125 MHz domain).

    Fields
    ------
    run: enable (1, RW), mask_busy (1, R)
        The master datapath enable and a read-back of the keep-bitmap clear-sweep
        busy flag (poll it after pulsing ``mask_load.clear`` before loading bins).
    quant: shift (shift_width)
        Quantise right-shift shared by the A/B quantisers.
    dma_control: dma_start (Wpulse), dma_stop (Wpulse)
        One-cycle pulses (in the sync domain) to start/stop the streaming DMA.
    dma_next_address: next_address (axi_awidth, R)
        The DMA's next write address; after a run, ``next_address - base`` is the
        number of record bytes written (the PS uses it to size the block count).
        ``base`` here is the runtime ``dma_base`` the server programmed (below),
        which ``stage1_top`` also adds to this read-back so it stays absolute.
    dma_base: base (axi_awidth, RW)  [mapped only when n_dds < 2]
        Runtime AXI write-base for the egress DMA (Option B / udmabuf zero-copy):
        the PL DMA's internal address counter is 0-based over the ring size, and
        ``stage1_top`` adds this register to the AXI ``awaddr`` (and to the
        ``dma_next_address`` read-back).  The board server sets it to the physical
        address of the page-backed u-dma-buf ring it allocated, so the PL DMA
        writes into pinnable pages the NIC can zero-copy from -- instead of the
        old fixed ``/dev/mem`` reserved region baked into the bitstream.  Must be
        burst-aligned (64 B for the 32-bit/16-beat burst; CMA gives page
        alignment).  Bank slot 0b110 collides with ``dds2`` (two-tone injection),
        so it is present only for the production (n_dds=0) and single-tone
        HW-verify (n_dds=1) builds -- the only ones that use the zero-copy path.
    dds[/dds2/dds3]: phase_inc (dds_phase_bits), enable (1)
        Input-scene injection: replaces the ADC samples with a sum of PL complex
        tones (one per enabled DDS generator; ``n_dds`` of them).  ``phase_inc =
        k << (dds_phase_bits - order_log2)`` places a tone exactly on FFT bin
        ``k``; ``enable`` adds that generator's tone.  Two enabled generators give
        a two-tone / two-window scene (e.g. lighting two disjoint mask windows at
        once, or a two-tone IMD test).  ``dds`` at 0b100; ``dds2`` at 0b110,
        ``dds3`` at 0b111 when built with n_dds>=2/3.  All off in normal operation.
    mask_load: addr (order_log2), keep_a (1), keep_b (1), load (Wpulse),
               clear (Wpulse)
        Per-ADC keep-bitmap loader (replaces the old contiguous k0/w_run/adc_mask).
        Writing with ``load=1`` stores ``{keep_b, keep_a}`` at bin ``addr`` (one
        AXI write per kept bin).  ``clear=1`` starts a sweep that zeroes the whole
        4096-bin memory (watch ``run.mask_busy``).  Load/clear only while
        ``enable=0`` (quasi-static); crossed into the fft domain in the wideband
        build (see ``stage1_top``).
    """
    def dds_reg():
        return Register('dds', [
            Field('phase_inc', Access.RW, dds_phase_bits, 0),
            Field('enable', Access.RW, 1, 0),
        ])

    registers = {
        0b000: Register('run', [
            Field('enable', Access.RW, 1, 0),
            Field('mask_busy', Access.R, 1, 0),
        ]),
        0b001: Register('quant', [
            Field('shift', Access.RW, shift_width, 0),
        ]),
        0b010: Register('dma_control', [
            Field('dma_start', Access.Wpulse, 1, 0),
            Field('dma_stop', Access.Wpulse, 1, 0),
        ]),
        0b011: Register('dma_next_address', [
            Field('next_address', Access.R, axi_awidth, 0),
        ]),
        0b100: dds_reg(),                       # 'dds' (always mapped)
        0b101: Register('mask_load', [
            Field('addr', Access.RW, order_log2, 0),
            Field('keep_a', Access.RW, 1, 0),
            Field('keep_b', Access.RW, 1, 0),
            Field('load', Access.Wpulse, 1, 0),
            Field('clear', Access.Wpulse, 1, 0),
        ]),
    }
    # Runtime DMA write-base (Option B / udmabuf zero-copy) @ 0b110.  Present for
    # the builds that use the zero-copy egress path (production n_dds=0 and the
    # single-tone HW-verify n_dds=1); it shares slot 0b110 with dds2, so it is
    # dropped for the rare two-tone (n_dds>=2) bench builds, which keep the
    # compile-time base baked into the bitstream.
    if n_dds < 2:
        registers[0b110] = Register('dma_base', [
            Field('base', Access.RW, axi_awidth, dma_base_init),
        ])
        # Ring-lap counter @ 0b111 (shares the slot with dds3, dropped for the
        # two-tone bench builds).  The PL counts every wrap of the circular DMA
        # write pointer -- it sees dma_next_address on EVERY clock edge, so its
        # lap count is exact and alias-proof, unlike the board server's old
        # software wrap heuristic (which polled the modulo pointer and could miss
        # or fake a wrap near the ring end -> a one-lap misregistration that
        # de-framed the UDP egress).  The server pairs this with next_address to
        # form an exact monotonic committed-byte position.  Reset on dma_start.
        registers[0b111] = Register('dma_laps', [
            Field('laps', Access.R, 32, 0),
        ])
    # Extra DDS generators (two-tone / multi-window injection): dds2 @ 0b110,
    # dds3 @ 0b111.  Mapped when the build carries them (n_dds >= 2 / 3); the
    # 3-bit bank has no slots beyond these.
    if n_dds >= 2:
        r = dds_reg(); r.name = 'dds2'; registers[0b110] = r
    if n_dds >= 3:
        r = dds_reg(); r.name = 'dds3'; registers[0b111] = r
    return Registers('fcfb', registers, 3)


def param_registers(*, r_hop, t_frames, n_fft, wmax, bin_width, n_dds,
                    features=0, build_id=0):
    """Read-only parameter block (s_axi_lite domain) at byte offset 0x40.

    The board's FIXED analysis parameters, exposed so a host client can read them
    (the FPRM params query) and verify its synthesis kernel g was fit for the SAME
    (R, T) before streaming -- a mismatched kernel reconstructs silent garbage.
    Every field is a build-time constant, so this is a constant read mux: no
    datapath, ~0 logic, timing-trivial.  One 32-bit register per value keeps the
    host read trivial (mirror the offsets in host/fcfb_server.c).

    ``r_hop`` is the analysis hop R the host kernel must match (the wideband
    oversampled hop, or N for a critically-sampled build); ``build_id`` is a
    64-bit provenance stamp (e.g. the git short hash) split lo/hi, 0 if unset.
    """
    R = Access.R
    return Registers('params', {
        0b0000: Register('param_magic',
                         [Field('magic', R, 32, PARAM_BLOCK_MAGIC)]),
        0b0001: Register('param_version', [
            Field('param_ver', R, 16, PARAM_BLOCK_VER),
            Field('reserved', R, 16, 0),
        ]),
        0b0010: Register('param_r', [Field('r_hop', R, 32, r_hop)]),
        0b0011: Register('param_t', [Field('t_frames', R, 32, t_frames)]),
        0b0100: Register('param_n', [Field('n_fft', R, 32, n_fft)]),
        0b0101: Register('param_wmax', [Field('wmax', R, 32, wmax)]),
        0b0110: Register('param_bin_width',
                         [Field('bin_width', R, 32, bin_width)]),
        0b0111: Register('param_n_dds', [Field('n_dds', R, 32, n_dds)]),
        0b1000: Register('param_features', [Field('features', R, 32, features)]),
        0b1001: Register('param_build_id_lo',
                         [Field('build_id_lo', R, 32, build_id & 0xffffffff)]),
        0b1010: Register('param_build_id_hi',
                         [Field('build_id_hi', R, 32, (build_id >> 32) & 0xffffffff)]),
    }, 4)


def register_map(control, fcfb, params=None):
    """SVD-generating register map (control @ 0x00, fcfb @ 0x20, params @ 0x40)."""
    metadata = {
        'vendor': 'fcfb project',
        'vendorID': 'fcfb',
        'name': 'fcfb Stage-1',
        'series': 'fcfb',
        'version': VERSION,
        'description': 'fcfb distributed fast-convolution filter bank, Stage-1',
        'licenseText': 'SPDX-License-Identifier: MIT',
    }
    banks = {0x0: control, 0x20: fcfb}
    if params is not None:
        banks[0x40] = params
    return RegisterMap(banks, metadata)
