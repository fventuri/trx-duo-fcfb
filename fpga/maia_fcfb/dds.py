# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""Numerically-controlled oscillator (DDS) for HW input-scene injection.

Step-6e/Phase-4 verification needs a *deterministic, exactly-known* input so the
board's Stage-1 output can be compared against the golden model (the ADC front
end has no sample-injection path, and a file-playback scene would need BRAM the
design does not have).  This DDS synthesises a
complex tone ``z[n] = A*(cos(theta_n) + j*sin(theta_n))`` in the PL, selectable
over the ADC via a register.

Why a LINEAR-INTERPOLATED LUT
-----------------------------
The original 64-entry non-interpolated ROM was fine for ON-BIN tones (the whole
point of step 6e: prove byte-exactness on an exact-bin tone), but its worst-case
phase-truncation spur floor is only ~ -6.02*LUT_BITS = -36 dBc.  For the wideband
SFDR PAYOFF measurement we must sweep OFF-grid tones to expose the bank's
inter-bin images at ~ -79 dBc; a -36 dBc source would mask them entirely (the
on-bin case escapes because a bin-k tone's LUT spurs land on *other* exact FFT
bins, outside the narrow kept run, and are filtered out -- an off-grid tone's
spurs smear in-band).  Linear interpolation between a 2**LUT_BITS base table with
FRAC_BITS of sub-index fraction lifts the phase-spur floor to roughly
-(12*LUT_BITS) dBc (~ -120 dBc at LUT_BITS=10), far below both the -79 bank floor
and the int16 amplitude-quant ceiling (~ -116 dBc at amplitude 8192).

Exact-bin property (unchanged)
------------------------------
With ``PHASE_BITS = 24`` and ``phase_inc = k << (PHASE_BITS - order_log2)`` the
phase advances by exactly ``k`` full turns over one ``N = 2**order_log2`` sample
block, so the tone sits *exactly* on FFT bin ``k``.  For N=4096 that is
``phase_inc = k << 12``.  An arbitrary 24-bit ``phase_inc`` is a legal OFF-grid
tone at ``f = phase_inc * FS / 2**24`` (resolution FS/2**24 ~ 7.45 Hz).

Bit-exact model twin
--------------------
``nco_samples()`` is the single source of truth for the emitted sample stream;
the host/model side imports it so the injected samples match the hardware to the
LSB (a constant *pipeline* phase offset -- the RTL registers the ROM read +
interpolation multiply -- is a pure per-sample constant on a single tone, so it
is absorbed by the best-fit-gain compare and does not appear in a magnitude
spectrum / SFDR).  Phase resets to 0 with ``sdr_reset`` and advances on
``strobe``.
"""

import math

import numpy as np
from amaranth import *
from amaranth.lib.memory import Memory

PHASE_BITS = 24
LUT_BITS = 8            # base cos table: 2**8 = 256 entries (small LUTRAM)
FRAC_BITS = 12          # sub-index interpolation fraction bits
# 24 - 8 - 12 = 4 unused low phase bits (sub-LSB of the fraction).
# Linear interp off-grid SFDR ~ -88 dBc (comfortably below the -79 bank floor);
# kept small so the distributed-LUTRAM read closes timing with wide margin.


def cos_lut(amplitude):
    """Base cosine table of 2**LUT_BITS integer samples, scaled to `amplitude`.

    Single source of truth shared by the RTL ROM init and the model twin. Uses
    Python round-half-to-even; both sides call this exact function."""
    n = 1 << LUT_BITS
    return [int(round(amplitude * math.cos(2 * math.pi * i / n))) for i in range(n)]


def sine_tables(amplitude):
    """Back-compat: (cos_table, sin_table) of 2**LUT_BITS entries.

    sin_t[i] == cos_t[(i - n/4) mod n]. Retained for callers/tests that inspect
    the raw ROM; the emitted samples now come from `nco_samples` (interpolated)."""
    cos_t = cos_lut(amplitude)
    n = 1 << LUT_BITS
    sin_t = [cos_t[(i - (n >> 2)) % n] for i in range(n)]
    return cos_t, sin_t


def _interp(phase, lut, amplitude):
    """Linear-interpolated LUT read for a 24-bit phase (scalar or array).

    idx = top LUT_BITS; frac = next FRAC_BITS; out = lut[idx] +
    round(frac*(lut[idx+1]-lut[idx]) / 2**FRAC_BITS), round-half-up matching the
    RTL ``(delta*frac + 2**(FRAC_BITS-1)) >> FRAC_BITS`` (arithmetic shift)."""
    n = 1 << LUT_BITS
    phase = np.asarray(phase, dtype=np.int64) & ((1 << PHASE_BITS) - 1)
    idx = (phase >> (PHASE_BITS - LUT_BITS)) & (n - 1)
    frac = (phase >> (PHASE_BITS - LUT_BITS - FRAC_BITS)) & ((1 << FRAC_BITS) - 1)
    lut = np.asarray(lut, dtype=np.int64)
    y0 = lut[idx]
    y1 = lut[(idx + 1) & (n - 1)]
    delta = y1 - y0
    rnd = (delta * frac + (1 << (FRAC_BITS - 1))) >> FRAC_BITS   # floor => match RTL
    return y0 + rnd


def nco_samples(nsamp, phase_inc, amplitude, phase0=0):
    """Integer complex tone z[n] = A*cos + j*A*sin, phase = phase0 + n*phase_inc.

    THE bit-exact twin of the RTL output stream (up to the RTL's constant
    pipeline phase, which a single tone carries as a per-sample constant).
    ``phase_inc = k << (PHASE_BITS - log2 N)`` is the exact-bin word; any 24-bit
    value is a legal off-grid tone."""
    lut = np.asarray(cos_lut(amplitude), dtype=np.int64)
    n = 1 << LUT_BITS
    quarter = n >> 2                                              # -pi/2 in idx
    ph = (phase0 + np.arange(nsamp, dtype=np.int64) * int(phase_inc))
    ph &= (1 << PHASE_BITS) - 1
    re = _interp(ph, lut, amplitude)
    # sin(theta) = cos(theta - pi/2): shift phase by a quarter turn (an integer
    # number of LUT steps, so the SAME frac interpolates the imaginary component).
    ph_sin = (ph - (quarter << (PHASE_BITS - LUT_BITS))) & ((1 << PHASE_BITS) - 1)
    im = _interp(ph_sin, lut, amplitude)
    return re + 1j * im


class Dds(Elaboratable):
    """Complex-tone NCO with a linear-interpolated cosine LUT.

    Ports (unchanged from the original 64-entry version)
    ----------------------------------------------------
    enable : in    -- mux/accumulate enable (the register bit)
    strobe : in    -- advance the phase this cycle (datapath consumed-sample tick)
    phase_inc : in, PHASE_BITS -- frequency control word (k << 12 for exact bin k)
    re, im : out, signed(out_width) -- A*cos, A*sin at the current phase

    The ROM read + interpolation is PIPELINED (registered BRAM read, registered
    DSP multiply). On a single tone this is a constant phase, absorbed by the
    best-fit-gain compare; the few-sample startup is inside the WOLA warm-up.
    """
    def __init__(self, out_width=16, amplitude=8192, domain='sync'):
        self.out_width = out_width
        self.amplitude = amplitude
        self._domain = domain
        self.enable = Signal()
        self.strobe = Signal()
        self.phase_inc = Signal(PHASE_BITS)
        self.re = Signal(signed(out_width))
        self.im = Signal(signed(out_width))

    def _read_interp(self, mem_name, table, phase, en_s):
        """One interpolated component: COMB (LUTRAM) reads of lut[idx],lut[idx+1]
        + a DSP interp, pipelined PER SAMPLE (registers advance on ``en_s`` =
        enable&strobe, NOT every clock).

        Per-sample (not per-clock) pipelining is essential: the consumed-sample
        strobe is not every clock, so a per-clock pipeline would sample a phase a
        non-integer number of SAMPLES behind and distort the tone (this bit the
        first HW build). Advancing on en_s makes the output exactly z[m-latency]
        for ANY strobe cadence -- a per-sample constant phase, absorbed by the
        best-fit-gain compare.

        Stage 1 (en_s): capture the comb LUT reads + frac.
        Stage 2 (en_s): DSP  prod = (y1-y0)*frac + round; carry y0.
        Comb:           out = y0 + (prod >> FRAC_BITS).
        The comb LUTRAM read (no BRAM clk-to-out) sits in its own stage before the
        DSP, so no path spans read + subtract + multiply."""
        d = self._domain
        n = 1 << LUT_BITS
        # ram_style=distributed FORCES LUTRAM (async read): without it Vivado
        # infers a RAMB18 and absorbs the stage-1 register into the BRAM output,
        # putting the slow BRAM clk-to-out (~2.5 ns) straight into the DSP -- the
        # path that failed timing (bin-dependent HW corruption). LUTRAM keeps the
        # read async and fast, with the fabric register between it and the DSP.
        mem = Memory(shape=signed(self.out_width), depth=n, init=table,
                     attrs={"ram_style": "distributed"})
        setattr(self.m.submodules, mem_name, mem)
        rd0 = mem.read_port(domain="comb")            # lut[idx]   (LUTRAM, comb)
        rd1 = mem.read_port(domain="comb")            # lut[idx+1] (LUTRAM, comb)
        idx = phase[PHASE_BITS - LUT_BITS:]
        frac = phase[PHASE_BITS - LUT_BITS - FRAC_BITS: PHASE_BITS - LUT_BITS]
        self.m.d.comb += [rd0.addr.eq(idx), rd1.addr.eq(idx + 1)]

        # stage 1: capture comb reads + frac (per sample)
        y0_1 = Signal(signed(self.out_width))
        y1_1 = Signal(signed(self.out_width))
        frac_1 = Signal(FRAC_BITS)
        # stage 2: DSP pre-adder + multiply (registered inputs y0_1/y1_1/frac_1)
        prod = Signal(signed(self.out_width + 1 + FRAC_BITS))
        y0_2 = Signal(signed(self.out_width))
        with self.m.If(en_s):
            self.m.d[d] += [
                y0_1.eq(rd0.data), y1_1.eq(rd1.data), frac_1.eq(frac),
                prod.eq((y1_1 - y0_1) * frac_1 + (1 << (FRAC_BITS - 1))),
                y0_2.eq(y0_1),
            ]
        out = Signal(signed(self.out_width))
        self.m.d.comb += out.eq(y0_2 + (prod >> FRAC_BITS))     # round-half-up
        return out

    def elaborate(self, platform):
        m = Module()
        self.m = m
        en_s = Signal()
        m.d.comb += en_s.eq(self.enable & self.strobe)
        phase = Signal(PHASE_BITS)
        with m.If(en_s):
            m.d[self._domain] += phase.eq(phase + self.phase_inc)

        table = cos_lut(self.amplitude)
        n = 1 << LUT_BITS
        # sin(theta) = cos(theta - pi/2): a quarter-turn phase offset (an integer
        # number of LUT steps -> identical frac). Two LUTRAM copies give the 4
        # simultaneous comb reads (idx, idx+1 for cos and sin); both hold `table`.
        quarter_phase = (n >> 2) << (PHASE_BITS - LUT_BITS)
        phase_sin = Signal(PHASE_BITS)
        m.d.comb += phase_sin.eq(phase - quarter_phase)
        m.d.comb += [
            self.re.eq(self._read_interp("cos_rom", table, phase, en_s)),
            self.im.eq(self._read_interp("sin_rom", table, phase_sin, en_s)),
        ]
        return m
