# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Reuses the maia-hdl FFT (© Daniel Estevez, MIT) and this project's ABSplit
# (itself derived from the maia-sdr-trx-duo RealRecovery, MIT).
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``stage1_core`` — the fcfb dual-ADC analysis seam: windowed FFT + A/B split.

This is the fcfb analog of the trx-duo ``RealSpectrometerCore``, but for the
PLAN §3 packing: the two real ADCs enter one complex FFT as ``z = adc0 + j*adc1``
(``re_in = adc0``, ``im_in = adc1`` — that IS the "pack"), and the bit-reversed
FFT output is split into the two SEPARATE ADC spectra A (ADC0) and B (ADC1) by
:class:`ABSplit` — no twiddle recombine (that is what makes fcfb differ from the
even/odd-packed real spectrometer).

The FFT is configured like the real design (R2², per-stage truncate ``[0,1]``,
16-bit twiddles) so the numerics match ``fpga/models/fixed_point_sim.py``. The
T=4 WOLA prefilter and the bin-select / quantise / stream-format back end wrap
around this core at the next level; this module isolates the FFT↔split seam so it
can be simulated on its own (as ``RealSpectrometerCore`` did for recovery).

For M3 first-light this can run with ``window=None`` (rectangular, T=1) and
``cmult3x=False`` (single clock domain). The shipping config uses a window and
``cmult3x=True`` (2×/3× domains) for resource sharing; the numerics are identical.
"""

from amaranth import *
import numpy as np

from .ab_split import ABSplit


class Stage1Core(Elaboratable):
    """Packed dual-ADC FFT + A/B split.

    Parameters
    ----------
    width_in : int
        FFT input sample width. fcfb Stage-1 = 17 (see fixed_point_sim.py: two
        16-bit ADCs pack to |z|<=sqrt2*32767, which needs a 17-bit input).
    order_log2 : int
        log2 of the complex FFT size N.
    twiddle_width : int
        Twiddle width for the FFT.
    window : str or None
        FFT window (scipy.signal.windows name), or None for rectangular (T=1).
    cmult3x : bool
        Time-share one DSP at 3× for the twiddle multiply (needs domain_3x).
    domain_2x, domain_3x : str or None
        2×/3× clock domains (required when window / cmult3x are used).

    Attributes
    ----------
    clken : Signal(), in
    common_edge_2x, common_edge_3x : Signal(), in    (only when 2×/3× used)
    re_in : Signal(signed(width_in)), in    ADC0 sample (packed real part).
    im_in : Signal(signed(width_in)), in    ADC1 sample (packed imag part).
    a_re, a_im : Signal(signed(width_out)), out    ADC0 spectrum A[k], natural order.
    b_re, b_im : Signal(signed(width_out)), out    ADC1 spectrum B[k], natural order.
    out_valid, out_last : Signal(), out
    """
    def __init__(self, width_in=17, order_log2=12, twiddle_width=16,
                 window=None, cmult3x=False, domain_2x=None, domain_3x=None,
                 ab_extra_pipe=False, ab_share_write_port=False):
        from maia_hdl.fft import FFT
        self.width_in = width_in
        self.order_log2 = order_log2
        self.tw = twiddle_width

        truncates = [[0, 1]] * (order_log2 // 2)
        self.fft = FFT(width_in, order_log2, 'R22',
                       width_twiddle=twiddle_width, truncates=truncates,
                       use_bram_reg=True, window=window, cmult3x=cmult3x,
                       domain_2x=domain_2x, domain_3x=domain_3x)
        self.width_out = len(self.fft.re_out)
        self.ab = ABSplit(self.width_out, order_log2, extra_pipe=ab_extra_pipe,
                          share_write_port=ab_share_write_port)

        self._use_2x = window is not None
        self._use_3x = cmult3x

        self.clken = Signal()
        if self._use_2x:
            self.common_edge_2x = Signal()
        if self._use_3x:
            self.common_edge_3x = Signal()
        self.re_in = Signal(signed(width_in))
        self.im_in = Signal(signed(width_in))
        self.a_re = Signal(signed(self.width_out))
        self.a_im = Signal(signed(self.width_out))
        self.b_re = Signal(signed(self.width_out))
        self.b_im = Signal(signed(self.width_out))
        self.out_valid = Signal()
        self.out_last = Signal()

    @property
    def latency(self):
        return self.fft.delay + self.ab.latency

    def model(self, z_re, z_im):
        """One packed frame z=(adc0)+j(adc1) -> (A, B), both natural-order N bins.

        Matches the fixed-point FFT (truncation schedule + window) followed by the
        exact A/B split; A is the ADC0 spectrum, B the ADC1 spectrum.
        """
        from maia_hdl.util import bit_invert
        N = 1 << self.order_log2
        Cre, Cim = self.fft.model(np.asarray(z_re), np.asarray(z_im))
        C_stream = np.asarray(Cre) + 1j * np.asarray(Cim)
        inv = np.array([bit_invert(k, self.order_log2, 1) for k in range(N)])
        C = C_stream[inv]                       # bit-reversed -> natural order
        return self.ab.model(C)

    def elaborate(self, platform):
        m = Module()
        m.submodules.fft = fft = self.fft
        m.submodules.ab = ab = self.ab
        m.d.comb += [
            fft.clken.eq(self.clken),
            fft.re_in.eq(self.re_in),
            fft.im_in.eq(self.im_in),
            ab.clken.eq(self.clken),
            ab.re_in.eq(fft.re_out),
            ab.im_in.eq(fft.im_out),
            ab.input_last.eq(fft.out_last),
            self.a_re.eq(ab.a_re), self.a_im.eq(ab.a_im),
            self.b_re.eq(ab.b_re), self.b_im.eq(ab.b_im),
            self.out_valid.eq(ab.out_valid),
            self.out_last.eq(ab.out_last),
        ]
        if self._use_2x:
            m.d.comb += fft.common_edge_2x.eq(self.common_edge_2x)
        if self._use_3x:
            m.d.comb += fft.common_edge_3x.eq(self.common_edge_3x)
        return m
