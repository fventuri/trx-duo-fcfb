# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""``quantise`` — scale a wide complex bin down to ``out_width`` I/Q for the wire.

The M1 emitter (`sim/emit_stream.py`) ships `round(bin / bin_scale)` clipped to
int16, with `bin_scale = max|S_run| / 32000`. In hardware an arbitrary-float
division is expensive; instead this block does a **programmable arithmetic
right shift** by ``shift`` bits with round-to-nearest (half-up) and saturation
to a signed ``out_width`` output. The host's advertised ``bin_scale`` is then
``2**shift`` times the Stage-1 constant gain (the peak-normalized-window /
A-B-split factors), so the "gain absorbed into bin_scale" decision
(fixed_point_sim.py) holds end to end.

  q = sat( (bin + (1 << (shift-1))) >>> shift )            # shift >= 1
  q = sat(  bin )                                           # shift == 0

``out_width`` is the on-wire component width: 16 for the shipping critical build,
24 for the wideband Option-B build (which streams the full 23-bit internal bin so
the reconstructed-channel SFDR reaches the fixed-point ceiling ~-102 dBc instead
of the ~-96 dBc the int16 truncation caps at -- see sim/precision_study.py).
``shift`` is a runtime input (set from the board server's level/admission calc).
"""

from amaranth import *


I16_MAX = 2 ** 15 - 1          # 32767
I16_MIN = -(2 ** 15)           # -32768


class Quantise(Elaboratable):
    """Round + saturate a signed(width_in) complex value to signed(out_width) I/Q.

    Parameters
    ----------
    width_in : int
        Input component width (fcfb Stage-1 A/B width = 23).
    shift_width : int
        Bit width of the ``shift`` control input.
    extra_pipe : bool
        Split the round+saturate into TWO register stages (latency 1 -> 2).
        Needed to close the fast ``fft`` clock (166.67 MHz): the single-cycle
        chain (add round-offset -> variable ``>>shift`` barrel -> saturate) is
        ~15 logic levels and misses 6 ns.  Stage 1 registers the shifted value
        (add + barrel); stage 2 registers the saturate.  Default off = shipping
        125 MHz single-stage path.  Numerics are identical (pipelining preserves
        values), so ``model`` is unchanged.
    out_width : int
        On-wire component width (16 shipping, 24 for Option-B int24).  With
        out_width >= width_in (24 vs 23) a shift of 0 streams the raw internal
        bin unclipped; a nonzero shift still gives headroom/AGC for strong scenes.

    Attributes
    ----------
    clken : Signal(), in           Clock enable (registered datapath).
    shift : Signal(shift_width), in   Right-shift amount (>=0).
    in_valid : Signal(), in        Input strobe; registered out to out_valid.
    re_in, im_in : Signal(signed(width_in)), in
    re_out, im_out : Signal(signed(out_width)), out   Saturated I/Q.
    out_valid : Signal(), out
    """
    def __init__(self, width_in, shift_width=5, extra_pipe=False, out_width=16):
        self.w = width_in
        self.sw = shift_width
        self._extra_pipe = extra_pipe
        self.out_width = out_width
        self.omax = 2 ** (out_width - 1) - 1
        self.omin = -(2 ** (out_width - 1))
        self.clken = Signal()
        self.shift = Signal(shift_width)
        self.in_valid = Signal()
        self.re_in = Signal(signed(self.w))
        self.im_in = Signal(signed(self.w))
        self.re_out = Signal(signed(out_width))
        self.im_out = Signal(signed(out_width))
        self.out_valid = Signal()

    @property
    def latency(self):
        return 2 if self._extra_pipe else 1

    def model(self, x, shift):
        """Bit-exact reference: round-half-up by ``shift``, saturate to out_width.

        Matches the RTL (Python ``>>`` floors toward -inf, as Amaranth's signed
        ``>>``); mirrors ``test_quantise.q_ref``.
        """
        x = int(x)
        y = x if shift == 0 else (x + (1 << (shift - 1))) >> shift
        return int(max(self.omin, min(self.omax, y)))

    def _round(self, m, x):
        """Combinational round-half-up by ``shift`` (add offset + arithmetic >>).

        Returns the shifted value (signed(w+1)); the saturate is ``_sat``.  Split
        so ``extra_pipe`` can register between the two (the barrel-shift is the
        long half; the saturate is a pair of compares).
        """
        # round offset = (1 << (shift-1)) when shift>0 else 0
        off = Signal(signed(self.w + 1))
        with m.If(self.shift == 0):
            m.d.comb += off.eq(0)
        with m.Else():
            # shift >= 1 here, so (shift-1) is a valid unsigned shift amount.
            m.d.comb += off.eq(Const(1) << (self.shift - 1).as_unsigned())
        shifted = Signal(signed(self.w + 1))
        m.d.comb += shifted.eq((x + off) >> self.shift)
        return shifted

    def _sat(self, m, shifted):
        """Combinational saturate a shifted value to signed(out_width)."""
        sat = Signal(signed(self.out_width))
        with m.If(shifted > self.omax):
            m.d.comb += sat.eq(self.omax)
        with m.Elif(shifted < self.omin):
            m.d.comb += sat.eq(self.omin)
        with m.Else():
            m.d.comb += sat.eq(shifted)
        return sat

    def elaborate(self, platform):
        m = Module()
        re_shifted = self._round(m, self.re_in)
        im_shifted = self._round(m, self.im_in)
        if not self._extra_pipe:
            # 1-stage (125 MHz): round -> saturate -> register out.
            with m.If(self.clken):
                m.d.sync += [
                    self.re_out.eq(self._sat(m, re_shifted)),
                    self.im_out.eq(self._sat(m, im_shifted)),
                    self.out_valid.eq(self.in_valid),
                ]
        else:
            # 2-stage (166.67 MHz): stage 1 registers the shifted value (add +
            # barrel); stage 2 registers the int16 saturate.  Same numerics.
            re_sh_q = Signal(signed(self.w + 1))
            im_sh_q = Signal(signed(self.w + 1))
            valid_q = Signal()
            with m.If(self.clken):
                m.d.sync += [
                    re_sh_q.eq(re_shifted),
                    im_sh_q.eq(im_shifted),
                    valid_q.eq(self.in_valid),
                    self.re_out.eq(self._sat(m, re_sh_q)),
                    self.im_out.eq(self._sat(m, im_sh_q)),
                    self.out_valid.eq(valid_q),
                ]
        return m
