# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# System-level timing exceptions for the fcfb Stage-1 datapath at 125 MHz.
#
# The client-run configuration registers (k0, w_run, adc_mask, shift) are
# QUASI-STATIC: the PS board-server writes them once from the client's bin
# selection, then asserts `enable` and streams; they do not change during a
# capture.  Their fan-out into the datapath (shift -> the quantiser barrel
# shifters; k0/w_run -> bin-select; adc_mask -> stream framing) therefore does
# not need to meet the full 8 ns (125 MHz) single-cycle budget.  Relax those
# paths to a 2-cycle multicycle so the tool need not over-replicate/route them.
# (`enable` is deliberately NOT relaxed: it gates the datapath clock-enable and
# must take effect coherently.)
#
# Safe because the protocol is set-config -> enable -> stream -> disable: the
# values are stable for many cycles around any datapath capture of them.

set fcfb_cfg [get_cells -quiet -hierarchical -filter { \
    NAME =~ *fcfb_registers*field_shift_reg* || \
    NAME =~ *fcfb_registers*field_k0_reg* || \
    NAME =~ *fcfb_registers*field_w_run_reg* || \
    NAME =~ *fcfb_registers*field_adc_mask_reg* }]

set_multicycle_path -setup 2 -from $fcfb_cfg
set_multicycle_path -hold  1 -from $fcfb_cfg

# NOTE: the wideband (Phase-3b) sync<->fft async clock groups + the fft-side
# control-CDC multicycles live in cfg/timing_wideband.xdc, added to constrs_1
# ONLY for the wideband build (build.tcl, gated on $wideband).  They cannot be
# guarded here with Tcl `if` -- the XDC constraint reader rejects control flow
# (Designutils 20-1307) and would silently drop a guarded constraint.
