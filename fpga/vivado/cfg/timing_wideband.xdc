# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Wideband (Phase-3b) timing exceptions.  Added to constrs_1 ONLY for the
# wideband two-rate build (build.tcl, gated on $wideband); the critical single-
# 125 build never sees this file, so it may reference clk_out3 / the *_cdc regs
# unconditionally.  Pure XDC (no Tcl `if`: the constraint reader rejects control
# flow -- Designutils 20-1307 -- and would silently drop a guarded constraint).

# ---- sync (clk_out1, 125) <-> fft (clk_out3, 166.67) are asynchronous ----
# The WOLA + FFT + backend run in fft; FS-rate inputs, the record stream, and the
# control registers cross through AsyncFIFOs + FFSynchronizers.  The gray-pointer
# and control syncs carry amaranth's false_path attribute, but the AsyncFIFO's
# own storage-RAM read (written in one domain, read in the other) is a real
# cross-clock net that STA otherwise tries (and fails) to time single-cycle.
# Declaring the domains asynchronous ignores those safe-by-construction paths.
set_clock_groups -asynchronous \
    -group [get_clocks -of_objects \
        [get_pins -hierarchical -filter {NAME =~ *pll_0*clk_out1}]] \
    -group [get_clocks -of_objects \
        [get_pins -hierarchical -filter {NAME =~ *pll_0*clk_out3}]]

# ---- quasi-static control CDC outputs (fft domain) -> datapath ----
# k0 / w_run / adc_mask / shift are written once per retune (while the datapath
# is disabled), then held for the whole capture.  In the critical build they are
# relaxed to a 2-cycle multicycle at the source register (cfg/timing.xdc's
# fcfb_cfg); in wideband they reach the datapath through the *_cdc FFSynchronizer
# second stage, so relax THOSE flops instead (else e.g. shift -> the quantiser
# barrel shifter is a full 6 ns single-cycle path that misses badly).  enable is
# deliberately NOT relaxed -- it gates the datapath clock-enable and must take
# effect coherently.
set wb_cfg [get_cells -hierarchical -filter { \
    NAME =~ *shift_cdc*stage1_reg* || \
    NAME =~ *k0_cdc*stage1_reg* || \
    NAME =~ *w_run_cdc*stage1_reg* || \
    NAME =~ *adc_mask_cdc*stage1_reg* }]
set_multicycle_path -setup 2 -from $wb_cfg
set_multicycle_path -hold  1 -from $wb_cfg
