# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Block design for the fcfb Stage-1 datapath, in the red-pitaya-notes `cell`/
# `wire` idiom (helpers from TRX_DUO_125-16 scripts/project.tcl, © Pavel Demin,
# MIT).  Adapted from the trx-duo MaiaSDR system BD (© Daniel Estevez / MaiaSDR
# port, MIT): the SINGLE-125 clock plan (no clk2x/clk3x), and the streaming DMA
# to HP0 with a 32->64 width conversion in the interconnect.
#
# NOTE on register-path clocking/reset (inherited from trx-duo): the PS7 GP0
# register path is clocked by the PL MMCM (pll_0/clk_out1), NOT by a PS fabric
# clock (FCLK_CLK0), and its reset is derived from a tied-off constant gated by
# the MMCM lock, NOT from FCLK_RESET0_N.  This matches every RedPitaya/TRX-duo
# reference design and is required because the PL is programmed at runtime by the
# Linux FPGA manager, which does not re-run ps7_init: clocking/resetting the
# AXI-Lite slave from FCLK_* hangs every register read.
#
# Expects globals: preset_file (path to red_pitaya.xml).

# ---- Clocking: MMCM from the 125 MHz differential ADC (DCO) clock ----
# Single-125 plan: 125 -> VCO -> clk_out1 125 (sync, s_axi_lite, DMA, GP0/HP0),
# clk_out2 125 (sampling / ADC IOB capture, phase-aligned to sync).  No 2x/3x.
cell xilinx.com:ip:clk_wiz pll_0 {
  PRIMITIVE MMCM
  PRIM_IN_FREQ.VALUE_SRC USER
  PRIM_IN_FREQ 125.0
  PRIM_SOURCE Differential_clock_capable_pin
  CLKOUT1_USED true
  CLKOUT1_REQUESTED_OUT_FREQ 125.0
  CLKOUT2_USED true
  CLKOUT2_REQUESTED_OUT_FREQ 125.0
  USE_RESET false
} {
  clk_in1_p adc_clk_p_i
  clk_in1_n adc_clk_n_i
}

# Wideband (Phase-3b): add clk_out3 = 166.67 MHz (= 125 x 4/3) for the fast fft
# domain.  VCO = 1000 (M=8 from the 125 DCO, integer): clk_out1/2 = 1000/8 = 125,
# clk_out3 = 1000/6 = 166.67 -- all integer dividers, VCO in the -1 part range.
if {$wideband} {
  set_property -dict [list \
    CONFIG.CLKOUT3_USED true \
    CONFIG.CLKOUT3_REQUESTED_OUT_FREQ 166.666 \
  ] [get_bd_cells pll_0]
}

# ---- Processing system ----
cell xilinx.com:ip:processing_system7 ps_0 {
  PCW_IMPORT_BOARD_PRESET $preset_file
  PCW_USE_S_AXI_HP0 1
  PCW_S_AXI_HP0_DATA_WIDTH 64
  PCW_USE_FABRIC_INTERRUPT 1
  PCW_IRQ_F2P_INTR 1
} {
  M_AXI_GP0_ACLK pll_0/clk_out1
  S_AXI_HP0_ACLK pll_0/clk_out1
}

# DDR / FIXED_IO to external ports.
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {
  make_external {FIXED_IO, DDR}
  Master Disable
  Slave Disable
} [get_bd_cells ps_0]

# ---- Resets ----
# Tie-off "external reset never asserted" (active-low ext_reset_in driven high),
# so the reset controller is sequenced solely by the MMCM lock - exactly as the
# RedPitaya reference designs do.  Avoids depending on FCLK_RESET0_N, which the
# Linux-FPGA-manager programming flow does not guarantee to deassert.
cell xilinx.com:ip:xlconstant const_0 {
  CONST_WIDTH 1
  CONST_VAL 1
} {}
# Single reset for the whole PL: sync (clk_out1) domain, held until the MMCM
# locks.  Drives both the register/GP0 path and the streaming DMA path.
cell xilinx.com:ip:proc_sys_reset rst_adc {} {
  slowest_sync_clk pll_0/clk_out1
  ext_reset_in const_0/dout
  dcm_locked pll_0/locked
}

# ---- fcfb Stage-1 IP core ----
cell fcfb:user:fcfb_stage1 fcfb_0 {} {
  adc_dat_a adc_dat_a_i
  adc_dat_b adc_dat_b_i
  adc_csn adc_csn_o
  sampling_clk pll_0/clk_out2
  clk pll_0/clk_out1
  s_axi_lite_clk pll_0/clk_out1
  s_axi_lite_rst rst_adc/peripheral_reset
}

# Wideband: the fast fft clock is an extra IP input from clk_out3.  (Its reset is
# internal to the IP -- driven from sdr_reset -- so nothing else to wire.)
if {$wideband} {
  wire fcfb_0/fft_clk pll_0/clk_out3
}

# ---- Register access: PS7 M_AXI_GP0 (AXI3) -> s_axi_lite (AXI4-Lite) ----
# Clocked/reset from the PL MMCM (clk_out1) + MMCM-lock reset (see header note).
cell xilinx.com:ip:axi_interconnect axi_gp0 {
  NUM_SI 1
  NUM_MI 1
} {
  S00_AXI ps_0/M_AXI_GP0
  M00_AXI fcfb_0/s_axi_lite
  ACLK pll_0/clk_out1
  ARESETN rst_adc/interconnect_aresetn
  S00_ACLK pll_0/clk_out1
  S00_ARESETN rst_adc/interconnect_aresetn
  M00_ACLK pll_0/clk_out1
  M00_ARESETN rst_adc/interconnect_aresetn
}

# ---- Streaming DMA: m_axi_dma (AXI3, 32-bit) -> PS7 S_AXI_HP0 (64-bit) ----
# The axi_interconnect performs the 32->64 data-width conversion between the
# fcfb record stream master and the 64-bit HP slave.
cell xilinx.com:ip:axi_interconnect axi_hp0 {
  NUM_SI 1
  NUM_MI 1
} {
  S00_AXI fcfb_0/m_axi_dma
  M00_AXI ps_0/S_AXI_HP0
  ACLK pll_0/clk_out1
  ARESETN rst_adc/interconnect_aresetn
  S00_ACLK pll_0/clk_out1
  S00_ARESETN rst_adc/interconnect_aresetn
  M00_ACLK pll_0/clk_out1
  M00_ARESETN rst_adc/interconnect_aresetn
}

# ---- Interrupt: interrupt_out -> IRQ_F2P[0] ----
cell xilinx.com:ip:xlconcat int_concat {
  NUM_PORTS 1
} {
  In0 fcfb_0/interrupt_out
  dout ps_0/IRQ_F2P
}

# ---- Address assignment ----
# Registers window (PS accesses fcfb s_axi_lite).
assign_bd_address -offset 0x40000000 -range 0x1000 \
  [get_bd_addr_segs fcfb_0/s_axi_lite/reg0]
# DMA window (fcfb master accesses PS DDR via HP0).  Covers the DDR ring buffer
# configured in the IP (0x1000_0000 .. 0x1a00_0000).
assign_bd_address -offset 0x00000000 -range 0x20000000 \
  [get_bd_addr_segs ps_0/S_AXI_HP0/HP0_DDR_LOWOCM] \
  -target_address_space [get_bd_addr_spaces fcfb_0/m_axi_dma]
