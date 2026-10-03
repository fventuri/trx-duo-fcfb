# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# External block-design ports for the fcfb Stage-1 project.  Only the ADC
# interface is exposed (dual 16-bit parallel data + differential DCO clock +
# chip-select); DDR / FIXED_IO come from the PS7 automation.  Pin locations are
# in cfg/adc_ports.xdc (from the TRX_DUO_125-16 board files, © Pavel Demin, MIT).

create_bd_port -dir I -from 15 -to 0 adc_dat_a_i
create_bd_port -dir I -from 15 -to 0 adc_dat_b_i
create_bd_port -dir I adc_clk_p_i
create_bd_port -dir I adc_clk_n_i
create_bd_port -dir O adc_csn_o
