# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
# Adapted from the trx-duo MaiaSDR IP xdc (© Daniel Estevez / MaiaSDR port, MIT).
#
# IP-level timing constraints for the fcfb Stage1Top core (clock-domain
# crossings): the RegisterCDC register bus, the FIFO18E1 in the write-gated
# IqCDC, and any Amaranth-tagged false paths (e.g. the DMA-interrupt
# PulseSynchronizer).

# False path for Amaranth-generated false_path attributes.
set_false_path -to [get_cells -hier -filter {amaranth.vivado.false_path == "TRUE"}]

# Register-bus CDC (RegisterCDC): the request/response data registers are
# quasi-static across the crossing.  (Amaranth flattens the hierarchy into
# dotted module names, so match the register names anywhere in the netlist.)
set_false_path -to [get_pins -hierarchical -filter {NAME =~ *cdc_request_data_dest*/D}]
set_false_path -to [get_pins -hierarchical -filter {NAME =~ *cdc_response_data_dest*/D}]

# False path for the RST of every FIFO18E1 (the write-gated IqCDC's async FIFO).
# Target by primitive so it is robust to Amaranth's flattened naming.  Written as
# a single XDC-native command: the constraint reader used during IP
# out-of-context synthesis rejects Tcl control flow and would silently drop a
# guarded constraint (Designutils 20-1307).  The FIFO18E1 cell always exists
# here, and -quiet keeps an empty match from warning.
set_false_path -to [get_pins -quiet -of_objects \
    [get_cells -quiet -hierarchical -filter {REF_NAME == FIFO18E1}] \
    -filter {REF_PIN_NAME == RST}]
