# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
# Adapted from the trx-duo MaiaSDR package_ip.tcl (© Daniel Estevez / MaiaSDR
# port, MIT): single streaming DMA (m_axi_dma), no clk2x/clk3x clocks.
#
# Package the Amaranth-generated Stage1Top Verilog as a Vivado IP core.
# Expects these globals set by the caller (build.tcl):
#   part_name    target part
#   verilog_file path to stage1_top.v
#   ip_xdc       path to maia_fcfb.xdc (IP-level constraints)
#   cores_dir    output IP repository dir (IP goes in $cores_dir/fcfb_stage1)
#   ip_version   version string, e.g. 0.1.0

set ip_dir $cores_dir/fcfb_stage1
file delete -force $ip_dir

create_project fcfb_stage1 $ip_dir -part $part_name -force
add_files -norecurse $verilog_file
add_files -norecurse -fileset constrs_1 $ip_xdc
set_property top top [current_fileset]

# The streaming DMA master uses id_bits=0, so AWID/WID/BID are zero-width and
# Amaranth/Yosys emit them as default-direction (input) ports.  On an AXI master
# AWID/WID are outputs, so inference trips a benign direction-mismatch
# (IP_Flow 19-3480) on these no-bit signals.  Suppress just that check while
# packaging; the stale port maps are removed below so the IP itself is clean.
set_msg_config -id {IP_Flow 19-3480} -suppress
ipx::package_project -import_files -root_dir $ip_dir \
    -vendor fcfb -library user -taxonomy /fcfb -force
reset_msg_config -id {IP_Flow 19-3480} -suppress
set core [ipx::current_core]
set_property name fcfb_stage1 $core
set_property library user $core
set_property display_name {fcfb Stage-1} $core
set_property description {fcfb distributed fast-convolution filter bank, Stage-1} $core
set_property vendor_display_name {fcfb project} $core
set_property version $ip_version $core

proc add_clock_if {core name} {
    ipx::add_bus_interface $name $core
    set bif [ipx::get_bus_interfaces $name -of_objects $core]
    set_property abstraction_type_vlnv xilinx.com:signal:clock_rtl:1.0 $bif
    set_property bus_type_vlnv xilinx.com:signal:clock:1.0 $bif
    ipx::add_bus_parameter FREQ_HZ $bif
    ipx::add_port_map CLK $bif
    set_property physical_name $name [ipx::get_port_maps CLK -of_objects $bif]
}

# Clock interfaces (Amaranth emits bare clock ports; declare them explicitly).
add_clock_if $core sampling_clk
add_clock_if $core s_axi_lite_clk
# 'clk' is the sync-domain clock (Amaranth's default domain -> port name 'clk').
add_clock_if $core clk
# Wideband (Phase-3b): the fast fft clock exists only in the two-rate IP.  Declare
# it as a clock interface when the generated Verilog carries the port.
if {[llength [ipx::get_ports -quiet fft_clk -of_objects $core]]} {
    add_clock_if $core fft_clk
}

# 'rst' output reset (internal sdr_reset on the sync domain).
ipx::add_bus_parameter POLARITY [ipx::get_bus_interfaces rst -of_objects $core]
set_property value ACTIVE_HIGH [ipx::get_bus_parameters POLARITY -of_objects \
    [ipx::get_bus_interfaces rst -of_objects $core]]

# s_axi_lite_rst input reset.
ipx::add_bus_interface s_axi_lite_rst $core
set bif [ipx::get_bus_interfaces s_axi_lite_rst -of_objects $core]
set_property abstraction_type_vlnv xilinx.com:signal:reset_rtl:1.0 $bif
set_property bus_type_vlnv xilinx.com:signal:reset:1.0 $bif
ipx::add_bus_parameter POLARITY $bif
set_property value ACTIVE_HIGH [ipx::get_bus_parameters POLARITY -of_objects $bif]
ipx::add_port_map RST $bif
set_property physical_name s_axi_lite_rst [ipx::get_port_maps RST -of_objects $bif]

# Associate the AXI interfaces with their clocks.  s_axi_lite has its own
# s_axi_lite_clk port; m_axi_dma runs in the sync ('clk') domain.  In the block
# design both s_axi_lite_clk and clk are driven by the PL MMCM (pll_0/clk_out1) -
# the register path is NOT on a PS FCLK (see block_design.tcl).
ipx::associate_bus_interfaces -busif s_axi_lite -clock clk -remove $core
ipx::associate_bus_interfaces -busif s_axi_lite -clock s_axi_lite_clk $core
ipx::associate_bus_interfaces -busif m_axi_dma -clock clk $core

# Drop the zero-width AWID/WID/BID port maps (id_bits=0): the signals carry no
# bits, so these stale maps are what caused the direction-mismatch above.
set dma_bif [ipx::get_bus_interfaces m_axi_dma -of_objects $core]
foreach pm {AWID WID BID} {
    if {[llength [ipx::get_port_maps $pm -of_objects $dma_bif]]} {
        ipx::remove_port_map $pm $dma_bif
    }
}

# Interrupt output.
ipx::add_bus_interface interrupt $core
set bif [ipx::get_bus_interfaces interrupt -of_objects $core]
set_property abstraction_type_vlnv xilinx.com:signal:interrupt_rtl:1.0 $bif
set_property bus_type_vlnv xilinx.com:signal:interrupt:1.0 $bif
set_property interface_mode master $bif
ipx::add_port_map INTERRUPT $bif
set_property physical_name interrupt_out [ipx::get_port_maps INTERRUPT -of_objects $bif]

ipx::create_xgui_files $core
ipx::update_checksums $core
ipx::save_core $core
close_project
puts "packaged fcfb_stage1 IP -> $ip_dir"
