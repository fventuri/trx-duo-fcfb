# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Adapted from the trx-duo MaiaSDR Vivado orchestrator (© Daniel Estevez /
# MaiaSDR port, MIT).  Packages the Amaranth-generated fcfb Stage-1 IP, builds
# the block design, and runs synth/impl/bitstream.  The cell/wire BD helpers are
# from the red-pitaya-notes flow (TRX_DUO_125-16 scripts/project.tcl,
# © Pavel Demin, MIT).
#
# Usage:  vivado -mode batch -source build.tcl -tclargs [all|package|bd|bit]
# Run from this directory (paths are resolved relative to the script).

set script_dir [file dirname [file normalize [info script]]]
set mode [expr {$argc >= 1 ? [lindex $argv 0] : "all"}]

# Wideband (Phase-3b): FCFB_HOP set (by build.sh, same env) -> the two-rate build
# with the fast fft clock (MMCM clk_out3 = 166.67 MHz).  Drives the conditional
# clk_out3 + fcfb_0/fft_clk wiring in block_design.tcl and the fft_clk clock
# interface in package_ip.tcl.  Unset -> the critical single-125 MHz build.
set wideband [expr {[info exists env(FCFB_HOP)] && $env(FCFB_HOP) ne ""}]

set part_name   xc7z010clg400-1
set ip_version  0.1.0
set verilog_file $script_dir/build/stage1_top.v
set ip_xdc       $script_dir/maia_fcfb.xdc
set cores_dir    $script_dir/tmp/cores
set preset_file  $script_dir/cfg/red_pitaya.xml
set proj_name    fcfb_stage1_system
set proj_dir     $script_dir/tmp/$proj_name

if {![file exists $verilog_file]} {
    error "generated Verilog not found: $verilog_file (run build.sh, which \
generates it first)"
}

# ---- BD helper procs (red-pitaya-notes) ----
proc wire {name1 name2} {
    set p1 [get_bd_pins $name1]
    set p2 [get_bd_pins $name2]
    if {[llength $p1] == 1 && [llength $p2] == 1} {
        connect_bd_net $p1 $p2
        return
    }
    set p1 [get_bd_intf_pins $name1]
    set p2 [get_bd_intf_pins $name2]
    if {[llength $p1] == 1 && [llength $p2] == 1} {
        connect_bd_intf_net $p1 $p2
        return
    }
    error "** ERROR: can't connect $name1 and $name2"
}

proc cell {cell_vlnv cell_name {cell_props {}} {cell_ports {}}} {
    set cell [create_bd_cell -type ip -vlnv $cell_vlnv $cell_name]
    set prop_list {}
    foreach {prop_name prop_value} [uplevel 1 [list subst $cell_props]] {
        lappend prop_list CONFIG.$prop_name $prop_value
    }
    if {[llength $prop_list] > 1} {
        set_property -dict $prop_list $cell
    }
    foreach {local_name remote_name} [uplevel 1 [list subst $cell_ports]] {
        wire $cell_name/$local_name $remote_name
    }
}

# ---- 1. Package the IP ----
if {$mode in {all package}} {
    source $script_dir/package_ip.tcl
}

if {$mode eq "package"} {
    puts "done (package)"
    return
}

# ---- 2. Create the system project + block design ----
file delete -force $proj_dir
create_project $proj_name $proj_dir -part $part_name -force
set_property IP_REPO_PATHS $cores_dir [current_project]
update_ip_catalog

create_bd_design system
source $script_dir/ports.tcl
source $script_dir/block_design.tcl
regenerate_bd_layout
validate_bd_design
save_bd_design

set system_bd [get_files system.bd]
set_property SYNTH_CHECKPOINT_MODE None $system_bd
generate_target all $system_bd
make_wrapper -files $system_bd -top
set wrapper [glob $proj_dir/$proj_name.gen/sources_1/bd/system/hdl/system_wrapper.v]
add_files -norecurse $wrapper
set_property TOP system_wrapper [current_fileset]

add_files -norecurse -fileset constrs_1 \
    [list $script_dir/cfg/adc_ports.xdc $script_dir/cfg/clocks.xdc \
          $script_dir/cfg/timing.xdc]
# Wideband: the sync<->fft async clock groups + fft-side control-CDC multicycles.
# Only for the two-rate build (references clk_out3 / the *_cdc regs, which do not
# exist in the critical single-125 build).
if {$wideband} {
    add_files -norecurse -fileset constrs_1 $script_dir/cfg/timing_wideband.xdc
}

if {$mode eq "bd"} {
    puts "done (bd)"
    return
}

# ---- 3. Synthesis, implementation, bitstream ----
# The datapath runs at 125 MHz (single-domain, no cmult3x).  The maia FFT's
# global clken and the sync reset are ~5000-fanout nets that need replication to
# close at 8 ns; enable phys_opt (post-place + post-route) with a replication-
# friendly directive so the tool fractures those nets into local driver copies.
# Timing-focused place + route directives.  The fft-domain (166.67 MHz) worst
# path is a StreamFormat jcnt carry chain (~8 logic levels) that sits right at the
# 6 ns edge; with the default directives it closes only marginally (+0.011 ns) and
# is sensitive to placement variance.  ExtraTimingOpt spends extra placement effort
# on the near-critical nets and AggressiveExplore routing recovers comfortable
# margin without any RTL change.
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore \
    [get_runs impl_1]

launch_runs impl_1 -to_step write_bitstream -jobs [expr {max(1,[exec nproc])}]
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "implementation did not complete (see runs)"
}
set bit [glob -nocomplain \
    $proj_dir/$proj_name.runs/impl_1/system_wrapper.bit]
file mkdir $script_dir/build
file copy -force $bit $script_dir/build/fcfb_stage1.bit
puts "bitstream -> $script_dir/build/fcfb_stage1.bit"

# Also emit a raw .bin (config data, no .bit header) for the mainline Zynq
# fpga-manager / fpga-region overlay.  Generated from the routed checkpoint.
open_checkpoint $proj_dir/$proj_name.runs/impl_1/system_wrapper_routed.dcp
write_bitstream -force -bin_file $proj_dir/$proj_name.runs/impl_1/system_wrapper
set bin [glob -nocomplain \
    $proj_dir/$proj_name.runs/impl_1/system_wrapper.bin]
if {$bin ne ""} {
    file copy -force $bin $script_dir/build/fcfb_stage1.bit.bin
    puts "raw bitstream -> $script_dir/build/fcfb_stage1.bit.bin"
}
