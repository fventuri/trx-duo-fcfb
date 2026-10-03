# OOC synth+place+route timing/utilization spike.
# Usage: vivado -mode batch -source ooc_generic.tcl -tclargs <verilog> <top> <clkport> <period_ns> <tag>
set verilog [lindex $argv 0]
set top     [lindex $argv 1]
set clkport [lindex $argv 2]
set period  [lindex $argv 3]
set tag     [lindex $argv 4]
set part    xc7z010clg400-1

read_verilog $verilog
synth_design -top $top -part $part -mode out_of_context
create_clock -name clk -period $period [get_ports $clkport]
opt_design
place_design
route_design

puts "==== OOC RESULT ($tag) ===="
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "WNS @ ${period}ns : $wns"
puts "RAMB36 : [llength [get_cells -hierarchical -filter {REF_NAME == RAMB36E1}]]"
puts "RAMB18 : [llength [get_cells -hierarchical -filter {REF_NAME == RAMB18E1}]]"
puts "DSP48  : [llength [get_cells -hierarchical -filter {REF_NAME == DSP48E1}]]"
report_utilization
# WRITE_MODE of the bank BRAMs (Hazard-A READ_FIRST check for NB=T+1)
puts "---- bank BRAM WRITE_MODE / RDW (READ_FIRST check) ----"
foreach c [get_cells -hierarchical -filter {PRIMITIVE_GROUP == BLOCKRAM}] {
    set wm ""
    catch {set wm [get_property WRITE_MODE_A $c]}
    set wmb ""
    catch {set wmb [get_property WRITE_MODE_B $c]}
    puts "  $c  A=$wm B=$wmb"
}
puts "==== END ($tag) ===="
