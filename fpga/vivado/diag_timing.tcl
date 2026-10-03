# Per-subsystem WNS diagnostic on the routed checkpoint.
open_checkpoint tmp/fcfb_stage1_system/fcfb_stage1_system.runs/impl_1/system_wrapper_routed.dcp

proc wns_to {label filt} {
    set cells [get_cells -quiet -hierarchical -filter $filt]
    if {[llength $cells] == 0} { puts "$label: (no cells)"; return }
    set paths [get_timing_paths -quiet -setup -to $cells -max_paths 1 -nworst 1]
    if {[llength $paths] == 0} { puts "$label: (no paths)"; return }
    set s [get_property SLACK [lindex $paths 0]]
    puts "$label WNS = $s ns  (cells=[llength $cells])"
}

puts "==== per-subsystem setup WNS (to endpoints in each block) ===="
wns_to "WOLA"        {NAME =~ *fcfb_0*stage1/wola/*}
wns_to "FFT"         {NAME =~ *fcfb_0*stage1/core/fft/*}
wns_to "AB-split/bs" {NAME =~ *fcfb_0*stage1/bs/*}
wns_to "quantise"    {NAME =~ *fcfb_0*stage1/q*}
wns_to "streamfmt"   {NAME =~ *fcfb_0*stage1/sf/*}
wns_to "reg-CDC"     {NAME =~ *fcfb_registers_cdc*}
wns_to "egress_fifo" {NAME =~ *egress_fifo*}

puts "==== FFT-only, excluding WOLA (top 5 worst paths) ===="
set fftcells [get_cells -quiet -hierarchical -filter {NAME =~ *fcfb_0*stage1/core/fft/*}]
foreach p [get_timing_paths -quiet -setup -to $fftcells -max_paths 5 -nworst 5] {
    puts "  slack=[get_property SLACK $p]  from=[get_property STARTPOINT_PIN $p] -> to=[get_property ENDPOINT_PIN $p]"
}
