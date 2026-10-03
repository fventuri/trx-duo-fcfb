# Real-logic WNS (exclude the high-fanout sync reset startpoints) per subsystem.
open_checkpoint tmp/fcfb_stage1_system/fcfb_stage1_system.runs/impl_1/system_wrapper_routed.dcp

set rstcells [get_cells -quiet -hierarchical -filter {NAME =~ *sync_rst*rst_reg*}]
puts "sync_rst driver cells = [llength $rstcells]"
# High-fanout check: fanout of the sync reset net.
foreach c $rstcells {
    set pin [get_pins -quiet -of $c -filter {DIRECTION==OUT}]
    foreach net [get_nets -quiet -of $pin] {
        puts "  reset net [get_property NAME $net] fanout=[get_property FLAT_PIN_COUNT $net]"
    }
}

proc realwns {label filt rstcells} {
    set cells [get_cells -quiet -hierarchical -filter $filt]
    if {[llength $cells]==0} { puts "$label: (no cells)"; return }
    # worst 8 paths, skip any whose startpoint is in the reset tree
    set paths [get_timing_paths -quiet -setup -to $cells -max_paths 40 -nworst 40]
    foreach p $paths {
        set sp [get_property STARTPOINT_PIN $p]
        set spc [get_cells -quiet -of $sp]
        if {[lsearch -exact $rstcells $spc] >= 0} { continue }
        set spn [get_property NAME $sp]
        if {[string match *sync_rst* $spn]} { continue }
        puts "$label real WNS = [get_property SLACK $p]  from=$spn -> to=[get_property NAME [get_property ENDPOINT_PIN $p]]"
        return
    }
    puts "$label: (all top paths are reset paths)"
}

realwns "FFT"       {NAME =~ *fcfb_0*stage1/core/fft/*} $rstcells
realwns "streamfmt" {NAME =~ *fcfb_0*stage1/sf/*}       $rstcells
realwns "AB-split"  {NAME =~ *fcfb_0*stage1/bs/*}       $rstcells
realwns "egress"    {NAME =~ *egress_fifo*}             $rstcells
