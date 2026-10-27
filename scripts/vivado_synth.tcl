# vivado_synth.tcl -- out-of-context synthesis sanity check + resource proxy.
#   vivado -mode batch -source scripts/vivado_synth.tcl -tclargs <outdir>
# Not the ASIC flow (that is the Tiny Tapeout GDS action); this catches
# latches/loops/multi-drivers early and gives per-block flop counts.
set outdir [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/vivado"}]
set root   [file normalize [file dirname [info script]]/..]
file mkdir $outdir

foreach f {pw_fifo pw_mem pw_lane pw_core pw_host_spi pw_top tt_um_pinwheel} {
    read_verilog [list "$root/src/$f.v"]          ;# list: the path contains a space
}
synth_design -top tt_um_sashwath_s_pinwheel -part xc7z010clg400-1 -mode out_of_context \
             -flatten_hierarchy rebuilt -keep_equivalent_registers
create_clock -period 20.000 -name clk [get_ports clk]

report_utilization               -file $outdir/util.rpt
report_utilization -hierarchical -file $outdir/util_hier.rpt
report_timing_summary -max_paths 5 -file $outdir/timing.rpt
report_methodology               -file $outdir/methodology.rpt
puts "PW_SYNTH_DONE"
