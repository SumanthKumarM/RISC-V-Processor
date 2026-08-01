# OpenSTA static timing analysis on the Yosys/Nangate45 netlist produced by
# synth/synth.tcl. Independent tool from the synthesis step deliberately --
# ABC's own internal timing model (used during `abc -liberty` mapping) isn't
# cross-checked by anything else in the synth.tcl flow, so this is where
# setup/hold and real critical-path numbers actually get verified against the
# same .lib.
#
# Required env vars (same names/values as synth.tcl's):
#   LIBERTY, TOP, OUT_DIR   - OUT_DIR must already contain ${TOP}_netlist.v
#   CLK_PERIOD              - target clock period in ns
#
# Usage: LIBERTY=... TOP=... OUT_DIR=... CLK_PERIOD=... sta -no_init -exit synth/sta.tcl
#
# EXPECT A LARGE VIOLATION AT THE DEFAULT PERIOD. The multiply and divide
# units are fully combinational (spatially unrolled, not sequenced), so the
# critical path runs through the entire divider within a single cycle --
# about 199 ns, i.e. ~5 MHz, against the 10 ns default target. That is a
# property of the RTL, not a flaw in this script or in synthesis: closing
# timing at a useful frequency would mean making the RV32M units multi-cycle
# or pipelined. Pass a realistic CLK_PERIOD to get a non-violating report.

set liberty    $::env(LIBERTY)
set top        $::env(TOP)
set out_dir    $::env(OUT_DIR)
set clk_period $::env(CLK_PERIOD)

read_liberty $liberty
read_verilog $out_dir/${top}_netlist.v
link_design $top

# Top-level clock port name, per rtl/risc.v's risc_v module ports ('clock').
create_clock -name clk -period $clk_period [get_ports clock]

# Constrain the data inputs only. `all_inputs` also returns the clock port,
# and constraining a port relative to a clock defined on that same port is
# illegal ("Warning 441 ... not allowed") -- it is rejected, silently leaving
# that input unconstrained. OpenSTA has no remove_from_collection, so filter
# the clock out by name.
set data_inputs {}
foreach port [all_inputs] {
    if {[get_full_name $port] ne "clock"} {
        lappend data_inputs $port
    }
}
set_input_delay  0.0 -clock clk $data_inputs
set_output_delay 0.0 -clock clk [all_outputs]

# Setup (max) and hold (min) critical paths, with the clock network expanded
# so launch/capture edges are visible in the report.
report_checks -path_delay min_max -format full_clock_expanded \
    > $out_dir/timing.rpt
report_tns >> $out_dir/timing.rpt
report_wns >> $out_dir/timing.rpt

# Static (no switching-activity annotation) power estimate -- informational
# only, real numbers need a VCD/SAIF activity file which this flow doesn't
# produce yet.
report_power > $out_dir/power.rpt

# Echo the headline numbers to stdout (and so into sta.log) so a Makefile run
# is readable without opening the report files.
puts "----------------------------------------------------------------"
puts "STA summary (target clock period ${clk_period} ns)"
report_wns
report_tns
puts "Full reports: ${out_dir}/timing.rpt , ${out_dir}/power.rpt"
puts "----------------------------------------------------------------"
