# Yosys synthesis script: RV32I+RV32M core -> Nangate45 gate-level netlist.
#
# Tcl mode ('yosys -import' registers every Yosys command as a Tcl proc), so
# paths/top/etc. come in as environment variables rather than being hardcoded
# -- same override-via-env pattern the sim/Makefile already uses for
# TOOLCHAIN_PREFIX etc. See sim/Makefile's `synth` target for how these are set.
#
# Required env vars:
#   RTL_SRCS  - space-separated list of Verilog sources
#   LIBERTY   - path to the target .lib (Nangate45 typical corner)
#   TOP       - top module name (risc_v)
#   OUT_DIR   - where the netlist/log/report land
# Optional:
#   ABC_FAST  - "1" (default) uses `abc -fast`, "0" uses full ABC resynthesis.
#               See the runtime note below before setting this to 0.
#
# Usage: RTL_SRCS=... LIBERTY=... TOP=... OUT_DIR=... yosys -c synth/synth.tcl
#
#-----------------------------------------------------------------------------
# RUNTIME NOTE -- why `synth -noabc`, and why this used to run for 30+ minutes
#
# This design's ALU is large and fully combinational. Measured gate counts
# after coarse synthesis:
#
#     Booths_mult          ~35.7k cells
#     non_rest_div         ~19.3k cells
#     carry_look_ahead      ~1.1k cells   (never the bottleneck, despite its
#                                          O(n^3) triple-nested source loop)
#
# Both large units are deep XOR/carry chains, the known worst case for ABC's
# structural hashing and mapping passes.
#
# Yosys's stock `synth` command runs its OWN internal `abc -fast` pass, with
# no liberty file, to map to generic gates. On this RTL that internal pass is
# pathological. Measured on non_rest_div in isolation:
#
#     synth -top non_rest_div -noabc         2.8 s
#     synth -top non_rest_div              > 300 s  (never seen to finish)
#
# which is exactly where whole-design runs used to sit for 30+ minutes,
# printing "Extracting gate netlist of module `\non_rest_div'" and appearing
# hung. In an ASIC flow that internal mapping is discarded anyway -- the
# `abc -liberty` call below immediately re-maps everything onto real standard
# cells -- so `-noabc` removes pure waste rather than trading away quality.
#
#     synth -noabc + abc -fast -liberty      ~30 s for the whole design
#
# ABC_FAST=0 switches the explicit mapping call to full `abc -liberty`
# resynthesis. That is much slower here: a whole-design run was still going
# after 10 minutes and was abandoned. Treat it as an occasional quality
# experiment, not a default. It affects area/timing quality only, never
# correctness.
#-----------------------------------------------------------------------------

yosys -import

set rtl_srcs [split $::env(RTL_SRCS)]
set liberty  $::env(LIBERTY)
set top      $::env(TOP)
set out_dir  $::env(OUT_DIR)
set abc_fast [expr {![info exists ::env(ABC_FAST)] || $::env(ABC_FAST) != "0"}]

file mkdir $out_dir

foreach src $rtl_srcs {
    read_verilog -sv $src
}

# Coarse + fine synthesis down to generic cells, but WITHOUT yosys's internal
# library-less ABC pass (see RUNTIME NOTE above).
synth -top $top -noabc

# Map flops, then combinational logic, onto real Nangate45 cells so that
# `stat -liberty` and the netlist handed to OpenSTA reflect actual cell areas
# and delays rather than Yosys's internal $_DFF_/$_AND_ primitives.
dfflibmap -liberty $liberty
if {$abc_fast} {
    abc -fast -liberty $liberty
} else {
    abc -liberty $liberty
}

# Resolve any remaining x/z to a defined constant before the netlist is
# written: OpenSTA reads structural Verilog and has no notion of 'x, so an
# undriven bit would otherwise surface as a dangling net at link_design time.
setundef -zero

# Tidy up after tech-mapping: split any remaining multi-bit nets left over
# from bit-blasted synth/abc passes, then drop now-dangling wires/cells.
splitnets
opt_clean

# Area/cell-count summary, using real Nangate45 cell areas instead of the
# generic bit-count estimate `stat` gives without -liberty.
tee -o $out_dir/area.rpt stat -liberty $liberty

# Gate-level netlist for OpenSTA (see synth/sta.tcl).
write_verilog -noattr $out_dir/${top}_netlist.v
