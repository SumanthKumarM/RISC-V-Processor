# Vivado RTL lint pass, replacing the old `verilator --lint-only` target.
#
# `synth_design -rtl` elaborates the design (no technology mapping) and
# surfaces the classic lint findings -- truncated/unconnected ports, trimmed
# registers, inferred latches, incomplete case statements, width mismatches.
# Those arrive as Vivado *messages*, so the readable artifact is the warnings
# digest that sim/Makefile extracts from vivado.log into lint_warnings.rpt;
# this script's job is to produce the messages and the methodology report.
#
# Must run with Vivado on PATH: `venv` (see ~/.config/rtl_py_venv.sh) sources
# both the Python venv and /opt/vivado/2025.2/Vivado/settings64.sh -- see
# sim/Makefile's `lint` target for how this is invoked non-interactively.
#
# Required env vars:
#   RTL_SRCS   - space-separated list of Verilog sources
#   REPORT_DIR - where methodology.rpt lands (vivado.log/.jou are placed here
#                too, via the -log/-journal flags on the command line)
#   PART       - target part (elaboration-only, so any part works; pick one
#                that's actually installed -- xc7a100tcsg324-1 is the Basys3
#                Artix-7 part and is present in a stock Vivado install)
#   TOP        - top module name (risc_v)
#
# Usage: RTL_SRCS=... REPORT_DIR=... PART=... TOP=... \
#        vivado -mode batch -source lint/lint.tcl
#
#-----------------------------------------------------------------------------
# WHY methodology.rpt IS (CORRECTLY) NEARLY EMPTY
#
# `report_methodology` runs Vivado's UG949 design-methodology DRCs, and
# essentially all of them -- the TIMING-*, SYNTH-*, and XDCH-* families --
# need a *constrained, fully synthesized or implemented* netlist to have
# anything to check. Against an `-rtl` elaborated design with no XDC, every
# rule is inapplicable, so the report legitimately comes back "Checks
# found: 0" with empty tables. It is kept here because it costs nothing once
# the design is elaborated and becomes meaningful if constraints and a full
# (non `-rtl`) synthesis are ever added -- but it is NOT the lint signal.
# The real findings are the elaboration warnings; see lint_warnings.rpt.
#
# WHY `synth_design -lint` IS NOT USED
#
# Vivado 2025.2 does have a dedicated RTL linter (`synth_design -lint -file
# <rpt>`, rules ASSIGN-1..7, INFER-1..3, CLOCK-1, RESET-1..2). It was tested
# here and its report table comes back EMPTY even on RTL written to violate
# it deliberately (an inferred latch plus an incomplete case statement) --
# with or without `config_linter -rule ... -severity ...`, and in both
# project and non-project mode. Vivado's own elaborator flagged that same
# test RTL ("[Synth 8-155] case statement is not full and has no default"),
# so the checks run but never reach the report. Worse, invoking `-lint`
# ahead of `-rtl` in one session suppresses most of the `-rtl` warnings
# (Vivado de-duplicates message IDs per session): it drops this design from
# 54 warnings to 3. So `-lint` is omitted deliberately -- it adds nothing and
# actively hides real findings.
#-----------------------------------------------------------------------------

set rtl_srcs   [split $::env(RTL_SRCS)]
set report_dir $::env(REPORT_DIR)
set part       $::env(PART)
set top        $::env(TOP)

file mkdir $report_dir

read_verilog -sv $rtl_srcs

# Elaboration. This is the step that emits the real lint findings as Vivado
# messages; -name is passed so the run is reproducible in GUI mode too.
synth_design -rtl -name rtl_1 -part $part -top $top

report_methodology -file $report_dir/methodology.rpt
