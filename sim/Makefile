# =============================================================================
# RV32I + RV32M multi-cycle processor - build / simulation
#
#   make            compile and run the full self-checking testbench
#   make compile    elaborate only
#   make run        run the compiled simulation
#   make wave       run (if needed) and open the VCD in GTKWave
#   make lint       Vivado RTL-elaboration lint + methodology check
#   make synth      Yosys synthesis against the Nangate45 typical-corner lib
#   make sta        OpenSTA static timing analysis of the synthesized netlist
#   make run_asm PROG=<name>   assemble asm/<name>.s with the RISC-V GNU
#                               toolchain and run it on the core, lockstep
#                               against the golden model (see run_asm below)
#   make clean      remove build artefacts
#
# Options:
#   make DUMP=0        run without writing a VCD (faster, no waveform)
#   make SEED=1234     override the testbench random seed
# =============================================================================

# bash, not the default /bin/sh (dash): Vivado's settings64.sh (sourced by
# the `lint` target) uses bash-only syntax internally.
SHELL := /bin/bash

IVERILOG  ?= iverilog
VVP       ?= vvp
GTKWAVE   ?= gtkwave

RTL_DIR     := ../rtl
TB_DIR      := ../tb
ASM_DIR     := ../asm
SYNTH_DIR   := ../synth
LINT_DIR    := ../lint
BUILD       := .

RTL_SRCS := $(RTL_DIR)/risc.v
TB_SRCS  := $(TB_DIR)/riscv_ref.v $(TB_DIR)/risc_tb.v
SRCS     := $(RTL_SRCS) $(TB_SRCS)

TOP := risc_tb
OUT := $(BUILD)/$(TOP).vvp
VCD := $(BUILD)/$(TOP).vcd

# VCD dumping is on by default; DUMP=0 turns it off.
DUMP ?= 1

IVFLAGS := -g2005 -s $(TOP)
ifeq ($(DUMP),1)
  IVFLAGS += -DDUMP
endif
ifdef SEED
  IVFLAGS += -Prisc_tb.SEED=$(SEED)
endif

#-----------------------------------------------------------------------------
# Real assembly programs, run via the RISC-V GNU toolchain (Phase 3)
#
#   make run_asm PROG=fibonacci
#
# Flow: asm/<PROG>.s --gcc(as+ld)--> .elf --objcopy--> .text .bin
#       --scripts/bin2mem.py--> .mem --iverilog/vvp(risc_tb_asm)--> lockstep
#       run against riscv_ref.v (same golden model risc_tb.v uses).
#
# TOOLCHAIN_PREFIX points at this machine's installed riscv64-unknown-elf-*
# toolchain; override on the command line if yours lives elsewhere:
#   make run_asm PROG=fibonacci TOOLCHAIN_PREFIX=/usr/bin/riscv64-unknown-elf-
#-----------------------------------------------------------------------------
TOOLCHAIN_PREFIX ?= /opt/riscv-gnu/riscv/bin/riscv64-unknown-elf-
CC      := $(TOOLCHAIN_PREFIX)gcc
OBJCOPY := $(TOOLCHAIN_PREFIX)objcopy
OBJDUMP := $(TOOLCHAIN_PREFIX)objdump
PYTHON  ?= python3

# -march=rv32im (+no zicsr/zifencei): the core has no CSR/trap/fence
# infrastructure, so any program that needs them must be rejected at
# assemble time rather than silently misbehaving in simulation.
ASFLAGS := -march=rv32im -mabi=ilp32 -nostdlib -nostartfiles \
           -Wl,-Ttext=0x0 -Wl,--no-relax

ASM_MEM_DIR := $(BUILD)/asm_build
ASM_TOP     := risc_tb_asm
ASM_TB_SRCS := $(RTL_DIR)/risc.v $(TB_DIR)/riscv_ref.v $(TB_DIR)/risc_tb_asm.v
ASM_OUT     := $(BUILD)/$(ASM_TOP).vvp

ASM_IVFLAGS := -g2005 -s $(ASM_TOP)
ifeq ($(DUMP),1)
  ASM_IVFLAGS += -DDUMP
endif

.PHONY: run_asm
run_asm:
ifndef PROG
	$(error usage: make run_asm PROG=<name>  (assembles $(ASM_DIR)/<name>.s))
endif
	@test -f $(ASM_DIR)/$(PROG).s || \
		{ echo "*** $(ASM_DIR)/$(PROG).s not found ***"; exit 1; }
	@mkdir -p $(ASM_MEM_DIR)
	$(CC) $(ASFLAGS) -o $(ASM_MEM_DIR)/$(PROG).elf $(ASM_DIR)/$(PROG).s
	$(OBJCOPY) -O binary -j .text $(ASM_MEM_DIR)/$(PROG).elf $(ASM_MEM_DIR)/$(PROG).bin
	$(OBJDUMP) -d $(ASM_MEM_DIR)/$(PROG).elf > $(ASM_MEM_DIR)/$(PROG).dis
	$(PYTHON) $(BUILD)/bin2mem.py $(ASM_MEM_DIR)/$(PROG).bin $(ASM_MEM_DIR)/$(PROG).mem
	$(IVERILOG) $(ASM_IVFLAGS) -o $(ASM_OUT) $(ASM_TB_SRCS)
	@cd $(BUILD) && $(VVP) $(ASM_TOP).vvp +MEMFILE=asm_build/$(PROG).mem 2>&1 | tee asm_build/$(PROG).log
	@grep -q "^TB_STATUS: PASS$$" $(ASM_MEM_DIR)/$(PROG).log \
		|| { echo "*** $(PROG): DUT diverged from golden model ***"; exit 1; }

#-----------------------------------------------------------------------------
# ASIC synthesis (Yosys) + static timing analysis (OpenSTA)
#
#   make synth                  synthesize rtl/risc.v against Nangate45
#   make sta CLK_PERIOD=8        run STA on the synthesized netlist (ns)
#
# Flow: rtl/*.v --yosys(synth+dfflibmap+abc)--> gate netlist + area report
#       --sta(read_liberty/read_verilog/link_design)--> timing/power reports.
# synth/synth.tcl and synth/sta.tcl take their inputs as env vars rather than
# -tclargs so both scripts stay runnable by hand outside make too.
#-----------------------------------------------------------------------------
YOSYS       ?= yosys
STA         ?= sta
NANGATE_LIB ?= /opt/pdks/nangate45/lib/NangateOpenCellLibrary_typical.lib
SYNTH_TOP   := risc_v
# Logs/reports/netlist land in synth/reports/ (not sim/) so everything synth-
# related stays under synth/; `cd` into it before invoking yosys/sta so any
# incidental files the tools drop (not just the ones the scripts explicitly
# write) land there too. RTL_SRCS/SYNTH_DIR are $(abspath)'d since they're
# normally relative to sim/, which stops being correct after the `cd`.
SYNTH_OUT   := $(SYNTH_DIR)/reports
CLK_PERIOD  ?= 10
# 1 (default) = abc -fast, quick but worse QoR; 0 = full ABC resynthesis,
# which on this design's non_rest_div/carry_look_ahead can run for a very
# long time (see synth/synth.tcl's header comment). Override with:
#   make synth ABC_FAST=0
ABC_FAST    ?= 1

SYNTH_NETLIST := $(SYNTH_OUT)/$(SYNTH_TOP)_netlist.v

$(SYNTH_NETLIST): $(RTL_SRCS) $(SYNTH_DIR)/synth.tcl
	@mkdir -p $(SYNTH_OUT)
	@echo "==> Synthesizing $(SYNTH_TOP) (ABC_FAST=$(ABC_FAST)); expect ~30s"
	cd $(SYNTH_OUT) && \
	RTL_SRCS="$(abspath $(RTL_SRCS))" LIBERTY=$(NANGATE_LIB) TOP=$(SYNTH_TOP) OUT_DIR=. ABC_FAST=$(ABC_FAST) \
		$(YOSYS) -c $(abspath $(SYNTH_DIR))/synth.tcl 2>&1 | tee yosys.log
	@echo "==> Netlist: $(SYNTH_NETLIST)   Area report: $(SYNTH_OUT)/area.rpt"

.PHONY: synth sta
synth: $(SYNTH_NETLIST)

# Depends on the netlist, so a bare `make sta` will run synthesis first if the
# netlist is missing or stale. That is intentional, but it means the first
# output you see is Yosys's, not OpenSTA's -- the banner below makes clear
# which tool is actually running, since a silent synthesis run here previously
# looked like STA itself had hung.
sta: $(SYNTH_NETLIST)
	@echo "==> Running OpenSTA on $(SYNTH_NETLIST) at CLK_PERIOD=$(CLK_PERIOD) ns"
	cd $(SYNTH_OUT) && \
	LIBERTY=$(NANGATE_LIB) TOP=$(SYNTH_TOP) OUT_DIR=. CLK_PERIOD=$(CLK_PERIOD) \
		$(STA) -no_init -exit $(abspath $(SYNTH_DIR))/sta.tcl 2>&1 | tee sta.log

#-----------------------------------------------------------------------------
# Lint (Vivado RTL elaboration + report_methodology)
#
# Needs Vivado on PATH, which on this machine means the Python venv +
# Vivado settings script both sourced first (the `venv` shell function in
# ~/.config/rtl_py_venv.sh does this interactively; replicated here so `make
# lint` works non-interactively). Override VENV_ACTIVATE/VIVADO_SETTINGS if
# your setup differs.
#-----------------------------------------------------------------------------
VENV_ACTIVATE   ?= $(HOME)/.venv/bin/activate
VIVADO_SETTINGS ?= /opt/vivado/2025.2/Vivado/settings64.sh
VIVADO_PART     ?= xc7a100tcsg324-1
# Logs/reports land in lint/reports/ (not sim/) so everything lint-related stays
# under lint/, same reasoning as SYNTH_OUT above -- this also keeps Vivado's
# own working files (e.g. .Xil/) reports of sim/.
LINT_OUT        := $(LINT_DIR)/reports

.PHONY: all compile run wave lint synth sta clean help

all: run

compile: $(OUT)

$(OUT): $(SRCS)
	@mkdir -p $(BUILD)
	$(IVERILOG) $(IVFLAGS) -o $@ $(SRCS)

# Run from $(BUILD) so the VCD is written there rather than the repo root.
# Exits non-zero unless the testbench reports PASS, so this is usable in CI.
run: $(OUT)
	@cd $(BUILD) && $(VVP) $(TOP).vvp 2>&1 | tee run.log
	@grep -q "^TB_STATUS: PASS$$" $(BUILD)/run.log \
		|| { echo "*** TESTBENCH REPORTED FAILURES ***"; exit 1; }

wave: run
	@test -f $(VCD) || { echo "no VCD - was DUMP=0 used?"; exit 1; }
	$(GTKWAVE) $(VCD) &

# The lint findings arrive as Vivado messages rather than in a report file
# (see lint/lint.tcl's header for why methodology.rpt is legitimately empty),
# so distil the log into lint_warnings.rpt and print a count. Vivado's own
# exit status ignores warnings, so this is the artifact worth reading.
lint:
	@mkdir -p $(LINT_OUT)
	. $(VENV_ACTIVATE) && . $(VIVADO_SETTINGS) && \
	cd $(LINT_OUT) && \
	RTL_SRCS="$(abspath $(RTL_SRCS))" REPORT_DIR=. PART=$(VIVADO_PART) TOP=$(SYNTH_TOP) \
	vivado -mode batch -log vivado.log -journal vivado.jou \
		-source $(abspath $(LINT_DIR))/lint.tcl
	@grep -E '^(WARNING|CRITICAL WARNING|ERROR)' $(LINT_OUT)/vivado.log \
		> $(LINT_OUT)/lint_warnings.rpt || true
	@echo "-----------------------------------------------------------"
	@w=$$(grep -c '^WARNING'          $(LINT_OUT)/lint_warnings.rpt || true); \
	 c=$$(grep -c '^CRITICAL WARNING' $(LINT_OUT)/lint_warnings.rpt || true); \
	 e=$$(grep -c '^ERROR'            $(LINT_OUT)/lint_warnings.rpt || true); \
	 echo "lint: $$w warning(s), $$c critical warning(s), $$e error(s)"
	@echo "lint: findings  -> $(LINT_OUT)/lint_warnings.rpt"
	@echo "lint: full log  -> $(LINT_OUT)/vivado.log"
	@echo "-----------------------------------------------------------"
	@! grep -qE '^(CRITICAL WARNING|ERROR)' $(LINT_OUT)/lint_warnings.rpt

clean:
	rm -f $(BUILD)/*.vvp $(BUILD)/*.vcd $(BUILD)/*.log
	rm -rf $(ASM_MEM_DIR) $(SYNTH_OUT) $(LINT_OUT)

help:
	@echo "targets : all compile run wave lint synth sta clean run_asm"
	@echo "options : DUMP=0 (no VCD)   SEED=<n> (random seed)"
	@echo "          run_asm PROG=<name>  (assembles asm/<name>.s and runs it)"
	@echo "          sta CLK_PERIOD=<ns>  (synth target period, default 10)"
	@echo "          synth ABC_FAST=0     (full ABC opt instead of -fast; slow)"
