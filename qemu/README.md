# qemu-riscv32 as a second, independent oracle

`make run_asm` checks the core against `tb/riscv_ref.v`. That model is
independent *by construction* — it computes with the simulator's native
`* / % >>>` operators while the DUT uses a hand-built carry-lookahead adder,
Booth recoder and non-restoring divider — but it is still Verilog written in
this repository, from one author's one reading of the RISC-V spec.

That leaves a blind spot lockstep cannot see. If `risc.v` and `riscv_ref.v`
misread the same corner of the spec in the same way, they agree, and the run
reports a clean pass. Divide-by-zero results, the `INT_MIN / -1` overflow case,
sub-word load sign-extension and the signed/unsigned branch pairs are all
places where a single misreading would propagate into both files at once.

`qemu-riscv32` has no relationship to this codebase. Where it and the DUT
agree, that agreement is evidence.

This directory adds it as a second oracle **alongside** the existing one.
`make run_asm` is unchanged; `make run_qemu` runs both and reports each
separately.

## Flow

```
                asm/<prog>.s   (one source, two builds)
                      │
        ┌─────────────┴──────────────┐
        │ -DQEMU_BUILD=0             │ -DQEMU_BUILD=1
        │ -Ttext=0x0                 │ -Ttext=0x10000
        ▼                            ▼
   <prog>.elf                   <prog>.qemu.elf
        │ objcopy -j .text            │ qemu-riscv32 -plugin trace_plugin.so
        ▼                            ▼
   <prog>.mem                     raw trace  (S/I/E lines)
        │                            │ qemu_trace.py  (normalise + clip)
        │                            ▼
        │                       <prog>.qtrace
        └──────────────┬─────────────┘
                       ▼
              tb/risc_tb_asm.v
       compares the DUT against riscv_ref.v
       AND the qtrace, every retired instruction
```

## The three things that must be reconciled

The core and a Linux user-mode process cannot execute byte-identical images.
Exactly three differences exist, and each is handled in one place:

| # | Difference | Why it exists | Where it is resolved |
|---|---|---|---|
| 1 | Data lives at `DATA_BASE` (0x12000), not address 0 | Linux will not map page 0 (`vm.mmap_min_addr` = 65536, and `-R` does not relax it), so a store to 0 takes SIGSEGV. The core does not care: `data_mem` is indexed by `addr[7:2]`, so `DATA_BASE+k` aliases onto exactly the word plain `k` used to. | `asm/core_defs.inc` (`DATA_BASE`, `CORE_DATA_REGION`), same constant in **both** builds so address-holding registers match |
| 2 | Text is linked at 0x10000 for qemu, 0x0 for the core | Same page-0 restriction. All control flow in these programs is pc-relative, so the relocation is a pure offset. | `qemu_trace.py` subtracts `__trace_start` |
| 3 | Entry and exit differ | qemu hands a process a non-zero `sp`; the core comes out of reset all-zero. The core halts in a self-loop; qemu must `exit()` or it spins forever. | `CORE_ENTRY` zeroes the register file (qemu build only); `__trace_start` / `__trace_end` bracket the compared region and `qemu_trace.py` drops everything outside it |

Because `DATA_BASE` is identical on both sides, the dmem contents the core
produces are **unchanged** by the port — only registers that hold addresses
now carry `DATA_BASE+k` instead of `k`.

## Why a TCG plugin rather than `-d cpu`

`qemu-riscv32 -one-insn-per-tb -d cpu` does print the register file before
every instruction, but QEMU has no log item that reports **store addresses and
data**. The core's data memory is a large part of what we want checked, so the
text log cannot express the needed trace. The plugin API can, via
`qemu_plugin_register_vcpu_mem_cb()` — and it is far faster than parsing ~35
lines of pretty-printed text per instruction.

## What is checked

Per retired instruction: **pc**, **all 32 GPRs**, and **all 64 data-memory
words**. Memory is not inferred from later loads — stores from the trace are
folded into a shadow memory using the core's own `addr[7:2]` aliasing, then
compared word-for-word against `DUT.data_memory`.

Not checked: anything the core does not implement anyway (CSRs, traps, fences),
and instruction-memory contents (both sides are built from the same source).

## Files

| File | Role |
|---|---|
| `trace_plugin.c` | QEMU TCG plugin. Emits `I` (pc + 32 regs), `S` (store), `E` (count) records. |
| `Makefile` | Builds `trace_plugin.so` against `qemu-plugin.h` (API v6). |
| `qemu_trace.py` | Runs qemu under the plugin, normalises pc, clips to the compared region, shifts records so each pairs an instruction with the state it produced. Also does the register-relocation normalisation described below. |
| `tb_batch_gen.py` | Turns one `risc_tb.v` batch dump into a synthetic qemu-runnable image (see next section). |
| `../asm/core_defs.inc` | The only place the two `asm/*.s` builds differ. |
| `../tb/risc_tb_asm.v` | Consumes `+QEMUTRACE=<path>`; inactive without it. |
| `../tb/risc_tb.v` | Consumes `+QEMU_DUMP=<dir>` / `+QEMU_TRACE_DIR=<dir>`; inactive without either. |

## Usage

```sh
cd sim
make qemu_plugin              # build the plugin once
make run_qemu PROG=fibonacci  # one asm/*.s program, both oracles
make run_qemu_all             # every program under asm/
make run_asm PROG=fibonacci   # golden model only, exactly as before
make run_tb_qemu              # the risc_tb.v directed/random suite, both oracles
```

The summary names each oracle separately. The informative case is
`riscv_ref.v: 0 errors` together with `qemu: N errors` — that is the shared-
misreading blind spot this whole directory exists to expose.

## Second flow: qemu vs the risc_tb.v directed/random suite

`run_qemu` above checks one *compiled program* (`asm/*.s`) against qemu. But
the bulk of this core's verification - 6453 instructions across 54 batches -
comes from `tb/risc_tb.v`'s hand-generated directed and randomised instruction
streams, and those are not standalone programs: each batch starts from an
arbitrary SEEDED register file and data memory (not a zero reset), picked to
stress corner cases (INT_MIN, shift amounts that wrap, div-by-zero, ...) that
a compiled C-like program would rarely hit by chance. `make run_tb_qemu` gets
the same second-oracle coverage for that suite.

### Flow

```
risc_tb.v +QEMU_DUMP=<dir>            (pass 1: dump every batch's program + seeds,
        |                              deterministically, same SEED as always)
        v
  <dir>/batch_NNN.dump
        | tb_batch_gen.py             (synthesize a qemu image that reproduces
        v                              the batch's seeded state, see below)
  <dir>/batch_NNN.s  -->  .elf  -->  qemu_trace.py  -->  <dir>/batch_NNN.qtrace
        |
        v
risc_tb.v +QEMU_TRACE_DIR=<dir>       (pass 2: re-run with the SAME seed, so the
                                        same batches are generated in the same
                                        order, and compare each against both
                                        riscv_ref.v and its qtrace)
```

`tb_batch_gen.py`'s synthetic `_start` reproduces a batch's initial state by
brute force: `sw` a value into every non-zero seeded data word, then `li` (as
an unconditional `lui`+`addi` pair - see "exact instruction budget" below)
every register 1..31 to its seeded value, then the batch's own instructions
verbatim, then `exit(0)`. The batch's own appended halt self-loop (`beq
x0,x0,0`) is never emitted for qemu - a real self-loop would hang it forever -
so it is simply excluded from both sides of the comparison, the same way
`risc_tb_asm.v` excludes the core's post-`CORE_HALT` self-loop.

### Three problems that only showed up here, not in asm/*.s

None of `asm/*.s`'s seven programs use `JALR`, or keep a `JAL`/`AUIPC` result
in a register, or let a random instruction read the load/store base register
as an ordinary ALU operand. `risc_tb.v`'s `test_jumps` and `test_random`
exercise exactly those cases, and each one broke the qemu comparison in a
different, non-obvious way before landing on the design above:

1. **A register holding an absolute address is qemu-relocated even though
   only `pc` is supposed to need that.** `qemu_trace.py`'s pc-normalisation
   (subtract `__trace_start`) undoes the core-starts-at-0-but-qemu-can't
   relocation for the `pc` field, but a `JAL`/`JALR` link register or an
   `AUIPC` result captures a raw qemu-absolute address into a GPR, which the
   same relocation never touched - so that register would never match the
   DUT even on a fully correct core. Worse, the point at which a register
   becomes address-relative is dynamic (it's whichever instruction last wrote
   it), so this can't be a mask that's constant for the whole trace: applying
   the offset to a record *before* the writing instruction retires corrupts a
   comparison that was supposed to be exact. `qemu_trace.py --text-reloc-mask`
   (registers already address-relative in record 0, from an input seed) and
   `--text-reloc-events` (`offset:reg` - a register newly becomes
   address-relative starting at the record produced by the instruction at
   that byte offset) fix this; `tb_batch_gen.py` computes the events with a
   static scan for `JAL`/`JALR`/`AUIPC` with `rd != x0`.
2. **The load/store base register (`x1` by this suite's convention) can't
   just get `DATA_BASE` added to its seed and left at that.** `test_random`
   picks ALU source registers uniformly across all 32 registers, including
   `x1`; an ordinary `ADD`/`AND`/`MUL`/etc. reading a `DATA_BASE`-shifted `x1`
   bakes that qemu-only offset into whatever register receives the result -
   and for anything other than pure `+`/`-` it isn't even a constant offset
   anymore (`AND` does not distribute over addition), so it cannot be
   corrected after the fact. `risc_tb.v`'s `test_random` now excludes `x1`
   from `rs1`/`rs2` selection the same way it already excluded it from `rd`;
   x1 still gets full load/store coverage as the dedicated base.
3. **A batch retires exactly `prog_len` instructions, full stop - even if
   that means stopping mid-loop.** `test_jumps`'s `jal-backward` batch
   genuinely loops within its own instruction window forever if allowed to
   keep running (`risc_tb.v` only ever executes it for a handful of
   iterations before its fixed per-batch instruction budget runs out). qemu
   has no such external bound, so `tb_batch_gen.py` computes the exact
   instruction count - prologue length (deterministic, because every seed
   load is a fixed-size `lui`+`addi` pair rather than the size-optimising
   `li` pseudo-op) plus the batch's own instruction count plus one - and
   passes it as the plugin's `max=`, stopping qemu at exactly the same point
   the DUT stops, not one loop iteration later.

There is also a mundane one carried over from the general design: a batch's
data memory is seeded before execution, and words the program never stores to
stay at that seed forever. `qemu_trace.py` only ever sees *stores*, so
`risc_tb.v`'s shadow-memory reconstruction (`qt_dmem`, used purely for the
testbench's own comparison bookkeeping) has to start from the same seed
`DUT.data_memory` and `REF.dmem` do, not zero.

None of this is asm/*.s-specific paranoia: any RV32I program that calls a
subroutine (`JAL`+return via `JALR`) hits problem 1, so the fix belongs in
`qemu_trace.py` itself, not in `tb_batch_gen.py` alone.

## Gotchas worth knowing

**These sources go through cpp.** A comment line whose first word is a cpp
directive *is* a cpp directive: `#       if i == j: continue` in a pseudocode
block fails the build with "unterminated #if". `nested_loops.s` prefixes such
lines with `#   | `.

**QEMU does not publish `x8` as `s0`; it publishes it as `fp`.** Matching on a
single ABI name per register left `x8` unresolved and reading as a constant
zero — a register silently excluded from every comparison. `reg_aliases[]` in
`trace_plugin.c` now lists every accepted spelling and an unresolved register
is **fatal** rather than silently zero. Run with `verbose=1` to list the names
a given QEMU publishes.

**The trace is expected to be exactly one record short.** The core retires its
halt self-loop; the qemu build exits by syscall instead. Any larger shortfall
is reported as a control-flow divergence.

## Confirming the harness can fail

A comparison that never fires reports zero errors forever. The check was
fault-injected by mutating a known-good `.qtrace` and confirming the run turns
red, with `riscv_ref.v` still reporting zero — i.e. the qemu path alone caught
it:

| Injected fault | Result |
|---|---|
| register value changed | caught, register named |
| pc changed | caught, pc mismatch reported |
| store data changed | caught, dmem word named |
| store address changed (non-zero data) | caught, both vacated and wrongly-written words named |
| store address changed (**zero** data) | **not caught — correctly so** |

The last row is not a hole. Fibonacci's first store writes the value `0`, and
data memory is already all zeros, so redirecting that store to another zero
word is a genuine no-op with no observable consequence. When the same fault is
injected on a store carrying a non-zero value it is caught immediately. Worth
remembering when designing future fault-injection checks: a mutation that
happens to be semantically invisible proves nothing either way.

The `run_tb_qemu` path got the same treatment: a register field in one
`batch_NNN.qtrace` record was corrupted and `risc_tb.v` was re-run against the
(otherwise still correct) traces in place. The run turned red, naming the
exact batch, retired-instruction index and register (`QEMU ERROR #1 @
[random] instr 2 ... x4 : dut=b8f88a71 qemu=deadbeef`), with `riscv_ref.v`
still reporting zero errors for the same run - confirming the qemu compare
path in `risc_tb.v` (`qt_step_batch`) fires correctly and independently of the
golden-model check.
