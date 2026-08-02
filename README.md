# RV32I+M Multi-Cycle RISC-V Processor

A synthesizable, single-issue, multi-cycle 32-bit RISC-V processor
implementing the **RV32I** base integer instruction set and the **RV32M**
standard extension for integer multiplication and division.

The design executes each instruction through a seven-state finite-state
machine (fetch, decode, register read, execute, memory access, write-back,
next-PC), and its arithmetic core is built from first-principles hardware
algorithms rather than inferred operators: a carry-lookahead adder/
subtractor, a radix-2 Booth multiplier, and a non-restoring divider.

Correctness is established through lockstep simulation against two
independent oracles — an in-repo behavioral golden model and
`qemu-riscv32` — and the design additionally carries a synthesis and static
timing analysis flow (Yosys + OpenSTA against a Nangate45 standard-cell
library).

## Repository Layout

| Path | Contents |
|---|---|
| `rtl/` | Synthesizable processor RTL (`risc.v`). |
| `tb/` | Testbenches and the behavioral golden model (`riscv_ref.v`). |
| `asm/` | Example RISC-V assembly programs used in verification. |
| `qemu/` | `qemu-riscv32`-based second-oracle verification flow (trace plugin, trace normalization, batch generation). |
| `sim/` | Build and simulation entry point (`Makefile`). |
| `synth/` | Yosys synthesis scripts and reports. |
| `lint/` | Vivado RTL lint scripts and reports. |
| `docs/` | Full design specification (AsciiDoc source and generated HTML). |

## Documentation

This README is intentionally brief. The complete design specification —
instruction set coverage, microarchitecture, control-signal encodings,
arithmetic algorithms, the full verification methodology, and the
build/synthesis/timing flow — is maintained in
[`docs/design_spec.adoc`](docs/design_spec.adoc).

To generate a browsable HTML copy of the specification run:

```sh
asciidoctor docs/design_spec.adoc -o docs/design_spec.html
```
