#!/usr/bin/env python3
"""
tb_batch_gen.py - turn one tb/risc_tb.v batch dump into a qemu-runnable
assembly source, so the hand-generated directed/random instruction streams in
risc_tb.v can be checked against qemu-riscv32 exactly like the asm/*.s
programs are (see README.md, `make run_tb_qemu` in ../sim/Makefile).

Why this is needed instead of just reusing the asm/*.s flow
-------------------------------------------------------------------------
A risc_tb.v batch is not a standalone program: it starts from an arbitrary
SEEDED register file and data memory (not a zero reset), and a few batches
hold small absolute "core address" values in specific registers - JALR
targets, and the load/store base register - that only make sense in the
core's zero-based address space. This script builds a synthetic _start that
reproduces the same architectural state under qemu:

  1. seed data memory, using x1/x2 as scratch (safe: step 2 unconditionally
     overwrites every register with its real seeded value afterwards)
  2. load every register x1..x31 with its seeded value, relocating the few
     registers the dump flags as holding a "core address":
       - TRELOC bit r: register r holds an offset from the batch's own
         pc=0 start (a JALR target) -> becomes __trace_start+offset
       - DRELOC bit r: register r holds an offset into the 64-word data
         window (the load/store base register, x1 by this testbench's
         convention) -> becomes DATA_BASE+offset
  3. __trace_start, then the batch's instructions verbatim (.word) MINUS the
     halt self-loop risc_tb.v appends to every batch - a real self-loop would
     hang qemu forever, so it is simply never traced, on either side
  4. __trace_end, then exit(0)

The result is assembled/linked exactly like asm/*.s's qemu build (same
DATA_BASE, same QEMU_TEXT, same -march=rv32im), then run through the existing
qemu_trace.py to produce the canonical trace risc_tb.v's compare pass
(+QEMU_TRACE_DIR=<dir>) reads.

Dump format written by risc_tb.v's dump_batch task (whitespace-separated
tokens; the exact line breaks don't matter, only the token order):
    <prog_len>
    <prog_len hex instruction words>   (prog[0..prog_len-1]; the LAST word is
                                         always the appended halt self-loop)
    <32 hex words>                     (seed_reg[0..31])
    <64 hex words>                     (seed_mem[0..63])
    <hex text_reloc_mask>
    <hex data_reloc_mask>
"""
import argparse
import sys

NREG = 32
NMEM = 64


def die(msg):
    sys.stderr.write("tb_batch_gen: error: %s\n" % msg)
    raise SystemExit(1)


def read_dump(path):
    with open(path) as f:
        toks = f.read().split()
    pos = 0

    def next_tok():
        nonlocal pos
        if pos >= len(toks):
            die("%s: truncated dump" % path)
        t = toks[pos]
        pos += 1
        return t

    prog_len = int(next_tok())
    if prog_len < 1:
        die("%s: empty program (prog_len=%d)" % (path, prog_len))
    prog = [int(next_tok(), 16) for _ in range(prog_len)]
    regs = [int(next_tok(), 16) for _ in range(NREG)]
    mem = [int(next_tok(), 16) for _ in range(NMEM)]
    treloc = int(next_tok(), 16)
    dreloc = int(next_tok(), 16)
    return prog, regs, mem, treloc, dreloc


JAL_OP, JALR_OP, AUIPC_OP = 0x6f, 0x67, 0x17


def compute_text_reloc_events(traced):
    """Find every point in the trace where a register newly becomes a
    'distance from this program's own start' value, needing trace_start
    subtracted at compare time from then on (see qemu_trace.py's normalise()
    for why this must be an event list keyed by offset, not a mask constant
    for the whole trace: a register compares raw right up until the
    instruction that actually writes a pc-derived value into it).

    A static scan, not a simulation: it does not matter whether a flagged
    instruction is actually reached (a skipped branch target scanned as if
    executed only widens the event set harmlessly), only that its encoding is
    JAL/JALR/AUIPC with rd != x0. Once flagged, a register is assumed to stay
    address-flavoured for the rest of the trace, which holds for every batch
    risc_tb.v currently generates (none re-purpose such a register for plain
    data afterwards).

    Returns {byte_offset_from_trace_start: [reg, ...]}.
    """
    events = {}
    for i, w in enumerate(traced):
        op = w & 0x7f
        rd = (w >> 7) & 0x1f
        if op in (JAL_OP, JALR_OP, AUIPC_OP) and rd != 0:
            events.setdefault(4 * i, []).append(rd)
    return events


def li32(lines, reg, value):
    """Emit a value into `reg` as an ALWAYS-exactly-2-instruction lui+addi
    pair (%hi/%lo on a plain constant, not the `li` pseudo-op, which
    optimises away the lui when the value fits a 12-bit immediate). The
    instruction count must be exactly predictable so main() can compute a
    qemu instruction cap that stops the trace at precisely the batch's own
    instruction count (see the --max discussion in main())."""
    lines.append("    lui  x%d, %%hi(%d)" % (reg, value))
    lines.append("    addi x%d, x%d, %%lo(%d)" % (reg, reg, value))


def gen_asm(prog, regs, mem, treloc, dreloc, data_base):
    # The batch's own halt self-loop (always the last word) has no qemu
    # counterpart; trace only the real instructions ahead of it.
    traced = prog[:-1]
    if not traced:
        die("program has no instructions besides the halt self-loop")

    lines = []
    prologue_insns = 0   # exact machine-instruction count before __trace_start
    lines.append("# auto-generated by tb_batch_gen.py - do not edit by hand")
    lines.append(".option norelax")
    lines.append(".section .bss")
    lines.append(".balign 16")
    lines.append(".globl __data_base")
    lines.append("__data_base:")
    lines.append("    .space %d" % (NMEM * 4))
    lines.append(".text")
    lines.append(".globl _start")
    lines.append("_start:")

    lines.append("    # ---- seed data memory (x1/x2 scratch; overwritten below) ----")
    for k, v in enumerate(mem):
        if v == 0:
            continue
        li32(lines, 2, data_base + 4 * k)
        li32(lines, 1, v)
        lines.append("    sw   x1, 0(x2)")
        prologue_insns += 5

    lines.append("    # ---- seed registers x1..x31 (overwrites the scratch above) ----")
    for r in range(1, NREG):
        v = regs[r]
        if (treloc >> r) & 1:
            off = v
            if off > 0x7fffffff:
                off -= 1 << 32   # treat as signed for the range check below
            if not (-2048 <= off <= 2047):
                die("register x%d: treloc offset %d does not fit a 12-bit "
                    "immediate; extend tb_batch_gen.py's la+addi sequence" % (r, off))
            # `la` on a local symbol in a static, non-PIC link is always
            # exactly auipc+addi (2 instructions) - never optimised further,
            # same determinism reasoning as li32.
            lines.append("    la   x%d, __trace_start" % r)
            prologue_insns += 2
            if off != 0:
                lines.append("    addi x%d, x%d, %d" % (r, r, off))
                prologue_insns += 1
        elif (dreloc >> r) & 1:
            li32(lines, r, (data_base + v) & 0xffffffff)
            prologue_insns += 2
        else:
            li32(lines, r, v)
            prologue_insns += 2

    lines.append("    .globl __trace_start")
    lines.append("__trace_start:")
    for w in traced:
        lines.append("    .word 0x%08x" % w)
    lines.append("    .globl __trace_end")
    lines.append("__trace_end:")
    lines.append("    li   a7, 93   # __NR_exit")
    lines.append("    li   a0, 0")
    lines.append("    ecall")
    lines.append("")
    return "\n".join(lines), prologue_insns


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True, help="risc_tb.v batch dump (batch_NNN.dump)")
    ap.add_argument("--out", required=True, help="assembly file to write")
    ap.add_argument("--data-base", type=lambda s: int(s, 0), default=0x12000)
    args = ap.parse_args()

    prog, regs, mem, treloc, dreloc = read_dump(args.dump)
    asm, prologue_insns = gen_asm(prog, regs, mem, treloc, dreloc, args.data_base)
    with open(args.out, "w") as f:
        f.write(asm)

    # data_reloc_mask needs no auto-detection: every dreloc-flagged register
    # in this testbench (the load/store base, x1) is never overwritten mid-
    # batch, so the input-time mask is valid for the whole trace unchanged.
    # text_reloc_mask (treloc) is likewise valid from record 0 - those
    # registers were seeded address-relative before __trace_start. Registers
    # that only BECOME address-relative partway through (a JAL/JALR/AUIPC
    # result) need the event-based form; see compute_text_reloc_events.
    events = compute_text_reloc_events(prog[:-1])
    events_str = ",".join("%d:%d" % (off, r) for off, regs_ in sorted(events.items())
                          for r in regs_)

    # risc_tb.v's own execution model retires EXACTLY prog_len instructions
    # per batch, regardless of where control flow goes - a batch like
    # "jal-backward" (test_jumps) genuinely loops within its own instruction
    # window forever if allowed to keep running. qemu has no such external
    # bound, so it must be told to stop after exactly the same number of real
    # instructions: prologue_insns to clear the seeding code, plus
    # len(traced) for the batch body, plus 1 so the plugin's pre-instruction
    # register snapshot captures the LAST traced instruction's result. Too
    # low truncates the batch; too high (mattering only for a looping batch)
    # would silently re-enter the trace window and record extra, unwanted
    # repeats of it.
    max_insn = prologue_insns + len(prog[:-1]) + 1

    reloc_path = args.out + ".reloc"
    with open(reloc_path, "w") as f:
        f.write("%#010x\n" % treloc)
        f.write("%#010x\n" % dreloc)
        f.write("%s\n" % events_str)
        f.write("%d\n" % max_insn)

    print("tb_batch_gen: %s -> %s (%d traced instruction(s)), reloc info -> %s" %
          (args.dump, args.out, len(prog) - 1, reloc_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
