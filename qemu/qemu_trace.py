#!/usr/bin/env python3
"""
Run a program under qemu-riscv32 with trace_plugin.so and normalise the result
into the canonical trace that tb/risc_tb_asm.v compares the DUT against.

Why a normalisation step is needed
----------------------------------
The core and qemu cannot execute byte-identical images, so the two builds
differ in exactly two places and this script undoes both:

  1. Link address. The core fetches from address 0 after reset, but Linux will
     not map page 0 (vm.mmap_min_addr), so the qemu build is linked at 0x10000.
     Every control transfer in these programs is pc-relative, which makes the
     relocation pure offset - subtracting __trace_start converts a qemu pc back
     into the core's pc.

  2. Entry and exit. qemu hands a process a non-zero sp, so the qemu build runs
     a register-zeroing prologue to reach the core's post-reset state; and it
     exits with a syscall where the core parks in a self-loop. Neither tail is
     the same code on both sides, so neither is compared: records outside
     [__trace_start, __trace_end) are dropped. What remains is a region where
     both sides are running identical instructions from identical state.

Record alignment
----------------
The plugin logs the register file BEFORE each instruction, so the state that
follows instruction j is the one logged with instruction j+1. This script does
that shift, pairing each retired instruction with the state it produced and the
store it performed, so a canonical record reads as "after this instruction
retired: pc, registers, memory writes".

Canonical output format
-----------------------
    <n_records>                                     (decimal, first line)
    <pc> <r0> ... <r31> <nstores> [<addr> <size> <data>]*   (hex, one per line)

Store addresses stay absolute; the testbench applies the core's addr[7:2]
aliasing when it folds them into the shadow data memory.
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_QEMU = "/opt/riscv-gnu/riscv/bin/qemu-riscv32"
DEFAULT_NM = "/opt/riscv-gnu/riscv/bin/riscv64-unknown-elf-nm"
DEFAULT_PLUGIN = os.path.join(HERE, "trace_plugin.so")

NREG = 32


def die(msg):
    sys.stderr.write("qemu_trace: error: %s\n" % msg)
    raise SystemExit(1)


def symbols(nm, elf):
    """Return {name: address} for the ELF's global symbols."""
    try:
        out = subprocess.run([nm, elf], capture_output=True, text=True,
                             check=True).stdout
    except FileNotFoundError:
        die("nm not found: %s" % nm)
    except subprocess.CalledProcessError as e:
        die("nm failed on %s:\n%s" % (elf, e.stderr))

    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            syms[parts[2]] = int(parts[0], 16)
    return syms


def run_qemu(qemu, plugin, elf, raw_path, max_insn, timeout):
    if not os.path.exists(plugin):
        die("plugin not found: %s\n"
            "       build it first:  make -C %s" % (plugin, os.path.dirname(plugin) or "."))

    cmd = [qemu,
           "-plugin", "%s,outfile=%s,max=%d" % (plugin, raw_path, max_insn),
           elf]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout)
    except FileNotFoundError:
        die("qemu not found: %s" % qemu)
    except subprocess.TimeoutExpired:
        die("qemu did not finish within %ds.\n"
            "       A program that never reaches CORE_HALT will spin forever; "
            "raise --timeout if the program is simply long." % timeout)

    if proc.stderr.strip():
        sys.stderr.write(proc.stderr)

    # A guest that dies on a bad access takes a signal rather than exiting.
    # Report it plainly: the usual cause is a store outside the mapped data
    # window, which means the program is not using DATA_BASE correctly.
    if proc.returncode < 0:
        die("qemu terminated by signal %d while running %s.\n"
            "       Most often this is a store outside the DATA_BASE window "
            "(the .bss page qemu maps); check the program's addressing."
            % (-proc.returncode, os.path.basename(elf)))
    if proc.returncode != 0:
        die("qemu exited %d while running %s" % (proc.returncode, elf))


def parse_raw(raw_path):
    """Parse the plugin log into [(pc, [regs], [stores])] in execution order.

    Each returned tuple is a raw sample: the state observed BEFORE the
    instruction at pc ran, plus the stores performed since the previous sample
    (i.e. by the previous instruction).
    """
    samples = []
    pending = []
    capped = False
    n_reported = None

    with open(raw_path) as f:
        for line in f:
            if line.startswith("I "):
                parts = line.split()
                if len(parts) != 2 + NREG:
                    die("malformed I record in %s: %s" % (raw_path, line.strip()))
                pc = int(parts[1], 16)
                regs = [int(x, 16) for x in parts[2:]]
                samples.append((pc, regs, pending))
                pending = []
            elif line.startswith("S "):
                _, addr, size, val = line.split()
                pending.append((int(addr, 16), int(size), int(val, 16)))
            elif line.startswith("E "):
                n_reported = int(line.split()[1])
            elif "max-insn cap" in line:
                capped = True

    # Stores from the final instruction are flushed by the plugin's atexit
    # hook, after the last I record; attach them to a trailing sample so they
    # are not lost.
    if pending:
        samples.append((None, None, pending))

    return samples, capped, n_reported


def normalise(samples, trace_start, trace_end, text_reloc_mask=0, data_reloc_mask=0,
             data_base=None, text_reloc_events=None):
    """Shift raw samples into per-retired-instruction canonical records.

    Register normalisation
    -----------------------
    Only `pc` is inherently qemu/core-relocatable: the core always resets to
    pc=0, qemu is linked at a nonzero address (page 0 is unmappable), and the
    subtraction above undoes that for every record's own pc. But the SAME
    problem silently reappears whenever a register captures an absolute
    address: JAL/JALR write pc+4 into rd, AUIPC writes pc+imm. On the core
    side that value is trace-relative (small); on the qemu side it is a real
    qemu-absolute address - so a register holding such a value would never
    match the DUT even though both sides computed the architecturally correct
    result. This never showed up in asm/*.s (none of those programs use JALR
    or keep a JAL/AUIPC result), but it is not optional once a test exercises
    subroutine-call-style code.

    Crucially this cannot be a mask that is constant for the whole trace: a
    register only becomes address-relative from the instant some instruction
    actually writes a pc-derived value into it. Applying the offset to
    earlier records - where the same register still holds whatever plain
    value it was seeded with - corrupts a comparison that was supposed to be
    exact. (Concretely: `jal x2,...; ...; auipc x2,0` - the record produced by
    the jal must compare x2 raw; only the record produced by the auipc, and
    everything after it, needs the offset.)

    So the caller supplies:
      text_reloc_mask   registers already address-relative in record 0 (an
                         input seed that is itself a "distance from program
                         start", e.g. a JALR target loaded before the trace
                         began)
      text_reloc_events {byte-offset-from-trace_start: [registers]} - at the
                         record produced by the instruction at that offset,
                         each listed register newly becomes (and remains)
                         address-relative. Computed by the caller from a
                         static scan for JAL/JALR/AUIPC with rd!=0.
      data_reloc_mask   registers holding a "distance into the data window"
                         value throughout the WHOLE trace (e.g. a load/store
                         base register seeded relative to DATA_BASE and never
                         overwritten) - gets data_base subtracted, constant
                         for the whole trace. There is currently no data-side
                         equivalent of text_reloc_events because no register
                         in this repo's test suite is ever written with a
                         data-window address mid-trace; extend analogously to
                         text_reloc_events if that ever changes.
    All default to off (0 / empty).
    """
    if data_reloc_mask and data_base is None:
        die("--data-reloc-mask given without --data-base")
    text_reloc_events = text_reloc_events or {}

    records = []
    active_text_mask = text_reloc_mask
    for j in range(len(samples) - 1):
        pc_j = samples[j][0]
        if pc_j is None or not (trace_start <= pc_j < trace_end):
            continue
        rel = pc_j - trace_start
        if rel in text_reloc_events:
            for r in text_reloc_events[rel]:
                active_text_mask |= (1 << r)
        nxt_pc, nxt_regs, stores = samples[j + 1]
        if nxt_pc is None:
            # Final instruction of the program: its stores were flushed at
            # exit, and there is no following state sample. Reuse this
            # instruction's own successor address.
            die("trace ended immediately after an in-range instruction; the "
                "program appears to have exited without passing through "
                "CORE_HALT")
        if active_text_mask or data_reloc_mask:
            nxt_regs = list(nxt_regs)
            for r in range(32):
                if (active_text_mask >> r) & 1:
                    nxt_regs[r] = (nxt_regs[r] - trace_start) & 0xffffffff
                elif (data_reloc_mask >> r) & 1:
                    nxt_regs[r] = (nxt_regs[r] - data_base) & 0xffffffff
        records.append((nxt_pc - trace_start, nxt_regs, stores))
    return records


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", required=True, help="qemu-build ELF to run")
    ap.add_argument("--out", required=True, help="canonical trace to write")
    ap.add_argument("--raw", help="keep the plugin's raw log at this path")
    ap.add_argument("--qemu", default=DEFAULT_QEMU)
    ap.add_argument("--plugin", default=DEFAULT_PLUGIN)
    ap.add_argument("--nm", default=DEFAULT_NM)
    ap.add_argument("--data-base", type=lambda s: int(s, 0),
                    help="assert __data_base sits at this address")
    ap.add_argument("--text-reloc-mask", type=lambda s: int(s, 0), default=0,
                    help="registers (bitmask) already 'distance from this "
                         "program's start' valued in record 0 (see normalise())")
    ap.add_argument("--text-reloc-events", default="",
                    help="comma-separated offset:reg pairs (decimal or 0x-hex) - "
                         "register `reg` becomes text-relocated starting at the "
                         "record produced by the instruction at byte `offset` "
                         "from __trace_start, e.g. '0:1,8:2,16:3'")
    ap.add_argument("--data-reloc-mask", type=lambda s: int(s, 0), default=0,
                    help="registers (bitmask) holding a 'distance into the "
                         "data window' value - --data-base is subtracted from "
                         "them in every record; requires --data-base")
    ap.add_argument("--max", type=int, default=200000,
                    help="instruction cap inside the plugin (default 200000)")
    ap.add_argument("--timeout", type=int, default=120,
                    help="wall-clock limit for the qemu run (default 120s)")
    args = ap.parse_args()

    syms = symbols(args.nm, args.elf)
    for name in ("__trace_start", "__trace_end"):
        if name not in syms:
            die("%s does not define %s - was it built from a program using "
                "CORE_ENTRY/CORE_HALT from asm/core_defs.inc?"
                % (args.elf, name))
    trace_start = syms["__trace_start"]
    trace_end = syms["__trace_end"]

    # DATA_BASE is a compile-time constant in core_defs.inc but the .bss page
    # is placed by the linker; if the two ever drift apart the core and qemu
    # would be addressing different words, so check rather than assume.
    if args.data_base is not None:
        actual = syms.get("__data_base")
        if actual is None:
            die("%s does not define __data_base (missing CORE_DATA_REGION?)"
                % args.elf)
        if actual != args.data_base:
            die("__data_base is at 0x%x but DATA_BASE is 0x%x; the linker's "
                "-Tbss and core_defs.inc disagree" % (actual, args.data_base))

    raw_path = args.raw or (args.out + ".raw")
    run_qemu(args.qemu, args.plugin, args.elf, raw_path, args.max, args.timeout)

    samples, capped, n_reported = parse_raw(raw_path)
    if not samples:
        die("plugin produced no instruction records in %s" % raw_path)
    if capped:
        sys.stderr.write(
            "qemu_trace: warning: hit the --max instruction cap; the trace is "
            "truncated and the comparison will stop early rather than reach "
            "CORE_HALT\n")

    text_reloc_events = {}
    for tok in args.text_reloc_events.split(","):
        tok = tok.strip()
        if not tok:
            continue
        off_s, reg_s = tok.split(":")
        text_reloc_events.setdefault(int(off_s, 0), []).append(int(reg_s, 0))

    records = normalise(samples, trace_start, trace_end,
                        text_reloc_mask=args.text_reloc_mask,
                        data_reloc_mask=args.data_reloc_mask,
                        data_base=args.data_base,
                        text_reloc_events=text_reloc_events)
    if not records:
        die("no instructions fell inside [__trace_start, __trace_end) = "
            "[0x%x, 0x%x); the compared region is empty"
            % (trace_start, trace_end))

    with open(args.out, "w") as f:
        f.write("%d\n" % len(records))
        for pc, regs, stores in records:
            fields = ["%08x" % pc]
            fields += ["%08x" % r for r in regs]
            fields.append("%x" % len(stores))
            for addr, size, val in stores:
                fields += ["%08x" % addr, "%x" % size, "%08x" % val]
            f.write(" ".join(fields) + "\n")

    n_stores = sum(len(s) for _, _, s in records)
    print("qemu_trace: %s -> %s" % (os.path.basename(args.elf), args.out))
    print("qemu_trace: %d instruction(s) traced in "
          "[__trace_start=0x%x, __trace_end=0x%x), %d store(s); "
          "qemu executed %s in total"
          % (len(records), trace_start, trace_end, n_stores,
             n_reported if n_reported is not None else "?"))
    if not args.raw:
        os.remove(raw_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
