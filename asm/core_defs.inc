/*
 * core_defs.inc - shared definitions for programs that must run BOTH on the
 * RTL core (rtl/risc.v, via `make run_asm`) and under qemu-riscv32 (via
 * `make run_qemu`).
 *
 * One source, two builds. The build is selected by -DQEMU_BUILD=1; everything
 * that has to differ between the two targets is confined to this file, so the
 * program bodies stay identical and a divergence can never be blamed on the
 * two sides having been assembled from different logic.
 *
 * Assembled with `gcc -x assembler-with-cpp`, so cpp conditionals work while
 * ordinary '#' RISC-V comments still pass through untouched.
 *
 * One pitfall that comes with that: a comment whose first word happens to be a
 * cpp directive IS a cpp directive. A line like
 *     #       if i == j: continue
 * in a pseudocode comment block is read as a real #if and fails the build
 * ("unterminated #if"). Prefix such lines with a marker - nested_loops.s uses
 * "#   | " - so the '#' is never directly followed by if/else/endif/define.
 *
 * ---------------------------------------------------------------------------
 * Why the data region cannot live at address 0 any more
 * ---------------------------------------------------------------------------
 * The core has no MMU: data_mem (rtl/risc.v) is 64 words indexed by
 * addr[7:2], so every address aliases into a 256-byte window and address 0 is
 * a perfectly good place to store things.
 *
 * qemu-riscv32 is a Linux user-mode emulator, and Linux will not map page 0
 * (vm.mmap_min_addr is 65536 on this machine, and `-R` does not relax it), so
 * a program that stores to address 0 takes SIGSEGV before it retires its
 * first interesting instruction.
 *
 * The fix is to move the data region up to DATA_BASE, which is
 *   - a real mapped .bss page under qemu, and
 *   - transparent to the core, because DATA_BASE[7:2] == 0, so DATA_BASE+k
 *     lands on exactly the same dmem word that plain k used to.
 * Core-side dmem contents are therefore unchanged by the port; only the
 * registers that hold addresses now carry DATA_BASE+k instead of k.
 *
 * DATA_BASE must stay identical in BOTH builds (it is a compile-time constant
 * here, and the linker is told -Tbss=DATA_BASE so the .bss page actually
 * lands there), otherwise address-holding registers would differ between the
 * core and qemu and every comparison would report a false divergence.
 */

#ifndef CORE_DEFS_INC
#define CORE_DEFS_INC

#ifndef QEMU_BUILD
#define QEMU_BUILD 0
#endif

/* Data window base. Requirements:
 *   - >= 0x10000            so qemu can map it (vm.mmap_min_addr)
 *   - (DATA_BASE & 0xFF)==0 so it maps onto dmem word 0, keeping every
 *                           program's existing byte offsets pointing at the
 *                           same dmem words they always did
 *   - low 12 bits zero      so `lui` alone materialises it (one instruction,
 *                           identical encoding in both builds)
 * Kept in sync with -Wl,-Tbss= in sim/Makefile; the build asserts they agree.
 */
#define DATA_BASE 0x12000
#define DATA_SIZE 256          /* == 64 words == the core's whole data_mem */

/* Register holding DATA_BASE throughout a program. x31 is free in all of the
 * example programs, so the port only had to rewrite `k(x0)` as `k(DATA_REG)`.
 */
#define DATA_REG x31

/*
 * CORE_DATA_REGION - reserve the .bss window that DATA_BASE points at.
 *
 * Only qemu needs this (it is what makes the page exist); the core build
 * carries it harmlessly, since `objcopy -j .text` drops .bss entirely and the
 * .mem image is unaffected.
 */
.macro CORE_DATA_REGION
    .bss
    .balign 16
    .globl __data_base
__data_base:
    .space DATA_SIZE
    .text
.endm

/*
 * CORE_ENTRY - program entry point, and the start of the compared region.
 *
 * qemu hands a fresh process a non-zero sp (and may seed other argument
 * registers), whereas the core comes out of reset with all 32 registers at
 * zero. Left alone that single difference would make every subsequent
 * comparison mismatch, so the qemu build zeroes the file first. Those
 * zeroing instructions are NOT part of the comparison: __trace_start marks
 * where the two sides become architecturally identical, and qemu_trace.py
 * discards every record before it.
 *
 * The core build needs no prologue (reset already zeroed everything), which
 * is why __trace_start sits at offset 0 of the core's .text and the PC
 * normalisation in qemu_trace.py is a simple subtraction of __trace_start.
 */
.macro CORE_ENTRY
    .text
    .globl _start
_start:
#if QEMU_BUILD
    addi x1,x0,0;  addi x2,x0,0;  addi x3,x0,0;  addi x4,x0,0
    addi x5,x0,0;  addi x6,x0,0;  addi x7,x0,0;  addi x8,x0,0
    addi x9,x0,0;  addi x10,x0,0; addi x11,x0,0; addi x12,x0,0
    addi x13,x0,0; addi x14,x0,0; addi x15,x0,0; addi x16,x0,0
    addi x17,x0,0; addi x18,x0,0; addi x19,x0,0; addi x20,x0,0
    addi x21,x0,0; addi x22,x0,0; addi x23,x0,0; addi x24,x0,0
    addi x25,x0,0; addi x26,x0,0; addi x27,x0,0; addi x28,x0,0
    addi x29,x0,0; addi x30,x0,0; addi x31,x0,0
#endif
    .globl __trace_start
__trace_start:
    /* Explicit lui rather than the `li` pseudo-op: DATA_BASE's low 12 bits
     * are zero, so this is exactly one instruction with an encoding that is
     * bit-identical in both builds. */
    lui DATA_REG, (DATA_BASE >> 12)
.endm

/*
 * CORE_HALT - end of program, and the end of the compared region.
 *
 * The core has no way to stop, so it parks in a self-loop; risc_tb_asm.v
 * detects that (next pc == the pc the instruction was fetched from) and ends
 * the run. Under qemu a self-loop would simply spin forever, so the qemu
 * build exits via the Linux exit syscall instead.
 *
 * Those two tails are different instruction sequences, so neither is
 * compared: __trace_end marks the cut, and qemu_trace.py drops every record
 * from there on. Everything before it is identical code on both sides.
 */
.macro CORE_HALT
    .globl __trace_end
__trace_end:
#if QEMU_BUILD
    addi x17, x0, 93        # __NR_exit
    addi x10, x0, 0         # status = 0
    ecall
#else
1:  jal x0, 1b
#endif
.endm

#endif /* CORE_DEFS_INC */
