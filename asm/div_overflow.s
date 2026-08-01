# DIV/DIVU/REM/REMU edge cases: divide-by-zero (spec-mandated results, not
# traps -- this core has no trap infra) and the one signed-overflow case in
# RV32M, INT_MIN / -1. Also covers DIVU/REMU treating a negative bit pattern
# as a large unsigned value, which a naive divider (reusing the signed path)
# tends to get wrong.
#
# Spec results used below (RISC-V unprivileged spec, table for M-extension):
#   x / 0   (DIV)  = -1 (all ones)      x % 0  (REM)  = x
#   x / 0   (DIVU) = 0xFFFFFFFF          x % 0  (REMU) = x
#   INT_MIN / -1 (DIV) = INT_MIN (overflow, no trap)
#   INT_MIN % -1 (REM) = 0
#
# mem[0..60] holds the 16 results below, in order.
#
# run: make run_asm PROG=div_overflow   (from RISC-V-Processor/sim)
.text
.globl _start
_start:
    addi x1, x0, 17          # dividend, positive
    addi x2, x0, 5            # divisor, positive
    addi x3, x0, -17           # dividend, negative
    addi x5, x0, 0               # zero divisor
    lui  x6, 0x80000               # INT_MIN = 0x80000000
    addi x7, x0, -1                 # -1
    addi x8, x6, -1                  # INT_MAX = 0x7FFFFFFF

    div  x10, x1, x2      # 17 / 5 = 3
    rem  x11, x1, x2      # 17 % 5 = 2
    div  x12, x3, x2      # -17 / 5 = -3 (truncate toward zero)
    rem  x13, x3, x2      # -17 % 5 = -2

    div  x14, x1, x5      # 17 / 0 -> -1 (0xFFFFFFFF)
    rem  x15, x1, x5      # 17 % 0 -> 17
    divu x16, x1, x5      # 17 /u 0 -> 0xFFFFFFFF
    remu x17, x1, x5      # 17 %u 0 -> 17

    div  x18, x6, x7      # INT_MIN / -1 -> INT_MIN (overflow, no trap)
    rem  x19, x6, x7      # INT_MIN % -1 -> 0 (overflow)

    divu x20, x6, x2      # 0x80000000 /u 5 = 429496729
    remu x21, x6, x2      # 0x80000000 %u 5 = 3

    divu x22, x3, x2      # (-17 as unsigned 0xFFFFFFEF) /u 5 = 858993455
    remu x23, x3, x2      # (-17 as unsigned)            %u 5 = 4

    div  x24, x8, x7      # INT_MAX / -1 = -INT_MAX = 0x80000001 (no overflow)
    rem  x25, x8, x7      # INT_MAX % -1 = 0

    sw   x10, 0(x0)
    sw   x11, 4(x0)
    sw   x12, 8(x0)
    sw   x13, 12(x0)
    sw   x14, 16(x0)
    sw   x15, 20(x0)
    sw   x16, 24(x0)
    sw   x17, 28(x0)
    sw   x18, 32(x0)
    sw   x19, 36(x0)
    sw   x20, 40(x0)
    sw   x21, 44(x0)
    sw   x22, 48(x0)
    sw   x23, 52(x0)
    sw   x24, 56(x0)
    sw   x25, 60(x0)

halt:
    jal x0, halt
