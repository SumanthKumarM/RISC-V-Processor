# 2x2 matrix multiply, C = A * B, exercising RV32M (mul) alongside loads/stores.
#
#   A = [1 2; 3 4]   (mem[0..12], row-major)
#   B = [5 6; 7 8]   (mem[16..28])
#   C = A*B          (mem[32..44])
#
# Expected: C = [19 22; 43 50]
#   C00 = 1*5+2*7=19  C01 = 1*6+2*8=22
#   C10 = 3*5+4*7=43  C11 = 3*6+4*8=50
#
# "mem[k]" means the dmem word DATA_BASE+k aliases onto (see core_defs.inc).
#
# run: make run_asm  PROG=matrix_mult   (core vs riscv_ref.v)
#      make run_qemu PROG=matrix_mult   (core vs riscv_ref.v AND qemu-riscv32)
#include "core_defs.inc"

    CORE_DATA_REGION
    CORE_ENTRY

    # ---- initialize A ----
    addi x1, x0, 1
    sw   x1, 0(DATA_REG)      # A00
    addi x1, x0, 2
    sw   x1, 4(DATA_REG)      # A01
    addi x1, x0, 3
    sw   x1, 8(DATA_REG)      # A10
    addi x1, x0, 4
    sw   x1, 12(DATA_REG)     # A11

    # ---- initialize B ----
    addi x1, x0, 5
    sw   x1, 16(DATA_REG)     # B00
    addi x1, x0, 6
    sw   x1, 20(DATA_REG)     # B01
    addi x1, x0, 7
    sw   x1, 24(DATA_REG)     # B10
    addi x1, x0, 8
    sw   x1, 28(DATA_REG)     # B11

    # ---- load operands ----
    lw   x2,  0(DATA_REG)     # A00
    lw   x3,  4(DATA_REG)     # A01
    lw   x4,  8(DATA_REG)     # A10
    lw   x5, 12(DATA_REG)     # A11
    lw   x6, 16(DATA_REG)     # B00
    lw   x7, 20(DATA_REG)     # B01
    lw   x8, 24(DATA_REG)     # B10
    lw   x9, 28(DATA_REG)     # B11

    # ---- C00 = A00*B00 + A01*B10 ----
    mul  x10, x2, x6
    mul  x11, x3, x8
    add  x10, x10, x11
    sw   x10, 32(DATA_REG)

    # ---- C01 = A00*B01 + A01*B11 ----
    mul  x10, x2, x7
    mul  x11, x3, x9
    add  x10, x10, x11
    sw   x10, 36(DATA_REG)

    # ---- C10 = A10*B00 + A11*B10 ----
    mul  x10, x4, x6
    mul  x11, x5, x8
    add  x10, x10, x11
    sw   x10, 40(DATA_REG)

    # ---- C11 = A10*B01 + A11*B11 ----
    mul  x10, x4, x7
    mul  x11, x5, x9
    add  x10, x10, x11
    sw   x10, 44(DATA_REG)

    CORE_HALT
