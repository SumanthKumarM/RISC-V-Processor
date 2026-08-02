# Sweep all six branch types (BEQ, BNE, BLT, BGE, BLTU, BGEU) plus SLT/SLTU,
# hitting the classic signed-vs-unsigned divergence corners that a shared
# comparator (as opposed to a proper signed/unsigned adder) tends to get
# wrong: -1 vs 1 and INT_MIN vs INT_MAX.
#
#   x20 = taken-count: incremented once per branch that WAS taken.
#         11 of the 12 branches below are taken -> expect x20 == 11.
#   mem[0] = x20, mem[4] = x21 (SLT/SLTU scratch, see corner_test below),
#         where "mem[k]" is the dmem word DATA_BASE+k aliases onto - see
#         core_defs.inc for why the data window had to move off address 0.
#
# run: make run_asm  PROG=branch_sweep   (core vs riscv_ref.v)
#      make run_qemu PROG=branch_sweep   (core vs riscv_ref.v AND qemu-riscv32)
#include "core_defs.inc"

    CORE_DATA_REGION
    CORE_ENTRY
    addi x1, x0, 5              # equal operands
    addi x2, x0, 5
    addi x3, x0, 7              # x3 > x1 (both positive small)
    addi x4, x0, -1              # 0xFFFFFFFF: max unsigned, min-ish signed
    addi x5, x0, 1                 # small positive
    lui  x6, 0x80000                 # INT_MIN = 0x80000000
    addi x7, x6, -1                    # INT_MAX = 0x7FFFFFFF (min - 1, wraps)
    addi x20, x0, 0                     # taken-count
    addi x21, x0, 0                      # scratch result word

    # ---- BEQ: equal -> taken ----
    beq  x1, x2, t_beq
    jal  x0, f_beq
t_beq: addi x20, x20, 1
f_beq:

    # ---- BNE: unequal -> taken ----
    bne  x1, x3, t_bne
    jal  x0, f_bne
t_bne: addi x20, x20, 1
f_bne:

    # ---- BEQ: unequal -> NOT taken (the one branch we expect to skip) ----
    beq  x1, x3, f_beq2
    jal  x0, t_beq2
f_beq2: addi x20, x20, 999    # would corrupt the count if wrongly taken
t_beq2:

    # ---- BLT signed: -1 < 1 -> taken (this is the corner: unsigned says NO) ----
    blt  x4, x5, t_blt1
    jal  x0, f_blt1
t_blt1: addi x20, x20, 1
f_blt1:

    # ---- BLTU unsigned: 0xFFFFFFFF < 1 -> NOT taken (opposite of BLT above) ----
    bltu x4, x5, f_bltu1
    jal  x0, t_bltu1
f_bltu1: addi x20, x20, 999
t_bltu1: addi x20, x20, 1

    # ---- BGE signed: 1 >= -1 -> taken ----
    bge  x5, x4, t_bge1
    jal  x0, f_bge1
t_bge1: addi x20, x20, 1
f_bge1:

    # ---- BGEU unsigned: 1 >= 0xFFFFFFFF -> NOT taken (opposite of BGE above) ----
    bgeu x5, x4, f_bgeu1
    jal  x0, t_bgeu1
f_bgeu1: addi x20, x20, 999
t_bgeu1: addi x20, x20, 1

    # ---- BLT signed: INT_MIN < INT_MAX -> taken ----
    blt  x6, x7, t_blt2
    jal  x0, f_blt2
t_blt2: addi x20, x20, 1
f_blt2:

    # ---- BLTU unsigned: 0x80000000 < 0x7FFFFFFF -> NOT taken (0x80000000 is
    # the larger *unsigned* value, opposite of the signed comparison above) ----
    bltu x6, x7, f_blt3
    jal  x0, t_blt3
f_blt3: addi x20, x20, 999
t_blt3: addi x20, x20, 1

    # ---- BGE signed: INT_MAX >= INT_MIN -> taken ----
    bge  x7, x6, t_bge2
    jal  x0, f_bge2
t_bge2: addi x20, x20, 1
f_bge2:

    # ---- BGEU unsigned: 0x7FFFFFFF >= 0x80000000 -> NOT taken ----
    bgeu x7, x6, f_bge3
    jal  x0, t_bge3
f_bge3: addi x20, x20, 999
t_bge3: addi x20, x20, 1

    # ---- SLT/SLTU on the same -1-vs-1 corner, mirroring BLT/BLTU above ----
    slt  x21, x4, x5           # signed: -1 < 1 -> 1
    sltu x22, x4, x5           # unsigned: 0xFFFFFFFF < 1 -> 0
    add  x21, x21, x22         # expect x21 == 1 (1+0)

    # ---- BEQ zero-register corner: x0 vs x0 -> always taken ----
    beq  x0, x0, t_beqz
    jal  x0, f_beqz
t_beqz: addi x20, x20, 1
f_beqz:

    sw   x20, 0(DATA_REG)     # taken-count, expect 11
    sw   x21, 4(DATA_REG)     # slt+sltu sum, expect 1

    CORE_HALT
