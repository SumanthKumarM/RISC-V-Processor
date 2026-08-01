# Sub-word load/store corner cases: per-byte stores into one word, sign vs
# zero extension for LB/LBU and LH/LHU at their sign-bit boundaries, and
# negative-immediate addressing for both store and load.
#
# run: make run_asm PROG=load_store_corners   (from RISC-V-Processor/sim)
.text
.globl _start
_start:
    addi x1, x0, 0          # base pointer, byte 0

    # ---- four SBs into one word, confirm neighbouring bytes survive ----
    addi x2, x0, 0x11
    sb   x2, 0(x1)
    addi x2, x0, 0x22
    sb   x2, 1(x1)
    addi x2, x0, 0x33
    sb   x2, 2(x1)
    addi x2, x0, 0x44
    sb   x2, 3(x1)
    lw   x3, 0(x1)          # expect 0x44332211

    # ---- byte sign-extension corners: 0x80 (negative) / 0x7F (positive) ----
    addi x4, x0, 0x80
    sb   x4, 4(x1)
    lb   x5, 4(x1)          # expect 0xFFFFFF80 (sign-extended)
    lbu  x6, 4(x1)          # expect 0x00000080 (zero-extended)

    addi x7, x0, 0x7F
    sb   x7, 5(x1)
    lb   x8, 5(x1)          # expect 0x0000007F

    # ---- halfword sign-extension corners: 0x8000 / 0x7FFF ----
    lui  x9, 0x8             # 0x8000
    sh   x9, 8(x1)
    lh   x10, 8(x1)          # expect 0xFFFF8000 (sign-extended)
    lhu  x11, 8(x1)          # expect 0x00008000 (zero-extended)

    lui  x12, 0x8
    addi x12, x12, -1         # 0x7FFF
    sh   x12, 10(x1)
    lh   x13, 10(x1)          # expect 0x00007FFF
    lhu  x14, 10(x1)          # expect 0x00007FFF

    # ---- negative-offset addressing (immS / immI sign extension) ----
    addi x15, x0, 100          # base = byte 100
    addi x16, x0, 0x5AD
    sw   x16, -4(x15)           # store at byte 96
    lw   x17, -4(x15)           # load back, expect 0x5AD
    sw   x16, -36(x15)           # store at byte 64
    lw   x18, -36(x15)           # load back, expect 0x5AD

halt:
    jal x0, halt
