# Array reduction over a mix of positive/negative values: sum, running max
# (via BLT) and a negative-count (via SLT + branch), pointer-walked with LW
# instead of the beq-only counters the earlier examples used.
#
#   array[8] @ mem[32..60] = { 5, -3, 12, -7, 100, -100, 1, -1 }
#   sum = 7          (5-3+12-7+100-100+1-1)
#   max = 100         (BLT-based running max)
#   neg_count = 4      (SLT rd,val,0 -> 1 iff val<0, then branch on rd)
#
# mem[0]=sum, mem[4]=max, mem[8]=neg_count.
#
# run: make run_asm PROG=array_sum   (from RISC-V-Processor/sim)
.text
.globl _start
_start:
    addi x1, x0, 8            # N
    addi x2, x0, 0             # i = 0
    addi x3, x0, 32             # array base address
    addi x4, x3, 0                # addr = base, walks by +4
    addi x5, x0, 0                 # sum
    lui  x10, 0x80000                # max = INT_MIN (running max seed)
    addi x11, x0, 0                   # neg_count

    # ---- populate the array ----
    addi x6, x0, 5
    sw   x6, 0(x3)
    addi x6, x0, -3
    sw   x6, 4(x3)
    addi x6, x0, 12
    sw   x6, 8(x3)
    addi x6, x0, -7
    sw   x6, 12(x3)
    addi x6, x0, 100
    sw   x6, 16(x3)
    addi x6, x0, -100
    sw   x6, 20(x3)
    addi x6, x0, 1
    sw   x6, 24(x3)
    addi x6, x0, -1
    sw   x6, 28(x3)

sum_loop:
    bge  x2, x1, sum_done       # i >= N -> done
    lw   x6, 0(x4)               # val = array[i]
    add  x5, x5, x6                # sum += val

    blt  x10, x6, new_max            # max < val -> update
    jal  x0, after_max
new_max:
    add  x10, x6, x0
after_max:

    slt  x7, x6, x0                  # x7 = 1 iff val < 0 (SLT)
    beq  x7, x0, after_neg
    addi x11, x11, 1
after_neg:

    addi x4, x4, 4
    addi x2, x2, 1
    jal  x0, sum_loop

sum_done:
    sw   x5,  0(x0)      # sum
    sw   x10, 4(x0)      # max
    sw   x11, 8(x0)      # neg_count

halt:
    jal x0, halt
