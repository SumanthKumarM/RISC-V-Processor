# Triple-nested loop with a conditional skip, exercising forward/backward
# jal branches and multiple beq comparisons per level.
#
#   for i in 0..3:
#     for j in 0..3:
#       if i == j: continue        # skip_j corner: conditional skip of the
#                                   # entire inner loop, not just one iter
#       for k in 0..3:
#         sum += i+j+k
#         count += 1
#
# i!=j pairs: 4*4-4=12, each contributing 4 k-iterations -> count = 48.
# mem[0] = sum, mem[4] = count(=48).
#
# run: make run_asm PROG=nested_loops   (from RISC-V-Processor/sim)
.text
.globl _start
_start:
    addi x1,  x0, 0     # i
    addi x2,  x0, 4      # I_MAX = J_MAX = K_MAX
    addi x10, x0, 0       # sum
    addi x11, x0, 0       # count

outer_i:
    beq  x1, x2, done_i
    addi x3, x0, 0         # j = 0
outer_j:
    beq  x3, x2, next_i
    beq  x1, x3, skip_j     # skip the whole k-loop when i == j
    addi x4, x0, 0            # k = 0
inner_k:
    beq  x4, x2, next_j
    add  x5, x1, x3
    add  x5, x5, x4
    add  x10, x10, x5          # sum += i+j+k
    addi x11, x11, 1            # count++
    addi x4, x4, 1               # k++
    jal  x0, inner_k
skip_j:
next_j:
    addi x3, x3, 1
    jal  x0, outer_j
next_i:
    addi x1, x1, 1
    jal  x0, outer_i
done_i:
    sw   x10, 0(x0)     # sum
    sw   x11, 4(x0)     # count

halt:
    jal x0, halt
