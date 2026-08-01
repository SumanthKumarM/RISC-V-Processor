# Iterative Fibonacci: compute the first N terms and store them to memory,
# leaving fib(N) itself in x2 when the loop exits.
#
#   x1 = N (term count)      x2 = a (running fib value, stored each iter)
#   x3 = b (next fib value)  x4 = i (loop counter)
#   x5 = byte address, advances by 4 each iteration
#
# N=10 -> mem[0..36] = 0,1,1,2,3,5,8,13,21,34 and x2 = fib(10) = 55 on exit.
#
# run: make run_asm PROG=fibonacci   (from RISC-V-Processor/sim)
.text
.globl _start
_start:
    addi x1, x0, 10        # N
    addi x2, x0, 0         # a = fib(0)
    addi x3, x0, 1         # b = fib(1)
    addi x4, x0, 0         # i = 0
    addi x5, x0, 0         # addr = 0

fib_loop:
    beq  x4, x1, fib_done
    sw   x2, 0(x5)         # mem[i] = a
    add  x6, x2, x3        # next = a + b
    add  x2, x0, x3        # a = b
    add  x3, x0, x6        # b = next
    addi x4, x4, 1         # i++
    addi x5, x5, 4         # addr += 4
    jal  x0, fib_loop

fib_done:
halt:
    jal x0, halt
