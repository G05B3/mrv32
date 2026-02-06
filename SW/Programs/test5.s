.section .text
.global _start
_start:
    lui  x4, 0x10000

    addi x1, x0, 10           # N = 10
    addi x2, x0, 0            # sum = 0

loop:
    add  x2, x2, x1           # sum += N
    addi x1, x1, -1           # N--
    bne  x1, x0, loop

    sw   x2, 0(x4)            # signature = 55
halt:
    j halt

