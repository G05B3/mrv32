.section .text
.global _start
_start:
    li   x1, 10
    li   x2, 20

    lui  x4, 0x00001        # signature base = 0x1000
    addi x0, x0, 0
    addi x0, x0, 0

    add  x3, x1, x2

    addi x0, x0, 0
    addi x0, x0, 0
    addi x0, x0, 0

    sw   x3, 0(x4)

halt:
    j halt