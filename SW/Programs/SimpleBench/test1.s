.section .text
.global _start
_start:
    li   x1, 10
    li   x2, 20
    add  x3, x1, x2

    lui  x4, 0x10000        # signature base = 0x10000000
    sw   x3, 0(x4)

halt:
    j halt