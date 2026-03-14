.section .text
.global _start
_start:

    addi x1, x0, 1

    lui  x4, 0x00001
    addi x5, x0, -16
    addi x0, x0, 0

    slli x2, x1, 8            # x2 = 0x00000100

    addi x0, x0, 0
    addi x0, x0, 0
    srai x6, x5, 2            # x6 = 0xFFFFFFFC (-4)

    srli x3, x2, 4            # x3 = 0x00000010    

    addi x0, x0, 0
    addi x0, x0, 0
    sw   x6, 4(x4)

    sw   x3, 0(x4)

    

halt:
    j halt