.section .text
.global _start
_start:
    lui  x4, 0x10000

    addi x1, x0, 1
    slli x2, x1, 8            # x2 = 0x00000100
    srli x3, x2, 4            # x3 = 0x00000010

    addi x5, x0, -16          # x5 = 0xFFFFFFF0
    srai x6, x5, 2            # x6 = 0xFFFFFFFC (-4)

    # store two words
    sw   x3, 0(x4)            # 0x10
    sw   x6, 4(x4)            # 0xFFFFFFFC
halt:
    j halt

