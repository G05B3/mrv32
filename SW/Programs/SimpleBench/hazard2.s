.section .text
.global _start
_start:
    lui   x1, 1        # x1 = 0x00001000
    add   x2, x1, x0  # MEM->EX on x1
    auipc x3, 0        # x3 = pc = 0x08
    add   x4, x3, x0  # MEM->EX on x3
halt:
    j halt
nop
nop
nop
nop