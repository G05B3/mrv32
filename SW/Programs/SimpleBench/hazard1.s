.section .text
.global _start
_start:
    addi x1, x0, 10
    addi x2, x0, 20
    add  x3, x1, x2    # MEM->EX on x1, EX->EX on x2
    add  x4, x3, x2    # EX->EX on x3, MEM->EX on x2
    sub  x5, x4, x3    # EX->EX on x4, MEM->EX on x3
halt:
    j halt
nop
nop
nop
nop