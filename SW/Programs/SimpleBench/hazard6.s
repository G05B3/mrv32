.section .text
.global _start
_start:
    addi x1, x0, 0x80     # base addr
    addi x2, x0, 5
    addi x3, x0, 10
    sw   x2, 0(x1)         # mem[0x80] = 5
    sw   x3, 4(x1)         # mem[0x84] = 10
    lw   x4, 0(x1)         # x4 = 5, load-use stall
    lw   x5, 4(x1)         # x5 = 10, load-use stall
    add  x6, x4, x5        # RAW on x4(MEM->EX) and x5(EX->EX)
    beq  x6, x0, wrong     # should NOT be taken (x6=15)
    addi x7, x0, 0x42
    j halt
wrong:
    addi x7, x0, 0xFF      # must NOT execute
halt:
    j halt