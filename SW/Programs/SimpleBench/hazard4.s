.section .text
.global _start
_start:
    addi x1, x0, 0x40    # x1 = 64 = base address
    addi x2, x0, 0xAB
    sw   x2, 0(x1)        # mem[64] = 0xAB
    lw   x3, 0(x1)        # x3 = mem[64] = 0xAB, load-use stall
    add  x4, x3, x3       # RAW on x3, needs stall + forward
    addi x5, x3, 1        # RAW on x3 via MEM->EX forward
halt:
    j halt
nop
nop
nop
nop