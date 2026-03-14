.section .text
.global _start
_start:
    jal  x1, target       # x1 = 0x04, jumps to target
    addi x2, x1, 0        # in pipeline behind JAL, MEM->EX forward of x1
target:
    addi x3, x1, 0        # x1 should be 0x04
    jalr x4, x1, 0        # jump to x1=0x04, x4=pc+4
    nop
    nop
    nop
    nop
    nop
halt:
    j halt
nop
nop
nop