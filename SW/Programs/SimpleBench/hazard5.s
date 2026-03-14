.section .text
.global _start
_start:
    addi x1, x0, 1
    addi x2, x0, 1
    beq  x1, x2, taken    # should be taken, flush pipeline
    addi x3, x0, 0xFF     # must NOT execute
    addi x4, x0, 0xFF     # must NOT execute
taken:
    addi x3, x0, 0x42     # should execute with x3=0x42
    addi x4, x0, 0x24
halt:
    j halt