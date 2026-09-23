.section .text
.global _start
_start:
    beq  x0, x0, label
    addi x7, x0, 0x42
    j halt
label:
    addi x7, x0, 0xFF
halt:
    j halt