.section .text
.global _start
_start:
    lui  x4, 0x10000          # base = 0x10000000

    addi x1, x0, 123
    sw   x1, 8(x4)            # write 123 at base+8
    lw   x2, 8(x4)            # read back
    addi x2, x2, 1            # 124
    sw   x2, 0(x4)            # signature = 124
halt:
    j halt

