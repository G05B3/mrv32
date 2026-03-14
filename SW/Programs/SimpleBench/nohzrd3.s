.section .text
.global _start
_start:
    lui  x4, 0x00001          # base = 0x1000

    addi x1, x0, 123

    addi x0, x0, 0
    addi x0, x0, 0
    addi x0, x0, 0

    sw   x1, 8(x4)            # write 123

    addi x0, x0, 0
    addi x0, x0, 0
    addi x0, x0, 0

    lw   x2, 8(x4)            # read back

    addi x0, x0, 0
    addi x0, x0, 0
    addi x0, x0, 0

    addi x2, x2, 1            # 124

    addi x0, x0, 0
    addi x0, x0, 0
    addi x0, x0, 0

    sw   x2, 0(x4)            # signature = 124

halt:
    j halt
    addi x0, x0, 0
    addi x0, x0, 0
    addi x0, x0, 0
    
    addi x0, x0, 0