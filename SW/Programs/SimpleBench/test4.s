.section .text
.global _start
_start:
    lui  x4, 0x10000

    addi x1, x0, 5
    addi x2, x0, 5
    beq  x1, x2, equal

    # not taken path
    addi x3, x0, 0x111
    j done

equal:
    addi x3, x0, 0x222

done:
    sw   x3, 0(x4)            # signature
halt:
    j halt

