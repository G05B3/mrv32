.section .text
.global _start
_start:
    addi x1, x0, 1
    addi x2, x0, 2
    add  x3, x1, x2        # x3=3, EX->EX on x2, MEM->EX on x1
    add  x3, x3, x3        # x3=6, EX->EX on x3
    add  x3, x3, x3        # x3=12, EX->EX on x3
    addi x4, x0, 0x100
    sw   x3, 0(x4)         # mem[0x100]=12
    lw   x5, 0(x4)         # x5=12, load-use stall
    add  x6, x5, x3        # x6=24, RAW on x5 and x3
    bne  x6, x0, skip      # taken, flush pipeline
    addi x6, x0, 0xFF      # must NOT execute
skip:
    jal  x7, done          # x7=pc+4
    addi x8, x0, 0xFF      # must NOT execute
done:
    add  x8, x6, x5        # x8=24+12=36
halt:
    j halt