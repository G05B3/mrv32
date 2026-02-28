    .section .text
    .globl _start
_start:
    /* Set up stack pointer */
    la   sp, _stack_top

    /* Clear BSS: [__bss_start, __bss_end) */
    la   t0, __bss_start
    la   t1, __bss_end
1:
    bgeu t0, t1, 2f
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    1b
2:
    /* Call main() */
    jal  ra, main

    /* a0 = return code; write to tohost and halt */
    lui  t0, %hi(MRV_TOHOST)
    addi t0, t0, %lo(MRV_TOHOST)
    sw   a0, 0(t0)

3:  j 3b
