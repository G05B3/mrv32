.section .text
.global _start
_start:
    lui  x4, 0x10000

    # call func: jalr x1, x5, 0 where x5 has func address
    la   x5, func             # pseudo-op (assembler expands)
    jalr x1, x5, 0            # x1 = return addr, jump to func

    # returned here
    sw   x6, 0(x4)            # signature written by func
halt:
    j halt

func:
    addi x6, x0, 77
    jalr x0, x1, 0            # return

