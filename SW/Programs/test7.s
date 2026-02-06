.section .text
.global _start
_start:
    # base = 0x10000000
    lui   x4, 0x10000

    # Basic immediates / shifts
    addi  x1, x0, 1
    slli  x2, x1, 8          # x2 = 0x00000100
    srli  x3, x2, 4          # x3 = 0x00000010
    addi  x5, x0, -16        # x5 = 0xFFFFFFF0
    srai  x6, x5, 2          # x6 = 0xFFFFFFFC

    # Logical immediates
    ori   x7, x0, 0x55       # x7 = 0x55
    xori  x7, x7, 0x0F       # x7 = 0x5A
    andi  x7, x7, 0x3F       # x7 = 0x1A

    # slti/sltiu
    slti  x8, x5, 0          # -16 < 0 => 1
    sltiu x9, x5, 1          # 0xFFFFFFF0 < 1 (unsigned) => 0

    # R-type mix
    add   x10, x3, x7        # 0x10 + 0x1A = 0x2A
    sub   x11, x10, x1       # 0x29
    sll   x12, x1, x3        # 1 << 0x10 => shift masked => 1<<16 = 0x00010000 (depends on your mask correctness)
    slt   x13, x5, x1        # -16 < 1 => 1
    sltu  x14, x5, x1        # 0xFFFFFFF0 < 1 unsigned => 0
    xor   x15, x11, x3       # 0x29 ^ 0x10 = 0x39
    srl   x16, x15, x1       # 0x39 >> 1 = 0x1C
    sra   x17, x6, x1        # 0xFFFFFFFC >> 1 arith = 0xFFFFFFFE
    or    x18, x16, x3       # 0x1C | 0x10 = 0x1C
    and   x19, x18, x7       # 0x1C & 0x1A = 0x18

    # Store bytes/halves/words at base
    sb    x7,  0(x4)         # [0] = 0x1A
    sh    x19, 2(x4)         # [2..3] = 0x0018
    sw    x6,  4(x4)         # [4..7] = 0xFFFFFFFC

    # Load back (sign/zero tests)
    lb    x20, 0(x4)         # x20 = 0x0000001A
    lbu   x21, 0(x4)         # x21 = 0x0000001A
    lh    x22, 2(x4)         # x22 = 0x00000018
    lhu   x23, 2(x4)         # x23 = 0x00000018
    lw    x24, 4(x4)         # x24 = 0xFFFFFFFC

    # jal / jalr test: call func via jal, return via jalr
    jal   x1, func
after:
    # signature = x25 written by func
    sw    x25, 0x10(x4)      # signature at 0x10000010
halt:
    jal   x0, halt

func:
    # auipc just to exercise it (result unused)
    auipc x26, 0

    # compute signature in x25 (sum of a few key regs)
    # x20=0x1A, x22=0x18, x24=0xFFFFFFFC
    add   x25, x20, x22      # 0x32
    add   x25, x25, x24      # 0x2E (since 0x32 + 0xFFFFFFFC = 0x2E)
    jalr  x0, x1, 0

