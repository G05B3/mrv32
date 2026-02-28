# TODO

Provide formal documentation for the considered ISA Subset, memory structure and map and bus protocol

## Considered Instructions

### R-Type
- add, sub
- and, or, xor
- sll, srl, sra
- slt, sltu

### I-Type
- addi
- andi, ori, xori
- slli, srli, srai
- slti, sltiu
- lb, lbu, lh, lhu, lw
- jalr

### S-Type
- sb, sh, sw

### B-Type
- beq, bne, blt, bltu, bge, bgeu

### U-Type
- lui, auipc

### J-Type
- jal

## Memory Address Map

Memory Address Map can be changed in SW/Link/memmap.mk [REQUIRES RECOMPILING WITH "make all" @ SW/ISS]

### RAM

0x00000000 - 0x00100000

### MMIOs

MRV_MMIO_BASE   := 0x10000000

### UART

MRV_UART_TX     := 0x10000000
MRV_UART_STATUS := 0x10000004

### Program end call

MRV_TOHOST      := 0x10000010