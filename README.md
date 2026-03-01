# MRV32

MRV32 is a minimal RV32I-based RISC-V processor project built from the ground up using open-source tools.  
The project explores the complete hardware development flow, from architectural modeling to RTL implementation and physical design.

---

## Project Overview

MRV32 currently consists of two primary components:

1. **Instruction Set Simulator (ISS)**  
   A functional reference model of the RV32I ISA used for architectural validation and differential testing.

2. **SystemVerilog RTL Implementation**  
   A synthesizable hardware implementation of the processor, developed incrementally from a serialized bring-up core toward a fully pipelined design.

---

## Current Status

### Instruction Set Simulator (ISS)
- RV32I functional simulator (excluding FENCE, ECALL, EBREAK, CSRs)
- Used as architectural golden reference
- Supports memory mapped IOs, with a custom printf implementation

### RTL v1.0 – Serialized Core
- Single instruction in flight
- Structured IF/ID/EX/MEM/WB stage registers
- Branch resolution in MEM stage

This version behaves like a multi-cycle processor while maintaining a pipelined structure to enable future upgrades.

---

## Roadmap

### Version 2.0
- True 5-stage pipelined core
- Hazard detection unit
- Forwarding paths
- Pipeline flush on branch
- Memory hierarchy (Caches)
- Improved IPC

---

## Long-Term Goals
- Memory hierarchy refinement
- Memory mapped IO expansion
- Accelerator system integration (CGRA)
- Synthesis and ASIC-style place-and-route for a complete chip

---

## Toolchain

- SystemVerilog RTL
- Open-source simulation tools (iverilog, yosys, openROAD)
- Custom ISS for differential testing
- ASIC-oriented synthesis and PnR flow

---

## Purpose

MRV32 is primarily a learning and exploration project in:

- Computer Architecture
- Microarchitecture design
- RTL development
- Verification methodology
- Hardware implementation flow

Performance is secondary to architectural clarity and design discipline.