# MRV32

MRV32 is a minimal RV32I-based RISC-V processor project built from the ground up using open-source tools.

The project explores the complete hardware development flow, from architectural modeling to RTL implementation and physical design.

---

## Project Overview

MRV32 currently consists of two primary components:

1. **Instruction Set Simulator (ISS)**

   A functional reference model of the RV32I ISA used for architectural validation and differential testing.

2. **SystemVerilog RTL Implementation**

   A synthesizable hardware implementation of the processor, developed incrementally from a serialized bring-up core into a fully pipelined design.

---

## Current Status

### Instruction Set Simulator (ISS)

- RV32I functional simulator (excluding FENCE, ECALL, EBREAK, and CSRs)
- Used as architectural golden reference
- Supports memory-mapped I/O
- Includes a custom `mrv_printf()` implementation for bare-metal software
- Supports configurable instruction tracing, memory/register-file dumps, and execution statistics

### RTL v1.0 – Serialized Core

- Single instruction in flight
- Structured IF/ID/EX/MEM/WB stage registers
- Blocking memory transactions
- Branch and jump resolution in the MEM stage
- Provides the baseline implementation for architectural and RTL bring-up

### RTL v1.1 – Fully Pipelined Core

- True 5-stage in-order pipeline: IF / ID / EX / MEM / WB
- Multiple instructions in flight
- Hazard detection for data dependencies
- Forwarding paths from later pipeline stages to EX
- Load-use hazard detection and pipeline stalling
- Pipeline flushing following taken branches and jumps
- Branch resolution remains in the MEM stage
- Retains the blocking LSU and memory interface used by the bring-up implementation

The v1.1 core has been validated against the current MRVBench suite (`vadd`, `memcpy`, and `dotprod`) and achieves an approximately 3× speedup over v1.0 on these benchmarks.

---

## Roadmap

### Version 2.0

- Memory hierarchy and cache integration
- Further pipeline and control-flow optimization
- Improved memory-system performance
- Expanded performance characterization

---

## Long-Term Goals

- Memory hierarchy refinement
- Memory-mapped I/O expansion
- Accelerator system integration (e.g. CGRA or systolic array)
- Synthesis and ASIC-style place-and-route for a complete chip

---

## Toolchain

- SystemVerilog RTL
- Open-source simulation and synthesis tools
- Custom C++ instruction set simulator
- ASIC-oriented synthesis and physical-design flow
- OpenROAD-based place-and-route

---

## Purpose

MRV32 is primarily a learning and exploration project in:

- Computer Architecture
- Microarchitecture design
- RTL development
- Verification methodology
- Hardware implementation flow

Performance is secondary to architectural clarity and design discipline.