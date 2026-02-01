# MRV32
A very minimal RISC-V 32b CPU design equipped with a simple memory systems and memory mapped peripherals. Includes an Instruction Set Simulator, to simulate and compare the code execution.

More concretely, it is a CPU design project focused on building a minimal RV32I-based processor from the ground up using open-source tools. The project spans the full development flow, starting from a custom instruction-set simulator (ISS) and progressing through SystemVerilog RTL implementation, verification, synthesis, and ASIC-style place-and-route.

The processor is intentionally simple and in-order, with a small on-chip memory system and memory-mapped peripherals, and is developed alongside a reference simulator to enable differential testing and early validation. The primary goal of MRV32 is learning and exploration of computer architecture and hardware design, rather than performance or full ISA compliance.
