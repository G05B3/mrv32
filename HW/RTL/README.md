# MRV32 RTL

This directory contains the SystemVerilog implementation of the MRV32 processor.

The RTL is developed incrementally through explicitly versioned core implementations. Each `core_v<version>` directory represents a specific architectural/microarchitectural milestone and is preserved as a historical reference. The RTL sources located directly in this directory represent the **current development version** on the active Git branch.

---

## Directory Organization

```text
HW/
└── RTL/
    ├── README.md
    ├── core_v1.0/
    ├── core_v1.1/
    └── <current RTL sources>
```

### Versioned cores

The `core_v<version>` directories are snapshots of specific MRV32 core versions. They are intended to provide stable reference points for:

- comparing microarchitectural changes
- reproducing previous implementations
- measuring performance improvements
- debugging regressions between versions

A versioned directory should be treated as a historical implementation rather than as the active development tree.

### Current RTL

RTL files located directly inside `HW/RTL/` correspond to the implementation currently being developed on the active Git branch. These files may contain changes that are not yet represented by a new versioned snapshot.

When a new core version is considered stable, the corresponding RTL can be preserved in a new `core_v<version>` directory.

---

# Core Versions

## v1.0 — Serialized Core

The first MRV32 RTL implementation was designed primarily as a bring-up and verification core.

### Architecture

- Single-issue execution
- One instruction in flight at a time
- Explicit IF, ID, EX, MEM, and WB stage registers
- RV32I subset implemented through a dedicated decode and ALU datapath
- Branch and jump resolution in the MEM stage
- Writeback and architectural progress controlled through the WB stage

Although the datapath is organized around the familiar five stages, execution is effectively serialized because instructions do not overlap.

### Memory system

The v1.0 implementation uses a blocking load-store unit. Memory operations progress through a small FSM and stall the processor until the transaction completes.

The LSU supports:

- `LB`, `LBU`, `LH`, `LHU`, `LW`
- `SB`, `SH`, `SW`
- byte-enable generation
- sign and zero extension
- alignment checking

### I/O

The core exposes a memory-mapped peripheral interface used by the MRV32 software environment. The simulation environment currently provides UART output and a TOHOST termination mechanism.

### Purpose

v1.0 serves as the baseline implementation against which later microarchitectural revisions can be compared.

---

## v1.1 — Fully Pipelined Core

v1.1 converts the serialized datapath into a true 5-stage in-order pipeline, allowing multiple instructions to be in flight simultaneously.

### Pipeline

```text
IF → ID → EX → MEM → WB
```

Each stage communicates through dedicated pipeline registers carrying both datapath values and control information.

The design retains the same basic RV32I datapath and memory interface while introducing the mechanisms required for overlapping instruction execution. The implemented pipeline explicitly carries instruction-valid, control, register, immediate, and result information between stages.

### Hazard Detection

v1.1 includes a dedicated hazard detection unit responsible for identifying dependencies between instructions in the pipeline.

The current implementation handles:

- register data hazards
- load-use hazards
- detection of whether `rs1`/`rs2` are actually consumed
- pipeline stalling when a load result is not yet available

The load-use dependency is handled separately from normal forwarding because the loaded value is not available early enough for the immediately following dependent instruction.

### Forwarding

The pipeline includes forwarding paths that bypass values from later stages directly back to the EX-stage operands.

The current implementation provides forwarding from:

- MEM-stage results
- WB-stage results

The forwarding logic covers both source operands and accounts for values produced by ALU operations, loads, and jumps.

### Control Hazards

Branches and jumps are resolved in the MEM stage, with the resulting target address fed back to the fetch subsystem.

When a control-flow change is taken, instructions that were fetched along the wrong path are invalidated. The fetch logic maintains an instruction queue and clears queued entries on a taken branch, preventing stale instructions from being delivered to the pipeline.

### Memory Stalls

The v1.1 pipeline retains the blocking LSU inherited from the bring-up design.

A memory instruction can therefore stall the pipeline while its transaction is in progress. The MEM/WB boundary only advances when the LSU reports completion.

This intentionally keeps the memory interface simple while allowing the core datapath itself to benefit from instruction-level overlap.

### Performance

The v1.1 implementation has been validated against the current MRVBench suite:

- `vadd`
- `memcpy`
- `dotprod`

Across these benchmarks, v1.1 achieves an approximately **3× speedup** over v1.0.

The benchmark suite is also used as the first systematic functional regression set for comparing different MRV32 core versions.

---

# Future Versions

Future core versions will build incrementally on the v1.1 pipeline rather than replacing the architectural model wholesale.

Possible areas of evolution include:

- deeper or more aggressive pipelining
- improved branch handling
- more complete forwarding and hazard optimization
- caches and a more sophisticated memory hierarchy
- higher-throughput memory interfaces
- hardware performance counters
- accelerator interfaces
- additional ISA extensions

Each new version will be preserved as a separate `core_v<version>` snapshot once its implementation is sufficiently stable.

---

# Relationship with the ISS

The C++ ISS serves as the architectural reference model for MRV32.

The goal of the RTL versions is to preserve the architectural behavior of the ISS while progressively changing the underlying microarchitecture.

In particular:

```text
             Architectural behavior
                      │
                      ▼
                 MRV32 ISS
                      │
                      │ verification reference
                      ▼
              ┌───────────────┐
              │    RTL v1.0   │
              │   serialized  │
              └───────────────┘
                      │
                      ▼
              ┌───────────────┐
              │    RTL v1.1   │
              │  5-stage pipe │
              └───────────────┘
                      │
                      ▼
                future cores
```

The architecture should remain compatible across versions unless a change is explicitly introduced as part of a future architectural revision.