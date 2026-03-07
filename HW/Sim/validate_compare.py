#!/usr/bin/env python3
"""
validate.py — Compare ISS vs RTL simulation output.

Usage:
    python3 validate.py -f <source.c/.s> [options]

Options:
    -f <file>       Source file (.c or .s)
    -n <num>        Max instructions to retire (default: 1000)
    -s <hex_addr>   Memory dump start address (default: 0x00000000)
    -l <num>        Memory dump length in bytes (default: 64)
    -c <version>    Select specific core version
    -t              Enable RTL retirement trace
    -h              Show help
"""

import subprocess
import argparse
import re
import sys
import os

# ---------------------------------------------------------------
# Paths
# ---------------------------------------------------------------
ROOT     = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
HW_SIM   = os.path.join(ROOT, "HW", "Sim")
HW_RTL   = os.path.join(ROOT, "HW", "RTL")
SW_DIR   = os.path.join(ROOT, "SW")
ISS_DIR  = os.path.join(ROOT, "SW", "ISS")
LINK_DIR = os.path.join(SW_DIR, "Link")
ISS_BIN  = os.path.join(ISS_DIR, "mrv_iss")

PROG_ELF = os.path.join(HW_SIM, "validation.elf")
PROG_HEX = os.path.join(HW_SIM, "validation.hex")

CORE_FILES = [
    "mrv32_pkg.sv",
    "mrv32_alu.sv",
    "mrv32_fetch.sv",
    "mrv32_id.sv",
    "mrv32_rf.sv",
    "mrv32_imm_gen.sv",
    "mrv32_lsu.sv",
    "mrv32_bru.sv",
    "mrv32_wb.sv",
    "mrv32_core.sv",
    "mrv32_periph.sv",
]

INFRA_SOURCES = [
    "../RTL/mem_dual_port.sv",
    "../RTL/peripherals.sv",
    "core_tb.sv",
]

# ---------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------
def run(cmd, cwd=None, desc=None):
    if desc:
        print(f"  {desc}...")
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
    if result.returncode != 0:
        print(f"\nFailed: {' '.join(cmd)}")
        print(result.stderr)
        sys.exit(0)
    return result.stdout

def banner(title):
    print(f"\n{'='*50}")
    print(f"  {title}")
    print(f"{'='*50}")

# ---------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------
def parse_iss_regs(text):
    """Parse RF(all) section from ISS output."""
    regs = {}
    for line in text.splitlines():
        # Each line has up to 8 registers: x0=0x... x1=0x... etc
        for m in re.finditer(r"x(\d+)=0x([0-9a-fA-F]+)", line):
            regs[int(m.group(1))] = int(m.group(2), 16)
    return regs

def parse_rtl_regs(text):
    """Parse register dump from RTL output."""
    regs = {}
    for line in text.splitlines():
        m = re.match(r"x(\d+)\s*=\s*0x([0-9a-fA-F]+)", line)
        if m:
            regs[int(m.group(1))] = int(m.group(2), 16)
    return regs

def parse_iss_mem(text):
    """Parse MEM(All) section from ISS output."""
    mem = {}
    for line in text.splitlines():
        m = re.match(r"0x([0-9a-fA-F]+):\s*0x([0-9a-fA-F]+)", line)
        if m:
            mem[int(m.group(1), 16)] = int(m.group(2), 16)
    return mem

def parse_rtl_mem(text):
    """Parse memory dump from RTL output."""
    mem = {}
    for line in text.splitlines():
        m = re.match(r"0x([0-9a-fA-F]+)\s*:.*\(0x([0-9a-fA-F]+)\)", line)
        if m:
            mem[int(m.group(1), 16)] = int(m.group(2), 16)
    return mem

def parse_iss_instret(text):
    m = re.search(r"instret:\s*(\d+)", text)
    return int(m.group(1)) if m else None

def parse_rtl_instret(text):
    m = re.search(r"Retired:\s*(\d+)", text)
    return int(m.group(1)) if m else None

# ---------------------------------------------------------------
# Build steps
# ---------------------------------------------------------------
def compile_source(src):
    banner("Step 1: Compile")
    ext = os.path.splitext(src)[1]
    if ext == ".c":
        run([
            "riscv64-unknown-elf-gcc",
            "-march=rv32i", "-mabi=ilp32",
            "-ffreestanding", "-nostdlib", "-nostartfiles",
            f"-I{ISS_DIR}/Include",
            f"-I{ROOT}/include",
            f"-T{LINK_DIR}/link.ld",
            f"-I{SW_DIR}/Lib",
            f"-Wl,-L{LINK_DIR}",
            f"{LINK_DIR}/crt0.s",
            os.path.abspath(src),
            "-o", PROG_ELF,
            "-lgcc"
        ], cwd=HW_SIM, desc=f"Compiling {src} → validation.elf")

    elif ext == ".s":
        obj = os.path.join(HW_SIM, "asm_obj.o")
        run([
            "riscv64-unknown-elf-as",
            "-march=rv32i", "-mabi=ilp32",
            os.path.abspath(src), "-o", obj
        ], cwd=HW_SIM, desc=f"Assembling {src} → asm_obj.o")
        run([
            "riscv64-unknown-elf-gcc",
            "-march=rv32i", "-mabi=ilp32",
            "-ffreestanding", "-nostdlib", "-nostartfiles",
            f"-T{LINK_DIR}/link.ld",
            f"{LINK_DIR}/crt0.s",
            obj, "-o", PROG_ELF, "-lgcc"
        ], cwd=HW_SIM, desc="Linking → validation.elf")
    else:
        print(f"Unsupported source type: {ext}")
        sys.exit(0)

def elf_to_hex():
    run(["./getProgMem.sh", PROG_ELF, "-o", PROG_HEX],
        cwd=HW_SIM, desc="Converting ELF → HEX")

def run_iss(max_instrs, mem_start, mem_end):
    banner("Step 2: ISS")
    out = run([
        ISS_BIN,
        PROG_ELF,
        "--show-register-file=all",
        "--show-memory=all",
        f"--mem-addr-base=0x{mem_start:08x}",
        f"--mem-addr-end=0x{mem_end:08x}",
        f"--max-steps={max_instrs}",
        f"--trace",
    ], cwd=HW_SIM, desc=f"Running ISS ({max_instrs} instrs)")
    return out

def compile_rtl(core_version=None):
    banner("Step 3: Compile RTL")

    if core_version is None:
        core_dir = "../RTL"
    else:
        core_dir = f"../RTL/core_{core_version}"
        if not os.path.isdir(os.path.join(HW_SIM, core_dir)):
            print(f"\nError: core version '{core_version}' not found at {core_dir}")
            available = [
                d.replace("core_", "")
                for d in os.listdir(os.path.join(HW_SIM, "../RTL"))
                if d.startswith("core_")
            ]
            if available:
                print(f"Available versions: {', '.join(sorted(available))}")
            sys.exit(1)

    core_srcs = [f"{core_dir}/{f}" for f in CORE_FILES]
    all_srcs  = core_srcs + INFRA_SOURCES

    print(f"  Core directory: {core_dir}")
    run(["iverilog", "-g2012", "-o", "sim"] + all_srcs,
        cwd=HW_SIM, desc="Compiling RTL")

def run_rtl(max_instrs, mem_start, mem_len, trace):
    banner("Step 4: RTL Simulation")
    out = run([
        "vvp", "sim",
        f"+PROG={PROG_HEX}",
        f"+MAX_INSTRS={max_instrs}",
        f"+DUMP_START={mem_start:08x}",
        f"+DUMP_LEN={mem_len}",
        f"+TRACE={int(trace)}",
    ], cwd=HW_SIM, desc=f"Running RTL ({max_instrs} instrs)")
    return out

# ---------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------
def compare(iss_out, rtl_out):
    banner("Step 5: Comparison")

    iss_regs   = parse_iss_regs(iss_out)
    rtl_regs   = parse_rtl_regs(rtl_out)
    iss_mem    = parse_iss_mem(iss_out)
    rtl_mem    = parse_rtl_mem(rtl_out)
    iss_retire = parse_iss_instret(iss_out)
    rtl_retire = parse_rtl_instret(rtl_out)

    errors = []

    # instret
    print(f"\n  instret — ISS: {iss_retire}  RTL: {rtl_retire}", end="  ")
    if iss_retire != rtl_retire:
        print("✗ MISMATCH")
        errors.append(f"instret: ISS={iss_retire} RTL={rtl_retire}")
    else:
        print("✓")

    # registers
    print("\n  Registers:")
    reg_ok = True
    for i in range(32):
        iv = iss_regs.get(i, 0)
        rv = rtl_regs.get(i, 0)
        if iv != rv:
            print(f"    x{i:02d}  ISS=0x{iv:08x}  RTL=0x{rv:08x}  ✗")
            errors.append(f"x{i}: ISS=0x{iv:08x} RTL=0x{rv:08x}")
            reg_ok = False
    if reg_ok:
        print("    all match ✓")

    # memory
    print("\n  Memory:")
    mem_ok = True
    all_addrs = sorted(set(iss_mem) | set(rtl_mem))
    for addr in all_addrs:
        iv = iss_mem.get(addr, 0)
        rv = rtl_mem.get(addr, 0)
        if iv != rv:
            print(f"    0x{addr:08x}  ISS=0x{iv:08x}  RTL=0x{rv:08x}  ✗")
            errors.append(f"mem[0x{addr:08x}]: ISS=0x{iv:08x} RTL=0x{rv:08x}")
            mem_ok = False
    if mem_ok:
        print("    all match ✓")

    # verdict
    banner("Result")
    if errors:
        print(f"  FAILED — {len(errors)} mismatch(es)\n")
        sys.exit(0)
    else:
        print("  PASSED ✓\n")
        sys.exit(0)

# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Validate RTL against ISS",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-f", metavar="FILE",    required=True,              help="Source file (.c or .s)")
    parser.add_argument("-n", metavar="NUM",     type=int, default=1000,     help="Max instructions (default: 1000)")
    parser.add_argument("-s", metavar="ADDR",    default="0x00000000",       help="Memory dump start, hex (default: 0x00000000)")
    parser.add_argument("-l", metavar="BYTES",   type=int, default=64,       help="Memory dump length in bytes (default: 64)")
    parser.add_argument("-c", metavar="VERSION", default=None,               help="Core version, e.g. v1.0 (default: working copy)")
    parser.add_argument("-t", action="store_true", default=False,            help="Enable RTL trace")
    args = parser.parse_args()

    mem_start = int(args.s, 16)
    mem_end   = mem_start + args.l

    compile_source(args.f)
    elf_to_hex()
    iss_out = run_iss(args.n, mem_start, mem_end)
    compile_rtl(args.c)
    rtl_out = run_rtl(args.n, mem_start, args.l, args.t)
    compare(iss_out, rtl_out)

if __name__ == "__main__":
    main()