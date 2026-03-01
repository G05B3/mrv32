#!/usr/bin/env python3
import subprocess
import re
import sys
import os

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
HW_SIM       = os.path.join(PROJECT_ROOT, "HW", "Sim")
ISS_DIR      = os.path.join(PROJECT_ROOT, "SW", "ISS")
SW_DIR       = os.path.join(PROJECT_ROOT, "SW")

if len(sys.argv) < 2:
    print("Usage: python3 validate.py <source_program.c/.s>")
    sys.exit(1)

SRC      = sys.argv[1]
PROG_ELF = "validation.elf"
PROG_HEX = "validation.hex"

MEM_START  = 0x000fff00
MEM_END    = 0x00100000
MAX_RETIRE = 1000

# ------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------
def run(cmd, cwd=None):
    """Run a shell command and return stdout. Exit on error."""
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
    if result.returncode != 0:
        print("Command failed:\n", result.stderr)
        sys.exit(1)
    return result.stdout

def parse_regs(text):
    regs = {}
    for line in text.splitlines():
        m = re.match(r"x(\d+)\s*=\s*0x([0-9a-fA-F]+)", line)
        if m:
            regs[int(m.group(1))] = int(m.group(2), 16)
    return regs

def parse_mem(text):
    mem = {}
    for line in text.splitlines():
        m = re.match(r"0x([0-9a-fA-F]+)\s*:\s*.*\(0x([0-9a-fA-F]+)\)", line)
        if m:
            addr = int(m.group(1), 16)
            val  = int(m.group(2), 16)
            mem[addr] = val
    return mem

def parse_retire(text):
    m = re.search(r"FINAL_RETIRE_COUNT=(\d+)", text)
    if m:
        return int(m.group(1))
    return None

# ------------------------------------------------------------
# Step 0: Compile C/ASM → ELF
# ------------------------------------------------------------
print(f"Compiling {SRC} → {PROG_ELF}...")

if SRC.endswith(".c"):
    # Run the exact command you would type in terminal for C
    compile_cmd = [
        "riscv64-unknown-elf-gcc",
        "-march=rv32i", "-mabi=ilp32",
        "-ffreestanding", "-nostdlib", "-nostartfiles",
        "-I" + os.path.join(ISS_DIR, "Include"),
        "-I" + os.path.join(PROJECT_ROOT, "include"),
        "-I" + os.path.join(ISS_DIR, "/../Lib"),
        "-T" + os.path.join(SW_DIR, "Link", "link.ld"),
        "-Wl,-L" + os.path.join(SW_DIR, "Link"),
        "-Wl,-I" + os.path.join(SW_DIR, "Link"),
        os.path.join(SW_DIR, "Link", "crt0.s"),
        SRC,
        "-o", PROG_ELF,
        "-lgcc"
    ]
elif SRC.endswith(".s"):
    src_path = os.path.abspath(SRC)
    crt0_path = os.path.join(SW_DIR, "Link", "crt0.s")
    link_ld   = os.path.join(SW_DIR, "Link", "link.ld")
    elf_path  = os.path.join(HW_SIM, PROG_ELF)
    asm_obj   = "asm_obj.o"

    # Step 1: Assemble .s → .o
    run([
        "riscv64-unknown-elf-as",
        "-march=rv32i",
        "-mabi=ilp32",
        src_path,
        "-o", asm_obj
    ], cwd=HW_SIM)

    # Step 2: Link using GCC (not ld) so it finds crt0, libgcc, and linker paths
    compile_cmd = [
        "riscv64-unknown-elf-gcc",
        "-march=rv32i", "-mabi=ilp32",
        "-ffreestanding", "-nostdlib", "-nostartfiles",
        "-T", os.path.join(SW_DIR, "Link", "link.ld"),
        os.path.join(SW_DIR, "Link", "crt0.s"),
        asm_obj,  # absolute path
        "-o", os.path.join(HW_SIM, PROG_ELF),
        "-lgcc"
    ]
    run(compile_cmd, cwd=SW_DIR)
else:
    raise ValueError("Unsupported source file type")

run(compile_cmd, cwd=HW_SIM)

# ------------------------------------------------------------
# Step 1: Convert ELF → HEX for RTL
# ------------------------------------------------------------
print(f"Converting {PROG_ELF} → {PROG_HEX}...")
run(["./getProgMem.sh", PROG_ELF, "-o", PROG_HEX], cwd=HW_SIM)

# ------------------------------------------------------------
# Step 2: Run ISS
# ------------------------------------------------------------
print("Running ISS...")
iss_path = os.path.join(ISS_DIR, "mrv_iss")
elf_path = os.path.join(HW_SIM, PROG_ELF)  # absolute path to ELF

iss_cmd = [
    iss_path,
    elf_path,
    "--show-register-file=all",
    "--show-memory=all",
    f"--mem-addr-base=0x{MEM_START:08x}",
    f"--mem-addr-end=0x{MEM_END:08x}",
    f"--max-steps={MAX_RETIRE}",
]

iss_out = run(iss_cmd, cwd=HW_SIM)  # run from the directory containing the ELF

# ------------------------------------------------------------
# Step 3: Compile RTL
# ------------------------------------------------------------
print("Compiling RTL...")
rtl_compile_cmd = [
    "iverilog", "-g2012", "-o", "sim",
    "../RTL/mrv32_pkg.sv",
    "../RTL/mrv32_alu.sv",
    "../RTL/mrv32_fetch.sv",
    "../RTL/mrv32_id.sv",
    "../RTL/mrv32_rf.sv",
    "../RTL/mrv32_imm_gen.sv",
    "../RTL/mrv32_lsu.sv",
    "../RTL/mrv32_bru.sv",
    "../RTL/mrv32_wb.sv",
    "../RTL/mrv32_core.sv",
    "../RTL/mem_dual_port.sv",
    "validate.sv"
]
run(rtl_compile_cmd, cwd=HW_SIM)

# ------------------------------------------------------------
# Step 4: Run RTL
# ------------------------------------------------------------
print("Running RTL...")
rtl_run_cmd = [
    "vvp", "sim",
    f"+PROG={PROG_HEX}",
    f"+MAX_RETIRE={MAX_RETIRE}",
    f"+MEM_START={MEM_START:08x}",
    f"+MEM_END={MEM_END:08x}"
]
rtl_out = run(rtl_run_cmd, cwd=HW_SIM)

# ------------------------------------------------------------
# Step 5: Parse outputs
# ------------------------------------------------------------
iss_regs   = parse_regs(iss_out)
rtl_regs   = parse_regs(rtl_out)
iss_mem    = parse_mem(iss_out)
rtl_mem    = parse_mem(rtl_out)
iss_retire = MAX_RETIRE  # ISS always uses max-steps
rtl_retire = parse_retire(rtl_out)

# ------------------------------------------------------------
# Step 6: Compare
# ------------------------------------------------------------
error = False

print("\n===== RETIRE COUNT =====")
print(f"ISS: {iss_retire}")
print(f"RTL: {rtl_retire}")
if iss_retire != rtl_retire:
    print("Retire mismatch")
    error = True
else:
    print("Retire match")

print("\n===== REGISTERS =====")
for i in range(32):
    iv = iss_regs.get(i, 0)
    rv = rtl_regs.get(i, 0)
    if iv != rv:
        print(f"MISMATCH: x{i:02d}: ISS=0x{iv:08x} RTL=0x{rv:08x}")
        error = True

if not error:
    print("Registers match")

print("\n===== MEMORY =====")
for addr in sorted(iss_mem.keys()):
    iv = iss_mem.get(addr, 0)
    rv = rtl_mem.get(addr, 0)
    if iv != rv:
        print(f"MISMATCH: MEM[0x{addr:08x}]: ISS=0x{iv:08x} RTL=0x{rv:08x}")
        error = True

if not error:
    print("Memory match")

print("\n=================================")
if error:
    print("VALIDATION FAILED")
    sys.exit(1)
else:
    print("VALIDATION PASSED")
    sys.exit(0)