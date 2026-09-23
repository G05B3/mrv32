#!/usr/bin/env bash
set -e

# Usage check
if [[ $# -lt 3 ]]; then
    echo "Usage: $0 <source.elf> -o <output.hex>"
    exit 1
fi

SRC="$1"
OUTPUT="$3"
shift

riscv64-unknown-elf-objcopy -O verilog "$SRC" "$OUTPUT"