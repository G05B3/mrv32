#!/usr/bin/env bash
set -e

CC=riscv64-unknown-elf-gcc

ARCH="-march=rv32i -mabi=ilp32"
CFLAGS="-ffreestanding -nostdlib -nostartfiles"
INCLUDES="-I$(pwd)/Include -I../../include -I../Lib"
LINKER="-T ../Link/link.ld"
LDFLAGS="-Wl,-L../Link -Wl,-I../Link"
CRT="../Link/crt0.s"

# Usage check
if [[ $# -lt 3 ]]; then
    echo "Usage: $0 <source.c> -o <output.elf>"
    exit 1
fi

SRC="$1"
shift

$CC \
  $ARCH \
  $CFLAGS \
  $INCLUDES \
  $LINKER \
  $LDFLAGS \
  $CRT \
  "$SRC" \
  -lgcc \
  "$@"