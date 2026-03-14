#!/bin/bash

# Usage: ./run.sh [options]
# Options:
#   -f <file>     Hex file to load (default: hex_files/test1.hex)
#   -n <num>      Max instructions to retire (default: 10000)
#   -s <addr>     Memory dump start address, hex (default: 0x00000000)
#   -l <num>      Memory dump length in bytes (default: 64)
#   -t            Enable retirement trace (default: off)
#   -c <version>  Core version to simulate (default: latest in ../RTL/)
#   -h            Show this help
#
# Core version examples: -c v1.0  -c v2.0  -c v1.1
# If -c is omitted, uses the working copy directly in ../RTL/
# All mrv32_*.sv files in the core directory are compiled automatically.
# mrv32_pkg.sv is always first, mrv32_core.sv is always last.

PROG="hex_files/test1.hex"
MAX_INSTRS=10000
DUMP_START="00000000"
DUMP_LEN=64
TRACE=0
CORE_VERSION=""

usage() {
  grep '^#' "$0" | grep -v '#!/' | sed 's/^# \{0,1\}//'
  exit 0
}

while getopts "f:n:s:l:c:th" opt; do
  case $opt in
    f) PROG="$OPTARG" ;;
    n) MAX_INSTRS="$OPTARG" ;;
    s) DUMP_START="${OPTARG#0x}" ;;
    l) DUMP_LEN="$OPTARG" ;;
    c) CORE_VERSION="$OPTARG" ;;
    t) TRACE=1 ;;
    h) usage ;;
    *) echo "Unknown flag. Use -h for help."; exit 0 ;;
  esac
done

# Resolve core directory
if [ -z "$CORE_VERSION" ]; then
  CORE_DIR="../RTL"
else
  CORE_DIR="../RTL/core_${CORE_VERSION}"
  if [ ! -d "$CORE_DIR" ]; then
    echo "Error: core version '${CORE_VERSION}' not found at ${CORE_DIR}"
    echo "Available versions:"
    ls ../RTL/ | grep '^core_' | sed 's/core_/  /'
    exit 0
  fi
fi

echo "Using core: ${CORE_DIR}"

if [ ! -f "$PROG" ]; then
  echo "Error: hex file '$PROG' not found."
  exit 0
fi

# Build core source list: pkg first, core last, everything else in between
PKG="${CORE_DIR}/mrv32_pkg.sv"
CORE="${CORE_DIR}/mrv32_core.sv"

if [ ! -f "$PKG" ]; then
  echo "Error: mrv32_pkg.sv not found in ${CORE_DIR}"
  exit 1
fi
if [ ! -f "$CORE" ]; then
  echo "Error: mrv32_core.sv not found in ${CORE_DIR}"
  exit 1
fi

MIDDLE=$(ls ${CORE_DIR}/mrv32_*.sv 2>/dev/null \
  | grep -v 'mrv32_pkg\.sv' \
  | grep -v 'mrv32_core\.sv')

CORE_SRCS="$PKG $MIDDLE $CORE"

#../RTL/mem_dual_port.sv \  <--- add this instead of instant_mem.sv in the final version
iverilog -g2012 -o sim \
  $CORE_SRCS \
  ../RTL/instant_mem.sv \
  ../RTL/peripherals.sv \
  core_tb.sv \
  || exit 0

vvp sim \
  +PROG=$PROG \
  +MAX_INSTRS=$MAX_INSTRS \
  +DUMP_START=$DUMP_START \
  +DUMP_LEN=$DUMP_LEN \
  +TRACE=$TRACE