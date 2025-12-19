#!/bin/bash
################################################################################
# Build RISC-V Compliance Tests Script
#
# This script clones the riscv-tests repository (if needed) and compiles
# all RV32I compliance tests, then converts them to hex format for use
# with the RISC-V processor simulator.
#
# Usage:
#   ./scripts/build_compliance_tests.sh
#
# Requirements:
#   - RISC-V GCC toolchain (riscv64-unknown-elf-gcc)
#   - Make
#   - Git
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RISCV_TESTS_DIR="$PROJECT_ROOT/riscv-tests"
TESTS_DIR="$PROJECT_ROOT/tests/compliance"
ELF_DIR="$TESTS_DIR/elf"
HEX_DIR="$TESTS_DIR/hex"

# RISC-V toolchain prefix
RISCV_PREFIX="${RISCV_PREFIX:-riscv64-unknown-elf-}"

echo "=========================================="
echo "RISC-V Compliance Tests Build Script"
echo "=========================================="
echo "Project Root: $PROJECT_ROOT"
echo "Tests Directory: $TESTS_DIR"
echo "Toolchain Prefix: $RISCV_PREFIX"
echo ""

# Check if RISC-V toolchain is available
if ! command -v "${RISCV_PREFIX}gcc" &> /dev/null; then
    echo -e "${RED}Error: RISC-V GCC toolchain not found!${NC}"
    echo "Please install RISC-V GNU toolchain:"
    echo "  Ubuntu/Debian: sudo apt-get install gcc-riscv64-unknown-elf"
    echo "  macOS: brew install riscv-gnu-toolchain"
    echo "  Or build from: https://github.com/riscv-collab/riscv-gnu-toolchain"
    exit 1
fi

echo -e "${GREEN}✓${NC} RISC-V toolchain found: $(${RISCV_PREFIX}gcc --version | head -n1)"
echo ""

# Create directories
mkdir -p "$TESTS_DIR"
mkdir -p "$ELF_DIR"
mkdir -p "$HEX_DIR"

# Clone riscv-tests repository if it doesn't exist
if [ ! -d "$RISCV_TESTS_DIR" ]; then
    echo "Cloning riscv-tests repository..."
    cd "$PROJECT_ROOT"
    git clone https://github.com/riscv/riscv-tests.git
    echo -e "${GREEN}✓${NC} Repository cloned"
else
    echo -e "${GREEN}✓${NC} riscv-tests repository already exists"
    echo "  Updating repository..."
    cd "$RISCV_TESTS_DIR"
    git pull || echo "  (Could not update, using existing version)"
fi

echo ""

# Set environment variables for riscv-tests build
export RISCV_PREFIX
export RISCV_TARGET=riscv

# Navigate to riscv-tests directory
cd "$RISCV_TESTS_DIR"

# Build ISA tests
echo "Building ISA tests (rv32ui)..."
cd isa

# Compile individual test files
# Note: riscv-tests uses a Makefile, but we'll compile manually for more control

# List of ISA tests to compile
ISA_TESTS=(
    "add" "addi" "and" "andi" "auipc"
    "beq" "bge" "bgeu" "blt" "bltu" "bne"
    "jal" "jalr"
    "lb" "lbu" "lh" "lhu" "lui" "lw"
    "or" "ori"
    "sb" "sh" "sw"
    "sll" "slli" "slt" "slti" "sltiu" "sltu"
    "sra" "srai" "srl" "srli"
    "sub" "xor" "xori"
)

# Compile each test
COMPILED_COUNT=0
FAILED_TESTS=()

for test in "${ISA_TESTS[@]}"; do
    test_file="rv32ui/${test}.S"
    
    if [ ! -f "$test_file" ]; then
        echo -e "${YELLOW}⚠${NC} Test file not found: $test_file (skipping)"
        continue
    fi
    
    echo -n "  Compiling $test... "
    
    # Compile test
    if "${RISCV_PREFIX}gcc" \
        -march=rv32i \
        -mabi=ilp32 \
        -static \
        -mcmodel=medany \
        -fvisibility=hidden \
        -nostdlib \
        -nostartfiles \
        -T ../env/p/link.ld \
        -I ../env/p \
        "$test_file" \
        -o "$ELF_DIR/${test}.elf" 2>&1 | grep -v "warning:"; then
        echo -e "${GREEN}✓${NC}"
        ((COMPILED_COUNT++))
    else
        echo -e "${RED}✗${NC}"
        FAILED_TESTS+=("$test")
    fi
done

echo ""
echo "Compiled $COMPILED_COUNT tests"
if [ ${#FAILED_TESTS[@]} -gt 0 ]; then
    echo -e "${YELLOW}Failed tests: ${FAILED_TESTS[*]}${NC}"
fi
echo ""

# Convert ELF files to hex format
echo "Converting ELF files to hex format..."
cd "$PROJECT_ROOT"

CONVERTED_COUNT=0
for elf_file in "$ELF_DIR"/*.elf; do
    if [ -f "$elf_file" ]; then
        test_name=$(basename "$elf_file" .elf)
        hex_file="$HEX_DIR/${test_name}.hex"
        
        echo -n "  Converting $test_name... "
        
        # Use Python script to convert ELF to hex
        if python3 "$SCRIPT_DIR/elf_to_hex.py" "$elf_file" "$hex_file"; then
            echo -e "${GREEN}✓${NC}"
            ((CONVERTED_COUNT++))
        else
            echo -e "${RED}✗${NC}"
        fi
    fi
done

echo ""
echo "=========================================="
echo "Build Summary"
echo "=========================================="
echo "Tests compiled: $COMPILED_COUNT"
echo "Tests converted: $CONVERTED_COUNT"
echo "ELF files: $ELF_DIR"
echo "Hex files: $HEX_DIR"
echo ""
echo -e "${GREEN}Build complete!${NC}"
echo ""
echo "To run a test:"
echo "  ./scripts/run_compliance_test.sh <test_name>"
echo ""
echo "To run all tests:"
echo "  ./scripts/run_all_compliance_tests.sh"

