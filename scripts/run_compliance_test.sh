#!/bin/bash
################################################################################
# Run Single RISC-V Compliance Test
#
# This script runs a single compliance test by:
# 1. Copying test hex file to instruction memory
# 2. Running the processor simulation
# 3. Checking results (register x1 = 1 for pass, 0 for fail)
#
# Usage:
#   ./scripts/run_compliance_test.sh <test_name> [options]
#
# Options:
#   --verbose    Enable verbose output
#   --debug      Enable debug output
#   --waveform   Generate waveform file
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TESTS_DIR="$PROJECT_ROOT/tests/compliance"
HEX_DIR="$TESTS_DIR/hex"
LOG_DIR="$TESTS_DIR/logs"
MEM_DIR="$PROJECT_ROOT/mem"

# Parse arguments
TEST_NAME=""
VERBOSE=0
DEBUG=0
WAVEFORM=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose)
            VERBOSE=1
            shift
            ;;
        --debug)
            DEBUG=1
            VERBOSE=1
            shift
            ;;
        --waveform)
            WAVEFORM=1
            shift
            ;;
        *)
            if [ -z "$TEST_NAME" ]; then
                TEST_NAME="$1"
            else
                echo "Unknown option: $1"
                exit 1
            fi
            shift
            ;;
    esac
done

if [ -z "$TEST_NAME" ]; then
    echo "Usage: $0 <test_name> [--verbose] [--debug] [--waveform]"
    echo ""
    echo "Available tests:"
    ls -1 "$HEX_DIR"/*.hex 2>/dev/null | xargs -n1 basename | sed 's/\.hex$//' | sed 's/^/  /'
    exit 1
fi

# Check if test file exists
TEST_HEX="$HEX_DIR/${TEST_NAME}.hex"
if [ ! -f "$TEST_HEX" ]; then
    echo -e "${RED}Error: Test file not found: $TEST_HEX${NC}"
    echo "Available tests:"
    ls -1 "$HEX_DIR"/*.hex 2>/dev/null | xargs -n1 basename | sed 's/\.hex$//' | sed 's/^/  /'
    exit 1
fi

echo "=========================================="
echo "Running Compliance Test: $TEST_NAME"
echo "=========================================="

# Create log directory
mkdir -p "$LOG_DIR"

# Copy test hex file to instruction memory
echo "Preparing instruction memory..."
cp "$TEST_HEX" "$MEM_DIR/inst_mem.hex"
echo -e "${GREEN}✓${NC} Copied $TEST_HEX to $MEM_DIR/inst_mem.hex"

# Run simulation
echo ""
echo "Running simulation..."

# Determine simulator
SIMULATOR=""
if command -v iverilog &> /dev/null; then
    SIMULATOR="iverilog"
elif command -v vvp &> /dev/null; then
    SIMULATOR="iverilog"
else
    echo -e "${RED}Error: No simulator found (iverilog or vvp)${NC}"
    exit 1
fi

# Compile and run with Icarus Verilog
cd "$PROJECT_ROOT"

LOG_FILE="$LOG_DIR/${TEST_NAME}.log"
VCD_FILE="$PROJECT_ROOT/${TEST_NAME}.vcd"

if [ "$SIMULATOR" = "iverilog" ]; then
    # Compile
    if [ $VERBOSE -eq 1 ]; then
        echo "  Compiling..."
    fi
    
    IVERILOG_CMD="iverilog -o ${TEST_NAME}_sim -I src src/*.sv tb/riscv_pipeline_tb.sv"
    if [ $DEBUG -eq 1 ]; then
        IVERILOG_CMD="$IVERILOG_CMD -DDEBUG"
    fi
    
    if ! eval "$IVERILOG_CMD" > "$LOG_FILE" 2>&1; then
        echo -e "${RED}✗${NC} Compilation failed"
        echo "See log: $LOG_FILE"
        exit 1
    fi
    
    # Run simulation
    if [ $VERBOSE -eq 1 ]; then
        echo "  Running simulation..."
    fi
    
    VVP_CMD="vvp ${TEST_NAME}_sim"
    if [ $WAVEFORM -eq 1 ]; then
        VVP_CMD="$VVP_CMD +dumpfile=${VCD_FILE}"
    fi
    
    if ! eval "$VVP_CMD" >> "$LOG_FILE" 2>&1; then
        echo -e "${RED}✗${NC} Simulation failed"
        echo "See log: $LOG_FILE"
        exit 1
    fi
fi

# Parse results from log file
echo ""
echo "Analyzing results..."

# Look for register x1 value in log file
# The test sets x1 = 1 for pass, x1 = 0 for fail
X1_VALUE=$(grep -i "x1\|register.*1" "$LOG_FILE" | tail -1 | grep -oE "0x[0-9a-fA-F]+|0x[0-9a-fA-F]{8}" | head -1 || echo "")

# Also check for pass/fail indicators
if grep -qi "pass\|PASS\|test.*passed" "$LOG_FILE"; then
    TEST_RESULT="PASSED"
    RESULT_COLOR="$GREEN"
elif grep -qi "fail\|FAIL\|test.*failed" "$LOG_FILE"; then
    TEST_RESULT="FAILED"
    RESULT_COLOR="$RED"
else
    # Try to infer from register x1 value
    if [ -n "$X1_VALUE" ]; then
        if echo "$X1_VALUE" | grep -qi "0x00000001\|0x1$"; then
            TEST_RESULT="PASSED"
            RESULT_COLOR="$GREEN"
        else
            TEST_RESULT="FAILED"
            RESULT_COLOR="$RED"
        fi
    else
        TEST_RESULT="UNKNOWN"
        RESULT_COLOR="$YELLOW"
    fi
fi

# Display results
echo "=========================================="
echo "Test Results"
echo "=========================================="
echo "Test: $TEST_NAME"
echo "Status: ${RESULT_COLOR}${TEST_RESULT}${NC}"
if [ -n "$X1_VALUE" ]; then
    echo "Register x1: $X1_VALUE"
fi
echo "Log file: $LOG_FILE"
if [ $WAVEFORM -eq 1 ] && [ -f "$VCD_FILE" ]; then
    echo "Waveform: $VCD_FILE"
fi
echo ""

# Cleanup
rm -f "${TEST_NAME}_sim"

# Exit with appropriate code
if [ "$TEST_RESULT" = "PASSED" ]; then
    exit 0
elif [ "$TEST_RESULT" = "FAILED" ]; then
    exit 1
else
    exit 2
fi

