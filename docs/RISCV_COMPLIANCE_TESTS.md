# RISC-V Compliance Tests Integration Guide

This guide explains how to integrate the official RISC-V compliance test suite (riscv-tests) with your processor to verify RV32I instruction set compliance.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Setup Instructions](#setup-instructions)
4. [Compiling Tests](#compiling-tests)
5. [Converting ELF to Hex](#converting-elf-to-hex)
6. [Running Tests](#running-tests)
7. [Interpreting Results](#interpreting-results)
8. [Automated Testing](#automated-testing)
9. [Troubleshooting](#troubleshooting)

---

## Overview

### What is riscv-tests?

The **riscv-tests** repository contains the official compliance test suite for RISC-V processors. These tests verify that your processor correctly implements the RISC-V ISA specification.

**Test Categories**:
- **ISA Tests**: Test individual instructions (ADD, SUB, AND, OR, etc.)
- **Compliance Tests**: Test instruction behavior and edge cases
- **Benchmark Tests**: Performance and correctness benchmarks

### Test Structure

Each test:
1. **Sets up initial state** (registers, memory)
2. **Executes instruction(s)** to test
3. **Checks results** (register values, memory contents)
4. **Reports pass/fail** via register x1 (1 = pass, 0 = fail)

### Integration with Your Processor

Your processor can run these tests by:
1. Loading test ELF files into instruction memory
2. Running the processor simulation
3. Checking final register values for pass/fail indication

---

## Prerequisites

### Required Tools

1. **RISC-V GCC Toolchain**
   - Cross-compiler for RISC-V
   - Includes: `riscv64-unknown-elf-gcc`, `riscv64-unknown-elf-objcopy`, etc.
   - Download: [RISC-V GNU Toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)

2. **Git**
   - For cloning repositories
   - Usually pre-installed on Linux/Mac
   - Download for Windows: [Git for Windows](https://git-scm.com/download/win)

3. **Make**
   - Build automation tool
   - Usually pre-installed on Linux/Mac
   - Windows: Use WSL or install via MSYS2

4. **Python 3** (optional)
   - For test automation scripts
   - Usually pre-installed

### Installation Options

#### Option 1: Pre-built Toolchain (Recommended)

**Linux (Ubuntu/Debian)**:
```bash
sudo apt-get update
sudo apt-get install gcc-riscv64-unknown-elf
```

**macOS (Homebrew)**:
```bash
brew install riscv-gnu-toolchain
```

**Windows (WSL)**:
```bash
# In WSL Ubuntu
sudo apt-get update
sudo apt-get install gcc-riscv64-unknown-elf
```

#### Option 2: Build from Source

See [RISC-V GNU Toolchain Repository](https://github.com/riscv-collab/riscv-gnu-toolchain) for build instructions.

**Quick Build**:
```bash
git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git
cd riscv-gnu-toolchain
./configure --prefix=/opt/riscv
make
```

---

## Setup Instructions

### Step 1: Clone riscv-tests Repository

```bash
# Navigate to your project directory
cd /path/to/RISC-V-5Stage-Processor

# Clone riscv-tests repository
git clone https://github.com/riscv/riscv-tests.git
cd riscv-tests
```

### Step 2: Verify Toolchain Installation

```bash
# Check if RISC-V GCC is installed
riscv64-unknown-elf-gcc --version

# Expected output:
# riscv64-unknown-elf-gcc (GCC) 11.x.x or later
```

If not found, add toolchain to PATH:
```bash
export PATH=/opt/riscv/bin:$PATH
```

### Step 3: Create Test Directory Structure

```bash
# In your project root
mkdir -p tests/compliance
mkdir -p tests/compliance/elf
mkdir -p tests/compliance/hex
mkdir -p tests/compliance/logs
```

---

## Compiling Tests

### Manual Compilation

#### Basic ISA Test Compilation

```bash
# Navigate to riscv-tests directory
cd riscv-tests

# Compile a single test (e.g., ADD test)
riscv64-unknown-elf-gcc \
    -march=rv32i \
    -mabi=ilp32 \
    -static \
    -mcmodel=medany \
    -fvisibility=hidden \
    -nostdlib \
    -nostartfiles \
    -T riscv_test.h/p/link.ld \
    -I riscv_test.h/p \
    isa/rv32ui/add.S \
    -o add.elf

# Convert to hex format
riscv64-unknown-elf-objcopy -O verilog add.elf add.hex
```

#### Compile All ISA Tests

```bash
# Set environment variables
export RISCV_PREFIX=riscv64-unknown-elf-
export RISCV_TARGET=riscv

# Build all tests
make -j$(nproc)
```

### Using Provided Script

Use the provided `scripts/build_compliance_tests.sh` script:

```bash
# Make script executable
chmod +x scripts/build_compliance_tests.sh

# Run script
./scripts/build_compliance_tests.sh
```

This script:
- Clones riscv-tests if needed
- Compiles all RV32I tests
- Converts ELF to hex format
- Organizes files in `tests/compliance/`

---

## Converting ELF to Hex

### Why Convert?

Your processor uses hex files for memory initialization, but riscv-tests produces ELF files. We need to convert ELF to hex format.

### Conversion Methods

#### Method 1: objcopy (Verilog Format)

```bash
# Convert ELF to Verilog hex format
riscv64-unknown-elf-objcopy -O verilog test.elf test.hex
```

**Output Format**:
```
@00000000
13 00 00 00
93 00 00 00
...
```

**Note**: This format may need adjustment for your memory initialization.

#### Method 2: Custom Python Script

Use `scripts/elf_to_hex.py` to convert ELF to your processor's hex format:

```bash
python3 scripts/elf_to_hex.py test.elf test.hex
```

**Output Format** (one 32-bit instruction per line):
```
00100093
00200113
...
```

#### Method 3: readelf + Custom Script

```bash
# Extract text section
riscv64-unknown-elf-objcopy -O binary --only-section=.text test.elf test.bin

# Convert binary to hex (using provided script)
python3 scripts/bin_to_hex.py test.bin test.hex
```

---

## Running Tests

### Manual Test Execution

#### Step 1: Prepare Test File

```bash
# Copy test hex file to memory directory
cp tests/compliance/hex/add.hex mem/inst_mem.hex
```

#### Step 2: Run Simulation

```bash
# Using Icarus Verilog
iverilog -o riscv_test -I src src/*.sv tb/riscv_pipeline_tb.sv
vvp riscv_test

# Or using ModelSim
vsim -do scripts/run_test.do
```

#### Step 3: Check Results

The test sets register x1 to indicate pass/fail:
- **x1 = 1**: Test passed
- **x1 = 0**: Test failed

Check final register values in simulation output or waveform.

### Using Test Runner Script

Use `scripts/run_compliance_test.sh`:

```bash
# Run a single test
./scripts/run_compliance_test.sh add

# Run all tests
./scripts/run_all_compliance_tests.sh
```

---

## Interpreting Results

### Test Output Format

#### Passed Test

```
Test: add
Status: PASSED
Register x1: 0x00000001
Cycles: 150
```

#### Failed Test

```
Test: add
Status: FAILED
Register x1: 0x00000000
Cycles: 150
Expected: 0x00000001
```

### Understanding Test Behavior

#### Test Structure

1. **Setup Phase**:
   - Initialize registers
   - Set up test data
   - Load test values

2. **Execution Phase**:
   - Execute instruction(s) under test
   - Perform operations

3. **Verification Phase**:
   - Check results
   - Set x1 = 1 if pass, x1 = 0 if fail

#### Common Test Patterns

**Arithmetic Test** (e.g., ADD):
```assembly
# Test: ADD x1, x2, x3
li x2, 5        # Load 5 into x2
li x3, 3        # Load 3 into x3
add x1, x2, x3  # x1 = x2 + x3 = 8
li x4, 8        # Expected result
bne x1, x4, fail # Compare
li x1, 1        # Pass
j pass
fail:
li x1, 0        # Fail
pass:
```

**Memory Test** (e.g., LW):
```assembly
# Test: LW x1, 0(x2)
li x2, 0x1000   # Base address
sw x3, 0(x2)    # Store value
lw x1, 0(x2)    # Load value
beq x1, x3, pass # Compare
li x1, 0        # Fail
j end
pass:
li x1, 1        # Pass
end:
```

### Analyzing Failures

#### Common Failure Causes

1. **Instruction Not Implemented**
   - Test fails immediately
   - Check if instruction is supported

2. **Incorrect Result**
   - Wrong value in result register
   - Check ALU/instruction logic

3. **Memory Access Error**
   - Load/store not working
   - Check memory interface

4. **Control Flow Error**
   - Branch/jump not working
   - Check PC update logic

5. **Hazard Handling Error**
   - Forwarding not working
   - Check forwarding unit

#### Debugging Failed Tests

1. **Enable Verbose Output**:
   ```bash
   ./scripts/run_compliance_test.sh add --verbose
   ```

2. **Generate Waveform**:
   ```bash
   # Enable VCD generation in testbench
   # View waveform to see instruction execution
   ```

3. **Check Register Values**:
   - Compare expected vs actual register values
   - Identify where test diverges

4. **Trace Instruction Execution**:
   - Add debug prints in processor
   - Trace instruction flow

---

## Automated Testing

### Python Test Runner (Recommended)

The `scripts/run_compliance_tests.py` script provides comprehensive automation:

1. **Test Discovery**: Finds all test hex files
2. **Test Execution**: Runs each test in simulation
3. **Result Parsing**: Extracts register x1 value (pass/fail indicator)
4. **Result Comparison**: Compares against expected values
5. **Report Generation**: Creates text, HTML, and JSON reports

### Usage

```bash
# Run all compliance tests
python3 scripts/run_compliance_tests.py

# Run with verbose output
python3 scripts/run_compliance_tests.py --verbose

# Generate HTML report
python3 scripts/run_compliance_tests.py --report compliance_report.html

# Generate JSON report
python3 scripts/run_compliance_tests.py --json results.json

# Generate waveforms for failed tests
python3 scripts/run_compliance_tests.py --waveform
```

### Shell Script Alternative

The `scripts/run_all_compliance_tests.sh` script also automates testing:

```bash
# Run all compliance tests
./scripts/run_all_compliance_tests.sh

# Run specific test category
./scripts/run_all_compliance_tests.sh --category isa

# Generate detailed report
./scripts/run_all_compliance_tests.sh --report compliance_report.html
```

### Test Report Format

**Text Report**:
```
============================================================
RISC-V Compliance Test Report
============================================================
Date: 2024-01-15T10:30:00
Processor: RISC-V 5-Stage Pipeline
ISA: RV32I

Summary:
------------------------------------------------------------
Total Tests:  50
Passed:       48
Failed:       2
Pass Rate:    96.00%
Duration:     120.45s

Failed Tests:
------------------------------------------------------------
  and:
    x1 = 0x00000000 (expected 0x00000001)
    Duration: 2.45s
```

**HTML Report**: Styled report with tables and links to log files

**JSON Report**: Machine-readable format for CI/CD integration

---

## Troubleshooting

### Common Issues

#### Issue 1: Toolchain Not Found

**Error**: `riscv64-unknown-elf-gcc: command not found`

**Solution**:
```bash
# Check if installed
which riscv64-unknown-elf-gcc

# If not found, install or add to PATH
export PATH=/opt/riscv/bin:$PATH
```

#### Issue 2: Test Compilation Fails

**Error**: `undefined reference to 'main'`

**Solution**:
- Ensure using correct linker script
- Check test file includes proper headers
- Verify architecture flags (`-march=rv32i`)

#### Issue 3: Hex File Format Mismatch

**Error**: Memory not initialized correctly

**Solution**:
- Check hex file format matches your memory initialization
- Verify byte order (little-endian vs big-endian)
- Use provided `elf_to_hex.py` script

#### Issue 4: Test Runs But Always Fails

**Error**: All tests report x1 = 0

**Solution**:
- Check reset behavior (PC should start at 0x00000000)
- Verify instruction memory is loaded correctly
- Check if test completion code is reached
- Enable debug output to trace execution

#### Issue 5: Simulation Hangs

**Error**: Test never completes

**Solution**:
- Add timeout to test runner
- Check for infinite loops in test
- Verify branch/jump instructions work
- Check if test expects specific memory layout

### Debug Tips

1. **Enable Verbose Mode**:
   ```bash
   ./scripts/run_compliance_test.sh add --verbose --debug
   ```

2. **Generate Waveform**:
   ```systemverilog
   // In testbench
   initial begin
       $dumpfile("test.vcd");
       $dumpvars(0, riscv_pipeline_tb);
   end
   ```

3. **Add Debug Prints**:
   ```systemverilog
   // In processor module
   always_ff @(posedge clk) begin
       if (wb_RegWrite) begin
           $display("Cycle %0d: Write x%0d = 0x%08x", cycle, wb_rd_addr, wb_write_data);
       end
   end
   ```

4. **Check Memory Contents**:
   ```systemverilog
   // Dump memory contents
   initial begin
       #1000;
       for (int i = 0; i < 100; i++) begin
           $display("IMEM[%0d] = 0x%08x", i, instruction_memory.memory[i]);
       end
   end
   ```

---

## Test Categories

### ISA Tests (rv32ui)

**Arithmetic**:
- `add`, `sub`, `addi`, `subi`

**Logical**:
- `and`, `or`, `xor`, `andi`, `ori`, `xori`

**Shift**:
- `sll`, `srl`, `sra`, `slli`, `srli`, `srai`

**Comparison**:
- `slt`, `sltu`, `slti`, `sltiu`

**Memory**:
- `lw`, `sw`, `lb`, `lh`, `lbu`, `lhu`, `sb`, `sh`

**Branch**:
- `beq`, `bne`, `blt`, `bge`, `bltu`, `bgeu`

**Jump**:
- `jal`, `jalr`

**System**:
- `lui`, `auipc`

### Compliance Tests

- Instruction behavior verification
- Edge case testing
- Exception handling (if implemented)

---

## Next Steps

1. **Run Basic Tests**: Start with simple arithmetic tests
2. **Verify Results**: Check pass/fail indicators
3. **Debug Failures**: Use waveforms and debug output
4. **Expand Coverage**: Run all ISA tests
5. **Automate**: Use test runner scripts for CI/CD

---

## References

- [riscv-tests Repository](https://github.com/riscv/riscv-tests)
- [RISC-V GNU Toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)
- [RISC-V ISA Specification](https://riscv.org/technical/specifications/)
- [RISC-V Compliance Test Framework](https://github.com/riscv-non-isa/riscv-arch-test)

---

## Appendix: Test File Locations

After compilation, test files are located in:

```
tests/compliance/
├── elf/          # ELF files (original)
├── hex/          # Hex files (converted)
├── logs/         # Test execution logs
└── reports/      # Test reports
```

Individual test files:
- `tests/compliance/hex/add.hex`
- `tests/compliance/hex/sub.hex`
- `tests/compliance/hex/and.hex`
- etc.

