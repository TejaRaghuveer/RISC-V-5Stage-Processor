# RISC-V Compliance Tests Scripts

This directory contains scripts for building and running RISC-V compliance tests.

## Scripts

### `build_compliance_tests.sh`

Builds all RISC-V compliance tests from the riscv-tests repository.

**Usage**:
```bash
./scripts/build_compliance_tests.sh
```

**What it does**:
1. Clones riscv-tests repository (if needed)
2. Compiles all RV32I ISA tests
3. Converts ELF files to hex format
4. Organizes files in `tests/compliance/`

**Requirements**:
- RISC-V GCC toolchain (`riscv64-unknown-elf-gcc`)
- Make
- Git
- Python 3 (for ELF to hex conversion)

### `run_compliance_test.sh`

Runs a single compliance test.

**Usage**:
```bash
./scripts/run_compliance_test.sh <test_name> [options]
```

**Options**:
- `--verbose`: Enable verbose output
- `--debug`: Enable debug output
- `--waveform`: Generate waveform file

**Example**:
```bash
./scripts/run_compliance_test.sh add --verbose
```

### `run_all_compliance_tests.sh`

Runs all compliance tests and generates a summary report.

**Usage**:
```bash
./scripts/run_all_compliance_tests.sh [options]
```

**Options**:
- `--category <cat>`: Run only tests in category
- `--report <file>`: Generate HTML report
- `--verbose`: Enable verbose output

**Example**:
```bash
./scripts/run_all_compliance_tests.sh --report compliance_report.html
```

### `elf_to_hex.py`

Converts RISC-V ELF files to hex format for instruction memory initialization.

**Usage**:
```bash
python3 scripts/elf_to_hex.py input.elf output.hex
```

**Requirements**:
- Python 3
- Optional: `pyelftools` (for better ELF parsing)

**Install pyelftools**:
```bash
pip install pyelftools
```

## Quick Start

### Step 1: Install RISC-V Toolchain

**Linux (Ubuntu/Debian)**:
```bash
sudo apt-get install gcc-riscv64-unknown-elf
```

**macOS**:
```bash
brew install riscv-gnu-toolchain
```

**Windows (WSL)**:
```bash
# In WSL Ubuntu
sudo apt-get install gcc-riscv64-unknown-elf
```

### Step 2: Build Tests

```bash
./scripts/build_compliance_tests.sh
```

This will:
- Clone riscv-tests repository
- Compile all tests
- Convert to hex format
- Place files in `tests/compliance/hex/`

### Step 3: Run Tests

**Single test**:
```bash
./scripts/run_compliance_test.sh add
```

**All tests**:
```bash
./scripts/run_all_compliance_tests.sh --report report.html
```

## Windows Users

On Windows, use one of these options:

1. **WSL (Windows Subsystem for Linux)**: Recommended
   - Install WSL Ubuntu
   - Run scripts in WSL

2. **Git Bash**: Works for most scripts
   - Install Git for Windows
   - Run scripts in Git Bash

3. **Cygwin/MSYS2**: Alternative Unix-like environment

## Troubleshooting

### Toolchain Not Found

**Error**: `riscv64-unknown-elf-gcc: command not found`

**Solution**:
```bash
# Check if installed
which riscv64-unknown-elf-gcc

# If not found, add to PATH
export PATH=/opt/riscv/bin:$PATH
```

### Python Script Fails

**Error**: `ModuleNotFoundError: No module named 'elftools'`

**Solution**:
```bash
# Install pyelftools (optional but recommended)
pip install pyelftools

# Or use basic ELF reader (less reliable)
# Script will fall back automatically
```

### Test Compilation Fails

**Error**: `undefined reference to 'main'`

**Solution**:
- Ensure using correct linker script from riscv-tests
- Check test file includes proper headers
- Verify architecture flags (`-march=rv32i`)

## See Also

- [`docs/RISCV_COMPLIANCE_TESTS.md`](../docs/RISCV_COMPLIANCE_TESTS.md) - Complete guide
- [riscv-tests Repository](https://github.com/riscv/riscv-tests)
- [RISC-V GNU Toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)

