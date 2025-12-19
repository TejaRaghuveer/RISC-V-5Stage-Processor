# RISC-V Compliance Test Runner Script

Python script to automate running RISC-V compliance tests on the processor.

## Quick Start

```bash
# Run all tests
python3 scripts/run_compliance_tests.py

# Run with verbose output
python3 scripts/run_compliance_tests.py --verbose

# Generate HTML report
python3 scripts/run_compliance_tests.py --report report.html

# Generate JSON report
python3 scripts/run_compliance_tests.py --json report.json
```

## Features

- **Automated Test Execution**: Runs all tests in a directory
- **Result Parsing**: Extracts register x1 value (pass/fail indicator)
- **Multiple Report Formats**: Text, HTML, and JSON
- **Error Handling**: Handles timeouts and simulation errors
- **Detailed Logging**: Saves logs for each test
- **Waveform Generation**: Optional VCD files for failed tests

## Usage

### Basic Usage

```bash
python3 scripts/run_compliance_tests.py
```

Runs all tests in `tests/compliance/hex/` directory.

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--test-dir <dir>` | Directory containing test hex files | `tests/compliance/hex` |
| `--simulator <sim>` | Simulator: `iverilog` or `modelsim` | `iverilog` |
| `--timeout <sec>` | Timeout per test in seconds | `60` |
| `--verbose` | Enable verbose output | `False` |
| `--debug` | Enable debug output | `False` |
| `--report <file>` | Generate HTML report | `compliance_report.html` |
| `--json <file>` | Generate JSON report | None |
| `--waveform` | Generate waveform files for failed tests | `False` |

### Examples

**Run specific test directory**:
```bash
python3 scripts/run_compliance_tests.py --test-dir tests/my_tests
```

**Run with longer timeout**:
```bash
python3 scripts/run_compliance_tests.py --timeout 120
```

**Generate both HTML and JSON reports**:
```bash
python3 scripts/run_compliance_tests.py --report report.html --json report.json
```

**Debug failed tests with waveforms**:
```bash
python3 scripts/run_compliance_tests.py --waveform --verbose
```

## Output

### Console Output

```
Running 50 compliance tests...

Running test: add
  ✓ PASSED (x1=0x00000001, 2.34s)
Running test: sub
  ✓ PASSED (x1=0x00000001, 2.12s)
Running test: and
  ✗ FAILED (x1=0x00000000, expected 0x00000001, 2.45s)

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
```

### HTML Report

Generates a styled HTML report with:
- Summary statistics
- Test results table
- Failed tests details
- Links to log files

### JSON Report

Generates machine-readable JSON with:
- Summary statistics
- Detailed results for each test
- Timestamps and durations

## Result Interpretation

### Pass/Fail Indicator

RISC-V compliance tests use register x1 to indicate results:
- **x1 = 0x00000001**: Test PASSED
- **x1 = 0x00000000**: Test FAILED

### Test Status

- **PASSED**: x1 = 1, test completed successfully
- **FAILED**: x1 = 0, test failed
- **ERROR**: Simulation error (compilation or runtime)
- **UNKNOWN**: Could not determine result (x1 value not found)

## Log Files

Log files are saved in `tests/compliance/logs/`:
- `{test_name}_compile.log`: Compilation log
- `{test_name}_run.log`: Simulation log

## Troubleshooting

### Tests Not Found

**Error**: `No test files found in tests/compliance/hex`

**Solution**:
1. Build tests first: `./scripts/build_compliance_tests.sh`
2. Check test directory: `ls tests/compliance/hex/`
3. Specify custom directory: `--test-dir /path/to/tests`

### Simulation Timeout

**Error**: `Simulation timeout (60s)`

**Solution**:
- Increase timeout: `--timeout 120`
- Check for infinite loops in test
- Verify processor is executing correctly

### Cannot Parse Results

**Error**: Test shows "UNKNOWN" status

**Solution**:
- Enable verbose mode: `--verbose`
- Check log file: `tests/compliance/logs/{test_name}_run.log`
- Verify testbench outputs register values
- May need to modify testbench to print register x1

### Compilation Errors

**Error**: `Compilation failed`

**Solution**:
- Check Icarus Verilog is installed: `iverilog -v`
- Verify source files exist: `ls src/*.sv`
- Check compilation log: `tests/compliance/logs/{test_name}_compile.log`

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: Compliance Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y iverilog gcc-riscv64-unknown-elf python3
      - name: Build tests
        run: ./scripts/build_compliance_tests.sh
      - name: Run tests
        run: python3 scripts/run_compliance_tests.py --json results.json
      - name: Upload results
        uses: actions/upload-artifact@v2
        with:
          name: test-results
          path: results.json
```

## See Also

- [`docs/RISCV_COMPLIANCE_TESTS.md`](../docs/RISCV_COMPLIANCE_TESTS.md) - Complete guide
- [`scripts/build_compliance_tests.sh`](build_compliance_tests.sh) - Build script
- [`scripts/run_compliance_test.sh`](run_compliance_test.sh) - Single test runner

