# RISC-V Arithmetic Test Programs

This document describes three comprehensive RISC-V assembly test programs designed to verify arithmetic operations in the processor.

## Test Programs Overview

1. **ADD/SUB Test** (`mem/add_sub_test.hex`) - Tests addition and subtraction with positive/negative numbers and overflow cases
2. **ADDI Test** (`mem/addi_subi_test.hex`) - Tests immediate arithmetic operations (ADDI with positive/negative immediates)
3. **SLT/SLTU Test** (`mem/slt_sltu_test.hex`) - Tests signed and unsigned comparison operations

## 1. ADD/SUB Test Program

### Purpose
Tests register-register arithmetic operations (ADD, SUB) with:
- Positive and negative operands
- Zero operands
- Overflow cases (max positive + 1, max + max, min negative - 1)

### Test Cases

#### Basic ADD Operations
- `ADD x10, x1, x2`: 5 + 3 = 8
- `ADD x11, x3, x4`: -5 + (-3) = -8
- `ADD x12, x1, x3`: 5 + (-5) = 0
- `ADD x13, x1, x4`: 5 + (-3) = 2
- `ADD x14, x3, x2`: -5 + 3 = -2
- `ADD x15, x1, x7`: 5 + 0 = 5
- `ADD x16, x3, x7`: -5 + 0 = -5

#### Basic SUB Operations
- `SUB x20, x1, x2`: 5 - 3 = 2
- `SUB x21, x2, x1`: 3 - 5 = -2
- `SUB x22, x3, x4`: -5 - (-3) = -2
- `SUB x23, x1, x3`: 5 - (-5) = 10
- `SUB x24, x3, x1`: -5 - 5 = -10
- `SUB x25, x1, x7`: 5 - 0 = 5
- `SUB x26, x2, x7`: 3 - 0 = 3

#### Overflow Cases
- `ADD x30, x5, x8`: 0x7FFFFFFF + 1 = 0x80000000 (overflow wrap)
- `ADD x31, x5, x5`: 0x7FFFFFFF + 0x7FFFFFFF = 0xFFFFFFFE (overflow wrap)
- `SUB x28, x6, x8`: 0x80000000 - 1 = 0x7FFFFFFF (underflow wrap)
- `SUB x29, x6, x6`: 0x80000000 - 0x80000000 = 0

### Expected Results

| Register | Value (Hex) | Value (Decimal) | Description |
|----------|-------------|-----------------|-------------|
| x10 | 0x00000008 | 8 | 5 + 3 |
| x11 | 0xFFFFFFF8 | -8 | -5 + (-3) |
| x12 | 0x00000000 | 0 | 5 + (-5) |
| x13 | 0x00000002 | 2 | 5 + (-3) |
| x14 | 0xFFFFFFFE | -2 | -5 + 3 |
| x20 | 0x00000002 | 2 | 5 - 3 |
| x21 | 0xFFFFFFFE | -2 | 3 - 5 |
| x22 | 0xFFFFFFFE | -2 | -5 - (-3) |
| x23 | 0x0000000A | 10 | 5 - (-5) |
| x24 | 0xFFFFFFF6 | -10 | -5 - 5 |
| x30 | 0x80000000 | -2147483648 | Overflow: max+1 |
| x31 | 0xFFFFFFFE | -2 | Overflow: max+max |
| x28 | 0x7FFFFFFF | 2147483647 | Underflow: min-1 |
| x29 | 0x00000000 | 0 | min - min |

### Self-Checking
The program stores results in memory and performs comparisons:
- memory[0] = 8 (verified)
- memory[1] = -8 (verified)
- memory[10] = 0x80000000 (overflow verified)

## 2. ADDI Test Program

### Purpose
Tests immediate arithmetic operations using ADDI with:
- Positive immediates (0 to 2047)
- Negative immediates (-1 to -2048, acts as SUBI)
- Large positive and negative base values
- Overflow cases

### Test Cases

#### ADDI with Positive Immediates
- `ADDI x10, x1, 5`: 10 + 5 = 15
- `ADDI x11, x1, 0`: 10 + 0 = 10
- `ADDI x12, x0, 42`: 0 + 42 = 42
- `ADDI x13, x1, 1`: 10 + 1 = 11
- `ADDI x14, x1, 2047`: 10 + 2047 = 2057

#### ADDI with Negative Immediates (SUBI-like)
- `ADDI x20, x1, -5`: 10 + (-5) = 5
- `ADDI x21, x1, -10`: 10 + (-10) = 0
- `ADDI x22, x1, -1`: 10 + (-1) = 9
- `ADDI x23, x2, -3`: 5 + (-3) = 2
- `ADDI x24, x0, -1`: 0 + (-1) = -1
- `ADDI x25, x0, -2048`: 0 + (-2048) = -2048

#### Overflow Cases
- `ADDI x6, x4, 1`: 0x7FFFFFFF + 1 = 0x80000000 (overflow)
- `ADDI x7, x4, 2047`: 0x7FFFFFFF + 2047 = 0x800007FE (overflow)
- `ADDI x8, x5, -1`: 0x80000000 + (-1) = 0x7FFFFFFF (underflow)

### Expected Results

| Register | Value (Hex) | Value (Decimal) | Description |
|----------|-------------|-----------------|-------------|
| x10 | 0x0000000F | 15 | 10 + 5 |
| x11 | 0x0000000A | 10 | 10 + 0 |
| x12 | 0x0000002A | 42 | 0 + 42 |
| x13 | 0x0000000B | 11 | 10 + 1 |
| x14 | 0x00000809 | 2057 | 10 + 2047 |
| x20 | 0x00000005 | 5 | 10 - 5 |
| x21 | 0x00000000 | 0 | 10 - 10 |
| x22 | 0x00000009 | 9 | 10 - 1 |
| x23 | 0x00000002 | 2 | 5 - 3 |
| x24 | 0xFFFFFFFF | -1 | 0 - 1 |
| x25 | 0xFFFFF800 | -2048 | 0 - 2048 |
| x6 | 0x80000000 | -2147483648 | Overflow |
| x7 | 0x800007FE | -2147482626 | Overflow |
| x8 | 0x7FFFFFFF | 2147483647 | Underflow |

### Self-Checking
The program verifies:
- memory[0] = 15 (verified)
- memory[5] = 5 (verified)
- memory[9] = -1 (verified)
- memory[11] = 0x80000000 (overflow verified)

## 3. SLT/SLTU Test Program

### Purpose
Tests signed and unsigned comparison operations:
- **SLT**: Set Less Than (signed comparison)
- **SLTU**: Set Less Than Unsigned (unsigned comparison)

### Test Cases

#### SLT Tests (Signed Comparison)
- `SLT x10, x2, x1`: (3 < 5) = 1 ✓
- `SLT x11, x1, x2`: (5 < 3) = 0 ✓
- `SLT x12, x1, x1`: (5 < 5) = 0 ✓
- `SLT x13, x3, x4`: (-5 < -3) = 1 ✓
- `SLT x14, x4, x3`: (-3 < -5) = 0 ✓
- `SLT x15, x3, x1`: (-5 < 5) = 1 ✓
- `SLT x16, x1, x3`: (5 < -5) = 0 ✓
- `SLT x17, x5, x1`: (0 < 5) = 1 ✓
- `SLT x18, x1, x5`: (5 < 0) = 0 ✓
- `SLT x19, x5, x3`: (0 < -5) = 0 ✓
- `SLT x20, x3, x5`: (-5 < 0) = 1 ✓
- `SLT x21, x6, x7`: (0x7FFFFFFF < 0x80000000) = 0 ✓
- `SLT x22, x7, x6`: (0x80000000 < 0x7FFFFFFF) = 1 ✓
- `SLT x22, x7, x8`: (0x80000000 < 1) = 1 ✓
- `SLT x24, x6, x9`: (0x7FFFFFFF < 0xFFFFFFFF) = 0 ✓

#### SLTU Tests (Unsigned Comparison)
- `SLTU x10, x2, x1`: (3 < 5) = 1 ✓
- `SLTU x11, x1, x2`: (5 < 3) = 0 ✓
- `SLTU x12, x1, x1`: (5 < 5) = 0 ✓
- `SLTU x13, x3, x4`: (0xFFFFFFFB < 0xFFFFFFFD) = 1 ✓
- `SLTU x14, x4, x3`: (0xFFFFFFFD < 0xFFFFFFFB) = 0 ✓
- `SLTU x15, x3, x1`: (0xFFFFFFFB < 5) = 0 ✓ (large unsigned > small)
- `SLTU x16, x1, x3`: (5 < 0xFFFFFFFB) = 1 ✓ (small < large unsigned)
- `SLTU x17, x5, x1`: (0 < 5) = 1 ✓
- `SLTU x18, x1, x5`: (5 < 0) = 0 ✓
- `SLTU x19, x5, x3`: (0 < 0xFFFFFFFB) = 1 ✓
- `SLTU x20, x3, x5`: (0xFFFFFFFB < 0) = 0 ✓
- `SLTU x21, x6, x7`: (0x7FFFFFFF < 0x80000000) = 1 ✓
- `SLTU x22, x7, x6`: (0x80000000 < 0x7FFFFFFF) = 0 ✓
- `SLTU x22, x7, x8`: (0x80000000 < 1) = 0 ✓
- `SLTU x24, x6, x9`: (0x7FFFFFFF < 0xFFFFFFFF) = 1 ✓

### Key Differences: SLT vs SLTU

| Comparison | SLT (Signed) | SLTU (Unsigned) | Reason |
|------------|--------------|-----------------|--------|
| -5 < 5 | 1 (true) | 0 (false) | Signed: negative < positive<br>Unsigned: 0xFFFFFFFB > 5 |
| 5 < -5 | 0 (false) | 1 (true) | Signed: positive > negative<br>Unsigned: 5 < 0xFFFFFFFB |
| 0x7FFFFFFF < 0x80000000 | 0 (false) | 1 (true) | Signed: max positive > min negative<br>Unsigned: smaller < larger |
| 0x80000000 < 0x7FFFFFFF | 1 (true) | 0 (false) | Signed: min negative < max positive<br>Unsigned: larger > smaller |

### Expected Results

**SLT Results** (memory[0-12]):
- memory[0] = 1 (3 < 5)
- memory[1] = 0 (5 < 3)
- memory[2] = 0 (5 < 5)
- memory[3] = 1 (-5 < -3)
- memory[4] = 0 (-3 < -5)
- memory[5] = 1 (-5 < 5)
- memory[6] = 0 (5 < -5)
- memory[7] = 1 (0 < 5)
- memory[8] = 0 (5 < 0)
- memory[9] = 0 (0 < -5)
- memory[10] = 1 (-5 < 0)
- memory[11] = 1 (min < 1)
- memory[12] = 0 (max < -1)

**SLTU Results** (memory[15-29]):
- memory[15] = 1 (3 < 5)
- memory[16] = 0 (5 < 3)
- memory[17] = 0 (5 < 5)
- memory[18] = 1 (0xFFFFFFFB < 0xFFFFFFFD)
- memory[19] = 0 (0xFFFFFFFD < 0xFFFFFFFB)
- memory[20] = 0 (0xFFFFFFFB < 5)
- memory[21] = 1 (5 < 0xFFFFFFFB)
- memory[22] = 1 (0 < 5)
- memory[23] = 0 (5 < 0)
- memory[24] = 1 (0 < 0xFFFFFFFB)
- memory[25] = 0 (0xFFFFFFFB < 0)
- memory[26] = 1 (0x7FFFFFFF < 0x80000000)
- memory[27] = 0 (0x80000000 < 0x7FFFFFFF)
- memory[28] = 0 (0x80000000 < 1)
- memory[29] = 1 (0x7FFFFFFF < 0xFFFFFFFF)

### Self-Checking
The program verifies:
- memory[0] = 1 (SLT: 3 < 5, verified)
- memory[16] = 0 (SLT: 5 < 3, verified)
- memory[20] = 0 (SLTU: large < small, verified)
- memory[21] = 1 (SLTU: small < large, verified)

## Usage Instructions

### Loading Test Programs

1. **ADD/SUB Test**:
   ```systemverilog
   parameter IMEM_INIT_FILE = "mem/add_sub_test.hex";
   ```

2. **ADDI Test**:
   ```systemverilog
   parameter IMEM_INIT_FILE = "mem/addi_subi_test.hex";
   ```

3. **SLT/SLTU Test**:
   ```systemverilog
   parameter IMEM_INIT_FILE = "mem/slt_sltu_test.hex";
   ```

### Running Tests

1. Load the appropriate hex file in your testbench
2. Run simulation for sufficient cycles (typically 50-100 cycles)
3. Check register file contents using `display_register_state()`
4. Check memory contents using `dump_data_memory()`
5. Verify comparison results in memory (should be 0 for pass)

### Verification Checklist

**ADD/SUB Test**:
- [ ] x10 = 8 (5 + 3)
- [ ] x11 = -8 (-5 + -3)
- [ ] x20 = 2 (5 - 3)
- [ ] x21 = -2 (3 - 5)
- [ ] x30 = 0x80000000 (overflow)
- [ ] memory[14] = 0 (comparison pass)

**ADDI Test**:
- [ ] x10 = 15 (10 + 5)
- [ ] x20 = 5 (10 - 5)
- [ ] x24 = -1 (0 - 1)
- [ ] x6 = 0x80000000 (overflow)
- [ ] memory[19] = 0 (comparison pass)

**SLT/SLTU Test**:
- [ ] memory[0] = 1 (SLT: 3 < 5)
- [ ] memory[1] = 0 (SLT: 5 < 3)
- [ ] memory[20] = 0 (SLTU: large < small)
- [ ] memory[21] = 1 (SLTU: small < large)
- [ ] memory[30] = 0 (comparison pass)

## File Structure

```
mem/
├── add_sub_test.s         # Assembly source (ADD/SUB test)
├── add_sub_test.hex       # Machine code (ADD/SUB test)
├── addi_subi_test.s       # Assembly source (ADDI test)
├── addi_subi_test.hex     # Machine code (ADDI test)
├── slt_sltu_test.s         # Assembly source (SLT/SLTU test)
└── slt_sltu_test.hex      # Machine code (SLT/SLTU test)

docs/
└── ARITHMETIC_TEST_PROGRAMS.md  # This documentation
```

## Notes

- All test programs are self-checking where possible
- Results are stored in memory for easy verification
- Overflow cases demonstrate two's complement wraparound behavior
- SLT vs SLTU differences are critical for correct signed/unsigned handling
- Negative immediates in ADDI effectively provide SUBI functionality

## References

- RISC-V Instruction Set Manual: https://riscv.org/specifications/
- Testbench Guide: `docs/TESTBENCH_GUIDE.md`
- Main Test Program: `docs/TEST_PROGRAM.md`

