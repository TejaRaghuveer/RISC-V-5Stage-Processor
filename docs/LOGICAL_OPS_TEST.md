# RISC-V Logical Operations Test Program

This document describes a comprehensive RISC-V assembly test program for logical operations: AND, OR, XOR, ANDI, ORI, XORI.

## Test Program Overview

**File**: `mem/logical_ops_test.hex`  
**Assembly Source**: `mem/logical_ops_test.s`

### Purpose
Tests all bitwise logical operations with:
- Basic operations (AND, OR, XOR)
- Immediate operations (ANDI, ORI, XORI)
- Bit masking examples
- Complement operations
- Edge cases (all 1s, all 0s, alternating patterns)

## Test Patterns Initialized

| Register | Value (Hex) | Binary Pattern | Description |
|----------|------------|----------------|-------------|
| x1 | 0xAAAAAAAA | 1010 1010 1010... | Alternating pattern (bits 1,0,1,0...) |
| x2 | 0x55555555 | 0101 0101 0101... | Alternating pattern (bits 0,1,0,1...) |
| x3 | 0xFFFFFFFF | 1111 1111 1111... | All 1s |
| x4 | 0x00000000 | 0000 0000 0000... | All 0s |
| x5 | 0x000000FF | 0000 0000 1111 1111 | Lower byte mask |
| x6 | 0x000000F0 | 0000 0000 1111 0000 | Upper nibble mask |
| x7 | 0x0000000F | 0000 0000 0000 1111 | Lower nibble mask |
| x9 | 0x12345678 | 0001 0010 0011... | Test value |

## Test Cases

### 1. AND Operations (R-Type)

**Purpose**: Test bitwise AND operations for masking and bit extraction.

| Instruction | Operands | Result | Description |
|------------|----------|--------|-------------|
| `AND x11, x1, x2` | 0xAAAAAAAA & 0x55555555 | 0x00000000 | No common bits |
| `AND x12, x1, x1` | 0xAAAAAAAA & 0xAAAAAAAA | 0xAAAAAAAA | Identity operation |
| `AND x13, x1, x3` | 0xAAAAAAAA & 0xFFFFFFFF | 0xAAAAAAAA | Mask with all 1s (no change) |
| `AND x14, x1, x4` | 0xAAAAAAAA & 0x00000000 | 0x00000000 | Mask with all 0s (clear) |
| `AND x19, x9, x5` | 0x12345678 & 0x000000FF | 0x00000078 | Extract lower byte |

**Expected Results**:
- x11 = 0x00000000 (no bits in common)
- x12 = 0xAAAAAAAA (identity)
- x13 = 0xAAAAAAAA (preserved by all-1s mask)
- x14 = 0x00000000 (cleared by all-0s mask)
- x19 = 0x00000078 (lower byte extracted)

### 2. OR Operations (R-Type)

**Purpose**: Test bitwise OR operations for bit setting.

| Instruction | Operands | Result | Description |
|------------|----------|--------|-------------|
| `OR x22, x1, x2` | 0xAAAAAAAA \| 0x55555555 | 0xFFFFFFFF | All bits set |
| `OR x23, x1, x1` | 0xAAAAAAAA \| 0xAAAAAAAA | 0xAAAAAAAA | Identity operation |
| `OR x24, x1, x3` | 0xAAAAAAAA \| 0xFFFFFFFF | 0xFFFFFFFF | OR with all 1s (all set) |
| `OR x25, x1, x4` | 0xAAAAAAAA \| 0x00000000 | 0xAAAAAAAA | OR with all 0s (no change) |
| `OR x30, x9, x5` | 0x12345678 \| 0x000000FF | 0x123456FF | Set lower byte |

**Expected Results**:
- x22 = 0xFFFFFFFF (complementary patterns combine)
- x23 = 0xAAAAAAAA (identity)
- x24 = 0xFFFFFFFF (all bits set)
- x25 = 0xAAAAAAAA (unchanged)
- x30 = 0x123456FF (lower byte set)

### 3. XOR Operations (R-Type)

**Purpose**: Test bitwise XOR operations for complement and toggle operations.

| Instruction | Operands | Result | Description |
|------------|----------|--------|-------------|
| `XOR x10, x1, x2` | 0xAAAAAAAA ^ 0x55555555 | 0xFFFFFFFF | Complementary patterns |
| `XOR x11, x1, x1` | 0xAAAAAAAA ^ 0xAAAAAAAA | 0x00000000 | Self-XOR = 0 |
| `XOR x12, x1, x3` | 0xAAAAAAAA ^ 0xFFFFFFFF | 0x55555555 | Complement operation |
| `XOR x13, x1, x4` | 0xAAAAAAAA ^ 0x00000000 | 0xAAAAAAAA | XOR with 0 = identity |
| `XOR x18, x9, x3` | 0x12345678 ^ 0xFFFFFFFF | 0xEDCBA987 | Bitwise complement |
| `XOR x21, x9, x5` | 0x12345678 ^ 0x000000FF | 0x12345687 | Toggle lower byte |

**Expected Results**:
- x10 = 0xFFFFFFFF (complementary patterns XOR to all 1s)
- x11 = 0x00000000 (self-XOR always zero)
- x12 = 0x55555555 (complement of x1)
- x13 = 0xAAAAAAAA (unchanged)
- x18 = 0xEDCBA987 (bitwise complement)
- x21 = 0x12345687 (lower byte toggled)

### 4. ANDI Operations (I-Type)

**Purpose**: Test immediate AND operations for bit masking.

| Instruction | Operands | Result | Description |
|------------|----------|--------|-------------|
| `ANDI x22, x1, 0x555` | 0xAAAAAAAA & 0x555 | 0x00000000 | No match |
| `ANDI x23, x1, 0xAAA` | 0xAAAAAAAA & 0xAAA | 0x00000AAA | Pattern match |
| `ANDI x28, x9, 0xFF` | 0x12345678 & 0xFF | 0x00000078 | Extract lower byte |
| `ANDI x29, x9, 0xF0` | 0x12345678 & 0xF0 | 0x00000070 | Extract upper nibble |
| `ANDI x30, x9, 0x0F` | 0x12345678 & 0x0F | 0x00000008 | Extract lower nibble |

**Expected Results**:
- x22 = 0x00000000 (no matching bits)
- x23 = 0x00000AAA (pattern matched)
- x28 = 0x00000078 (lower byte)
- x29 = 0x00000070 (upper nibble)
- x30 = 0x00000008 (lower nibble)

### 5. ORI Operations (I-Type)

**Purpose**: Test immediate OR operations for bit setting.

| Instruction | Operands | Result | Description |
|------------|----------|--------|-------------|
| `ORI x10, x1, 0x555` | 0xAAAAAAAA \| 0x555 | 0xAAAAAFFF | Set bits |
| `ORI x14, x3, 0xFFF` | 0xFFFFFFFF \| 0xFFF | 0xFFFFFFFF | Already all 1s |
| `ORI x15, x4, 0xFFF` | 0x00000000 \| 0xFFF | 0x00000FFF | Set lower 12 bits |
| `ORI x16, x9, 0xFF` | 0x12345678 \| 0xFF | 0x123456FF | Set lower byte |
| `ORI x17, x4, 0xAAA` | 0x00000000 \| 0xAAA | 0x00000AAA | Set pattern |

**Expected Results**:
- x10 = 0xAAAAAFFF (bits set)
- x14 = 0xFFFFFFFF (unchanged, already all 1s)
- x15 = 0x00000FFF (lower 12 bits set)
- x16 = 0x123456FF (lower byte set)
- x17 = 0x00000AAA (pattern set)

### 6. XORI Operations (I-Type)

**Purpose**: Test immediate XOR operations for bit toggling and complement.

| Instruction | Operands | Result | Description |
|------------|----------|--------|-------------|
| `XORI x19, x1, 0x555` | 0xAAAAAAAA ^ 0x555 | 0xAAAAAFFF | Toggle bits |
| `XORI x23, x3, 0xFFF` | 0xFFFFFFFF ^ 0xFFF | 0xFFFFF000 | Complement lower 12 bits |
| `XORI x24, x4, 0xFFF` | 0x00000000 ^ 0xFFF | 0x00000FFF | Set lower 12 bits (XOR with 0) |
| `XORI x25, x9, 0xFFF` | 0x12345678 ^ 0xFFF | 0x12345887 | Toggle lower 12 bits |
| `XORI x27, x9, 0xFF` | 0x12345678 ^ 0xFF | 0x12345687 | Toggle lower byte |

**Expected Results**:
- x19 = 0xAAAAAFFF (bits toggled)
- x23 = 0xFFFFF000 (lower 12 bits complemented)
- x24 = 0x00000FFF (lower 12 bits set)
- x25 = 0x12345887 (lower 12 bits toggled)
- x27 = 0x12345687 (lower byte toggled)

## Bit Masking Examples

### Extract Lower Byte
```assembly
ANDI x28, x9, 0xFF    # x28 = 0x12345678 & 0xFF = 0x00000078
```

### Extract Upper Nibble of Lower Byte
```assembly
ANDI x29, x9, 0xF0    # x29 = 0x12345678 & 0xF0 = 0x00000070
```

### Extract Lower Nibble of Lower Byte
```assembly
ANDI x30, x9, 0x0F    # x30 = 0x12345678 & 0x0F = 0x00000008
```

### Set Lower Byte
```assembly
ORI x16, x9, 0xFF     # x16 = 0x12345678 | 0xFF = 0x123456FF
```

### Toggle Lower Byte
```assembly
XORI x27, x9, 0xFF    # x27 = 0x12345678 ^ 0xFF = 0x12345687
```

## Complement Operations

### Bitwise Complement (XOR with all 1s)
```assembly
XOR x18, x9, x3        # x18 = 0x12345678 ^ 0xFFFFFFFF = 0xEDCBA987
```

### Complement Lower 12 Bits
```assembly
XORI x23, x3, 0xFFF    # x23 = 0xFFFFFFFF ^ 0xFFF = 0xFFFFF000
```

## Expected Memory Contents

After execution, memory contains:

| Address | Value (Hex) | Description |
|---------|-------------|-------------|
| 0 | 0x00000000 | AND: alternating patterns (no match) |
| 1 | 0xAAAAAAAA | AND: identity operation |
| 2 | 0xAAAAAAAA | AND: mask with all 1s |
| 3 | 0x00000000 | AND: mask with all 0s |
| 4 | 0x00000078 | AND: extract lower byte |
| 5 | 0xFFFFFFFF | OR: complementary patterns |
| 6 | 0xAAAAAAAA | OR: identity |
| 7 | 0xFFFFFFFF | OR: with all 1s |
| 8 | 0xAAAAAAAA | OR: with all 0s |
| 9 | 0x123456FF | OR: set lower byte |
| 10 | 0xFFFFFFFF | XOR: complementary patterns |
| 11 | 0x00000000 | XOR: self-XOR |
| 12 | 0x55555555 | XOR: complement |
| 13 | 0xEDCBA987 | XOR: bitwise complement |
| 14 | 0x00000078 | ANDI: extract lower byte |
| 15 | 0x00000070 | ANDI: extract upper nibble |
| 16 | 0x00000008 | ANDI: extract lower nibble |
| 17 | 0x123456FF | ORI: set lower byte |
| 18 | 0x00000AAA | ORI: set pattern |
| 19 | 0x12345887 | XORI: toggle lower 12 bits |
| 20 | 0x12345687 | XORI: toggle lower byte |
| 21 | 0x00000000 | Comparison pass (AND test) |
| 22 | 0x00000000 | Comparison pass (OR test) |
| 23 | 0x00000000 | Comparison pass (XOR test) |
| 24 | 0x00000000 | Comparison pass (ANDI test) |

## Self-Checking Verification

The program includes self-checking code that:
1. Loads results from memory
2. Compares with expected values
3. Stores comparison results (0 = pass, non-zero = fail)

Verification locations:
- memory[21] = AND test result (should be 0)
- memory[22] = OR test result (should be 0)
- memory[23] = XOR test result (should be 0)
- memory[24] = ANDI test result (should be 0)

## Usage Instructions

### Loading the Test Program

```systemverilog
// In testbench, set:
parameter IMEM_INIT_FILE = "mem/logical_ops_test.hex";
```

### Running the Test

1. Load the hex file in your testbench
2. Run simulation for sufficient cycles (typically 60-80 cycles)
3. Check register file contents using `display_register_state()`
4. Check memory contents using `dump_data_memory()`
5. Verify comparison results in memory[21-24] (should all be 0)

### Verification Checklist

- [ ] x1 = 0xAAAAAAAA (alternating pattern initialized)
- [ ] x2 = 0x55555555 (alternating pattern initialized)
- [ ] x3 = 0xFFFFFFFF (all 1s initialized)
- [ ] x4 = 0x00000000 (all 0s initialized)
- [ ] x11 = 0x00000000 (AND: no common bits)
- [ ] x22 = 0xFFFFFFFF (OR: complementary patterns)
- [ ] x10 = 0xFFFFFFFF (XOR: complementary patterns)
- [ ] x18 = 0xEDCBA987 (XOR: bitwise complement)
- [ ] x28 = 0x00000078 (ANDI: extract lower byte)
- [ ] x16 = 0x123456FF (ORI: set lower byte)
- [ ] x27 = 0x12345687 (XORI: toggle lower byte)
- [ ] memory[21] = 0 (AND verification pass)
- [ ] memory[22] = 0 (OR verification pass)
- [ ] memory[23] = 0 (XOR verification pass)
- [ ] memory[24] = 0 (ANDI verification pass)

## Key Concepts Tested

1. **Bit Masking**: Using AND/ANDI to extract specific bits
2. **Bit Setting**: Using OR/ORI to set specific bits
3. **Bit Toggling**: Using XOR/XORI to toggle specific bits
4. **Complement Operations**: Using XOR with all 1s to complement bits
5. **Identity Operations**: Operations that preserve values
6. **Zero Operations**: Operations that clear values
7. **Edge Cases**: All 1s, all 0s, alternating patterns

## Instruction Encoding Reference

### R-Type Logical Operations
- **AND**: opcode=0x33, funct3=111, funct7=0000000
- **OR**: opcode=0x33, funct3=110, funct7=0000000
- **XOR**: opcode=0x33, funct3=100, funct7=0000000

### I-Type Logical Operations
- **ANDI**: opcode=0x13, funct3=111
- **ORI**: opcode=0x13, funct3=110
- **XORI**: opcode=0x13, funct3=100

## References

- RISC-V Instruction Set Manual: https://riscv.org/specifications/
- Testbench Guide: `docs/TESTBENCH_GUIDE.md`
- Arithmetic Test Programs: `docs/ARITHMETIC_TEST_PROGRAMS.md`

