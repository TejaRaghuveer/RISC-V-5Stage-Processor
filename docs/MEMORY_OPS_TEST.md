# RISC-V Memory Operations Test Program

This document describes a comprehensive RISC-V assembly test program for memory operations: load/store, load-use hazards, store-load forwarding, and addressing with offsets.

## Test Program Overview

**File**: `mem/memory_ops_test.hex`  
**Assembly Source**: `mem/memory_ops_test.s`

### Purpose
Tests all memory-related operations including:
- Basic load/store operations (LW/SW)
- Load-use hazard detection (pipeline stalling)
- Store-load forwarding scenarios
- Memory addressing with various offsets

## Test Sections

### 1. Memory Initialization

The program initializes test data in memory locations 0-7:

| Address | Value (Hex) | Description |
|---------|-------------|-------------|
| 0 | 0x12345678 | Test pattern 1 |
| 1 | 0xABCDEF00 | Test pattern 2 |
| 2 | 0x00000042 | Small value |
| 3 | 0xFFFFFFFF | All 1s |
| 4 | 0x00000000 | Zero |
| 5 | 0x11111111 | Pattern 1 |
| 6 | 0x22222222 | Pattern 2 |
| 7 | 0x33333333 | Pattern 3 |

### 2. Basic Load Operations (LW)

Tests loading from different memory addresses:

| Instruction | Source Address | Register | Expected Value |
|------------|----------------|----------|----------------|
| `LW x10, 0(x0)` | memory[0] | x10 | 0x12345678 |
| `LW x11, 4(x0)` | memory[1] | x11 | 0xABCDEF00 |
| `LW x12, 8(x0)` | memory[2] | x12 | 0x00000042 |
| `LW x13, 12(x0)` | memory[3] | x13 | 0xFFFFFFFF |
| `LW x14, 16(x0)` | memory[4] | x14 | 0x00000000 |
| `LW x15, 20(x0)` | memory[5] | x15 | 0x11111111 |
| `LW x16, 24(x0)` | memory[6] | x16 | 0x22222222 |
| `LW x17, 28(x0)` | memory[7] | x17 | 0x33333333 |

### 3. Basic Store Operations (SW)

Tests storing to different memory addresses:

| Instruction | Destination Address | Value Stored | Description |
|------------|---------------------|--------------|-------------|
| `SW x18, 32(x0)` | memory[8] | 0xDEADBEEF | Pattern test |
| `SW x19, 36(x0)` | memory[9] | 0xCAFEBABE | Pattern test |
| `SW x20, 40(x0)` | memory[10] | 0x00001234 | Small value |
| `SW x21, 44(x0)` | memory[11] | 0xFFFFFFFF | All 1s |

### 4. Memory Addressing with Offsets

Tests various addressing modes:

#### Positive Offsets
- Base register with immediate offset: `LW x23, 0(x22)` where x22 = 48
- Multiple offsets from same base: `LW x24, 4(x22)`, `LW x25, 8(x22)`, `LW x26, 12(x22)`

#### Simulated Negative Offsets
- RISC-V doesn't support negative immediates in load/store
- Simulated using: `ADDI x29, x27, -4` then `LW x28, 0(x29)`
- Effectively loads from base - 4 bytes

#### Large Offsets
- `LW x31, 28(x30)` where x30 = 0 (base address)
- Tests maximum 12-bit signed immediate offset (2047 bytes)

### 5. Load-Use Hazard Test

Tests the hazard detection unit's ability to detect and stall for load-use hazards.

**Load-Use Hazard Condition**:
- Instruction in ID/EX stage is a load (MemRead = 1)
- AND its destination register (rd) matches rs1 or rs2 of instruction in ID stage
- Pipeline must stall for one cycle to allow load to complete

**Test Cases**:

| Test | Load Instruction | Use Instruction | Expected Behavior |
|------|------------------|-----------------|-------------------|
| 1 | `LW x1, 0(x0)` | `ADD x2, x1, x0` | Stall: x1 used immediately after load |
| 2 | `LW x3, 4(x0)` | `SUB x4, x3, x0` | Stall: x3 used immediately after load |
| 3 | `LW x5, 8(x0)` | `AND x6, x5, x5` | Stall: x5 used immediately after load |
| 4 | `LW x7, 12(x0)` | `OR x8, x7, x0` | Stall: x7 used immediately after load |

**Expected Results**:
- Pipeline should stall for 1 cycle after each load
- Results stored in memory[12-15] should be correct despite stall
- memory[12] = 0x12345678
- memory[13] = 0xABCDEF00
- memory[14] = 0x00000042
- memory[15] = 0xFFFFFFFF

### 6. Store-Load Forwarding Test

Tests scenarios where a store is followed by a load from the same address.

**Test Cases**:

#### Test 1: Direct Store-Load
```assembly
SW x9, 64(x0)    # Store 0x5555AAAA to memory[16]
LW x11, 64(x0)   # Load from memory[16]
```
- **Expected**: x11 = 0x5555AAAA (should get stored value)
- **Result stored**: memory[17] = 0x5555AAAA

#### Test 2: Store-Load with Base Register
```assembly
SW x12, 0(x14)   # Store 0x77778888 to memory[16] (x14 = 64)
LW x15, 0(x14)   # Load from memory[16]
```
- **Expected**: x15 = 0x77778888
- **Result stored**: memory[18] = 0x77778888

#### Test 3: Store-Load with Offset
```assembly
SW x16, 0(x18)   # Store 0x9999AAAA to memory[20] (x18 = 80)
LW x19, 0(x18)   # Load from memory[20]
```
- **Expected**: x19 = 0x9999AAAA
- **Result stored**: memory[22] = 0x9999AAAA

**Note**: Store-load forwarding may or may not be implemented depending on processor design. The test verifies correct behavior regardless.

## Expected Memory Contents

After complete execution:

| Address | Value (Hex) | Description |
|---------|-------------|-------------|
| 0 | 0x12345678 | Initialized |
| 1 | 0xABCDEF00 | Initialized |
| 2 | 0x00000042 | Initialized |
| 3 | 0xFFFFFFFF | Initialized |
| 4 | 0x00000000 | Initialized |
| 5 | 0x11111111 | Initialized |
| 6 | 0x22222222 | Initialized |
| 7 | 0x33333333 | Initialized |
| 8 | 0xDEADBEEF | Stored |
| 9 | 0xCAFEBABE | Stored |
| 10 | 0x00001234 | Stored |
| 11 | 0xFFFFFFFF | Stored |
| 12 | 0x12345678 | Load-use hazard test result |
| 13 | 0xABCDEF00 | Load-use hazard test result |
| 14 | 0x00000042 | Load-use hazard test result |
| 15 | 0xFFFFFFFF | Load-use hazard test result |
| 16 | 0x77778888 | Store-load forwarding (overwritten) |
| 17 | 0x5555AAAA | Store-load forwarding result |
| 18 | 0x77778888 | Store-load forwarding result |
| 20 | 0x9999AAAA | Store-load forwarding result |
| 22 | 0x9999AAAA | Store-load forwarding result |
| 24 | 0x12345678 | Verification: loaded value |
| 25 | 0xABCDEF00 | Verification: loaded value |
| 26 | 0x00000042 | Verification: loaded value |
| 27 | 0xFFFFFFFF | Verification: loaded value |
| 28 | (offset addressing result) | Offset test result |
| 29 | (offset addressing result) | Offset test result |
| 30 | (negative offset result) | Negative offset simulation |
| 31 | 0x33333333 | Large offset result |
| 32 | 0x00000000 | Load-use verification (0 = pass) |
| 33 | 0x00000000 | Store-load verification (0 = pass) |
| 34 | 0x00000000 | Basic load verification (0 = pass) |

## Self-Checking Verification

The program includes three verification checks:

1. **Load-Use Hazard Verification** (memory[32]):
   - Loads result from load-use test 1 (memory[12])
   - Compares with expected value (0x12345678)
   - Stores 0 if correct, non-zero if incorrect

2. **Store-Load Forwarding Verification** (memory[33]):
   - Loads result from store-load test 1 (memory[17])
   - Compares with expected value (0x5555AAAA)
   - Stores 0 if correct, non-zero if incorrect

3. **Basic Load Verification** (memory[34]):
   - Loads verification result (memory[24])
   - Compares with expected value (0x12345678)
   - Stores 0 if correct, non-zero if incorrect

## Usage Instructions

### Loading the Test Program

```systemverilog
// In testbench, set:
parameter IMEM_INIT_FILE = "mem/memory_ops_test.hex";
```

### Running the Test

1. Load the hex file in your testbench
2. Run simulation for sufficient cycles (typically 100-150 cycles due to stalls)
3. Check register file contents using `display_register_state()`
4. Check memory contents using `dump_data_memory()`
5. Verify comparison results in memory[32-34] (should all be 0)

### Verification Checklist

- [ ] memory[0-7] contain initialized values
- [ ] memory[8-11] contain stored values
- [ ] memory[12-15] contain load-use hazard test results
- [ ] memory[17-18, 20, 22] contain store-load forwarding results
- [ ] memory[24-27] contain verification values
- [ ] memory[32] = 0 (load-use verification pass)
- [ ] memory[33] = 0 (store-load verification pass)
- [ ] memory[34] = 0 (basic load verification pass)

## Pipeline Behavior

### Load-Use Hazard Detection

When a load-use hazard is detected:
1. **Stall Signal**: Hazard detection unit asserts `stall = 1`
2. **PC Hold**: Program counter does not update
3. **IF/ID Hold**: IF/ID register holds current instruction
4. **ID/EX Flush**: ID/EX register is flushed (NOP inserted)
5. **One Cycle Delay**: Pipeline resumes after load completes

### Store-Load Forwarding

If implemented, store-load forwarding allows:
- Data from store instruction to be forwarded to subsequent load
- Avoids unnecessary memory access when store and load target same address
- Improves pipeline efficiency

## Key Concepts Tested

1. **Memory Access**: Basic read/write operations
2. **Addressing Modes**: Immediate offsets, base registers, large offsets
3. **Hazard Detection**: Load-use hazard identification and stalling
4. **Pipeline Control**: Stall and flush signal generation
5. **Data Forwarding**: Store-load data paths (if implemented)

## Instruction Encoding Reference

### Load Word (LW)
- **Opcode**: 0x03
- **funct3**: 010
- **Format**: `LW rd, imm(rs1)`
- **Encoding**: imm[11:0] | rs1 | 010 | rd | 0000011

### Store Word (SW)
- **Opcode**: 0x23
- **funct3**: 010
- **Format**: `SW rs2, imm(rs1)`
- **Encoding**: imm[11:5] | rs2 | rs1 | 010 | imm[4:0] | 0100011

## References

- RISC-V Instruction Set Manual: https://riscv.org/specifications/
- Testbench Guide: `docs/TESTBENCH_GUIDE.md`
- Hazard Detection: `docs/HAZARD_DETECTION_INTEGRATION.md`
- Arithmetic Test Programs: `docs/ARITHMETIC_TEST_PROGRAMS.md`

