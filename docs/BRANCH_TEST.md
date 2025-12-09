# Branch Instructions Test Program Documentation

## Overview

This test program comprehensively tests all six RISC-V branch instruction types:
- **BEQ** (Branch if Equal) - funct3 = 000
- **BNE** (Branch if Not Equal) - funct3 = 001
- **BLT** (Branch if Less Than, Signed) - funct3 = 100
- **BGE** (Branch if Greater or Equal, Signed) - funct3 = 101
- **BLTU** (Branch if Less Than, Unsigned) - funct3 = 110
- **BGEU** (Branch if Greater or Equal, Unsigned) - funct3 = 111

## Files

- `mem/branch_test.s` - Assembly source code with detailed annotations
- `mem/branch_test.hex` - Machine code in hex format for simulation

## Test Coverage

### 1. BEQ (Branch if Equal) Tests

**Test Cases**:
- **2.1**: BEQ taken - equal values (forward branch)
  - Condition: `x1 == x1` (always true)
  - Expected: Branch taken, `x10 = 3`
  
- **2.2**: BEQ not taken - unequal values (forward branch)
  - Condition: `x1 == x2` (10 == 20, false)
  - Expected: Branch not taken, `x10 = 30`
  
- **2.3**: BEQ taken with zero (forward branch)
  - Condition: `x3 == x3` (0 == 0, true)
  - Expected: Branch taken, `x10 = 300`
  
- **2.4**: BEQ backward branch (taken)
  - Creates backward branch loop (for testing branch direction)

### 2. BNE (Branch if Not Equal) Tests

**Test Cases**:
- **3.1**: BNE taken - unequal values (forward branch)
  - Condition: `x1 != x2` (10 != 20, true)
  - Expected: Branch taken, `x11 = 3`
  
- **3.2**: BNE not taken - equal values (forward branch)
  - Condition: `x1 != x1` (10 != 10, false)
  - Expected: Branch not taken, `x11 = 30`
  
- **3.3**: BNE taken with zero (forward branch)
  - Condition: `x1 != x3` (10 != 0, true)
  - Expected: Branch taken, `x11 = 300`

### 3. BLT (Branch if Less Than, Signed) Tests

**Test Cases**:
- **4.1**: BLT taken - positive < positive
  - Condition: `x1 < x2` (10 < 20, true)
  - Expected: Branch taken, `x12 = 3`
  
- **4.2**: BLT not taken - positive >= positive
  - Condition: `x2 < x1` (20 < 10, false)
  - Expected: Branch not taken, `x12 = 30`
  
- **4.3**: BLT taken - negative < positive
  - Condition: `x4 < x1` (-10 < 10, true)
  - Expected: Branch taken, `x12 = 300`
  
- **4.4**: BLT taken - negative < negative
  - Condition: `x5 < x4` (-20 < -10, true)
  - Expected: Branch taken, `x12 = 3000`
  
- **4.5**: BLT not taken - positive < negative
  - Condition: `x1 < x4` (10 < -10, false)
  - Expected: Branch not taken, `x12 = 30000`
  
- **4.6**: BLT edge case - zero comparison
  - Condition: `x3 < x1` (0 < 10, true)
  - Expected: Branch taken, `x12 = 300000`

### 4. BGE (Branch if Greater or Equal, Signed) Tests

**Test Cases**:
- **5.1**: BGE taken - positive > positive
  - Condition: `x2 >= x1` (20 >= 10, true)
  - Expected: Branch taken, `x13 = 3`
  
- **5.2**: BGE taken - equal values
  - Condition: `x1 >= x1` (10 >= 10, true)
  - Expected: Branch taken, `x13 = 30`
  
- **5.3**: BGE not taken - positive < positive
  - Condition: `x1 >= x2` (10 >= 20, false)
  - Expected: Branch not taken, `x13 = 300`
  
- **5.4**: BGE taken - negative >= negative
  - Condition: `x4 >= x5` (-10 >= -20, true)
  - Expected: Branch taken, `x13 = 3000`
  
- **5.5**: BGE taken - positive >= negative
  - Condition: `x1 >= x4` (10 >= -10, true)
  - Expected: Branch taken, `x13 = 30000`

### 5. BLTU (Branch if Less Than, Unsigned) Tests

**Test Cases**:
- **6.1**: BLTU taken - positive < positive
  - Condition: `x1 < x2` (10 < 20, true)
  - Expected: Branch taken, `x14 = 3`
  
- **6.2**: BLTU not taken - positive >= positive
  - Condition: `x2 < x1` (20 < 10, false)
  - Expected: Branch not taken, `x14 = 30`
  
- **6.3**: BLTU edge case - negative as large unsigned
  - Condition: `x1 < x4` (10 < 0xFFFFFFF6, true)
  - Note: x4 = -10 = 0xFFFFFFF6 (unsigned: 4294967286)
  - Expected: Branch taken, `x14 = 300`
  
- **6.4**: BLTU taken - negative < negative (as unsigned)
  - Condition: `x5 < x4` (0xFFFFFFEC < 0xFFFFFFF6, true)
  - Note: Both treated as unsigned (4294967276 < 4294967286)
  - Expected: Branch taken, `x14 = 3000`
  
- **6.5**: BLTU edge case - zero comparison
  - Condition: `x3 < x1` (0 < 10, true)
  - Expected: Branch taken, `x14 = 30000`

### 6. BGEU (Branch if Greater or Equal, Unsigned) Tests

**Test Cases**:
- **7.1**: BGEU taken - positive > positive
  - Condition: `x2 >= x1` (20 >= 10, true)
  - Expected: Branch taken, `x15 = 3`
  
- **7.2**: BGEU taken - equal values
  - Condition: `x1 >= x1` (10 >= 10, true)
  - Expected: Branch taken, `x15 = 30`
  
- **7.3**: BGEU not taken - positive < positive
  - Condition: `x1 >= x2` (10 >= 20, false)
  - Expected: Branch not taken, `x15 = 300`
  
- **7.4**: BGEU edge case - negative as large unsigned
  - Condition: `x4 >= x1` (0xFFFFFFF6 >= 10, true)
  - Note: x4 = -10 = 0xFFFFFFF6 (unsigned: 4294967286)
  - Expected: Branch taken, `x15 = 3000`
  
- **7.5**: BGEU taken - negative >= negative (as unsigned)
  - Condition: `x4 >= x5` (0xFFFFFFF6 >= 0xFFFFFFEC, true)
  - Note: Both treated as unsigned (4294967286 >= 4294967276)
  - Expected: Branch taken, `x15 = 30000`
  
- **7.6**: BGEU edge case - zero comparison
  - Condition: `x1 >= x3` (10 >= 0, true)
  - Expected: Branch taken, `x15 = 300000`

## Initial Register Values

| Register | Value | Description |
|----------|-------|-------------|
| x1 | 10 | Positive test value |
| x2 | 20 | Positive test value |
| x3 | 0 | Zero value |
| x4 | -10 (0xFFFFFFF6) | Negative test value |
| x5 | -20 (0xFFFFFFEC) | Negative test value |
| x6 | 0x7FFFFFFF | Maximum positive signed |
| x7 | 0x80000000 | Minimum negative signed / Maximum unsigned |

## Expected Final Register State

| Register | Value | Description |
|----------|-------|-------------|
| x10 | 300 | BEQ test result |
| x11 | 300 | BNE test result |
| x12 | 300000 | BLT test result |
| x13 | 30000 | BGE test result |
| x14 | 30000 | BLTU test result |
| x15 | 300000 | BGEU test result |

## Expected Final Memory State

| Address | Value | Description |
|---------|-------|-------------|
| memory[0] | 300 | BEQ test result |
| memory[1] | 300 | BNE test result |
| memory[2] | 300000 | BLT test result |
| memory[3] | 30000 | BGE test result |
| memory[4] | 30000 | BLTU test result |
| memory[5] | 300000 | BGEU test result |

## Branch Instruction Encoding

### B-Type Instruction Format

```
[31:25] imm[12|10:5]  - Immediate bits [12, 10:5]
[24:20] rs2           - Source register 2
[19:15] rs1           - Source register 1
[14:12] funct3        - Function field (branch type)
[11:7]  imm[4:1|11]   - Immediate bits [4:1, 11]
[6:0]   opcode        - Opcode (1100011 = 0x63)
```

### Branch Types (funct3)

| funct3 | Mnemonic | Condition (Signed) | Condition (Unsigned) |
|--------|----------|-------------------|---------------------|
| 000 | BEQ | rs1 == rs2 | rs1 == rs2 |
| 001 | BNE | rs1 != rs2 | rs1 != rs2 |
| 100 | BLT | rs1 < rs2 | - |
| 101 | BGE | rs1 >= rs2 | - |
| 110 | BLTU | - | rs1 < rs2 |
| 111 | BGEU | - | rs1 >= rs2 |

### Branch Offset Calculation

Branch instructions use a 13-bit signed immediate offset:
- Immediate format: `imm[12|10:5|4:1|11]`
- Offset = sign-extended immediate × 2 (byte offset)
- PC-relative: `target = PC + offset`

**Example**: `BEQ x1, x1, 8`
- Immediate = 8 bytes forward
- Encoded as: `imm[12|10:5] = 000000`, `imm[4:1|11] = 01000`
- Full immediate: `000000001000` = 8 (sign-extended)

## Signed vs Unsigned Comparison

### Signed Comparison (BLT, BGE)
- Treats values as two's complement signed integers
- Negative values are less than positive values
- Example: -10 < 10 (true)

### Unsigned Comparison (BLTU, BGEU)
- Treats values as unsigned integers
- Negative values become large positive values
- Example: -10 (0xFFFFFFF6) = 4,294,967,286 > 10 (true)

### Key Differences

| Comparison | Signed Result | Unsigned Result |
|------------|---------------|-----------------|
| -10 < 10 | True | False (0xFFFFFFF6 > 10) |
| -20 < -10 | True | True (0xFFFFFFEC < 0xFFFFFFF6) |
| 0x80000000 < 0x7FFFFFFF | True (signed) | False (unsigned) |

## Pipeline Behavior

### Branch Resolution
- Branch conditions are evaluated in the **EX stage**
- ALU performs subtraction (rs1 - rs2) for comparison
- Zero flag indicates equality
- Signed/unsigned comparison uses different logic

### Pipeline Flush
- When branch is taken:
  - `PCSrc = 1` (select branch target)
  - `flush = 1` (clear IF/ID and ID/EX registers)
  - Instructions after branch are discarded

### Branch Prediction
- This implementation uses **always-not-taken** prediction
- Branch target computed in EX stage
- Pipeline flushes if prediction was wrong

## Waveform Signals to Monitor

### Branch Control Signals
- `Branch`: Branch instruction indicator
- `funct3[2:0]`: Branch type (000=BEQ, 001=BNE, etc.)
- `zero_flag`: ALU zero flag (for equality comparison)
- `PCSrc`: PC source select (0=PC+4, 1=branch target)
- `flush`: Pipeline flush signal (active high)
- `branch_taken`: Branch condition evaluation result

### Address Signals
- `branch_target[31:0]`: Computed branch target address
- `PC[31:0]`: Current program counter

### Data Signals
- `rs1_data[31:0]`: Source register 1 data (forwarded)
- `rs2_data[31:0]`: Source register 2 data (forwarded)

## Verification Checklist

1. **BEQ Tests**:
   - Verify branch taken when operands equal
   - Verify branch not taken when operands unequal
   - Check zero comparison works correctly

2. **BNE Tests**:
   - Verify branch taken when operands unequal
   - Verify branch not taken when operands equal
   - Check zero comparison works correctly

3. **BLT Tests**:
   - Verify signed comparison works correctly
   - Test positive/negative comparisons
   - Verify zero comparisons

4. **BGE Tests**:
   - Verify signed comparison works correctly
   - Test equality case (>= includes ==)
   - Test positive/negative comparisons

5. **BLTU Tests**:
   - Verify unsigned comparison works correctly
   - Test negative values treated as large unsigned
   - Verify zero comparisons

6. **BGEU Tests**:
   - Verify unsigned comparison works correctly
   - Test equality case (>= includes ==)
   - Test negative values treated as large unsigned

## Common Issues and Debugging

### Issue 1: Branch Not Taken When Should Be Taken

**Symptoms**:
- Branch condition evaluates incorrectly
- Wrong register values used

**Debugging Steps**:
1. Check `rs1_data` and `rs2_data` values (verify forwarding)
2. Verify `funct3` matches expected branch type
3. Check `zero_flag` for equality comparisons
4. Verify signed/unsigned comparison logic

### Issue 2: Wrong Branch Target Address

**Symptoms**:
- Branch jumps to wrong address
- Pipeline executes wrong instructions

**Debugging Steps**:
1. Verify `branch_target` calculation
2. Check immediate encoding in instruction
3. Verify PC value when branch executes
4. Check offset sign extension

### Issue 3: Pipeline Flush Not Working

**Symptoms**:
- Instructions after branch execute even when branch taken
- Wrong instructions in pipeline

**Debugging Steps**:
1. Verify `flush` signal is asserted when branch taken
2. Check IF/ID and ID/EX register clearing
3. Verify `PCSrc` selects correct PC source

## Related Documentation

- `docs/ARCHITECTURE.md` - Pipeline architecture details
- `docs/FORWARDING_WAVEFORM_GUIDE.md` - Forwarding verification guide
- `src/branch_jump_control.sv` - Branch control implementation

