# Jump Instructions Test Program Documentation

## Overview

This test program comprehensively tests RISC-V jump instructions:
- **JAL** (Jump and Link) - PC-relative unconditional jump
- **JALR** (Jump and Link Register) - Register + immediate unconditional jump

The program demonstrates procedure calls, returns, nested calls, and calculated address jumps.

## Files

- `mem/jump_test.s` - Assembly source code with detailed annotations
- `mem/jump_test.hex` - Machine code in hex format for simulation

## Instruction Types

### JAL (Jump and Link)

**Format**: `JAL rd, imm`

**Operation**:
- Jump to PC-relative address: `target = PC + sign_extend(imm)`
- Save return address: `rd = PC + 4`
- Always taken (unconditional)

**Encoding** (J-type):
```
[31]      [30:21]      [20]      [19:12]     [11:7]    [6:0]
imm[20]   imm[10:1]    imm[11]   imm[19:12]  rd        opcode
```
- Opcode: `1101111` (0x6F)
- Immediate: 21-bit signed offset (multiply by 2 for byte offset)
- rd: Destination register for return address

**Use Cases**:
- Procedure calls
- Long-range jumps
- PC-relative jumps

### JALR (Jump and Link Register)

**Format**: `JALR rd, imm(rs1)`

**Operation**:
- Jump to address: `target = (rs1 + sign_extend(imm)) & ~1`
- Save return address: `rd = PC + 4`
- Always taken (unconditional)
- Target address LSB is cleared (word-aligned)

**Encoding** (I-type):
```
[31:20]        [19:15]   [14:12]  [11:7]    [6:0]
imm[11:0]      rs1       funct3   rd        opcode
```
- Opcode: `1100111` (0x67)
- funct3: `000`
- Immediate: 12-bit signed offset
- rs1: Base register for target address calculation

**Use Cases**:
- Procedure returns
- Computed jumps
- Function pointers
- Dynamic address calculation

## Test Coverage

### Section 2: Basic JAL Tests

**Test 2.1**: JAL forward jump
- Jumps forward 8 bytes
- Saves return address in x1
- Skips 2 instructions
- Expected: x2 = 100, x1 = return address

**Test 2.2**: JAL with return address verification
- Jumps forward 8 bytes
- Verifies return address is saved correctly
- Expected: x3 = 30, x1 = return address

### Section 3: Basic JALR Tests

**Test 3.1**: JALR with base register
- Calculates target address using AUIPC + ADDI
- Jumps to calculated address
- Expected: x4 = 200, x1 = return address

**Test 3.2**: JALR with immediate offset
- Uses register + immediate offset
- Expected: x5 = 300, x1 = return address

### Section 4: Simple Procedure Call and Return

**Pattern**:
```assembly
JAL x1, procedure_a    # Call procedure
# ... code after return ...

procedure_a:
    # Procedure body
    JALR x0, 0(x1)     # Return
```

**Test 4.1**: Basic procedure call
- Calls `procedure_a` using JAL
- Returns using JALR
- Expected: x2 = 100 (procedure), then x2 = 50 (after return)

### Section 5: Nested Procedure Calls

**Pattern**:
```assembly
JAL x1, nested_func1   # Call func1
# ... main code ...

nested_func1:
    JAL x1, nested_func2   # Call func2
    # ... func1 code ...
    JALR x0, 0(x1)         # Return from func1

nested_func2:
    # ... func2 code ...
    JALR x0, 0(x1)         # Return from func2
```

**Test 5.1**: Nested calls
- Calls func1, which calls func2
- Both functions return correctly
- Expected sequence: x3 = 100 → 300 → 200 → 1000

### Section 6: Procedure Call with Calculated Address

**Pattern**:
```assembly
AUIPC x13, 0           # x13 = PC
ADDI x13, x13, offset  # x13 = PC + offset
JALR x1, 0(x13)        # Call procedure at calculated address
```

**Test 6.1**: Calculated address call
- Computes procedure address at runtime
- Calls procedure using JALR
- Expected: x4 = 600 (procedure), then x4 = 500 (after return)

### Section 7: Multiple Procedure Calls

**Test 7.1-7.3**: Sequential procedure calls
- Calls proc1, proc2, proc3 in sequence
- Each procedure returns correctly
- Expected sequence: x5 = 100 → 10 → 200 → 20 → 300 → 30 → 1000

### Section 8: Return Address Register (x1/ra) Usage

**Test 8.1**: Basic return address usage
- Uses x1 (ra) as return address register
- Procedure returns using JALR x0, 0(x1)
- Expected: x2 = 200 (procedure), then x2 = 100 (after return)

**Test 8.2**: Nested calls preserving return addresses
- Outer procedure saves x1 before nested call
- Restores x1 after nested call returns
- Expected sequence: x3 = 100 → 300 → 200 → 1000

### Section 9: Jump to Calculated Addresses

**Test 9.1**: Calculate and jump
- Computes target address using AUIPC + ADDI
- Jumps to calculated address using JALR
- Expected: x4 = 2

**Test 9.2**: Jump with offset
- Loads base address, adds offset
- Jumps to computed address
- Expected: x5 = 20

## Expected Final Register State

| Register | Value | Description |
|----------|-------|-------------|
| x1 | Return address | Last procedure return address |
| x2 | 300 | After return address tests |
| x3 | 2000 | After nested return address tests |
| x4 | 2 | After calculated address jump |
| x5 | 20 | After calculated address jump with offset |

## Expected Final Memory State

| Address | Value | Description |
|---------|-------|-------------|
| memory[0] | Return address | x1 value |
| memory[1] | 300 | x2 value |
| memory[2] | 2000 | x3 value |
| memory[3] | 2 | x4 value |
| memory[4] | 20 | x5 value |

## Procedure Call Convention

### Standard RISC-V Calling Convention

**Return Address Register (x1/ra)**:
- x1 is designated as the return address register (ra)
- JAL saves PC+4 in ra
- JALR uses ra to return

**Call Pattern**:
```assembly
JAL x1, function_name    # Call function
# ... code after return ...
```

**Return Pattern**:
```assembly
function_name:
    # Function body
    JALR x0, 0(x1)       # Return (x0 discards return address)
```

### Nested Call Pattern

When a function calls another function, it must preserve its return address:

```assembly
outer_function:
    ADDI x6, x1, 0       # Save current return address
    JAL x1, inner_function  # Call inner function
    ADDI x1, x6, 0       # Restore return address
    JALR x0, 0(x1)       # Return from outer function
```

## Address Calculation Methods

### Method 1: AUIPC + ADDI

```assembly
AUIPC x13, 0           # x13 = PC (upper 20 bits)
ADDI x13, x13, offset  # x13 = PC + offset
JALR x1, 0(x13)        # Jump to x13 + 0
```

**Use Case**: PC-relative address calculation

### Method 2: LUI + ADDI

```assembly
LUI x14, upper_20      # x14 = upper 20 bits
ADDI x14, x14, lower_12 # x14 = full address
JALR x1, 0(x14)        # Jump to x14 + 0
```

**Use Case**: Absolute address calculation

### Method 3: Register + Immediate

```assembly
JALR x1, offset(rs1)   # Jump to rs1 + offset
```

**Use Case**: Base register + offset

## Pipeline Behavior

### JAL Pipeline Stages

1. **IF**: Fetch JAL instruction
2. **ID**: Decode, extract immediate, compute target address
3. **EX**: Target address = PC + sign_extend(imm)
4. **MEM**: (No memory access)
5. **WB**: Write PC+4 to rd register

### JALR Pipeline Stages

1. **IF**: Fetch JALR instruction
2. **ID**: Decode, read rs1, extract immediate
3. **EX**: Target address = (rs1 + sign_extend(imm)) & ~1
4. **MEM**: (No memory access)
5. **WB**: Write PC+4 to rd register

### Pipeline Flush

When jump is taken:
- `PCSrc = 1` (select jump target)
- `flush = 1` (clear IF/ID and ID/EX registers)
- Instructions after jump are discarded

## Waveform Signals to Monitor

### Jump Control Signals
- `Jump`: Jump instruction indicator
- `PCSrc`: PC source select (0=PC+4, 1=jump target)
- `flush`: Pipeline flush signal (active high)

### Address Signals
- `jump_target[31:0]`: Computed jump target address
- `PC[31:0]`: Current program counter
- `PC_plus_4[31:0]`: PC + 4 (return address)

### Data Signals
- `rd_data[31:0]`: Return address written to rd
- `rs1_data[31:0]`: Base register data (for JALR)

### Register Signals
- `rd_addr[4:0]`: Destination register (usually x1/ra)
- `rs1_addr[4:0]`: Source register (for JALR)

## Verification Checklist

1. **JAL Tests**:
   - Verify PC jumps to correct target address
   - Verify return address (PC+4) is saved in rd
   - Check forward jumps work correctly

2. **JALR Tests**:
   - Verify target address calculation: (rs1 + imm) & ~1
   - Verify return address is saved in rd
   - Check LSB is cleared (word alignment)

3. **Procedure Calls**:
   - Verify procedure is called correctly
   - Verify return address is preserved
   - Check procedure returns to correct location

4. **Nested Calls**:
   - Verify inner procedure is called
   - Verify return addresses are preserved
   - Check correct return sequence

5. **Calculated Addresses**:
   - Verify address calculation is correct
   - Verify jump to calculated address works
   - Check AUIPC + ADDI pattern

## Common Issues and Debugging

### Issue 1: Wrong Jump Target Address

**Symptoms**:
- Jump goes to wrong address
- Procedure doesn't execute

**Debugging Steps**:
1. Verify immediate encoding in instruction
2. Check PC value when jump executes
3. Verify target address calculation
4. For JALR: Check rs1 value and immediate offset

### Issue 2: Return Address Not Saved

**Symptoms**:
- rd register doesn't contain PC+4
- Procedure can't return

**Debugging Steps**:
1. Verify rd register write enable
2. Check PC+4 value when jump executes
3. Verify register file write operation
4. Check rd address matches instruction

### Issue 3: Procedure Doesn't Return

**Symptoms**:
- Procedure executes but doesn't return
- Execution continues incorrectly

**Debugging Steps**:
1. Verify return address in x1/ra
2. Check JALR instruction encoding
3. Verify target address calculation: (rs1 + imm) & ~1
4. Check LSB clearing for alignment

### Issue 4: Nested Calls Fail

**Symptoms**:
- Inner call works but outer return fails
- Return addresses overwritten

**Debugging Steps**:
1. Verify return address preservation
2. Check register save/restore logic
3. Verify x1 is saved before nested call
4. Check x1 is restored after nested call returns

## Related Documentation

- `docs/ARCHITECTURE.md` - Pipeline architecture details
- `docs/BRANCH_TEST.md` - Branch instruction tests
- `src/branch_jump_control.sv` - Branch/jump control implementation
- `src/imm_gen.sv` - Immediate generation (JAL/JALR encoding)

