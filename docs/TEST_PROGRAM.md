# RISC-V Test Program Documentation

## Overview

This document describes a comprehensive RISC-V assembly test program (`mem/test_program.hex`) designed to verify basic processor functionality. The test program covers all major instruction types in the RISC-V RV32I instruction set.

## Test Program Structure

The test program is organized into 8 sections, each testing different instruction types:

1. **Register Initialization** (ADDI)
2. **Arithmetic Operations** (ADD, SUB)
3. **Logical Operations** (AND, OR, XOR)
4. **Store Operations** (SW)
5. **Load Operations** (LW)
6. **Branch Instructions** (BEQ)
7. **Jump Instructions** (JAL)
8. **End Marker** (NOPs)

## Instruction Encoding Reference

### RISC-V Instruction Formats

#### R-Type (Register-Register Operations)
```
|31     25|24   20|19   15|14    12|11    7|6      0|
| funct7  |  rs2  |  rs1  | funct3 |  rd   | opcode |
```

#### I-Type (Immediate Operations)
```
|31          20|19   15|14    12|11    7|6      0|
|   imm[11:0]  |  rs1  | funct3 |  rd   | opcode |
```

#### S-Type (Store Operations)
```
|31     25|24   20|19   15|14    12|11    7|6      0|
|imm[11:5]|  rs2  |  rs1  | funct3 |imm[4:0]| opcode |
```

#### B-Type (Branch Operations)
```
|31     25|24   20|19   15|14    12|11    7|6      0|
|imm[12] |imm[10:5]| rs2 | rs1 |funct3|imm[4:1]|imm[11]| opcode |
```

#### J-Type (Jump Operations)
```
|31     20|19   12|11   11|10     1|6      0|
|imm[20] |imm[10:1]|imm[11]|imm[19:12]|  rd   | opcode |
```

### Opcodes Used
- `0x13` - OP-IMM (ADDI, ANDI, ORI, XORI)
- `0x33` - OP (ADD, SUB, AND, OR, XOR)
- `0x03` - LOAD (LW)
- `0x23` - STORE (SW)
- `0x63` - BRANCH (BEQ, BNE, BLT, BGE, BLTU, BGEU)
- `0x6F` - JAL (Jump and Link)

### Function Codes (funct3)
- `000` - ADD/ADDI, BEQ, LW, SW
- `010` - LW (load word)
- `100` - XOR
- `110` - OR
- `111` - AND

### Function Codes (funct7)
- `0000000` - ADD
- `0100000` - SUB

## Detailed Instruction Breakdown

### Section 1: Register Initialization (ADDI)

**Purpose**: Initialize registers with known values for testing.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `00500093` | `0x00500093` | `ADDI x1, x0, 5` | x1 = 5 |
| `00300113` | `0x00300113` | `ADDI x2, x0, 3` | x2 = 3 |
| `00000193` | `0x00000193` | `ADDI x3, x0, 0` | x3 = 0 (clear) |
| `00000213` | `0x00000213` | `ADDI x4, x0, 0` | x4 = 0 (clear) |
| `00000293` | `0x00000293` | `ADDI x5, x0, 0` | x5 = 0 (clear) |
| `00000313` | `0x00000313` | `ADDI x6, x0, 0` | x6 = 0 (clear) |
| `00000393` | `0x00000393` | `ADDI x7, x0, 0` | x7 = 0 (clear) |
| `00000413` | `0x00000413` | `ADDI x8, x0, 0` | x8 = 0 (clear) |
| `00000493` | `0x00000493` | `ADDI x9, x0, 0` | x9 = 0 (clear) |
| `00000513` | `0x00000513` | `ADDI x10, x0, 0` | x10 = 0 (clear) |

**Encoding**: `ADDI rd, rs1, imm12`
- Opcode: `0x13` (bits [6:0])
- funct3: `000` (bits [14:12])
- rd: destination register (bits [11:7])
- rs1: source register (bits [19:15])
- imm[11:0]: 12-bit immediate (bits [31:20])

**Example**: `ADDI x1, x0, 5` = `0x00500093`
- `0x13` = opcode
- `000` = funct3
- `00001` = rd = x1
- `00000` = rs1 = x0
- `000000000101` = imm = 5

### Section 2: Arithmetic Operations (ADD, SUB)

**Purpose**: Test register-register arithmetic operations.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `002081B3` | `0x002081B3` | `ADD x3, x1, x2` | x3 = x1 + x2 = 8 |
| `40210233` | `0x40210233` | `SUB x4, x2, x1` | x4 = x2 - x1 = -2 |

**Encoding**: `ADD/SUB rd, rs1, rs2`
- Opcode: `0x33` (bits [6:0])
- funct3: `000` (bits [14:12])
- funct7: `0000000` for ADD, `0100000` for SUB (bits [31:25])
- rd: destination register (bits [11:7])
- rs1: source register 1 (bits [19:15])
- rs2: source register 2 (bits [24:20])

**Example**: `ADD x3, x1, x2` = `0x002081B3`
- `0x33` = opcode
- `000` = funct3
- `00011` = rd = x3
- `00001` = rs1 = x1
- `00010` = rs2 = x2
- `0000000` = funct7 = ADD

**Expected Results**:
- x3 = 8 (5 + 3)
- x4 = -2 (3 - 5) = 0xFFFFFFFE (unsigned)

### Section 3: Logical Operations (AND, OR, XOR)

**Purpose**: Test bitwise logical operations.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `0020F2B3` | `0x0020F2B3` | `AND x5, x1, x2` | x5 = x1 & x2 = 1 |
| `0020E313` | `0x0020E313` | `OR x6, x1, x2` | x6 = x1 \| x2 = 7 |
| `0020C393` | `0x0020C393` | `XOR x7, x1, x2` | x7 = x1 ^ x2 = 6 |

**Encoding**: `AND/OR/XOR rd, rs1, rs2`
- Opcode: `0x33` (bits [6:0])
- funct3: `111` for AND, `110` for OR, `100` for XOR (bits [14:12])
- funct7: `0000000` (bits [31:25])
- rd: destination register (bits [11:7])
- rs1: source register 1 (bits [19:15])
- rs2: source register 2 (bits [24:20])

**Example**: `AND x5, x1, x2` = `0x0020F2B3`
- `0x33` = opcode
- `111` = funct3 = AND
- `00101` = rd = x5
- `00001` = rs1 = x1
- `00010` = rs2 = x2
- `0000000` = funct7

**Expected Results**:
- x5 = 1 (5 & 3 = 0b101 & 0b011 = 0b001)
- x6 = 7 (5 | 3 = 0b101 | 0b011 = 0b111)
- x7 = 6 (5 ^ 3 = 0b101 ^ 0b011 = 0b110)

### Section 4: Store Operations (SW)

**Purpose**: Test storing register values to memory.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `00302023` | `0x00302023` | `SW x3, 0(x0)` | Store x3 to memory[0] |
| `00402223` | `0x00402223` | `SW x4, 4(x0)` | Store x4 to memory[1] |
| `00502423` | `0x00502423` | `SW x5, 8(x0)` | Store x5 to memory[2] |
| `00602623` | `0x00602623` | `SW x6, 12(x0)` | Store x6 to memory[3] |
| `00702823` | `0x00702823` | `SW x7, 16(x0)` | Store x7 to memory[4] |

**Encoding**: `SW rs2, imm(rs1)`
- Opcode: `0x23` (bits [6:0])
- funct3: `010` (bits [14:12])
- rs1: base register (bits [19:15])
- rs2: source register (bits [24:20])
- imm[11:5]: upper 7 bits of immediate (bits [31:25])
- imm[4:0]: lower 5 bits of immediate (bits [11:7])

**Example**: `SW x3, 0(x0)` = `0x00302023`
- `0x23` = opcode
- `010` = funct3 = SW
- `00000` = rs1 = x0
- `00011` = rs2 = x3
- `0000000` = imm[11:5] = 0
- `00000` = imm[4:0] = 0
- Total immediate = 0

**Memory Layout After Stores**:
- memory[0] = 8 (x3)
- memory[1] = -2 (x4) = 0xFFFFFFFE
- memory[2] = 1 (x5)
- memory[3] = 7 (x6)
- memory[4] = 6 (x7)

### Section 5: Load Operations (LW)

**Purpose**: Test loading values from memory.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `00002403` | `0x00002403` | `LW x8, 0(x0)` | Load memory[0] into x8 |

**Encoding**: `LW rd, imm(rs1)`
- Opcode: `0x03` (bits [6:0])
- funct3: `010` (bits [14:12])
- rd: destination register (bits [11:7])
- rs1: base register (bits [19:15])
- imm[11:0]: 12-bit immediate (bits [31:20])

**Example**: `LW x8, 0(x0)` = `0x00002403`
- `0x03` = opcode
- `010` = funct3 = LW
- `01000` = rd = x8
- `00000` = rs1 = x0
- `000000000000` = imm = 0

**Expected Result**:
- x8 = 8 (loaded from memory[0])

### Section 6: Branch Test (BEQ)

**Purpose**: Test conditional branch instructions.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `00100493` | `0x00100493` | `ADDI x9, x0, 1` | x9 = 1 (before branch) |
| `00108463` | `0x00108463` | `BEQ x1, x1, 8` | Branch if x1 == x1 (always true) |
| `00200493` | `0x00200493` | `ADDI x9, x0, 2` | x9 = 2 (skipped if branch taken) |
| `00300493` | `0x00300493` | `ADDI x9, x0, 3` | x9 = 3 (branch target) |
| `00400493` | `0x00400493` | `ADDI x9, x0, 4` | x9 = 4 (after branch) |

**Encoding**: `BEQ rs1, rs2, imm`
- Opcode: `0x63` (bits [6:0])
- funct3: `000` (bits [14:12])
- rs1: source register 1 (bits [19:15])
- rs2: source register 2 (bits [24:20])
- imm[12\|10:5\|4:1\|11]: Branch offset (bits [31\|30:25\|11:8\|7])
  - imm[12] = bit [31]
  - imm[10:5] = bits [30:25]
  - imm[4:1] = bits [11:8]
  - imm[11] = bit [7]

**Example**: `BEQ x1, x1, 8` = `0x00108463`
- `0x63` = opcode
- `000` = funct3 = BEQ
- `00001` = rs1 = x1
- `00001` = rs2 = x1
- Immediate = 8 bytes = 2 instructions forward
- Encoding: `0000000` (imm[12\|10:5]) `00001` (rs2) `00001` (rs1) `000` (funct3) `0100` (imm[4:1]) `0` (imm[11]) `1100011` (opcode)
- Full: `0x00108463`

**Branch Behavior**:
- x1 == x1 is always true, so branch is taken
- PC jumps forward by 8 bytes (2 instructions)
- Instruction `ADDI x9, x0, 2` is skipped
- Execution continues at `ADDI x9, x0, 3`

**Expected Result**:
- x9 = 4 (final value after branch sequence)

### Section 7: Jump Test (JAL)

**Purpose**: Test unconditional jump instructions.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `0100006F` | `0x0100006F` | `JAL x10, 16` | Jump forward 16 bytes, save PC+4 in x10 |
| `00500513` | `0x00500513` | `ADDI x10, x0, 5` | x10 = 5 (skipped if jump taken) |
| `00600513` | `0x00600513` | `ADDI x10, x0, 6` | x10 = 6 (skipped if jump taken) |
| `04200513` | `0x04200513` | `ADDI x10, x0, 66` | x10 = 66 = 0x42 (jump target) |

**Encoding**: `JAL rd, imm`
- Opcode: `0x6F` (bits [6:0])
- rd: destination register for return address (bits [11:7])
- imm[20\|10:1\|11\|19:12]: Jump offset (bits [31\|30:21\|20\|19:12])
  - imm[20] = bit [31]
  - imm[10:1] = bits [30:21]
  - imm[11] = bit [20]
  - imm[19:12] = bits [19:12]

**Example**: `JAL x10, 16` = `0x0100006F`
- `0x6F` = opcode
- `01010` = rd = x10
- Immediate = 16 bytes = 4 instructions forward
- Encoding: `0` (imm[20]) `0000000001` (imm[10:1]) `0` (imm[11]) `0000` (imm[19:12]) `01010` (rd) `1101111` (opcode)
- Full: `0x0100006F`

**Jump Behavior**:
- PC jumps forward by 16 bytes (4 instructions)
- Return address (PC + 4) is saved in x10
- Instructions `ADDI x10, x0, 5` and `ADDI x10, x0, 6` are skipped
- Execution continues at `ADDI x10, x0, 66`

**Expected Result**:
- x10 = 66 = 0x42 (value set at jump target)

### Section 8: End Marker (NOPs)

**Purpose**: Mark end of program with NOP instructions.

| Instruction | Hex Code | Assembly | Description |
|------------|----------|----------|-------------|
| `00000013` | `0x00000013` | `ADDI x0, x0, 0` | NOP (end marker) |

## Expected Final Register State

After executing the complete test program:

| Register | Value (Hex) | Value (Decimal) | Description |
|----------|-------------|-----------------|-------------|
| x0 | 0x00000000 | 0 | Zero register (always 0) |
| x1 | 0x00000005 | 5 | Initialized value |
| x2 | 0x00000003 | 3 | Initialized value |
| x3 | 0x00000008 | 8 | ADD result (x1 + x2) |
| x4 | 0xFFFFFFFE | -2 | SUB result (x2 - x1) |
| x5 | 0x00000001 | 1 | AND result (x1 & x2) |
| x6 | 0x00000007 | 7 | OR result (x1 \| x2) |
| x7 | 0x00000006 | 6 | XOR result (x1 ^ x2) |
| x8 | 0x00000008 | 8 | Loaded from memory[0] |
| x9 | 0x00000004 | 4 | Branch test result |
| x10 | 0x00000042 | 66 | Jump test result (return address overwritten) |

## Expected Final Memory State

After executing the complete test program:

| Address | Value (Hex) | Value (Decimal) | Description |
|---------|-------------|-----------------|-------------|
| 0 | 0x00000008 | 8 | Stored x3 (ADD result) |
| 1 | 0xFFFFFFFE | -2 | Stored x4 (SUB result) |
| 2 | 0x00000001 | 1 | Stored x5 (AND result) |
| 3 | 0x00000007 | 7 | Stored x6 (OR result) |
| 4 | 0x00000006 | 6 | Stored x7 (XOR result) |

## Test Verification Checklist

Use this checklist to verify processor functionality:

- [ ] **ADDI**: x1 = 5, x2 = 3
- [ ] **ADD**: x3 = 8 (5 + 3)
- [ ] **SUB**: x4 = -2 (3 - 5) = 0xFFFFFFFE
- [ ] **AND**: x5 = 1 (5 & 3)
- [ ] **OR**: x6 = 7 (5 | 3)
- [ ] **XOR**: x7 = 6 (5 ^ 3)
- [ ] **SW**: memory[0] = 8, memory[1] = -2, etc.
- [ ] **LW**: x8 = 8 (loaded from memory[0])
- [ ] **BEQ**: Branch taken, x9 = 4
- [ ] **JAL**: Jump taken, x10 = 66

## Usage Instructions

1. **Load the test program**:
   ```systemverilog
   // In testbench, set:
   parameter IMEM_INIT_FILE = "mem/test_program.hex";
   ```

2. **Run simulation**:
   ```bash
   # Using ModelSim
   vlog -sv tb/riscv_pipeline_tb.sv src/*.sv
   vsim -c riscv_pipeline_tb
   
   # Using Icarus Verilog
   iverilog -g2012 -o riscv_pipeline_tb tb/riscv_pipeline_tb.sv src/*.sv
   vvp riscv_pipeline_tb
   ```

3. **Verify results**:
   - Check register file contents using `display_register_state()` task
   - Check data memory contents using `dump_data_memory()` task
   - Compare with expected values listed above

## Notes

- **Address Calculation**: RISC-V uses byte addressing, but instruction memory is word-aligned. The test program uses word addresses for clarity.
- **Branch Offset**: Branch offsets are in bytes and relative to the current PC.
- **Jump Offset**: Jump offsets are in bytes and relative to the current PC.
- **Signed vs Unsigned**: SUB operation produces signed result (-2) but stored as unsigned (0xFFFFFFFE).
- **Pipeline Effects**: Due to pipelining, results may appear a few cycles after instruction execution.

## Extending the Test Program

To add more test cases:

1. **Add new instructions** to the hex file
2. **Update expected results** in this document
3. **Add verification checks** in the testbench
4. **Document instruction encoding** for reference

## References

- RISC-V Instruction Set Manual: https://riscv.org/specifications/
- RISC-V Instruction Encoder: https://luplab.gitlab.io/rvcodecjs/
- SystemVerilog Testbench Guide: `docs/TESTBENCH_GUIDE.md`

