# RAW Data Hazard Test Program Documentation

## Overview

This test program is specifically designed to verify Read-After-Write (RAW) data hazard detection and resolution in the RISC-V 5-stage pipelined processor. It tests three critical hazard scenarios:

1. **EX Hazard**: Forwarding from EX/MEM stage
2. **MEM Hazard**: Forwarding from MEM/WB stage  
3. **Load-Use Hazard**: Pipeline stall required

## Files

- `mem/raw_hazard_test.s` - Assembly source code with detailed annotations
- `mem/raw_hazard_test.hex` - Machine code in hex format for simulation

## Pipeline Architecture

The RISC-V 5-stage pipeline consists of:
- **IF**: Instruction Fetch
- **ID**: Instruction Decode
- **EX**: Execute
- **MEM**: Memory Access
- **WB**: Write Back

## Hazard Types and Resolution

### 1. EX Hazard (Forward from EX/MEM)

**Definition**: An instruction in the EX stage uses a result from the immediately previous instruction that is currently in the MEM stage.

**Forwarding Path**: EX/MEM → EX  
**Forwarding Control**: ForwardA/ForwardB = `2'b10`

**Example**:
```assembly
ADDI x5, x0, 5         # x5 = 5 (written in MEM stage)
ADD x6, x5, x2         # x6 = x5 + x2 (reads x5 in EX stage)
```

**Pipeline Timeline**:
- Cycle N:   I1 in MEM, I2 in EX (I2 uses forwarded data from I1's EX/MEM)
- Cycle N+1: I1 in WB, I2 in MEM

**Test Cases**:
- **2.1**: rs1 dependency (ForwardA = 10)
- **2.2**: rs2 dependency (ForwardB = 10)
- **2.3**: Both rs1 and rs2 dependencies (ForwardA = 10, ForwardB = 10)
- **2.4**: Chain of EX hazards (each instruction depends on previous)

### 2. MEM Hazard (Forward from MEM/WB)

**Definition**: An instruction in the EX stage uses a result from 2 instructions back that is currently in the WB stage.

**Forwarding Path**: MEM/WB → EX  
**Forwarding Control**: ForwardA/ForwardB = `2'b01`

**Example**:
```assembly
ADDI x16, x0, 16       # x16 = 16 (written in WB stage)
ADDI x17, x0, 17       # x17 = 17 (independent instruction)
ADD x18, x16, x2       # x18 = x16 + x2 (reads x16 in EX stage)
```

**Pipeline Timeline**:
- Cycle N:   I1 in WB, I2 in MEM, I3 in EX (I3 uses forwarded data from I1's MEM/WB)
- Cycle N+1: I1 completes, I2 in WB, I3 in MEM

**Test Cases**:
- **3.1**: rs1 dependency (ForwardA = 01)
- **3.2**: rs2 dependency (ForwardB = 01)
- **3.3**: Both rs1 and rs2 dependencies (ForwardA = 01, ForwardB = 01)
- **3.4**: Mixed EX and MEM hazards (ForwardA = 01, ForwardB = 10)

### 3. Load-Use Hazard (Pipeline Stall)

**Definition**: A load instruction is followed by an instruction that immediately uses the loaded data. The data is not available until after the MEM stage completes.

**Resolution**: Pipeline stall for 1 cycle (insert bubble in ID/EX)

**Example**:
```assembly
LW x30, 0(x29)         # Load from memory into x30
ADD x31, x30, x2       # Use x30 immediately (hazard!)
```

**Pipeline Timeline**:
- Cycle N:   LW in EX, ADD in ID (hazard detected)
- Cycle N+1: LW in MEM, ADD stalled in ID (bubble inserted in ID/EX)
- Cycle N+2: LW in WB, ADD in EX (uses forwarded data from MEM/WB)

**Hazard Detection**:
- `id_ex_MemRead = 1` AND
- (`id_ex_rd = if_id_rs1` OR `id_ex_rd = if_id_rs2`)

**Stall Control**:
- `PCWrite = 0` (stall PC)
- `IF/ID enable = 0` (hold IF/ID register)
- `ID/EX flush = 1` (insert bubble/NOP)

**Test Cases**:
- **4.1**: Load-use hazard with rs1 dependency
- **4.2**: Load-use hazard with rs2 dependency
- **4.3**: Load-use hazard with both rs1 and rs2 dependencies
- **4.4**: Load-use hazard followed by normal EX forwarding

## Test Program Structure

### Section 1: Initialize Test Values
Sets up base values in registers x1-x4 for use in subsequent tests.

### Section 2: EX Hazard Tests
Tests forwarding from EX/MEM stage with various dependency patterns.

### Section 3: MEM Hazard Tests
Tests forwarding from MEM/WB stage with various dependency patterns.

### Section 4: Load-Use Hazard Tests
Tests pipeline stall behavior when load instructions are immediately followed by dependent instructions.

### Section 5: Complex Hazard Scenarios
Tests complex scenarios including:
- Multiple consecutive EX hazards
- Arithmetic operations with mixed hazards
- Store instructions (no stall needed)

### Section 6: Verification and Results
Stores final results to memory for verification.

## Expected Results

### Register Values

| Register | Value | Description |
|----------|-------|-------------|
| x1 | 20 | Load-use test result |
| x2 | 50 | Load-use test result |
| x3 | 30 | Base value |
| x4 | 40 | Independent instruction |
| x5 | 30 | Load result |
| x6 | 60 | Load-use test result |
| x7 | 10 | Load result |
| x8 | 10 | EX hazard after load-use |
| x9 | 10 | EX hazard chain |
| x10 | 100 | EX hazard chain base |
| x11 | 100 | EX hazard chain |
| x12 | 100 | EX hazard chain |
| x13 | 100 | EX hazard chain |
| x14 | 100 | EX hazard chain |
| x15 | 15 | Complex hazard base |
| x16 | 16 | Complex hazard base |
| x17 | 31 | Complex hazard (EX) |
| x18 | 16 | Complex hazard (EX) |
| x19 | 16 | Complex hazard (MEM) |
| x20 | 31 | Complex hazard (MEM) |
| x21 | 21 | Store test |
| x30 | 10 | Load result |
| x31 | 30 | Load-use hazard result |

### Memory Values

| Address | Value | Description |
|---------|-------|-------------|
| memory[0] | 10 | Initial store |
| memory[1] | 20 | Initial store |
| memory[2] | 30 | Initial store |
| memory[3] | 10 | Load-use test |
| memory[4] | 21 | Store test |
| memory[5] | 25 | EX hazard result |
| memory[6] | 27 | EX hazard result |
| memory[7] | 20 | EX hazard result |
| memory[8] | 36 | MEM hazard result |
| memory[9] | 39 | MEM hazard result |
| memory[10] | 45 | MEM hazard result |
| memory[11] | 30 | Load-use hazard result |
| memory[12] | 50 | Load-use hazard result |
| memory[13] | 60 | Load-use hazard result |
| memory[14] | 31 | Complex hazard result |
| memory[15] | 31 | Complex hazard result |

## Waveform Signals to Monitor

### Forwarding Unit Signals
- `ForwardA[1:0]`: Forwarding control for ALU operand A
  - `00`: No forwarding (use register file)
  - `01`: Forward from MEM/WB
  - `10`: Forward from EX/MEM
- `ForwardB[1:0]`: Forwarding control for ALU operand B
  - `00`: No forwarding (use register file)
  - `01`: Forward from MEM/WB
  - `10`: Forward from EX/MEM

### Hazard Detection Unit Signals
- `stall`: Pipeline stall signal (active high)
- `id_ex_flush`: ID/EX register flush signal (active high)

### Pipeline Control Signals
- `pipeline_stall_internal`: Combined stall signal (hazard + external)
- `pipeline_flush_internal`: Combined flush signal (branch + hazard)

### Pipeline Register Signals
- `id_ex_rs1_addr`, `id_ex_rs2_addr`: Source register addresses
- `ex_mem_rd_addr`, `mem_wb_rd_addr`: Destination register addresses
- `ex_mem_reg_write`, `mem_wb_reg_write`: Register write enables
- `id_ex_MemRead`: Load instruction indicator

### ALU Signals
- `rs1_data_forwarded`: Forwarded rs1 data to ALU
- `rs2_data_forwarded`: Forwarded rs2 data to ALU
- `alu_result`: ALU computation result

## Debugging Tips

1. **EX Hazard Not Working**:
   - Check `ForwardA`/`ForwardB` = `10` when expected
   - Verify `ex_mem_reg_write` and `ex_mem_rd_addr` match source registers
   - Ensure forwarding mux selects `ex_mem_alu_result`

2. **MEM Hazard Not Working**:
   - Check `ForwardA`/`ForwardB` = `01` when expected
   - Verify `mem_wb_reg_write` and `mem_wb_rd_addr` match source registers
   - Ensure no EX hazard is present (EX hazard has priority)

3. **Load-Use Hazard Not Stalling**:
   - Check `stall` signal is asserted when load-use hazard detected
   - Verify PC is not updating during stall (`PCWrite = 0`)
   - Check IF/ID register is held (`enable = 0`)
   - Verify bubble is inserted in ID/EX (`flush = 1`)

4. **Incorrect Results**:
   - Verify forwarding paths are correctly connected
   - Check register addresses match expected values
   - Ensure forwarding priority is correct (EX/MEM > MEM/WB)
   - Verify load-use stall timing (data available after MEM stage)

## Usage

1. Load `mem/raw_hazard_test.hex` into instruction memory
2. Run simulation for sufficient cycles to complete all tests
3. Monitor forwarding control signals (`ForwardA`, `ForwardB`)
4. Monitor hazard detection signals (`stall`, `id_ex_flush`)
5. Verify final register and memory values match expected results
6. Check pipeline stall behavior for load-use hazards

## Related Documentation

- `docs/ARCHITECTURE.md` - Pipeline architecture details
- `docs/MEMORY_DEBUG_GUIDE.md` - Memory operation debugging
- `src/forwarding_unit.sv` - Forwarding unit implementation
- `src/hazard_detection_unit.sv` - Hazard detection unit implementation

