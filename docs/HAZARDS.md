# Pipeline Hazards in RISC-V 5-Stage Processor

## Table of Contents

1. [Overview](#overview)
2. [Data Hazards](#data-hazards)
   - [RAW Hazards](#raw-hazards)
   - [Forwarding Unit](#forwarding-unit)
   - [EX/MEM Forwarding](#exmem-forwarding)
   - [MEM/WB Forwarding](#memwb-forwarding)
   - [Load-Use Hazards](#load-use-hazards)
3. [Control Hazards](#control-hazards)
   - [Branch Hazards](#branch-hazards)
   - [Jump Hazards](#jump-hazards)
   - [Pipeline Flush Mechanism](#pipeline-flush-mechanism)
4. [Hazard Resolution Summary](#hazard-resolution-summary)
5. [Performance Impact](#performance-impact)

---

## Overview

Pipeline hazards occur when the pipeline cannot proceed without stalling or inserting bubbles. The RISC-V 5-stage pipeline handles three types of hazards:

1. **Data Hazards**: Dependencies between instructions where one instruction needs data produced by a previous instruction
2. **Control Hazards**: Branch/jump instructions change the program counter, causing incorrect instructions to be fetched
3. **Structural Hazards**: Resource conflicts (not applicable in this design due to separate instruction and data memory)

### Pipeline Stage Overview

```
┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐
│ IF  │──▶│ ID  │──▶│ EX  │──▶│ MEM │──▶│ WB  │
└─────┘   └─────┘   └─────┘   └─────┘   └─────┘
   │         │         │         │         │
   │         │         │         │         │
   ▼         ▼         ▼         ▼         ▼
 IF/ID     ID/EX     EX/MEM    MEM/WB     RF
```

**Key Points**:
- Each stage completes in one clock cycle
- Multiple instructions are in flight simultaneously
- Data flows forward through pipeline registers
- Write-back occurs in WB stage, data available for ID stage in next cycle

---

## Data Hazards

### RAW Hazards

**Read-After-Write (RAW) Hazard**: An instruction reads a register that a previous instruction writes to.

**Example**:
```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x4, x1, x5    # I2: Reads x1 (depends on I1)
```

**Problem**: When I2 reaches the EX stage, I1's result may not yet be written to the register file.

**Solution**: **Forwarding (Bypassing)** - Forward data from later pipeline stages directly to the EX stage, bypassing the register file.

### Forwarding Unit

The Forwarding Unit detects RAW hazards and generates control signals to select the correct data source for ALU operands.

**Forwarding Paths**:
1. **EX/MEM → EX**: Forward ALU result from previous instruction (1 instruction ahead)
2. **MEM/WB → EX**: Forward data from memory or ALU result (2 instructions ahead)
3. **Register File → EX**: Normal case (no hazard)

**Forwarding Control Encoding**:
- `ForwardA/ForwardB = 00`: No forwarding (use register file data)
- `ForwardA/ForwardB = 01`: Forward from MEM/WB stage
- `ForwardA/ForwardB = 10`: Forward from EX/MEM stage
- `ForwardA/ForwardB = 11`: Reserved

**Priority**: EX/MEM forwarding has priority over MEM/WB forwarding (if both stages write to the same register, use EX/MEM data).

### EX/MEM Forwarding

**Scenario**: Instruction I2 needs data from instruction I1, where I1 is in MEM stage and I2 is in EX stage.

**Example**:
```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x4, x1, x5    # I2: Reads x1 (needs I1's result)
```

**Pipeline Timing (Without Forwarding)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   EX   MEM  WB
                    ↑
              I2 needs x1, but I1 hasn't written to RF yet!
```

**Pipeline Timing (With Forwarding)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   EX   MEM  WB
                    ↑
              Forward x1 from EX/MEM register → ALU operand A
```

**Forwarding Detection**:
- Condition: `ex_mem_reg_write = 1` AND `ex_mem_rd_addr == id_ex_rs1_addr` AND `ex_mem_rd_addr != 0`
- Action: `ForwardA = 10` (forward from EX/MEM)
- Result: ALU operand A uses `ex_mem_alu_result` instead of register file data

**Detailed Timing Diagram**:

```
Instruction Sequence:
  I1: ADD x1, x2, x3
  I2: ADD x4, x1, x5

Pipeline Stages:
┌─────────────────────────────────────────────────────────────┐
│ Cycle 1                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I1 (ADD x1, x2, x3)                            │
│ ID:  (empty)                                                │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 2                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I2 (ADD x4, x1, x5)                             │
│ ID:  Decode I1, read x2, x3 from RF                         │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 3                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I3                                               │
│ ID:  Decode I2, read x1, x5 from RF                        │
│      ⚠️  x1 not yet written by I1!                         │
│ EX:  Execute I1: x1 = x2 + x3                              │
│      ALU computes: x2 + x3 → alu_result                    │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 4 (WITHOUT FORWARDING - INCORRECT)                    │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I4                                               │
│ ID:  Decode I3                                              │
│ EX:  Execute I2: x4 = x1 + x5                              │
│      ❌ Uses OLD x1 value from RF (incorrect!)             │
│ MEM: I1: Store x1 result in EX/MEM register                │
│      ex_mem_alu_result = x2 + x3                            │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 4 (WITH FORWARDING - CORRECT)                        │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I4                                               │
│ ID:  Decode I3                                              │
│ EX:  Execute I2: x4 = x1 + x5                              │
│      ✅ Forwarding Unit detects hazard                      │
│      ✅ ForwardA = 10 (forward from EX/MEM)                 │
│      ✅ ALU operand A = ex_mem_alu_result (x2 + x3)        │
│      ✅ ALU computes: (x2 + x3) + x5 → alu_result          │
│ MEM: I1: Store x1 result in EX/MEM register                │
│      ex_mem_alu_result = x2 + x3                            │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘
```

**Forwarding Logic**:
```systemverilog
// Forwarding detection for rs1 (ForwardA)
if (ex_mem_reg_write && 
    (ex_mem_rd_addr != 0) && 
    (ex_mem_rd_addr == id_ex_rs1_addr)) begin
    ForwardA = 2'b10;  // Forward from EX/MEM
    alu_operand_a = ex_mem_alu_result;  // Use forwarded data
end
```

### MEM/WB Forwarding

**Scenario**: Instruction I2 needs data from instruction I1, where I1 is in WB stage and I2 is in EX stage.

**Example**:
```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x4, x1, x5    # I2: Reads x1 (needs I1's result)
```

**Pipeline Timing (With Forwarding)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   EX   MEM  WB
                         ↑
              Forward x1 from MEM/WB register → ALU operand A
```

**Forwarding Detection**:
- Condition: `mem_wb_reg_write = 1` AND `mem_wb_rd_addr == id_ex_rs1_addr` AND `mem_wb_rd_addr != 0` AND NOT (EX/MEM hazard)
- Action: `ForwardA = 01` (forward from MEM/WB)
- Result: ALU operand A uses `mem_wb_write_data` instead of register file data

**Detailed Timing Diagram**:

```
Instruction Sequence:
  I1: ADD x1, x2, x3
  I2: ADD x4, x1, x5

┌─────────────────────────────────────────────────────────────┐
│ Cycle 4                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I3                                               │
│ ID:  Decode I2, read x1, x5 from RF                        │
│      ⚠️  x1 not yet written by I1!                         │
│ EX:  Execute I1: x1 = x2 + x3                              │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 5                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I4                                               │
│ ID:  Decode I3                                              │
│ EX:  Execute I2: x4 = x1 + x5                              │
│      ✅ Forwarding Unit detects hazard                      │
│      ✅ ForwardA = 01 (forward from MEM/WB)                 │
│      ✅ ALU operand A = mem_wb_write_data (x2 + x3)         │
│      ✅ ALU computes: (x2 + x3) + x5 → alu_result          │
│ MEM: I1: Store x1 result in MEM/WB register                │
│      mem_wb_write_data = x2 + x3                           │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘
```

**Forwarding Logic**:
```systemverilog
// Forwarding detection for rs1 (ForwardA)
// Check MEM/WB hazard only if EX/MEM hazard not present
else if (mem_wb_reg_write && 
         (mem_wb_rd_addr != 0) && 
         (mem_wb_rd_addr == id_ex_rs1_addr)) begin
    ForwardA = 2'b01;  // Forward from MEM/WB
    alu_operand_a = mem_wb_write_data;  // Use forwarded data
end
```

### Forwarding Examples

#### Example 1: EX/MEM Forwarding (rs1)

```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x4, x1, x5    # I2: Reads x1
```

**Pipeline State at Cycle 3**:
- I1 in EX stage: Computing `x2 + x3`, result will be in EX/MEM register next cycle
- I2 in ID stage: Needs to read x1

**Pipeline State at Cycle 4**:
- I1 in MEM stage: `ex_mem_alu_result = x2 + x3`, `ex_mem_rd_addr = x1`
- I2 in EX stage: Needs x1 for ALU operand A
- **Forwarding Unit**: Detects `ex_mem_rd_addr == id_ex_rs1_addr`, sets `ForwardA = 10`
- **Result**: I2 uses `ex_mem_alu_result` (correct value) instead of old x1 from register file

#### Example 2: MEM/WB Forwarding (rs2)

```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x4, x5, x1    # I2: Reads x1 (as rs2)
```

**Pipeline State at Cycle 4**:
- I1 in MEM stage: `ex_mem_alu_result = x2 + x3`
- I2 in ID stage: Needs to read x1

**Pipeline State at Cycle 5**:
- I1 in WB stage: `mem_wb_write_data = x2 + x3`, `mem_wb_rd_addr = x1`
- I2 in EX stage: Needs x1 for ALU operand B
- **Forwarding Unit**: Detects `mem_wb_rd_addr == id_ex_rs2_addr`, sets `ForwardB = 01`
- **Result**: I2 uses `mem_wb_write_data` (correct value) instead of old x1 from register file

#### Example 3: Multiple Forwarding (Both Operands)

```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x4, x1, x1    # I2: Reads x1 twice (rs1 and rs2)
```

**Pipeline State at Cycle 4**:
- I1 in MEM stage: `ex_mem_alu_result = x2 + x3`, `ex_mem_rd_addr = x1`
- I2 in EX stage: Needs x1 for both ALU operands
- **Forwarding Unit**: 
  - Detects `ex_mem_rd_addr == id_ex_rs1_addr`, sets `ForwardA = 10`
  - Detects `ex_mem_rd_addr == id_ex_rs2_addr`, sets `ForwardB = 10`
- **Result**: Both ALU operands use `ex_mem_alu_result` (correct value)

#### Example 4: Forwarding Priority

```assembly
ADD x1, x2, x3    # I1: Writes x1
ADD x1, x4, x5    # I2: Writes x1 (overwrites I1)
ADD x6, x1, x7    # I3: Reads x1 (needs I2's result, not I1's)
```

**Pipeline State at Cycle 5**:
- I1 in WB stage: `mem_wb_write_data = x2 + x3`, `mem_wb_rd_addr = x1`
- I2 in MEM stage: `ex_mem_alu_result = x4 + x5`, `ex_mem_rd_addr = x1`
- I3 in EX stage: Needs x1 for ALU operand A
- **Forwarding Unit**: 
  - Detects EX/MEM hazard: `ex_mem_rd_addr == id_ex_rs1_addr`, sets `ForwardA = 10`
  - EX/MEM forwarding has priority over MEM/WB forwarding
- **Result**: I3 uses `ex_mem_alu_result` (I2's result) instead of `mem_wb_write_data` (I1's result)

### Load-Use Hazards

**Load-Use Hazard**: A load instruction followed by an instruction that uses the loaded data immediately.

**Problem**: Load instructions read data from memory in the MEM stage. The data is not available until after the MEM stage completes. Forwarding cannot resolve this hazard because the data is not yet available when the dependent instruction needs it in the EX stage.

**Example**:
```assembly
LW x1, 0(x2)      # I1: Load data into x1
ADD x3, x1, x4    # I2: Uses x1 immediately (hazard!)
```

**Pipeline Timing (Without Stall - INCORRECT)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   EX   MEM  WB
                    ↑
              I2 needs x1, but load data not available until MEM completes!
```

**Pipeline Timing (With Stall - CORRECT)**:
```
Cycle:  1    2    3    4    5    6    7
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   [STALL] EX   MEM  WB
                    ↑
              Stall for 1 cycle, then I2 can use x1 from register file
```

**Hazard Detection**:
- Condition: `id_ex_MemRead = 1` AND (`id_ex_rd_addr == if_id_rs1_addr` OR `id_ex_rd_addr == if_id_rs2_addr`) AND `id_ex_rd_addr != 0`
- Action: Assert `stall = 1` and `id_ex_flush = 1`

**Detailed Timing Diagram**:

```
Instruction Sequence:
  I1: LW x1, 0(x2)
  I2: ADD x3, x1, x4

┌─────────────────────────────────────────────────────────────┐
│ Cycle 1                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I1 (LW x1, 0(x2))                               │
│ ID:  (empty)                                                │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 2                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I2 (ADD x3, x1, x4)                             │
│ ID:  Decode I1, read x2 from RF                            │
│      Control: MemRead = 1, rd = x1                         │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 3                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I3 (would normally fetch next instruction)      │
│ ID:  Decode I2, read x1, x4 from RF                        │
│      ⚠️  Hazard Detection Unit detects load-use hazard!    │
│      - id_ex_MemRead = 1 (I1 is load)                      │
│      - id_ex_rd_addr = x1                                  │
│      - if_id_rs1_addr = x1 (I2 reads x1)                  │
│      - Hazard condition met!                               │
│ EX:  Execute I1: Compute address = x2 + 0                  │
│      ALU computes: x2 + 0 → alu_result (memory address)    │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 4 (STALL CYCLE)                                       │
├─────────────────────────────────────────────────────────────┤
│ IF:  [STALL] PC holds, no new instruction fetched          │
│      ✅ Stall signal prevents PC update                     │
│ ID:  [STALL] IF/ID register holds I2                       │
│      ✅ Stall signal prevents IF/ID register update        │
│      I2 remains in ID stage                                 │
│ EX:  [NOP] ID/EX register flushed (NOP inserted)           │
│      ✅ id_ex_flush signal clears ID/EX register           │
│      Control signals set to safe defaults (no operation)  │
│ MEM: I1: Read data from memory at address (x2 + 0)         │
│      Memory reads: Memory[x2] → mem_read_data             │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 5                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I3 (stall released)                             │
│ ID:  Decode I2 (still in ID stage, now can proceed)        │
│      Read x1, x4 from RF                                    │
│      ✅ x1 now available from register file!               │
│      (I1's result written in WB stage this cycle)           │
│ EX:  Execute I1: (NOP from previous cycle)                 │
│ MEM: I1: Store memory read data in MEM/WB register        │
│      mem_wb_mem_read_data = Memory[x2]                     │
│ WB:  I1: Write memory data to x1                            │
│      ✅ x1 = Memory[x2] (written to register file)         │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 6                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I4                                               │
│ ID:  Decode I3                                              │
│ EX:  Execute I2: x3 = x1 + x4                              │
│      ✅ Uses correct x1 value from register file           │
│      ALU computes: Memory[x2] + x4 → alu_result            │
│ MEM: I1: (NOP from cycle 4)                                 │
│ WB:  I1: (write-back completed in cycle 5)                 │
└─────────────────────────────────────────────────────────────┘
```

**Stall Mechanism**:
1. **PC Stall**: `stall = 1` prevents PC from updating, holds current instruction
2. **IF/ID Stall**: `stall = 1` prevents IF/ID register from updating, holds current instruction
3. **ID/EX Flush**: `id_ex_flush = 1` clears ID/EX register, inserts NOP bubble
4. **Result**: Load completes in MEM stage, data written to register file in WB stage, dependent instruction can proceed

**Hazard Detection Logic**:
```systemverilog
// Load-use hazard detection
assign rs1_hazard = id_ex_MemRead && 
                    (id_ex_rd_addr != 5'b00000) && 
                    (id_ex_rd_addr == if_id_rs1_addr);

assign rs2_hazard = id_ex_MemRead && 
                    (id_ex_rd_addr != 5'b00000) && 
                    (id_ex_rd_addr == if_id_rs2_addr);

assign load_use_hazard = rs1_hazard || rs2_hazard;

assign stall = load_use_hazard;
assign id_ex_flush = load_use_hazard;
```

**Why Forwarding Cannot Help**:
- Load data is read from memory in MEM stage
- Data is not available until MEM stage completes
- Dependent instruction needs data in EX stage
- EX stage executes before MEM stage completes
- **Solution**: Stall for one cycle to allow load to complete

---

## Control Hazards

### Branch Hazards

**Control Hazard**: Branch instructions change the program counter based on a condition evaluated in the EX stage. Instructions fetched after the branch may be incorrect if the branch is taken.

**Problem**: Branch condition is evaluated in EX stage, but PC update happens in IF stage. By the time we know if branch is taken, we've already fetched incorrect instructions.

**Example**:
```assembly
BEQ x1, x2, label  # I1: Branch if x1 == x2
ADD x3, x4, x5      # I2: Fetched assuming branch not taken
ADD x6, x7, x8      # I3: Fetched assuming branch not taken
```

**Pipeline Timing (Branch Not Taken - No Penalty)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   EX   MEM  WB
I3:            IF   ID   EX   MEM  WB
                    ↑
              Branch condition evaluated: NOT TAKEN
              Continue sequential execution (no flush)
```

**Pipeline Timing (Branch Taken - Flush Required)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   ID   [FLUSH] MEM  WB
I3:            IF   [FLUSH] EX   MEM  WB
                    ↑
              Branch condition evaluated: TAKEN
              Flush I2 and I3, fetch from branch target
```

**Detailed Timing Diagram (Branch Taken)**:

```
Instruction Sequence:
  I1: BEQ x1, x2, label  (assume x1 == x2, branch taken)
  I2: ADD x3, x4, x5     (incorrectly fetched)
  I3: ADD x6, x7, x8     (incorrectly fetched)
  label: ADD x9, x10, x11 (correct target)

┌─────────────────────────────────────────────────────────────┐
│ Cycle 1                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I1 (BEQ x1, x2, label)                          │
│      PC = address of I1                                     │
│ ID:  (empty)                                                │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 2                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I2 (ADD x3, x4, x5)                              │
│      PC = PC + 4 (sequential, assuming branch not taken)   │
│      ⚠️  Branch not yet evaluated!                          │
│ ID:  Decode I1, read x1, x2 from RF                        │
│      Control: Branch = 1                                    │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 3                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I3 (ADD x6, x7, x8)                              │
│      PC = PC + 4 (still sequential)                        │
│      ⚠️  Branch not yet evaluated!                          │
│ ID:  Decode I2 (ADD x3, x4, x5)                            │
│      ⚠️  This instruction will be flushed!                  │
│ EX:  Execute I1: Evaluate branch condition                │
│      ALU computes: x1 - x2 → zero_flag                     │
│      Branch/Jump Control Unit:                              │
│        - zero_flag = 1 (x1 == x2)                          │
│        - branch_taken = 1                                   │
│        - branch_target = PC + immediate                     │
│        - PCSrc = 1 (use branch target)                      │
│        - flush = 1 (flush incorrect instructions)           │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 4 (BRANCH TAKEN - FLUSH)                             │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch from branch target (label: ADD x9, x10, x11)    │
│      ✅ PC updated to branch_target                         │
│      ✅ PCSrc = 1 selects branch target                    │
│ ID:  [FLUSH] IF/ID register cleared (NOP inserted)         │
│      ✅ flush signal clears IF/ID register                 │
│      I2 instruction discarded                               │
│ EX:  [FLUSH] ID/EX register cleared (NOP inserted)         │
│      ✅ flush signal clears ID/EX register                 │
│      I3 instruction discarded                               │
│ MEM: I1: Store branch info in EX/MEM register              │
│      mem_PCSrc = 1, mem_branch_flush = 1                   │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 5                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch next instruction after label                      │
│      PC = branch_target + 4                                │
│ ID:  Decode label instruction (ADD x9, x10, x11)            │
│      ✅ Correct instruction now in pipeline                 │
│ EX:  Execute (NOP from cycle 4)                            │
│ MEM: I1: (NOP from cycle 3)                                 │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘
```

**Branch Condition Evaluation**:
- **BEQ (000)**: `rs1 == rs2` → `zero_flag == 1`
- **BNE (001)**: `rs1 != rs2` → `zero_flag == 0`
- **BLT (100)**: `rs1 < rs2` (signed) → signed comparison
- **BGE (101)**: `rs1 >= rs2` (signed) → signed comparison
- **BLTU (110)**: `rs1 < rs2` (unsigned) → unsigned comparison
- **BGEU (111)**: `rs1 >= rs2` (unsigned) → unsigned comparison

**Flush Mechanism**:
1. **Branch Evaluation**: Branch condition evaluated in EX stage
2. **PC Update**: PC updated to branch target when `PCSrc = 1`
3. **Pipeline Flush**: `flush = 1` clears IF/ID and ID/EX registers
4. **NOP Insertion**: Flushed registers contain NOP instructions
5. **Result**: Incorrect instructions discarded, correct instruction fetched from branch target

**Branch Control Logic**:
```systemverilog
// Branch condition evaluation
if (Branch) begin
    case (funct3)
        3'b000: branch_taken = zero_flag;        // BEQ
        3'b001: branch_taken = !zero_flag;       // BNE
        3'b100: branch_taken = signed_less_than;  // BLT
        3'b101: branch_taken = !signed_less_than; // BGE
        3'b110: branch_taken = unsigned_less_than; // BLTU
        3'b111: branch_taken = !unsigned_less_than; // BGEU
    endcase
end

// PC source selection
PCSrc = (Branch && branch_taken) || Jump;

// Pipeline flush
flush = (Branch && branch_taken) || Jump;
```

### Jump Hazards

**Jump Instructions**: JAL and JALR always change the PC (unconditional jumps).

**Example**:
```assembly
JAL x1, label     # I1: Jump to label, save return address in x1
ADD x2, x3, x4    # I2: Incorrectly fetched (will be flushed)
```

**Pipeline Timing (Jump)**:
```
Cycle:  1    2    3    4    5    6
I1:    IF   ID   EX   MEM  WB
I2:        IF   [FLUSH] EX   MEM  WB
                    ↑
              Jump always taken, flush I2, fetch from jump target
```

**Detailed Timing Diagram**:

```
Instruction Sequence:
  I1: JAL x1, label
  I2: ADD x2, x3, x4  (incorrectly fetched)
  label: ADD x5, x6, x7 (correct target)

┌─────────────────────────────────────────────────────────────┐
│ Cycle 1                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I1 (JAL x1, label)                              │
│ ID:  (empty)                                                │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 2                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I2 (ADD x2, x3, x4)                             │
│      PC = PC + 4 (sequential)                               │
│      ⚠️  Jump not yet evaluated!                            │
│ ID:  Decode I1, read (no source registers for JAL)         │
│      Control: Jump = 1                                      │
│ EX:  (empty)                                                │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 3                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch I3 (would normally fetch next instruction)       │
│      ⚠️  Jump not yet evaluated!                            │
│ ID:  Decode I2 (ADD x2, x3, x4)                            │
│      ⚠️  This instruction will be flushed!                  │
│ EX:  Execute I1: Calculate jump target                     │
│      jump_target = PC + sign_extend(immediate)             │
│      Branch/Jump Control Unit:                              │
│        - Jump = 1 (always taken)                           │
│        - PCSrc = 1 (use jump target)                       │
│        - flush = 1 (flush incorrect instructions)         │
│ MEM: (empty)                                                │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 4 (JUMP TAKEN - FLUSH)                               │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch from jump target (label: ADD x5, x6, x7)        │
│      ✅ PC updated to jump_target                           │
│      ✅ PCSrc = 1 selects jump target                      │
│ ID:  [FLUSH] IF/ID register cleared (NOP inserted)         │
│      ✅ flush signal clears IF/ID register                 │
│      I2 instruction discarded                               │
│ EX:  [FLUSH] ID/EX register cleared (NOP inserted)         │
│      ✅ flush signal clears ID/EX register                 │
│      I3 instruction discarded                               │
│ MEM: I1: Store jump info in EX/MEM register                │
│      mem_PCSrc = 1, mem_branch_flush = 1                   │
│      Also: mem_jump_target = jump_target                   │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 5                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch next instruction after label                     │
│      PC = jump_target + 4                                  │
│ ID:  Decode label instruction (ADD x5, x6, x7)             │
│      ✅ Correct instruction now in pipeline                 │
│ EX:  Execute (NOP from cycle 4)                            │
│ MEM: I1: Store return address (PC+4) in MEM/WB register   │
│      mem_wb_write_data = PC + 4 (return address)           │
│ WB:  (empty)                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Cycle 6                                                     │
├─────────────────────────────────────────────────────────────┤
│ IF:  Fetch next instruction                                 │
│ ID:  Decode next instruction                                │
│ EX:  Execute label instruction (ADD x5, x6, x7)           │
│ MEM: I1: (NOP from cycle 4)                                │
│ WB:  I1: Write return address to x1                        │
│      ✅ x1 = PC + 4 (return address saved)                 │
└─────────────────────────────────────────────────────────────┘
```

**JALR Instruction**:
- Similar to JAL, but target address is `(rs1 + immediate) & ~1`
- LSB is cleared to ensure word alignment
- Always taken (unconditional)

**Flush Logic**:
```systemverilog
// Jump handling
if (Jump) begin
    PCSrc = 1'b1;           // Always use jump target
    flush = 1'b1;           // Always flush incorrect instructions
    branch_taken = 1'b1;    // Jump is always taken
end

// Pipeline flush signal
pipeline_flush_internal = mem_branch_flush || pipeline_flush;
```

### Pipeline Flush Mechanism

**Flush Signal Generation**:
- **Source**: Branch/Jump Control Unit in EX stage
- **Registered**: Flush signal registered in EX/MEM register
- **Targets**: IF/ID and ID/EX pipeline registers

**Flush Actions**:
1. **IF/ID Register**: Cleared (instruction set to NOP = 0x00000013)
2. **ID/EX Register**: Cleared (all control signals set to safe defaults)
3. **PC Update**: PC updated to branch/jump target

**Flush Timing**:
- Branch/jump condition evaluated in EX stage (cycle N)
- Flush signal generated combinational in EX stage
- Flush signal registered in EX/MEM register (cycle N+1)
- Flush takes effect on cycle N+1, clearing IF/ID and ID/EX registers

**Flush Logic**:
```systemverilog
// In IF/ID register
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        instruction_reg <= 32'h00000000;
    end else if (flush) begin
        instruction_reg <= 32'h00000013;  // NOP instruction
    end else if (enable) begin
        instruction_reg <= if_instruction;
    end
end

// In ID/EX register
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset all registers
    end else if (flush) begin
        // Clear all registers (NOP)
        ex_RegWrite <= 1'b0;
        ex_MemRead <= 1'b0;
        ex_MemWrite <= 1'b0;
        // ... other control signals set to safe defaults
    end else if (enable) begin
        // Normal operation
    end
end
```

---

## Hazard Resolution Summary

### Data Hazard Resolution

| Hazard Type | Detection | Resolution | Penalty |
|-------------|-----------|------------|---------|
| RAW (EX/MEM) | Forwarding Unit | Forward from EX/MEM | 0 cycles |
| RAW (MEM/WB) | Forwarding Unit | Forward from MEM/WB | 0 cycles |
| Load-Use | Hazard Detection Unit | Stall + Flush ID/EX | 1 cycle |

### Control Hazard Resolution

| Hazard Type | Detection | Resolution | Penalty |
|-------------|-----------|------------|---------|
| Branch Not Taken | Branch/Jump Control | No action | 0 cycles |
| Branch Taken | Branch/Jump Control | Flush IF/ID, ID/EX | 1 cycle |
| Jump (JAL/JALR) | Branch/Jump Control | Flush IF/ID, ID/EX | 1 cycle |

### Forwarding Paths Summary

```
Forwarding Paths:
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  EX/MEM Register ────────┐                             │
│                          │                             │
│                          ▼                             │
│                    Forwarding Mux                       │
│                          │                             │
│                          ▼                             │
│  MEM/WB Register ────────┼───► Forwarding Mux          │
│                          │                             │
│                          ▼                             │
│                    Register File                       │
│                          │                             │
│                          ▼                             │
│                      ALU Operands                       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

**Forwarding Priority**:
1. **EX/MEM Forwarding** (highest priority): If EX/MEM stage writes to register needed by EX stage
2. **MEM/WB Forwarding**: If MEM/WB stage writes to register needed by EX stage (and EX/MEM doesn't)
3. **Register File**: Normal case (no forwarding needed)

---

## Performance Impact

### Ideal Performance

**Without Hazards**: 1 instruction per cycle (CPI = 1.0)

### Performance with Hazards

**Load-Use Hazard**: 1-cycle stall penalty
- Example: `LW x1, 0(x2)` followed by `ADD x3, x1, x4`
- CPI impact: +1 cycle per load-use hazard

**Branch Taken**: 1-cycle flush penalty
- Example: `BEQ x1, x2, label` (taken)
- CPI impact: +1 cycle per taken branch

**Branch Not Taken**: No penalty
- Example: `BEQ x1, x2, label` (not taken)
- CPI impact: 0 cycles

**Jump**: 1-cycle flush penalty
- Example: `JAL x1, label`
- CPI impact: +1 cycle per jump

### Example Performance Calculation

**Instruction Sequence**:
```assembly
ADD x1, x2, x3    # No hazard (forwarding resolves)
ADD x4, x1, x5    # EX/MEM forwarding (0 cycles penalty)
LW x6, 0(x7)      # Load instruction
ADD x8, x6, x9    # Load-use hazard (1 cycle stall)
BEQ x1, x2, label # Branch (assume taken, 1 cycle flush)
ADD x10, x11, x12 # After branch target
```

**Cycle-by-Cycle Execution**:
```
Cycle 1: ADD x1, x2, x3 (IF)
Cycle 2: ADD x1, x2, x3 (ID), ADD x4, x1, x5 (IF)
Cycle 3: ADD x1, x2, x3 (EX), ADD x4, x1, x5 (ID), LW x6, 0(x7) (IF)
Cycle 4: ADD x1, x2, x3 (MEM), ADD x4, x1, x5 (EX, uses forwarded x1), LW x6, 0(x7) (ID), ADD x8, x6, x9 (IF)
Cycle 5: ADD x1, x2, x3 (WB), ADD x4, x1, x5 (MEM), LW x6, 0(x7) (EX), ADD x8, x6, x9 (ID, STALL detected)
Cycle 6: ADD x4, x1, x5 (WB), LW x6, 0(x7) (MEM), [STALL], ADD x8, x6, x9 (ID, still stalled)
Cycle 7: LW x6, 0(x7) (WB), ADD x8, x6, x9 (EX, uses x6 from RF), BEQ x1, x2, label (IF)
Cycle 8: ADD x8, x6, x9 (MEM), BEQ x1, x2, label (ID), ADD x10, x11, x12 (IF, incorrect)
Cycle 9: ADD x8, x6, x9 (WB), BEQ x1, x2, label (EX, branch taken), [FLUSH]
Cycle 10: BEQ x1, x2, label (MEM), ADD x10, x11, x12 (IF, from branch target)
Cycle 11: BEQ x1, x2, label (WB), ADD x10, x11, x12 (ID)
Cycle 12: ADD x10, x11, x12 (EX)
...
```

**Total Cycles**: 12 cycles for 6 instructions
**CPI**: 12 / 6 = 2.0 cycles per instruction

**Breakdown**:
- 1 load-use hazard: +1 cycle
- 1 branch taken: +1 cycle
- Total penalty: +2 cycles
- Ideal cycles: 6 cycles
- Actual cycles: 8 cycles (excluding pipeline fill)

---

## Conclusion

The RISC-V 5-stage pipeline handles hazards through:

1. **Forwarding**: Resolves most RAW hazards without stalling (0-cycle penalty)
2. **Stalling**: Handles load-use hazards that cannot be resolved by forwarding (1-cycle penalty)
3. **Flushing**: Handles control hazards from branches/jumps (1-cycle penalty)

**Key Design Principles**:
- Forwarding eliminates most data hazard stalls
- Stalling only when necessary (load-use hazards)
- Flushing only when control flow changes (branches/jumps taken)
- Minimize performance impact while maintaining correctness

**Performance Characteristics**:
- **Ideal CPI**: 1.0 (no hazards)
- **Typical CPI**: 1.0 - 1.5 (with forwarding, minimal stalls)
- **Worst-case CPI**: Higher (many load-use hazards or branches)

---

*This document explains hazard handling in the RISC-V 5-stage pipeline processor. For implementation details, refer to the source code in `src/forwarding_unit.sv`, `src/hazard_detection_unit.sv`, and `src/branch_jump_control.sv`.*

