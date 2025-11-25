# RISC-V RV32I 5-Stage Pipeline Architecture

## Table of Contents

1. [Overview](#overview)
2. [Pipeline Stages](#pipeline-stages)
3. [Pipeline Registers](#pipeline-registers)
4. [Data Path](#data-path)
5. [Control Signals](#control-signals)
6. [Pipeline Hazards](#pipeline-hazards)
7. [Timing Considerations](#timing-considerations)
8. [Implementation Details](#implementation-details)

---

## Overview

The RISC-V RV32I processor implements a classic 5-stage pipeline architecture that processes one instruction per clock cycle (under ideal conditions). The pipeline consists of five distinct stages:

1. **Instruction Fetch (IF)**: Retrieves instructions from instruction memory
2. **Instruction Decode (ID)**: Decodes instructions and reads register file
3. **Execute (EX)**: Performs arithmetic/logic operations and calculates addresses
4. **Memory (MEM)**: Accesses data memory for load/store operations
5. **Writeback (WB)**: Writes results back to the register file

### Pipeline Philosophy

The pipeline improves instruction throughput by overlapping the execution of multiple instructions. While one instruction is being fetched, another is being decoded, a third is executing, and so on. This allows the processor to achieve an average of one instruction per cycle (IPC) in the absence of hazards.

### Clock Cycle Breakdown

Each stage completes its operation within a single clock cycle. The critical path determines the maximum clock frequency and typically lies in the Execute stage (ALU operations) or Memory stage (memory access).

---

## Pipeline Stages

### Stage 1: Instruction Fetch (IF)

**Purpose**: Fetch the next instruction from instruction memory and update the program counter.

**Operations**:
- Read instruction from instruction memory at address `PC`
- Increment PC: `PC = PC + 4` (for next sequential instruction)
- Handle branch/jump target calculation (preliminary)

**Inputs**:
- `PC`: Current program counter value
- `branch_target`: Branch/jump target address (from EX stage)
- `branch_taken`: Control signal indicating branch taken
- `stall`: Pipeline stall signal

**Outputs**:
- `instruction`: 32-bit instruction word
- `PC_plus_4`: PC + 4 for next instruction
- `PC_out`: Updated PC value

**Pipeline Register**: IF/ID

**Block Diagram**:
```
┌─────────────┐
│     PC      │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Instruction │
│   Memory    │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Instruction │
│  (32 bits)  │
└─────────────┘
```

**Key Components**:
- Program Counter (PC) register
- Instruction Memory (IMEM)
- PC increment logic (PC + 4)
- Branch target multiplexer

**Control Signals**:
- `PCWrite`: Enable PC update (disabled during stalls)
- `IF_flush`: Flush instruction on branch misprediction

---

### Stage 2: Instruction Decode (ID)

**Purpose**: Decode the instruction, read operands from register file, generate control signals, and prepare immediate values.

**Operations**:
- Decode instruction opcode and function fields
- Read register file: `rs1_data`, `rs2_data`
- Generate immediate value (I-type, S-type, B-type, U-type, J-type)
- Generate control signals for downstream stages
- Detect data hazards (read-after-write dependencies)

**Inputs**:
- `instruction`: 32-bit instruction from IF stage
- `PC_plus_4`: PC + 4 from IF stage
- `write_data`: Data to write to register file (from WB stage)
- `write_reg`: Destination register address (from WB stage)
- `reg_write_en`: Register write enable (from WB stage)

**Outputs**:
- `rs1_data`: Data from source register 1
- `rs2_data`: Data from source register 2
- `immediate`: Sign-extended immediate value
- `rd`: Destination register address
- `rs1_addr`: Source register 1 address
- `rs2_addr`: Source register 2 address
- Control signals (see Control Signals section)

**Pipeline Register**: ID/EX

**Block Diagram**:
```
┌─────────────┐
│ Instruction │
│  (32 bits)  │
└──────┬──────┘
       │
       ├─────────────┐
       │             │
       ▼             ▼
┌─────────────┐ ┌─────────────┐
│   Control   │ │    Imm      │
│    Unit     │ │   Generator │
└──────┬──────┘ └──────┬──────┘
       │               │
       │      ┌────────┴────────┐
       │      │                 │
       ▼      ▼                 ▼
┌─────────────────────────────────┐
│      Register File              │
│  ┌──────┐  ┌──────┐  ┌──────┐ │
│  │ rs1  │  │ rs2  │  │  rd  │ │
│  └──────┘  └──────┘  └──────┘ │
└─────────────────────────────────┘
```

**Key Components**:
- Control Unit (generates control signals)
- Register File (32 registers, x0 hardwired to 0)
- Immediate Generator (sign extension logic)
- Hazard Detection Unit (detects data hazards)

**Instruction Fields Decoded**:
- `opcode[6:0]`: Instruction opcode
- `rd[4:0]`: Destination register
- `funct3[2:0]`: Function field (for R/I/S/B types)
- `rs1[4:0]`: Source register 1
- `rs2[4:0]`: Source register 2
- `funct7[6:0]`: Function field (for R-type)

**Control Signals Generated**:
- `RegWrite`: Write to register file
- `MemRead`: Memory read enable
- `MemWrite`: Memory write enable
- `MemToReg`: Select memory data vs ALU result
- `ALUOp`: ALU operation control
- `ALUSrc`: ALU source selection (register vs immediate)
- `Branch`: Branch instruction detected
- `Jump`: Jump instruction detected

---

### Stage 3: Execute (EX)

**Purpose**: Perform arithmetic/logic operations, calculate memory addresses, evaluate branch conditions, and handle data forwarding.

**Operations**:
- Execute ALU operations (arithmetic, logic, shift, compare)
- Calculate memory addresses: `address = rs1_data + immediate`
- Evaluate branch conditions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
- Handle data forwarding from MEM and WB stages
- Select ALU operands (register data vs immediate)

**Inputs**:
- `rs1_data`: Source register 1 data (from ID stage or forwarded)
- `rs2_data`: Source register 2 data (from ID stage or forwarded)
- `immediate`: Immediate value from ID stage
- `PC_plus_4`: PC + 4 from ID stage
- `ALUOp`: ALU operation control
- `ALUSrc`: ALU source selection
- Forwarded data from MEM and WB stages
- Forward control signals

**Outputs**:
- `ALU_result`: Result of ALU operation
- `branch_target`: Calculated branch/jump target address
- `branch_taken`: Branch condition evaluation result
- `zero_flag`: ALU zero flag (for BEQ/BNE)
- `less_than_flag`: ALU less-than flag (for BLT/BGE)

**Pipeline Register**: EX/MEM

**Block Diagram**:
```
┌─────────────┐     ┌─────────────┐
│   rs1_data  │     │   rs2_data  │
└──────┬──────┘     └──────┬──────┘
       │                   │
       │    ┌──────────────┴──────┐
       │    │  Forwarding Muxes   │
       │    └──────────────┬──────┘
       │                   │
       ▼                   ▼
┌─────────────────────────────────┐
│         ALU                     │
│  ┌──────────────────────────┐  │
│  │  Operation:              │  │
│  │  ADD, SUB, AND, OR, XOR  │  │
│  │  SLL, SRL, SRA, SLT, etc │  │
│  └──────────────────────────┘  │
└──────────────┬──────────────────┘
               │
               ▼
        ┌─────────────┐
        │ ALU_result  │
        └─────────────┘
```

**Key Components**:
- Arithmetic Logic Unit (ALU)
- Forwarding Unit (data hazard resolution)
- Branch Unit (branch condition evaluation)
- Address calculation logic

**ALU Operations**:
- **Arithmetic**: ADD, SUB
- **Logic**: AND, OR, XOR
- **Shift**: SLL, SRL, SRA
- **Compare**: SLT, SLTU
- **Immediate variants**: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU

**Forwarding Paths**:
1. **EX→EX**: Forward ALU result from previous instruction (EX/MEM) to current instruction
2. **MEM→EX**: Forward memory data from MEM stage to current instruction
3. **WB→EX**: Forward writeback data to current instruction

**Forwarding Mux Control**:
- `ForwardA`: Select rs1_data source (00: ID/EX, 01: EX/MEM, 10: MEM/WB)
- `ForwardB`: Select rs2_data source (00: ID/EX, 01: EX/MEM, 10: MEM/WB)

---

### Stage 4: Memory Access (MEM)

**Purpose**: Access data memory for load/store operations, or pass through ALU results for non-memory instructions.

**Operations**:
- Read from data memory (LB, LH, LW, LBU, LHU)
- Write to data memory (SB, SH, SW)
- Pass ALU result through for non-memory instructions
- Handle memory alignment and byte/halfword operations

**Inputs**:
- `ALU_result`: Memory address (from EX stage)
- `rs2_data`: Data to write to memory (from EX stage, may be forwarded)
- `MemRead`: Memory read enable
- `MemWrite`: Memory write enable
- `MemWidth`: Memory access width (byte, halfword, word)
- `MemSignExt`: Sign extension control for loads

**Outputs**:
- `mem_read_data`: Data read from memory
- `ALU_result_out`: ALU result passed through

**Pipeline Register**: MEM/WB

**Block Diagram**:
```
┌─────────────┐
│ ALU_result  │ (address)
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    Data     │
│   Memory    │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ mem_read_   │
│    data     │
└─────────────┘
```

**Key Components**:
- Data Memory (DMEM)
- Memory control logic
- Byte/halfword/word selection logic
- Sign extension logic for loads

**Memory Operations**:
- **Loads**: LB, LH, LW, LBU, LHU
- **Stores**: SB, SH, SW

**Memory Access Widths**:
- **Byte (8 bits)**: LB, LBU, SB
- **Halfword (16 bits)**: LH, LHU, SH
- **Word (32 bits)**: LW, SW

**Sign Extension**:
- Signed loads (LB, LH): Sign-extend to 32 bits
- Unsigned loads (LBU, LHU): Zero-extend to 32 bits

---

### Stage 5: Writeback (WB)

**Purpose**: Write results back to the register file, selecting between ALU result and memory data.

**Operations**:
- Select data source: ALU result or memory data
- Write to register file (if RegWrite enabled)
- Update destination register (rd)

**Inputs**:
- `ALU_result`: ALU result from MEM stage
- `mem_read_data`: Data from memory (from MEM stage)
- `MemToReg`: Select ALU result vs memory data
- `rd`: Destination register address
- `RegWrite`: Register write enable

**Outputs**:
- `write_data`: Data to write to register file
- `write_reg`: Destination register address
- `reg_write_en`: Register write enable

**Pipeline Register**: None (final stage)

**Block Diagram**:
```
┌─────────────┐     ┌─────────────┐
│ ALU_result  │     │ mem_read_   │
│             │     │    data     │
└──────┬──────┘     └──────┬──────┘
       │                   │
       └─────────┬─────────┘
                 │
                 ▼
         ┌───────────────┐
         │  MemToReg Mux │
         └───────┬───────┘
                 │
                 ▼
         ┌───────────────┐
         │  Register     │
         │     File      │
         └───────────────┘
```

**Key Components**:
- Writeback multiplexer (MemToReg)
- Register file write port

**Data Source Selection**:
- `MemToReg = 0`: Write ALU result to register file
- `MemToReg = 1`: Write memory data to register file

**Register File Write**:
- Only occurs when `RegWrite = 1`
- Register x0 is hardwired to 0 (writes to x0 are ignored)

---

## Pipeline Registers

Pipeline registers store intermediate values between pipeline stages, allowing each stage to operate independently. They are clocked registers that capture data at the end of each clock cycle.

### IF/ID Pipeline Register

**Location**: Between IF and ID stages

**Contents**:
- `if_id_instruction[31:0]`: Instruction word
- `if_id_PC_plus_4[31:0]`: PC + 4 value

**Control Signals**:
- `IF_flush`: Clear register on branch taken
- `IF_stall`: Hold current values during stall

**Reset Behavior**: Cleared to zero on reset

### ID/EX Pipeline Register

**Location**: Between ID and EX stages

**Contents**:
- `id_ex_PC_plus_4[31:0]`: PC + 4
- `id_ex_rs1_data[31:0]`: Source register 1 data
- `id_ex_rs2_data[31:0]`: Source register 2 data
- `id_ex_immediate[31:0]`: Immediate value
- `id_ex_rd[4:0]`: Destination register address
- `id_ex_rs1_addr[4:0]`: Source register 1 address
- `id_ex_rs2_addr[4:0]`: Source register 2 address
- Control signals: `RegWrite`, `MemRead`, `MemWrite`, `MemToReg`, `ALUOp`, `ALUSrc`, `Branch`, `Jump`

**Control Signals**:
- `ID_stall`: Insert NOP (bubble) during stall
- `ID_flush`: Clear register on branch misprediction

**Reset Behavior**: Cleared to zero (NOP instruction)

### EX/MEM Pipeline Register

**Location**: Between EX and MEM stages

**Contents**:
- `ex_mem_ALU_result[31:0]`: ALU computation result
- `ex_mem_rs2_data[31:0]`: Source register 2 data (for stores)
- `ex_mem_rd[4:0]`: Destination register address
- `ex_mem_branch_target[31:0]`: Calculated branch target
- `ex_mem_branch_taken`: Branch taken signal
- Control signals: `RegWrite`, `MemRead`, `MemWrite`, `MemToReg`

**Control Signals**:
- `EX_stall`: Hold values during stall (rare)

**Reset Behavior**: Cleared to zero

### MEM/WB Pipeline Register

**Location**: Between MEM and WB stages

**Contents**:
- `mem_wb_ALU_result[31:0]`: ALU result
- `mem_wb_mem_read_data[31:0]`: Data read from memory
- `mem_wb_rd[4:0]`: Destination register address
- Control signals: `RegWrite`, `MemToReg`

**Control Signals**:
- `MEM_stall`: Hold values during stall (rare)

**Reset Behavior**: Cleared to zero

### Pipeline Register Timing

All pipeline registers are positive-edge triggered:
- **Setup Time**: Data must be stable before clock edge
- **Hold Time**: Data must remain stable after clock edge
- **Clock-to-Q Delay**: Output available after clock edge

---

## Data Path

The data path connects all pipeline stages and components, allowing data to flow from instruction fetch through writeback.

### Complete Data Path Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         INSTRUCTION FETCH                            │
│  ┌──────┐      ┌──────────────┐                                    │
│  │  PC  │─────▶│ Instruction  │                                    │
│  └──┬───┘      │    Memory    │                                    │
│     │          └──────┬───────┘                                    │
│     │                 │                                            │
│     │          ┌──────▼───────┐                                    │
│     │          │ Instruction  │                                    │
│     │          └──────┬───────┘                                    │
│     │                 │                                            │
│     └─────────────────┘                                            │
│              IF/ID Pipeline Register                                │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        INSTRUCTION DECODE                            │
│  ┌──────────────┐      ┌──────────────┐                           │
│  │   Control    │      │    Imm       │                           │
│  │    Unit      │      │  Generator   │                           │
│  └──────┬───────┘      └──────┬───────┘                           │
│         │                     │                                     │
│         │      ┌──────────────┴──────────────┐                    │
│         │      │                             │                    │
│         ▼      ▼                             ▼                    │
│  ┌──────────────────────────────────────────────────┐             │
│  │           Register File (32 regs)                │             │
│  │  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐        │             │
│  │  │ rs1  │  │ rs2  │  │  rd  │  │ ...  │        │             │
│  │  └──────┘  └──────┘  └──────┘  └──────┘        │             │
│  └──────────────────────────────────────────────────┘             │
│              ID/EX Pipeline Register                               │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                            EXECUTE                                   │
│  ┌──────────────┐      ┌──────────────┐                           │
│  │   Forwarding │      │   Forwarding │                           │
│  │     Unit     │      │     Muxes    │                           │
│  └──────┬───────┘      └──────┬───────┘                           │
│         │                     │                                     │
│         │      ┌──────────────┴──────────────┐                    │
│         │      │                             │                    │
│         ▼      ▼                             ▼                    │
│  ┌──────────────────────────────────────────────────┐             │
│  │                    ALU                           │             │
│  │  ┌──────────────────────────────────────────┐   │             │
│  │  │  ADD, SUB, AND, OR, XOR, SLL, SRL, etc  │   │             │
│  │  └──────────────────────────────────────────┘   │             │
│  └──────────────────────────────────────────────────┘             │
│              EX/MEM Pipeline Register                               │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         MEMORY ACCESS                                │
│  ┌──────────────┐                                                  │
│  │    Data      │                                                  │
│  │   Memory     │                                                  │
│  └──────┬───────┘                                                  │
│         │                                                           │
│         ▼                                                           │
│  ┌──────────────┐                                                  │
│  │ mem_read_data│                                                  │
│  └──────┬───────┘                                                  │
│              MEM/WB Pipeline Register                               │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                           WRITEBACK                                  │
│  ┌──────────────┐      ┌──────────────┐                           │
│  │ ALU_result   │      │ mem_read_data│                           │
│  └──────┬───────┘      └──────┬───────┘                           │
│         │                     │                                     │
│         └──────────┬──────────┘                                    │
│                    ▼                                               │
│            ┌──────────────┐                                        │
│            │ MemToReg Mux │                                        │
│            └──────┬───────┘                                        │
│                   │                                                │
│                   ▼                                                │
│            ┌──────────────┐                                        │
│            │  Register    │                                        │
│            │    File      │                                        │
│            └──────────────┘                                        │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Path Components

1. **Instruction Path**:
   - PC → Instruction Memory → IF/ID → Control Unit
   - Instruction flows from memory through decode to control generation

2. **Register Path**:
   - Register File → ID/EX → Forwarding Muxes → ALU
   - Register data flows to ALU with forwarding support

3. **ALU Path**:
   - ALU → EX/MEM → MEM/WB → Register File
   - ALU results flow through memory stage to writeback

4. **Memory Path**:
   - Data Memory → MEM/WB → MemToReg Mux → Register File
   - Memory data flows to register file for load instructions

5. **Immediate Path**:
   - Immediate Generator → ID/EX → ALU (via ALUSrc mux)
   - Immediate values flow to ALU for I-type instructions

6. **PC Update Path**:
   - Branch Target → PC (on branch taken)
   - PC increments normally, updates on branches/jumps

---

## Control Signals

Control signals are generated in the ID stage and propagate through pipeline registers to control operations in downstream stages.

### Control Signal Generation

Control signals are generated by the Control Unit based on the instruction opcode and function fields.

**Inputs to Control Unit**:
- `opcode[6:0]`: Instruction opcode
- `funct3[2:0]`: Function field (for R/I/S/B types)
- `funct7[6:0]`: Function field (for R-type)

### Control Signal List

#### Main Control Signals

| Signal | Width | Stage | Description |
|--------|-------|-------|-------------|
| `RegWrite` | 1 | ID→WB | Enable register file write |
| `MemRead` | 1 | ID→MEM | Enable memory read |
| `MemWrite` | 1 | ID→MEM | Enable memory write |
| `MemToReg` | 1 | ID→WB | Select memory data vs ALU result |
| `ALUOp` | 2 | ID→EX | ALU operation type |
| `ALUSrc` | 1 | ID→EX | Select immediate vs register |
| `Branch` | 1 | ID→EX | Branch instruction detected |
| `Jump` | 1 | ID→EX | Jump instruction detected |

#### ALU Control Signals

The ALU Control Unit generates specific ALU operations based on `ALUOp` and function fields:

| Signal | Width | Description |
|--------|-------|-------------|
| `ALU_control` | 4 | ALU operation code |

**ALU Operations**:
- `0000`: AND
- `0001`: OR
- `0010`: ADD
- `0110`: SUB
- `0111`: SLT (Set Less Than)
- `1100`: XOR
- `1000`: SLL (Shift Left Logical)
- `1001`: SRL (Shift Right Logical)
- `1010`: SRA (Shift Right Arithmetic)

### Control Signal Flow

```
Instruction (IF/ID)
       │
       ▼
┌──────────────┐
│   Control    │
│    Unit      │
└──────┬───────┘
       │
       ├─── RegWrite ────┐
       ├─── MemRead ─────┤
       ├─── MemWrite ────┤
       ├─── MemToReg ────┤
       ├─── ALUOp ───────┤
       ├─── ALUSrc ───────┤
       ├─── Branch ───────┤
       └─── Jump ─────────┤
                          │
                          ▼
              ID/EX Pipeline Register
                          │
                          ▼
              EX/MEM Pipeline Register (selected signals)
                          │
                          ▼
              MEM/WB Pipeline Register (selected signals)
```

### Control Signal Timing

Control signals are generated in the ID stage and must propagate through pipeline registers:

- **ID Stage**: All control signals generated
- **EX Stage**: Uses `ALUOp`, `ALUSrc`, `Branch`, `Jump`
- **MEM Stage**: Uses `MemRead`, `MemWrite`
- **WB Stage**: Uses `RegWrite`, `MemToReg`

### Control Signal Truth Table

| Instruction Type | RegWrite | MemRead | MemWrite | MemToReg | ALUOp | ALUSrc | Branch | Jump |
|-----------------|----------|---------|----------|----------|-------|--------|--------|------|
| R-type (ADD) | 1 | 0 | 0 | 0 | 10 | 0 | 0 | 0 |
| I-type (ADDI) | 1 | 0 | 0 | 0 | 10 | 1 | 0 | 0 |
| Load (LW) | 1 | 1 | 0 | 1 | 00 | 1 | 0 | 0 |
| Store (SW) | 0 | 0 | 1 | X | 00 | 1 | 0 | 0 |
| Branch (BEQ) | 0 | 0 | 0 | X | 01 | 0 | 1 | 0 |
| Jump (JAL) | 1 | 0 | 0 | 0 | XX | X | 0 | 1 |
| LUI | 1 | 0 | 0 | 0 | 11 | 1 | 0 | 0 |

---

## Pipeline Hazards

Pipeline hazards occur when the pipeline cannot proceed without stalling or inserting bubbles. Three types of hazards are handled:

### 1. Data Hazards

**Definition**: Dependencies between instructions where one instruction needs data produced by a previous instruction.

**Types**:
- **RAW (Read After Write)**: Instruction reads a register before previous instruction writes to it
- **WAW (Write After Write)**: Two instructions write to the same register (not possible in single-issue pipeline)
- **WAR (Write After Read)**: Instruction writes to a register before previous instruction reads it (not possible in single-issue pipeline)

**Resolution**:
- **Forwarding (Bypassing)**: Forward data from later pipeline stages to earlier stages
- **Pipeline Stall**: Insert NOP (bubble) when forwarding cannot resolve hazard

#### Forwarding Unit

The Forwarding Unit detects data hazards and selects the correct data source.

**Forwarding Conditions**:

1. **EX Hazard** (EX→EX forwarding):
   - `ex_mem_RegWrite = 1` AND
   - `ex_mem_rd ≠ 0` AND
   - `ex_mem_rd = id_ex_rs1` → ForwardA = 01
   - `ex_mem_rd = id_ex_rs2` → ForwardB = 01

2. **MEM Hazard** (MEM→EX forwarding):
   - `mem_wb_RegWrite = 1` AND
   - `mem_wb_rd ≠ 0` AND
   - `mem_wb_rd = id_ex_rs1` AND NOT (EX hazard) → ForwardA = 10
   - `mem_wb_rd = id_ex_rs2` AND NOT (EX hazard) → ForwardB = 10

**Forwarding Mux Control**:
```
ForwardA:
  00: id_ex_rs1_data (normal)
  01: ex_mem_ALU_result (EX→EX)
  10: mem_wb_write_data (MEM→EX)

ForwardB:
  00: id_ex_rs2_data (normal)
  01: ex_mem_ALU_result (EX→EX)
  10: mem_wb_write_data (MEM→EX)
```

#### Load-Use Hazard

**Definition**: Load instruction followed by instruction that uses loaded data.

**Example**:
```assembly
LW  x1, 0(x2)    # Load data into x1
ADD x3, x1, x4   # Use x1 (hazard!)
```

**Resolution**: Pipeline stall for one cycle (insert bubble in ID/EX)

**Detection**:
- `id_ex_MemRead = 1` AND
- (`id_ex_rd = if_id_rs1` OR `id_ex_rd = if_id_rs2`)

**Stall Control**:
- `PCWrite = 0` (stall PC)
- `IF_flush = 0` (don't flush)
- `ID_stall = 1` (insert NOP in ID/EX)

### 2. Control Hazards

**Definition**: Branch/jump instructions change the PC, causing incorrect instructions to be fetched.

**Resolution**:
- **Branch Prediction**: Predict branch taken/not taken (simple: assume not taken)
- **Pipeline Flush**: Flush incorrect instructions when branch is taken
- **Branch Delay**: 1-cycle penalty for taken branches

**Branch Resolution**:
- Branch condition evaluated in EX stage
- If branch taken: flush IF/ID and ID/EX registers
- Fetch instruction from branch target

**Flush Control**:
- `IF_flush = 1`: Clear IF/ID register
- `ID_flush = 1`: Clear ID/EX register (insert NOP)

### 3. Structural Hazards

**Definition**: Resource conflicts when multiple instructions need the same hardware resource.

**Resolution**: Separate instruction and data memory (Harvard architecture)

**Design Choice**: This processor uses separate instruction and data memory, eliminating structural hazards.

---

## Timing Considerations

### Clock Period

The clock period is determined by the longest critical path through the pipeline:

**Typical Critical Paths**:
1. **IF Stage**: PC → Instruction Memory → IF/ID register
2. **ID Stage**: Register File read → ID/EX register
3. **EX Stage**: Forwarding mux → ALU → EX/MEM register (usually longest)
4. **MEM Stage**: Data Memory access → MEM/WB register
5. **WB Stage**: MemToReg mux → Register File write

**Maximum Frequency**: `f_max = 1 / t_critical`

### Setup and Hold Times

All pipeline registers must meet setup and hold time requirements:

- **Setup Time**: Data must be stable before clock edge
- **Hold Time**: Data must remain stable after clock edge
- **Clock-to-Q Delay**: Output available after clock edge

### Pipeline Latency

- **Latency**: 5 cycles (one instruction per stage)
- **Throughput**: 1 instruction per cycle (ideal, no hazards)

### Hazard Penalties

- **Data Hazard (with forwarding)**: 0 cycles penalty
- **Load-Use Hazard**: 1 cycle penalty (stall)
- **Control Hazard (branch taken)**: 1 cycle penalty (flush)

---

## Implementation Details

### Register File

- **Size**: 32 registers (x0 through x31)
- **Width**: 32 bits per register
- **Ports**: 2 read ports, 1 write port
- **Special**: x0 hardwired to 0 (reads return 0, writes ignored)

### Memory Organization

- **Instruction Memory**: Separate, read-only during execution
- **Data Memory**: Separate, read/write capable
- **Address Space**: 32-bit addresses
- **Alignment**: Word-aligned addresses for LW/SW

### ALU Implementation

- **Operations**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **Width**: 32 bits
- **Flags**: Zero flag, Less-than flag
- **Latency**: 1 cycle

### Immediate Generation

- **I-type**: Sign-extend `immediate[11:0]`
- **S-type**: Sign-extend `{immediate[11:5], immediate[4:0]}`
- **B-type**: Sign-extend `{immediate[12], immediate[10:5], immediate[4:1], 1'b0}`
- **U-type**: `{immediate[31:12], 12'b0}`
- **J-type**: Sign-extend `{immediate[20], immediate[10:1], immediate[11], immediate[19:12], 1'b0}`

### Branch Target Calculation

- **Branch**: `PC + sign_extend(immediate)`
- **JAL**: `PC + sign_extend(immediate)`
- **JALR**: `rs1 + sign_extend(immediate)` (LSB cleared)

### Reset and Initialization

- **Reset**: All pipeline registers cleared to zero
- **PC Reset**: PC initialized to reset vector (typically 0x00000000 or configurable)
- **Register File**: All registers initialized to zero (except x0 which is always 0)

---

## Conclusion

This 5-stage pipeline architecture provides a balanced trade-off between performance and complexity. The pipeline achieves high instruction throughput while maintaining a relatively simple design that is easy to understand, verify, and synthesize.

The architecture supports the complete RV32I instruction set with efficient hazard handling through forwarding and minimal pipeline stalls. The modular design allows for future enhancements such as branch prediction, cache systems, and extended instruction sets.

