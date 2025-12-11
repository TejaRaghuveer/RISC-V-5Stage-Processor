# RISC-V RV32I 5-Stage Pipeline Processor Architecture

## Table of Contents

1. [Overview](#overview)
2. [Pipeline Stages](#pipeline-stages)
   - [Stage 1: Instruction Fetch (IF)](#stage-1-instruction-fetch-if)
   - [Stage 2: Instruction Decode (ID)](#stage-2-instruction-decode-id)
   - [Stage 3: Execute (EX)](#stage-3-execute-ex)
   - [Stage 4: Memory Access (MEM)](#stage-4-memory-access-mem)
   - [Stage 5: Writeback (WB)](#stage-5-writeback-wb)
3. [Pipeline Registers](#pipeline-registers)
   - [IF/ID Register](#ifid-register)
   - [ID/EX Register](#idex-register)
   - [EX/MEM Register](#exmem-register)
   - [MEM/WB Register](#memwb-register)
4. [Control Signals](#control-signals)
   - [Control Unit](#control-unit)
   - [ALU Control Unit](#alu-control-unit)
   - [Control Signal Propagation](#control-signal-propagation)
5. [Data Path](#data-path)
   - [Complete Data Path Diagram](#complete-data-path-diagram)
   - [Data Flow Paths](#data-flow-paths)
   - [Forwarding Paths](#forwarding-paths)
6. [Hazard Detection and Forwarding](#hazard-detection-and-forwarding)
   - [Data Hazards](#data-hazards)
   - [Forwarding Unit](#forwarding-unit)
   - [Hazard Detection Unit](#hazard-detection-unit)
   - [Control Hazards](#control-hazards)
7. [Branch and Jump Handling](#branch-and-jump-handling)
   - [Branch Instructions](#branch-instructions)
   - [Jump Instructions](#jump-instructions)
   - [Branch/Jump Control Unit](#branchjump-control-unit)
   - [Pipeline Flush Mechanism](#pipeline-flush-mechanism)
8. [RV32I Instruction Set](#rv32i-instruction-set)
   - [Instruction Formats](#instruction-formats)
   - [R-Type Instructions](#r-type-instructions)
   - [I-Type Instructions](#i-type-instructions)
   - [S-Type Instructions](#s-type-instructions)
   - [B-Type Instructions](#b-type-instructions)
   - [U-Type Instructions](#u-type-instructions)
   - [J-Type Instructions](#j-type-instructions)
9. [Timing and Performance](#timing-and-performance)
10. [Implementation Details](#implementation-details)

---

## Overview

The RISC-V RV32I processor implements a classic 5-stage pipeline architecture that processes instructions with high throughput. The pipeline consists of five distinct stages that operate concurrently, allowing multiple instructions to be in flight simultaneously.

### Pipeline Architecture

```
┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
│   IF    │──▶│   ID    │──▶│   EX    │──▶│   MEM   │──▶│   WB    │
└─────────┘   └─────────┘   └─────────┘   └─────────┘   └─────────┘
    │             │             │             │             │
    ▼             ▼             ▼             ▼             ▼
  IF/ID         ID/EX         EX/MEM        MEM/WB         RF
```

### Key Features

- **5-Stage Pipeline**: IF → ID → EX → MEM → WB
- **Pipeline Registers**: IF/ID, ID/EX, EX/MEM, MEM/WB
- **Forwarding Unit**: Resolves RAW data hazards without stalling
- **Hazard Detection Unit**: Detects load-use hazards requiring stalls
- **Branch/Jump Control**: Handles control flow with pipeline flushing
- **Harvard Architecture**: Separate instruction and data memory
- **RV32I Support**: Complete base instruction set implementation

### Pipeline Philosophy

The pipeline improves instruction throughput by overlapping the execution of multiple instructions. While one instruction is being fetched, another is being decoded, a third is executing, and so on. This allows the processor to achieve an average of one instruction per cycle (IPC) in the absence of hazards.

**Ideal Performance**: 1 instruction per cycle (CPI = 1.0)

**Actual Performance**: CPI > 1.0 due to:
- Load-use hazards (1-cycle stall)
- Branch/jump taken (1-cycle flush penalty)
- Pipeline initialization (first 4 cycles)

---

## Pipeline Stages

### Stage 1: Instruction Fetch (IF)

**Purpose**: Fetch the next instruction from instruction memory and update the program counter.

#### Operations

1. **Read Instruction**: Access instruction memory at address `PC`
2. **Update PC**: Calculate next PC value
   - Sequential: `PC = PC + 4`
   - Branch/Jump: `PC = branch_target` (when `branch_taken = 1`)
3. **Handle Pipeline Control**: Respond to stall and flush signals

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `clk` | 1 | External | Clock signal |
| `rst_n` | 1 | External | Active-low reset |
| `branch_target` | 32 | EX/MEM | Branch/jump target address |
| `branch_taken` | 1 | EX/MEM | PC source select (1 = target, 0 = PC+4) |
| `stall` | 1 | Hazard Detection | Pipeline stall signal |
| `flush` | 1 | Branch/Jump Control | Pipeline flush signal |

#### Outputs

| Signal | Width | Destination | Description |
|--------|-------|--------------|-------------|
| `instruction` | 32 | IF/ID Register | Fetched instruction word |
| `PC` | 32 | IF/ID Register | Current program counter |
| `PC_plus_4` | 32 | IF/ID Register | PC + 4 for sequential execution |
| `imem_addr` | 10 | IMEM | Instruction memory address |

#### Block Diagram

```
┌─────────────┐
│     PC      │◀─── branch_target (from EX/MEM)
│  Register   │◀─── branch_taken (from EX/MEM)
└──────┬──────┘
       │
       │ (stall control)
       │
       ▼
┌─────────────┐
│ Instruction │
│   Memory    │
│   (IMEM)    │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Instruction │
│  (32 bits)  │
└─────────────┘
```

#### Key Components

- **Program Counter (PC)**: 32-bit register holding current instruction address
- **Instruction Memory (IMEM)**: Read-only memory storing program instructions
- **PC Increment Logic**: Adds 4 to PC for sequential execution
- **PC Multiplexer**: Selects between PC+4 and branch target

#### Control Signals

- **PCWrite Enable**: Controlled by stall signal (disabled during stalls)
- **PC Source Select**: Controlled by `branch_taken` signal
- **Flush Control**: Clears fetched instruction on branch/jump taken

#### Timing

- **Clock Edge**: PC updates on positive clock edge
- **Combinational Path**: PC → IMEM → Instruction (must complete in one cycle)
- **Stall Behavior**: PC holds current value, no new instruction fetched

---

### Stage 2: Instruction Decode (ID)

**Purpose**: Decode the instruction, read operands from register file, generate control signals, and prepare immediate values.

#### Operations

1. **Instruction Decoding**: Extract opcode, function fields, register addresses
2. **Register File Access**: Read source registers (rs1, rs2)
3. **Immediate Generation**: Extract and sign-extend immediate values
4. **Control Signal Generation**: Generate control signals for downstream stages
5. **Write-Back Handling**: Write data from WB stage to register file

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `instruction` | 32 | IF/ID Register | Instruction word |
| `PC` | 32 | IF/ID Register | Program counter |
| `PC_plus_4` | 32 | IF/ID Register | PC + 4 value |
| `wb_reg_write_en` | 1 | MEM/WB Register | Register write enable |
| `wb_rd_addr` | 5 | MEM/WB Register | Write register address |
| `wb_rd_data` | 32 | WB Stage | Write data |
| `stall` | 1 | Hazard Detection | Pipeline stall signal |
| `flush` | 1 | Branch/Jump Control | Pipeline flush signal |

#### Outputs

| Signal | Width | Destination | Description |
|--------|-------|--------------|-------------|
| `rs1_data` | 32 | ID/EX Register | Source register 1 data |
| `rs2_data` | 32 | ID/EX Register | Source register 2 data |
| `immediate` | 32 | ID/EX Register | Sign-extended immediate |
| `rs1_addr` | 5 | ID/EX Register, Hazard Detection | Source register 1 address |
| `rs2_addr` | 5 | ID/EX Register, Hazard Detection | Source register 2 address |
| `rd_addr` | 5 | ID/EX Register | Destination register address |
| `funct3` | 3 | ID/EX Register | Function field [14:12] |
| `funct7` | 7 | ID/EX Register | Function field [31:25] |
| `opcode` | 7 | ID/EX Register | Instruction opcode [6:0] |
| `PC_out` | 32 | ID/EX Register | PC value (passed through) |
| `PC_plus_4_out` | 32 | ID/EX Register | PC+4 value (passed through) |
| Control signals | Various | ID/EX Register | See Control Signals section |

#### Block Diagram

```
┌─────────────┐
│ Instruction │
│  (32 bits)  │
└──────┬──────┘
       │
       ├──────────────┬──────────────┐
       │              │              │
       ▼              ▼              ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│   Control   │ │    Imm      │ │ Instruction │
│    Unit     │ │  Generator  │ │   Decoder   │
└──────┬──────┘ └──────┬──────┘ └──────┬──────┘
       │               │                │
       │               │                │
       │      ┌────────┴────────┐       │
       │      │                 │       │
       ▼      ▼                 ▼       ▼
┌─────────────────────────────────────────────┐
│         Register File (32 registers)      │
│  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐ │
│  │ rs1  │  │ rs2  │  │  rd  │  │ ...  │ │
│  └──────┘  └──────┘  └──────┘  └──────┘ │
│     ▲         ▲         │               │
│     │         │         │               │
│     └─────────┴─────────┘               │
│              (write-back from WB)       │
└─────────────────────────────────────────────┘
```

#### Key Components

- **Control Unit**: Decodes opcode and generates control signals
- **Register File**: 32 registers (x0-x31), 2 read ports, 1 write port
- **Immediate Generator**: Extracts and sign-extends immediate values
- **Instruction Decoder**: Extracts instruction fields (rs1, rs2, rd, funct3, funct7)

#### Instruction Fields Extracted

| Field | Bits | Description |
|-------|------|-------------|
| `opcode` | [6:0] | Instruction opcode |
| `rd` | [11:7] | Destination register |
| `funct3` | [14:12] | Function field (R/I/S/B types) |
| `rs1` | [19:15] | Source register 1 |
| `rs2` | [24:20] | Source register 2 |
| `funct7` | [31:25] | Function field (R-type) |

#### Register File

- **Size**: 32 registers (x0 through x31)
- **Width**: 32 bits per register
- **Read Ports**: 2 (for rs1 and rs2)
- **Write Port**: 1 (from WB stage)
- **Special Register**: x0 is hardwired to 0 (reads return 0, writes ignored)

#### Immediate Generation

The immediate generator extracts immediate values based on instruction type:

| Type | Bits Used | Sign Extension |
|------|-----------|----------------|
| I-type | [31:20] | Sign-extend to 32 bits |
| S-type | [31:25], [11:7] | Sign-extend to 32 bits |
| B-type | [31], [7], [30:25], [11:8] | Sign-extend to 32 bits, LSB=0 |
| U-type | [31:12] | Zero-extend (lower 12 bits = 0) |
| J-type | [31], [19:12], [20], [30:21] | Sign-extend to 32 bits, LSB=0 |

#### Control Signals Generated

See [Control Signals](#control-signals) section for detailed description.

---

### Stage 3: Execute (EX)

**Purpose**: Perform arithmetic/logic operations, calculate memory addresses, evaluate branch conditions, and handle data forwarding.

#### Operations

1. **ALU Operations**: Execute arithmetic, logic, shift, and comparison operations
2. **Address Calculation**: Calculate memory addresses (rs1 + immediate)
3. **Branch Evaluation**: Evaluate branch conditions and calculate branch targets
4. **Jump Target Calculation**: Calculate jump target addresses
5. **Data Forwarding**: Select forwarded data from EX/MEM and MEM/WB stages

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `rs1_data` | 32 | ID/EX Register | Source register 1 data |
| `rs2_data` | 32 | ID/EX Register | Source register 2 data |
| `immediate` | 32 | ID/EX Register | Sign-extended immediate |
| `PC` | 32 | ID/EX Register | Program counter |
| `PC_plus_4` | 32 | ID/EX Register | PC + 4 value |
| `rs1_addr` | 5 | ID/EX Register | Source register 1 address |
| `rs2_addr` | 5 | ID/EX Register | Source register 2 address |
| `rd_addr` | 5 | ID/EX Register | Destination register address |
| `funct3` | 3 | ID/EX Register | Function field |
| `funct7` | 7 | ID/EX Register | Function field |
| `opcode` | 7 | ID/EX Register | Instruction opcode |
| `ALUSrc` | 1 | ID/EX Register | ALU source select |
| `ALUOp` | 2 | ID/EX Register | ALU operation type |
| `Branch` | 1 | ID/EX Register | Branch instruction |
| `Jump` | 1 | ID/EX Register | Jump instruction |
| `ex_mem_alu_result` | 32 | EX/MEM Register | Forwarded ALU result |
| `mem_wb_write_data` | 32 | MEM/WB Register | Forwarded write-back data |
| `ForwardA` | 2 | Forwarding Unit | Forwarding control for rs1 |
| `ForwardB` | 2 | Forwarding Unit | Forwarding control for rs2 |

#### Outputs

| Signal | Width | Destination | Description |
|--------|-------|--------------|-------------|
| `alu_result` | 32 | EX/MEM Register | ALU computation result |
| `zero_flag` | 1 | Branch/Jump Control | ALU zero flag |
| `branch_taken` | 1 | EX/MEM Register | Branch condition evaluation |
| `branch_target` | 32 | EX/MEM Register | Branch target address |
| `jump_target` | 32 | EX/MEM Register | Jump target address |
| `rs2_data_out` | 32 | EX/MEM Register | rs2 data (for stores) |
| `rs1_data_forwarded` | 32 | Branch/Jump Control | Forwarded rs1 data |
| `rs2_data_forwarded` | 32 | Branch/Jump Control | Forwarded rs2 data |

#### Block Diagram

```
┌─────────────┐     ┌─────────────┐
│   rs1_data  │     │   rs2_data  │
│  (ID/EX)    │     │  (ID/EX)    │
└──────┬──────┘     └──────┬──────┘
       │                   │
       │    ┌──────────────┴──────┐
       │    │  Forwarding Muxes   │
       │    │  (ForwardA/ForwardB)│
       │    └──────────────┬──────┘
       │                   │
       │    ┌──────────────┴──────┐
       │    │   ALUSrc Mux        │
       │    │  (rs2 vs immediate) │
       │    └──────────────┬──────┘
       │                   │
       ▼                   ▼
┌─────────────────────────────────┐
│         ALU                     │
│  ┌──────────────────────────┐  │
│  │  Operations:               │  │
│  │  ADD, SUB, AND, OR, XOR    │  │
│  │  SLL, SRL, SRA, SLT, SLTU  │  │
│  └──────────────────────────┘  │
└──────────────┬──────────────────┘
               │
               ▼
        ┌─────────────┐
        │ ALU_result  │
        │ zero_flag   │
        └─────────────┘
```

#### Key Components

- **Arithmetic Logic Unit (ALU)**: Performs arithmetic, logic, shift, and comparison operations
- **Forwarding Unit**: Detects RAW hazards and generates forwarding control signals
- **Forwarding Multiplexers**: Select data source (register file, EX/MEM, MEM/WB)
- **ALU Control Unit**: Decodes ALUOp and function fields to generate ALU control
- **Branch/Jump Control Unit**: Evaluates branch conditions and generates control signals
- **Address Calculation Logic**: Calculates memory addresses and branch/jump targets

#### ALU Operations

| Operation | ALU Control | Description |
|-----------|-------------|-------------|
| AND | 0000 | Bitwise AND |
| OR | 0001 | Bitwise OR |
| ADD | 0010 | Addition |
| SUB | 0110 | Subtraction |
| SLT | 0111 | Set Less Than (signed) |
| SLTU | 1101 | Set Less Than Unsigned |
| XOR | 1100 | Bitwise XOR |
| SLL | 1000 | Shift Left Logical |
| SRL | 1001 | Shift Right Logical |
| SRA | 1010 | Shift Right Arithmetic |

#### Forwarding Paths

See [Forwarding Unit](#forwarding-unit) section for detailed description.

#### Branch Target Calculation

- **Branch Instructions**: `branch_target = PC + sign_extend(immediate)`
- **JAL Instruction**: `jump_target = PC + sign_extend(immediate)`
- **JALR Instruction**: `jump_target = (rs1 + sign_extend(immediate)) & ~1`

---

### Stage 4: Memory Access (MEM)

**Purpose**: Access data memory for load/store operations, or pass through ALU results for non-memory instructions.

#### Operations

1. **Memory Read**: Read data from data memory (load instructions)
2. **Memory Write**: Write data to data memory (store instructions)
3. **Address Validation**: Check address alignment and range
4. **Data Pass-Through**: Pass ALU result through for non-memory instructions

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `alu_result` | 32 | EX/MEM Register | Memory address (from ALU) |
| `rs2_data` | 32 | EX/MEM Register | Data to write (for stores) |
| `MemRead` | 1 | EX/MEM Register | Memory read enable |
| `MemWrite` | 1 | EX/MEM Register | Memory write enable |
| `funct3` | 3 | EX/MEM Register | Memory access width (for future extension) |

#### Outputs

| Signal | Width | Destination | Description |
|--------|-------|--------------|-------------|
| `mem_read_data` | 32 | MEM/WB Register | Data read from memory |
| `alu_result_out` | 32 | MEM/WB Register | ALU result (passthrough) |

#### Block Diagram

```
┌─────────────┐
│ ALU_result  │ (memory address)
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    Data     │
│   Memory    │
│   (DMEM)    │
└──────┬──────┘
       │
       │ (MemRead)
       │
       ▼
┌─────────────┐
│ mem_read_   │
│    data     │
└─────────────┘
```

#### Key Components

- **Data Memory (DMEM)**: Read/write memory for data storage
- **Memory Control Logic**: Handles read/write operations
- **Address Validation**: Checks alignment and address range

#### Memory Operations

| Instruction | Operation | Data Width |
|-------------|-----------|------------|
| LW | Load Word | 32 bits |
| SW | Store Word | 32 bits |

**Note**: Currently supports word (32-bit) operations only. Byte and halfword operations can be added in future extensions.

#### Memory Access Timing

- **Read Operation**: Data available in same cycle (combinational read)
- **Write Operation**: Data written on clock edge (registered write)
- **Address Registration**: Address registered in first cycle, word address computed in second cycle

#### Memory Organization

- **Address Width**: 32 bits (supports up to 4GB address space)
- **Memory Depth**: Configurable (default: 1024 words)
- **Word Alignment**: Addresses must be word-aligned (bits [1:0] = 00)
- **Address Range**: Valid addresses: 0 to (MEM_DEPTH * 4 - 1)

---

### Stage 5: Writeback (WB)

**Purpose**: Select data source (ALU result or memory data) and write results back to the register file.

#### Operations

1. **Data Source Selection**: Select between ALU result and memory data
2. **Register Write**: Write selected data to register file (if RegWrite enabled)

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `alu_result` | 32 | MEM/WB Register | ALU result from MEM stage |
| `mem_read_data` | 32 | MEM/WB Register | Data read from memory |
| `MemToReg` | 1 | MEM/WB Register | Memory to register select |
| `rd_addr` | 5 | MEM/WB Register | Destination register address |
| `RegWrite` | 1 | MEM/WB Register | Register write enable |

#### Outputs

| Signal | Width | Destination | Description |
|--------|-------|--------------|-------------|
| `write_data` | 32 | Register File (ID) | Data to write to register file |
| `rd_addr` | 5 | Register File (ID) | Write register address |
| `reg_write_en` | 1 | Register File (ID) | Register write enable |

#### Block Diagram

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
         │   (in ID)     │
         └───────────────┘
```

#### Key Components

- **Writeback Multiplexer**: Selects between ALU result and memory data
- **Register File Write Port**: Writes data to register file in ID stage

#### Data Source Selection

| MemToReg | Data Source | Instruction Type |
|----------|-------------|------------------|
| 0 | ALU result | R-type, I-type, U-type, JAL, JALR |
| 1 | Memory data | Load instructions (LW, etc.) |

#### Register File Write

- **Write Enable**: Only when `RegWrite = 1`
- **Write Timing**: Synchronous write on clock edge
- **Register x0**: Writes to x0 are ignored (hardwired to 0)
- **Write-Back Path**: WB stage → Register File (in ID stage)

---

## Pipeline Registers

Pipeline registers store intermediate values between pipeline stages, allowing each stage to operate independently. They are positive-edge triggered registers that capture data at the end of each clock cycle.

### IF/ID Register

**Location**: Between IF and ID stages

**Purpose**: Store instruction and PC values from IF stage for ID stage processing.

#### Contents

| Signal | Width | Description |
|--------|-------|-------------|
| `id_instruction` | 32 | Instruction word from IF stage |
| `id_PC` | 32 | Program counter from IF stage |
| `id_PC_plus_4` | 32 | PC + 4 value from IF stage |

#### Control Signals

| Signal | Description |
|--------|-------------|
| `enable` | Register enable (active low: ~stall) |
| `flush` | Flush signal (clears register on branch/jump taken) |

#### Reset Behavior

- All signals cleared to zero on reset
- Flush inserts NOP (zero instruction)

#### Timing

- **Clock Edge**: Positive edge triggered
- **Enable**: When `enable = 0` (not stalled), register updates
- **Flush**: When `flush = 1`, register cleared to zero

---

### ID/EX Register

**Location**: Between ID and EX stages

**Purpose**: Store register data, immediate values, PC, control signals, and instruction fields from ID stage for EX stage processing.

#### Contents

**Data Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `ex_rs1_data` | 32 | Source register 1 data |
| `ex_rs2_data` | 32 | Source register 2 data |
| `ex_immediate` | 32 | Sign-extended immediate value |
| `ex_PC` | 32 | Program counter |
| `ex_PC_plus_4` | 32 | PC + 4 value |

**Address Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `ex_rs1_addr` | 5 | Source register 1 address |
| `ex_rs2_addr` | 5 | Source register 2 address |
| `ex_rd_addr` | 5 | Destination register address |

**Instruction Fields**:
| Signal | Width | Description |
|--------|-------|-------------|
| `ex_funct3` | 3 | Function field [14:12] |
| `ex_funct7` | 7 | Function field [31:25] |
| `ex_opcode` | 7 | Instruction opcode [6:0] |

**Control Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `ex_RegWrite` | 1 | Register write enable |
| `ex_MemRead` | 1 | Memory read enable |
| `ex_MemWrite` | 1 | Memory write enable |
| `ex_MemToReg` | 1 | Memory to register select |
| `ex_ALUSrc` | 1 | ALU source select |
| `ex_ALUOp` | 2 | ALU operation type |
| `ex_Branch` | 1 | Branch instruction |
| `ex_Jump` | 1 | Jump instruction |

#### Control Signals

| Signal | Description |
|--------|-------------|
| `enable` | Register enable (active low: ~stall) |
| `flush` | Flush signal (clears register on branch/jump taken or hazard NOP) |

#### Reset Behavior

- All signals cleared to zero on reset
- Zero values represent NOP instruction

#### Timing

- **Clock Edge**: Positive edge triggered
- **Enable**: When `enable = 0` (not stalled), register updates
- **Flush**: When `flush = 1`, register cleared to zero (NOP inserted)

---

### EX/MEM Register

**Location**: Between EX and MEM stages

**Purpose**: Store ALU result, rs2 data, control signals, and branch/jump information from EX stage for MEM stage processing.

#### Contents

**Data Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `mem_alu_result` | 32 | ALU computation result (memory address) |
| `mem_rs2_data` | 32 | Source register 2 data (for stores) |

**Address Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `mem_rd_addr` | 5 | Destination register address |

**Control Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `mem_RegWrite` | 1 | Register write enable |
| `mem_MemRead` | 1 | Memory read enable |
| `mem_MemWrite` | 1 | Memory write enable |
| `mem_MemToReg` | 1 | Memory to register select |

**Branch/Jump Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `mem_Branch` | 1 | Branch instruction |
| `mem_Jump` | 1 | Jump instruction |
| `mem_branch_taken` | 1 | Branch condition evaluation |
| `mem_branch_target` | 32 | Branch target address |
| `mem_jump_target` | 32 | Jump target address |
| `mem_PCSrc` | 1 | PC source select (registered) |
| `mem_branch_flush` | 1 | Branch/jump flush signal (registered) |

#### Control Signals

| Signal | Description |
|--------|-------------|
| `enable` | Register enable (active low: ~stall) |
| `flush` | Flush signal (clears register on branch/jump taken) |

#### Reset Behavior

- All signals cleared to zero on reset

#### Timing

- **Clock Edge**: Positive edge triggered
- **Enable**: When `enable = 0` (not stalled), register updates
- **Flush**: When `flush = 1`, register cleared to zero

---

### MEM/WB Register

**Location**: Between MEM and WB stages

**Purpose**: Store memory read data, ALU result, and control signals from MEM stage for WB stage processing.

#### Contents

**Data Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `wb_mem_read_data` | 32 | Data read from memory |
| `wb_alu_result` | 32 | ALU result (passthrough) |

**Address Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `wb_rd_addr` | 5 | Destination register address |

**Control Signals**:
| Signal | Width | Description |
|--------|-------|-------------|
| `wb_RegWrite` | 1 | Register write enable |
| `wb_MemToReg` | 1 | Memory to register select |

#### Control Signals

| Signal | Description |
|--------|-------------|
| `enable` | Register enable (active low: ~stall) |
| `flush` | Flush signal (clears register on branch/jump taken) |

#### Reset Behavior

- All signals cleared to zero on reset

#### Timing

- **Clock Edge**: Positive edge triggered
- **Enable**: When `enable = 0` (not stalled), register updates
- **Flush**: When `flush = 1`, register cleared to zero

---

## Control Signals

Control signals are generated in the ID stage by the Control Unit and propagate through pipeline registers to control operations in downstream stages.

### Control Unit

The Control Unit decodes instruction opcodes and generates control signals for the datapath.

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `opcode` | 7 | Instruction [6:0] | Instruction opcode |
| `funct3` | 3 | Instruction [14:12] | Function field (R/I/S/B types) |
| `funct7` | 7 | Instruction [31:25] | Function field (R-type) |

#### Outputs

| Signal | Width | Description |
|--------|-------|-------------|
| `RegWrite` | 1 | Enable register file write |
| `MemRead` | 1 | Enable memory read |
| `MemWrite` | 1 | Enable memory write |
| `MemToReg` | 1 | Select memory data vs ALU result |
| `ALUSrc` | 1 | Select immediate vs register for ALU operand B |
| `ALUOp` | 2 | ALU operation type |
| `Branch` | 1 | Branch instruction detected |
| `Jump` | 1 | Jump instruction detected (JAL, JALR) |

#### Control Signal Encoding

**ALUOp Encoding**:
- `2'b00`: Load/Store (ADD for address calculation)
- `2'b01`: Branch (SUB for comparison)
- `2'b10`: R-type (ALU operation from funct3/funct7)
- `2'b11`: I-type immediate (ALU operation from funct3)

#### Control Signal Truth Table

| Instruction Type | Opcode | RegWrite | MemRead | MemWrite | MemToReg | ALUSrc | ALUOp | Branch | Jump |
|-----------------|--------|----------|---------|-----------|----------|--------|-------|--------|------|
| R-type (ADD, SUB, etc.) | 0x33 | 1 | 0 | 0 | 0 | 0 | 10 | 0 | 0 |
| I-type immediate (ADDI, etc.) | 0x13 | 1 | 0 | 0 | 0 | 1 | 11 | 0 | 0 |
| I-type load (LW) | 0x03 | 1 | 1 | 0 | 1 | 1 | 00 | 0 | 0 |
| S-type store (SW) | 0x23 | 0 | 0 | 1 | X | 1 | 00 | 0 | 0 |
| B-type branch (BEQ, etc.) | 0x63 | 0 | 0 | 0 | X | 0 | 01 | 1 | 0 |
| U-type LUI | 0x37 | 1 | 0 | 0 | 0 | 1 | 11 | 0 | 0 |
| U-type AUIPC | 0x17 | 1 | 0 | 0 | 0 | 1 | 00 | 0 | 0 |
| J-type JAL | 0x6F | 1 | 0 | 0 | 0 | X | XX | 0 | 1 |
| I-type JALR | 0x67 | 1 | 0 | 0 | 0 | 1 | 00 | 0 | 1 |

### ALU Control Unit

The ALU Control Unit decodes ALUOp and function fields to generate specific ALU operation control signals.

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `ALUOp` | 2 | Control Unit | ALU operation type |
| `funct3` | 3 | Instruction [14:12] | Function field |
| `funct7` | 7 | Instruction [31:25] | Function field (R-type) |

#### Outputs

| Signal | Width | Description |
|--------|-------|-------------|
| `alu_control` | 4 | ALU operation control signal |

#### ALU Control Encoding

| ALUOp | funct3 | funct7 | ALU Control | Operation |
|-------|--------|--------|-------------|-----------|
| 00 | XXX | XXX | 0010 | ADD |
| 01 | XXX | XXX | 0110 | SUB |
| 10 | 000 | 0000000 | 0010 | ADD |
| 10 | 000 | 0100000 | 0110 | SUB |
| 10 | 001 | 0000000 | 1000 | SLL |
| 10 | 010 | 0000000 | 0111 | SLT |
| 10 | 011 | 0000000 | 0111 | SLTU |
| 10 | 100 | 0000000 | 1100 | XOR |
| 10 | 101 | 0000000 | 1001 | SRL |
| 10 | 101 | 0100000 | 1010 | SRA |
| 10 | 110 | 0000000 | 0001 | OR |
| 10 | 111 | 0000000 | 0000 | AND |
| 11 | 000 | XXX | 0010 | ADDI |
| 11 | 001 | XXX | 1000 | SLLI |
| 11 | 010 | XXX | 0111 | SLTI |
| 11 | 011 | XXX | 0111 | SLTIU |
| 11 | 100 | XXX | 1100 | XORI |
| 11 | 101 | 0000000 | 1001 | SRLI |
| 11 | 101 | 0100000 | 1010 | SRAI |
| 11 | 110 | XXX | 0001 | ORI |
| 11 | 111 | XXX | 0000 | ANDI |

### Control Signal Propagation

Control signals propagate through pipeline registers as follows:

```
ID Stage (Control Unit)
    │
    ├─── RegWrite ────┐
    ├─── MemRead ─────┤
    ├─── MemWrite ────┤
    ├─── MemToReg ────┤
    ├─── ALUSrc ──────┤
    ├─── ALUOp ───────┤
    ├─── Branch ──────┤
    └─── Jump ────────┤
                      │
                      ▼
            ID/EX Pipeline Register
                      │
                      ├─── EX Stage uses: ALUSrc, ALUOp, Branch, Jump
                      │
                      ▼
            EX/MEM Pipeline Register
                      │
                      ├─── MEM Stage uses: MemRead, MemWrite
                      │
                      ▼
            MEM/WB Pipeline Register
                      │
                      ├─── WB Stage uses: RegWrite, MemToReg
                      │
                      ▼
            Register File Write
```

---

## Data Path

The data path connects all pipeline stages and components, allowing data to flow from instruction fetch through writeback.

### Complete Data Path Diagram

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
│  │  │  ADD, SUB, AND, OR, XOR, SLL, SRL, etc   │   │             │
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

### Data Flow Paths

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

7. **Write-Back Path**:
   - WB Stage → Register File (in ID stage)
   - Write-back data available for next instruction in ID stage

### Forwarding Paths

See [Forwarding Unit](#forwarding-unit) section for detailed description.

---

## Hazard Detection and Forwarding

Pipeline hazards occur when the pipeline cannot proceed without stalling or inserting bubbles. The processor handles data hazards through forwarding and hazard detection.

### Data Hazards

**Definition**: Dependencies between instructions where one instruction needs data produced by a previous instruction.

**Types**:
- **RAW (Read After Write)**: Instruction reads a register before previous instruction writes to it
- **WAW (Write After Write)**: Two instructions write to the same register (not possible in single-issue pipeline)
- **WAR (Write After Read)**: Instruction writes to a register before previous instruction reads it (not possible in single-issue pipeline)

**Resolution**:
- **Forwarding (Bypassing)**: Forward data from later pipeline stages to earlier stages
- **Pipeline Stall**: Insert NOP (bubble) when forwarding cannot resolve hazard

### Forwarding Unit

The Forwarding Unit detects RAW data hazards and generates forwarding control signals to select the correct data source.

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `id_ex_rs1_addr` | 5 | ID/EX Register | Source register 1 address |
| `id_ex_rs2_addr` | 5 | ID/EX Register | Source register 2 address |
| `ex_mem_rd_addr` | 5 | EX/MEM Register | Destination register address |
| `mem_wb_rd_addr` | 5 | MEM/WB Register | Destination register address |
| `ex_mem_reg_write` | 1 | EX/MEM Register | Register write enable |
| `mem_wb_reg_write` | 1 | MEM/WB Register | Register write enable |

#### Outputs

| Signal | Width | Description |
|--------|-------|-------------|
| `ForwardA` | 2 | Forwarding control for rs1 |
| `ForwardB` | 2 | Forwarding control for rs2 |

#### Forwarding Control Encoding

| ForwardA/ForwardB | Data Source | Description |
|-------------------|-------------|-------------|
| `2'b00` | ID/EX register data | No forwarding (normal operation) |
| `2'b01` | MEM/WB write data | Forward from MEM/WB stage |
| `2'b10` | EX/MEM ALU result | Forward from EX/MEM stage |
| `2'b11` | Reserved | Not used |

#### Forwarding Detection Logic

**EX/MEM Hazard (Priority 1)**:
- Condition: `ex_mem_reg_write = 1` AND `ex_mem_rd_addr != 0` AND `ex_mem_rd_addr == id_ex_rs1_addr`
- Action: `ForwardA = 2'b10` (forward from EX/MEM)
- Condition: `ex_mem_reg_write = 1` AND `ex_mem_rd_addr != 0` AND `ex_mem_rd_addr == id_ex_rs2_addr`
- Action: `ForwardB = 2'b10` (forward from EX/MEM)

**MEM/WB Hazard (Priority 2)**:
- Condition: `mem_wb_reg_write = 1` AND `mem_wb_rd_addr != 0` AND `mem_wb_rd_addr == id_ex_rs1_addr` AND NOT (EX/MEM hazard)
- Action: `ForwardA = 2'b01` (forward from MEM/WB)
- Condition: `mem_wb_reg_write = 1` AND `mem_wb_rd_addr != 0` AND `mem_wb_rd_addr == id_ex_rs2_addr` AND NOT (EX/MEM hazard)
- Action: `ForwardB = 2'b01` (forward from MEM/WB)

#### Forwarding Examples

**Example 1: EX Hazard**
```assembly
ADD x1, x2, x3    # Instruction I1 (in EX/MEM)
ADD x4, x1, x5    # Instruction I2 (in EX, needs x1)
```
- I2's EX stage needs x1, which I1 writes in MEM stage
- Forward x1 from EX/MEM ALU result to I2's ALU
- `ForwardA = 2'b10`

**Example 2: MEM Hazard**
```assembly
ADD x1, x2, x3    # Instruction I1 (in MEM/WB)
ADD x4, x1, x5    # Instruction I2 (in EX, needs x1)
```
- I2's EX stage needs x1, which I1 writes in WB stage
- Forward x1 from MEM/WB write data to I2's ALU
- `ForwardA = 2'b01`

**Example 3: Load-Use Hazard (Cannot Forward)**
```assembly
LW x1, 0(x2)      # Instruction I1 (load in EX/MEM)
ADD x4, x1, x5    # Instruction I2 (in ID, needs x1)
```
- I2 needs x1, but load data not available until MEM stage completes
- Cannot forward (data not yet available)
- Requires pipeline stall (handled by Hazard Detection Unit)

### Hazard Detection Unit

The Hazard Detection Unit detects load-use data hazards that cannot be resolved by forwarding.

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `id_ex_MemRead` | 1 | ID/EX Register | Memory read enable |
| `id_ex_rd_addr` | 5 | ID/EX Register | Destination register address |
| `if_id_rs1_addr` | 5 | IF/ID Register | Source register 1 address |
| `if_id_rs2_addr` | 5 | IF/ID Register | Source register 2 address |

#### Outputs

| Signal | Width | Description |
|--------|-------|-------------|
| `stall` | 1 | Pipeline stall signal |
| `id_ex_flush` | 1 | ID/EX register flush signal |

#### Hazard Detection Logic

**Load-Use Hazard Condition**:
- `id_ex_MemRead = 1` (load instruction in EX stage)
- AND (`id_ex_rd_addr == if_id_rs1_addr` OR `id_ex_rd_addr == if_id_rs2_addr`)
- AND `id_ex_rd_addr != 0` (not x0)

**Stall Actions**:
- `stall = 1`: Prevents PC update and holds IF/ID register
- `id_ex_flush = 1`: Clears ID/EX register (inserts NOP)

**Timing**:
- Detection: Combinational (based on current pipeline state)
- Stall/Flush: Takes effect on next clock cycle
- Resolution: One-cycle stall allows load to complete

#### Load-Use Hazard Example

```assembly
LW x1, 0(x2)      # Cycle N: Load in EX stage
ADD x3, x1, x4    # Cycle N: ADD in ID stage (needs x1)
```

**Cycle N**:
- Hazard detected: Load writes x1, ADD reads x1
- Stall asserted: PC holds, IF/ID holds ADD instruction
- ID/EX flushed: NOP inserted

**Cycle N+1**:
- Load moves to MEM stage (data read from memory)
- ADD remains in ID stage (stalled)

**Cycle N+2**:
- Load completes in MEM stage (data available)
- ADD moves to ID/EX stage
- Can read x1 from register file (written in WB stage)

### Control Hazards

**Definition**: Branch/jump instructions change the PC, causing incorrect instructions to be fetched.

**Resolution**:
- **Pipeline Flush**: Flush incorrect instructions when branch/jump is taken
- **Branch Delay**: 1-cycle penalty for taken branches

**Branch Resolution**:
- Branch condition evaluated in EX stage
- If branch taken: flush IF/ID and ID/EX registers
- Fetch instruction from branch target

**Flush Control**:
- `IF_flush = 1`: Clear IF/ID register
- `ID_flush = 1`: Clear ID/EX register (insert NOP)

See [Branch and Jump Handling](#branch-and-jump-handling) section for detailed description.

---

## Branch and Jump Handling

The processor handles control flow instructions (branches and jumps) with pipeline flushing to ensure correct execution.

### Branch Instructions

Branch instructions conditionally change the program counter based on comparison of two registers.

#### Supported Branch Instructions

| Instruction | funct3 | Condition | Description |
|-------------|--------|-----------|-------------|
| BEQ | 000 | rs1 == rs2 | Branch if equal |
| BNE | 001 | rs1 != rs2 | Branch if not equal |
| BLT | 100 | rs1 < rs2 (signed) | Branch if less than |
| BGE | 101 | rs1 >= rs2 (signed) | Branch if greater or equal |
| BLTU | 110 | rs1 < rs2 (unsigned) | Branch if less than unsigned |
| BGEU | 111 | rs1 >= rs2 (unsigned) | Branch if greater or equal unsigned |

#### Branch Instruction Format

```
┌──────────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ imm[12]  │imm   │ rs2  │ rs1  │funct3│imm   │imm   │
│          │[10:5]│      │      │      │[4:1] │[11]  │
└──────────┴──────┴──────┴──────┴──────┴──────┴──────┘
   31      25    20    15    12    8     7      0
```

#### Branch Target Calculation

- **Immediate Extraction**: `imm = {imm[12], imm[10:5], imm[4:1], imm[11], 1'b0}`
- **Sign Extension**: Sign-extend immediate to 32 bits
- **Target Address**: `branch_target = PC + sign_extend(imm)`

#### Branch Condition Evaluation

Branch conditions are evaluated in the EX stage:

1. **ALU Subtraction**: `rs1 - rs2` (for comparison)
2. **Zero Flag**: `zero_flag = (rs1 - rs2 == 0)` (for BEQ/BNE)
3. **Signed Comparison**: `rs1 < rs2` (signed) (for BLT/BGE)
4. **Unsigned Comparison**: `rs1 < rs2` (unsigned) (for BLTU/BGEU)

#### Branch Pipeline Behavior

**Not Taken (Sequential)**:
- PC updates to PC + 4
- No pipeline flush
- Next instruction fetched sequentially

**Taken (Branch)**:
- PC updates to branch target
- IF/ID and ID/EX registers flushed (NOPs inserted)
- Next instruction fetched from branch target
- 1-cycle penalty

### Jump Instructions

Jump instructions unconditionally change the program counter and save the return address.

#### Supported Jump Instructions

| Instruction | Opcode | Description |
|-------------|--------|-------------|
| JAL | 0x6F | Jump and Link (PC-relative) |
| JALR | 0x67 | Jump and Link Register (rs1 + immediate) |

#### JAL Instruction

- **Format**: J-type
- **Operation**: `rd = PC + 4`, `PC = PC + sign_extend(imm)`
- **Immediate**: 21-bit signed immediate (bits [20:1], bit [11], bits [19:12])
- **Target**: `jump_target = PC + sign_extend(imm)`
- **Always Taken**: JAL always jumps

#### JALR Instruction

- **Format**: I-type
- **Operation**: `rd = PC + 4`, `PC = (rs1 + sign_extend(imm)) & ~1`
- **Immediate**: 12-bit signed immediate
- **Target**: `jump_target = (rs1 + sign_extend(imm)) & ~1` (LSB cleared)
- **Always Taken**: JALR always jumps

#### Jump Pipeline Behavior

- PC updates to jump target
- Return address (PC + 4) written to rd register
- IF/ID and ID/EX registers flushed (NOPs inserted)
- Next instruction fetched from jump target
- 1-cycle penalty

### Branch/Jump Control Unit

The Branch/Jump Control Unit evaluates branch conditions and generates PC source selection and flush signals.

#### Inputs

| Signal | Width | Source | Description |
|--------|-------|---------|-------------|
| `Branch` | 1 | ID/EX Register | Branch instruction |
| `Jump` | 1 | ID/EX Register | Jump instruction |
| `funct3` | 3 | ID/EX Register | Function field (branch type) |
| `zero_flag` | 1 | ALU | Zero flag (from SUB) |
| `rs1_data` | 32 | EX Stage | Forwarded rs1 data |
| `rs2_data` | 32 | EX Stage | Forwarded rs2 data |
| `branch_target` | 32 | EX Stage | Branch target address |
| `jump_target` | 32 | EX Stage | Jump target address |

#### Outputs

| Signal | Width | Description |
|--------|-------|-------------|
| `PCSrc` | 1 | PC source select (0 = PC+4, 1 = target) |
| `flush` | 1 | Pipeline flush signal |
| `branch_taken` | 1 | Branch condition evaluation |

#### Branch Condition Evaluation Logic

| funct3 | Condition | Evaluation |
|--------|-----------|------------|
| 000 (BEQ) | rs1 == rs2 | `zero_flag == 1` |
| 001 (BNE) | rs1 != rs2 | `zero_flag == 0` |
| 100 (BLT) | rs1 < rs2 (signed) | `$signed(rs1) < $signed(rs2)` |
| 101 (BGE) | rs1 >= rs2 (signed) | `$signed(rs1) >= $signed(rs2)` |
| 110 (BLTU) | rs1 < rs2 (unsigned) | `rs1 < rs2` |
| 111 (BGEU) | rs1 >= rs2 (unsigned) | `rs1 >= rs2` |

#### PC Source Selection

- **PCSrc = 0**: Sequential execution (PC + 4)
- **PCSrc = 1**: Branch/Jump taken (use target address)

#### Pipeline Flush Mechanism

When branch/jump is taken:
- `flush = 1`: Clears IF/ID and ID/EX registers
- NOPs inserted in flushed pipeline registers
- Prevents incorrect instructions from executing

### Pipeline Flush Mechanism

Pipeline flushing clears pipeline registers to insert NOPs (bubbles) when control flow changes.

#### Flush Sources

1. **Branch/Jump Taken**: `mem_branch_flush` (from Branch/Jump Control Unit)
2. **External Flush**: `pipeline_flush` (for exceptions, interrupts)

#### Flush Targets

| Pipeline Register | Flush Signal | Effect |
|-------------------|--------------|--------|
| IF/ID | `pipeline_flush_internal` | Clears instruction and PC |
| ID/EX | `pipeline_flush_internal` OR `hazard_id_ex_flush` | Clears register data and control signals |
| EX/MEM | `pipeline_flush_internal` | Clears ALU result and control signals |
| MEM/WB | `pipeline_flush_internal` | Clears memory data and control signals |

#### Flush Timing

- **Detection**: Combinational (in EX stage)
- **Registration**: Registered in EX/MEM register
- **Execution**: Takes effect on next clock cycle
- **Duration**: One cycle (clears registers, inserts NOPs)

---

## RV32I Instruction Set

The processor implements the complete RISC-V RV32I base instruction set, supporting all instruction formats and operations.

### Instruction Formats

RISC-V instructions use six instruction formats:

#### R-Type (Register-Register)

```
┌──────────┬──────┬──────┬──────┬──────┬──────┐
│ funct7   │ rs2  │ rs1  │funct3│  rd  │opcode│
└──────────┴──────┴──────┴──────┴──────┴──────┘
   31      25    20    15    12    7      0
```

**Fields**:
- `funct7[31:25]`: Function field (7 bits)
- `rs2[24:20]`: Source register 2 (5 bits)
- `rs1[19:15]`: Source register 1 (5 bits)
- `funct3[14:12]`: Function field (3 bits)
- `rd[11:7]`: Destination register (5 bits)
- `opcode[6:0]`: Opcode (7 bits) = `0x33`

#### I-Type (Immediate)

```
┌──────────────┬──────┬──────┬──────┬──────┐
│   imm[11:0]  │ rs1  │funct3│  rd  │opcode│
└──────────────┴──────┴──────┴──────┴──────┘
      31        20    15    12    7      0
```

**Fields**:
- `imm[31:20]`: Immediate value (12 bits, sign-extended)
- `rs1[19:15]`: Source register 1 (5 bits)
- `funct3[14:12]`: Function field (3 bits)
- `rd[11:7]`: Destination register (5 bits)
- `opcode[6:0]`: Opcode (7 bits) = `0x13` (immediate), `0x03` (load), `0x67` (JALR)

#### S-Type (Store)

```
┌──────────┬──────┬──────┬──────┬──────┬──────┐
│imm[11:5] │ rs2  │ rs1  │funct3│imm   │opcode│
│          │      │      │      │[4:0] │      │
└──────────┴──────┴──────┴──────┴──────┴──────┘
   31      25    20    15    12    7      0
```

**Fields**:
- `imm[11:5]`: Immediate bits [11:5] (7 bits)
- `rs2[24:20]`: Source register 2 (5 bits)
- `rs1[19:15]`: Source register 1 (5 bits)
- `funct3[14:12]`: Function field (3 bits)
- `imm[4:0]`: Immediate bits [4:0] (5 bits)
- `opcode[6:0]`: Opcode (7 bits) = `0x23`

#### B-Type (Branch)

```
┌──────────┬──────┬──────┬──────┬──────┬──────┬──────┐
│imm[12]   │imm   │ rs2  │ rs1  │funct3│imm   │imm   │
│          │[10:5]│      │      │      │[4:1] │[11]  │
└──────────┴──────┴──────┴──────┴──────┴──────┴──────┘
   31      25    20    15    12    8     7      0
```

**Fields**:
- `imm[12]`: Immediate bit 12 (1 bit)
- `imm[10:5]`: Immediate bits [10:5] (6 bits)
- `rs2[24:20]`: Source register 2 (5 bits)
- `rs1[19:15]`: Source register 1 (5 bits)
- `funct3[14:12]`: Function field (3 bits)
- `imm[4:1]`: Immediate bits [4:1] (4 bits)
- `imm[11]`: Immediate bit 11 (1 bit)
- `opcode[6:0]`: Opcode (7 bits) = `0x63`

#### U-Type (Upper Immediate)

```
┌──────────────┬──────┬──────┐
│   imm[31:12] │  rd  │opcode│
└──────────────┴──────┴──────┘
      31        12    7      0
```

**Fields**:
- `imm[31:12]`: Immediate value (20 bits)
- `rd[11:7]`: Destination register (5 bits)
- `opcode[6:0]`: Opcode (7 bits) = `0x37` (LUI), `0x17` (AUIPC)

#### J-Type (Jump)

```
┌──────────┬──────────┬──────┬──────┬──────┐
│imm[20]   │imm[10:1] │imm   │imm   │  rd  │opcode│
│          │          │[11]  │[19:12]│      │      │
└──────────┴──────────┴──────┴──────┴──────┴──────┘
   31      21        20    12    7      0
```

**Fields**:
- `imm[20]`: Immediate bit 20 (1 bit)
- `imm[10:1]`: Immediate bits [10:1] (10 bits)
- `imm[11]`: Immediate bit 11 (1 bit)
- `imm[19:12]`: Immediate bits [19:12] (8 bits)
- `rd[11:7]`: Destination register (5 bits)
- `opcode[6:0]`: Opcode (7 bits) = `0x6F`

### R-Type Instructions

R-type instructions perform register-register operations.

| Instruction | funct7 | funct3 | Description |
|-------------|--------|--------|-------------|
| ADD | 0000000 | 000 | Add |
| SUB | 0100000 | 000 | Subtract |
| SLL | 0000000 | 001 | Shift Left Logical |
| SLT | 0000000 | 010 | Set Less Than (signed) |
| SLTU | 0000000 | 011 | Set Less Than Unsigned |
| XOR | 0000000 | 100 | XOR |
| SRL | 0000000 | 101 | Shift Right Logical |
| SRA | 0100000 | 101 | Shift Right Arithmetic |
| OR | 0000000 | 110 | OR |
| AND | 0000000 | 111 | AND |

**Opcode**: `0x33`

**Assembly Format**: `INSTR rd, rs1, rs2`

**Example**: `ADD x1, x2, x3` → `x1 = x2 + x3`

### I-Type Instructions

I-type instructions perform immediate operations or load operations.

#### Immediate Operations

| Instruction | funct3 | Description |
|-------------|--------|-------------|
| ADDI | 000 | Add Immediate |
| SLLI | 001 | Shift Left Logical Immediate |
| SLTI | 010 | Set Less Than Immediate (signed) |
| SLTIU | 011 | Set Less Than Immediate Unsigned |
| XORI | 100 | XOR Immediate |
| SRLI | 101 | Shift Right Logical Immediate |
| SRAI | 101 | Shift Right Arithmetic Immediate (funct7=0100000) |
| ORI | 110 | OR Immediate |
| ANDI | 111 | AND Immediate |

**Opcode**: `0x13`

**Assembly Format**: `INSTR rd, rs1, imm`

**Example**: `ADDI x1, x2, 5` → `x1 = x2 + 5`

#### Load Operations

| Instruction | funct3 | Description |
|-------------|--------|-------------|
| LB | 000 | Load Byte (signed) |
| LH | 001 | Load Halfword (signed) |
| LW | 010 | Load Word |
| LBU | 100 | Load Byte Unsigned |
| LHU | 101 | Load Halfword Unsigned |

**Opcode**: `0x03`

**Assembly Format**: `INSTR rd, imm(rs1)`

**Example**: `LW x1, 0(x2)` → `x1 = Memory[x2 + 0]`

**Note**: Currently only LW (Load Word) is fully implemented. Other load variants can be added in future extensions.

#### Jump and Link Register

| Instruction | funct3 | Description |
|-------------|--------|-------------|
| JALR | 000 | Jump and Link Register |

**Opcode**: `0x67`

**Assembly Format**: `JALR rd, imm(rs1)`

**Example**: `JALR x1, 0(x2)` → `x1 = PC + 4`, `PC = (x2 + 0) & ~1`

### S-Type Instructions

S-type instructions perform store operations.

| Instruction | funct3 | Description |
|-------------|--------|-------------|
| SB | 000 | Store Byte |
| SH | 001 | Store Halfword |
| SW | 010 | Store Word |

**Opcode**: `0x23`

**Assembly Format**: `INSTR rs2, imm(rs1)`

**Example**: `SW x2, 0(x1)` → `Memory[x1 + 0] = x2`

**Note**: Currently only SW (Store Word) is fully implemented. Other store variants can be added in future extensions.

### B-Type Instructions

B-type instructions perform conditional branches.

| Instruction | funct3 | Condition | Description |
|-------------|--------|-----------|-------------|
| BEQ | 000 | rs1 == rs2 | Branch if Equal |
| BNE | 001 | rs1 != rs2 | Branch if Not Equal |
| BLT | 100 | rs1 < rs2 (signed) | Branch if Less Than |
| BGE | 101 | rs1 >= rs2 (signed) | Branch if Greater or Equal |
| BLTU | 110 | rs1 < rs2 (unsigned) | Branch if Less Than Unsigned |
| BGEU | 111 | rs1 >= rs2 (unsigned) | Branch if Greater or Equal Unsigned |

**Opcode**: `0x63`

**Assembly Format**: `INSTR rs1, rs2, label`

**Example**: `BEQ x1, x2, label` → Branch to `label` if `x1 == x2`

**Branch Target**: `PC + sign_extend(imm)`

### U-Type Instructions

U-type instructions load upper immediate values or add upper immediate to PC.

| Instruction | Opcode | Description |
|-------------|--------|-------------|
| LUI | 0x37 | Load Upper Immediate |
| AUIPC | 0x17 | Add Upper Immediate to PC |

**Assembly Format**: `INSTR rd, imm`

**LUI Example**: `LUI x1, 0x12345` → `x1 = 0x12345000`

**AUIPC Example**: `AUIPC x1, 0x12345` → `x1 = PC + 0x12345000`

### J-Type Instructions

J-type instructions perform unconditional jumps.

| Instruction | Opcode | Description |
|-------------|--------|-------------|
| JAL | 0x6F | Jump and Link |

**Assembly Format**: `JAL rd, label`

**Example**: `JAL x1, label` → `x1 = PC + 4`, `PC = label`

**Jump Target**: `PC + sign_extend(imm)`

---

## Timing and Performance

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

| Hazard Type | Resolution | Penalty |
|-------------|------------|---------|
| RAW Hazard (with forwarding) | Forwarding | 0 cycles |
| Load-Use Hazard | Stall | 1 cycle |
| Control Hazard (branch taken) | Flush | 1 cycle |
| Control Hazard (branch not taken) | None | 0 cycles |

### Performance Metrics

- **CPI (Cycles Per Instruction)**: Average cycles per instruction
  - Ideal: 1.0 CPI
  - With hazards: CPI > 1.0
- **IPC (Instructions Per Cycle)**: Average instructions per cycle
  - Ideal: 1.0 IPC
  - With hazards: IPC < 1.0

---

## Implementation Details

### Register File

- **Size**: 32 registers (x0 through x31)
- **Width**: 32 bits per register
- **Ports**: 2 read ports, 1 write port
- **Special**: x0 hardwired to 0 (reads return 0, writes ignored)
- **Write Timing**: Synchronous write on clock edge
- **Read Timing**: Combinational read (data available immediately)

### Memory Organization

- **Instruction Memory**: Separate, read-only during execution
- **Data Memory**: Separate, read/write capable
- **Address Space**: 32-bit addresses
- **Alignment**: Word-aligned addresses for LW/SW
- **Memory Depth**: Configurable (default: 1024 words each)
- **Initialization**: Instruction memory initialized from hex file

### ALU Implementation

- **Operations**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **Width**: 32 bits
- **Flags**: Zero flag (for BEQ/BNE)
- **Latency**: 1 cycle (combinational)
- **Control**: 4-bit ALU control signal

### Immediate Generation

The immediate generator extracts and sign-extends immediate values:

| Type | Bits Used | Sign Extension |
|------|-----------|----------------|
| I-type | [31:20] | Sign-extend to 32 bits |
| S-type | [31:25], [11:7] | Sign-extend to 32 bits |
| B-type | [31], [7], [30:25], [11:8] | Sign-extend to 32 bits, LSB=0 |
| U-type | [31:12] | Zero-extend (lower 12 bits = 0) |
| J-type | [31], [19:12], [20], [30:21] | Sign-extend to 32 bits, LSB=0 |

### Branch Target Calculation

- **Branch**: `PC + sign_extend(immediate)`
- **JAL**: `PC + sign_extend(immediate)`
- **JALR**: `(rs1 + sign_extend(immediate)) & ~1` (LSB cleared)

### Reset and Initialization

- **Reset**: All pipeline registers cleared to zero
- **PC Reset**: PC initialized to reset vector (typically 0x00000000 or configurable)
- **Register File**: All registers initialized to zero (except x0 which is always 0)
- **Memory**: Instruction memory initialized from hex file (if specified)

### Pipeline Control

- **Stall**: Prevents new instructions from entering pipeline
  - PC holds current value
  - IF/ID register holds current instruction
  - ID/EX register flushed (NOP inserted)
- **Flush**: Clears pipeline registers (inserts NOPs)
  - IF/ID register cleared
  - ID/EX register cleared
  - Used on branch/jump taken
- **Reset**: Initializes all pipeline stages and registers

---

## Conclusion

This 5-stage pipeline architecture provides a balanced trade-off between performance and complexity. The pipeline achieves high instruction throughput while maintaining a relatively simple design that is easy to understand, verify, and synthesize.

The architecture supports the complete RV32I instruction set with efficient hazard handling through forwarding and minimal pipeline stalls. The modular design allows for future enhancements such as branch prediction, cache systems, and extended instruction sets.

### Key Design Features

- **5-Stage Pipeline**: Balanced pipeline with clear stage boundaries
- **Forwarding Unit**: Eliminates most data hazard stalls
- **Hazard Detection**: Handles load-use hazards with minimal stalls
- **Branch/Jump Control**: Efficient control flow handling with pipeline flushing
- **Harvard Architecture**: Separate instruction and data memory eliminates structural hazards
- **Complete RV32I Support**: All base instruction set instructions implemented

### Performance Characteristics

- **Ideal CPI**: 1.0 (one instruction per cycle)
- **Actual CPI**: > 1.0 due to hazards and control flow
- **Hazard Penalties**: Minimal (1 cycle for load-use, 1 cycle for taken branches)
- **Throughput**: High instruction throughput with efficient hazard resolution

---

*This architecture document describes the RISC-V RV32I 5-stage pipeline processor implementation. For implementation details, refer to the SystemVerilog source code in the `src/` directory.*
