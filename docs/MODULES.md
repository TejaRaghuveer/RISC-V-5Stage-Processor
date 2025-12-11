# RISC-V Processor SystemVerilog Modules Documentation

## Table of Contents

1. [Overview](#overview)
2. [Top-Level Module](#top-level-module)
3. [Pipeline Stage Modules](#pipeline-stage-modules)
   - [IF Stage](#if-stage)
   - [ID Stage](#id-stage)
   - [EX Stage](#ex-stage)
   - [MEM Stage](#mem-stage)
   - [WB Stage](#wb-stage)
4. [Pipeline Register Modules](#pipeline-register-modules)
   - [IF/ID Register](#ifid-register)
   - [ID/EX Register](#idex-register)
   - [EX/MEM Register](#exmem-register)
   - [MEM/WB Register](#memwb-register)
5. [Control Modules](#control-modules)
   - [Control Unit](#control-unit)
   - [Branch/Jump Control Unit](#branchjump-control-unit)
   - [Forwarding Unit](#forwarding-unit)
   - [Hazard Detection Unit](#hazard-detection-unit)
6. [Datapath Modules](#datapath-modules)
   - [ALU](#alu)
   - [Register File](#register-file)
   - [Immediate Generator](#immediate-generator)
7. [Memory Modules](#memory-modules)
   - [Instruction Memory (IMEM)](#instruction-memory-imem)
   - [Data Memory (DMEM)](#data-memory-dmem)
8. [Performance Monitoring](#performance-monitoring)

---

## Overview

This document provides comprehensive documentation for all SystemVerilog modules in the RISC-V 5-stage pipeline processor. Each module is documented with its purpose, interface, parameters, internal logic, and usage examples.

### Module Organization

Modules are organized by their role in the pipeline:
- **Pipeline Stages**: IF, ID, EX, MEM, WB
- **Pipeline Registers**: IF/ID, ID/EX, EX/MEM, MEM/WB
- **Control Units**: Control Unit, Branch/Jump Control, Forwarding Unit, Hazard Detection
- **Datapath Components**: ALU, Register File, Immediate Generator
- **Memory Modules**: Instruction Memory, Data Memory

---

## Top-Level Module

### `riscv_pipeline`

**Purpose**: Top-level module that integrates all 5 stages of the RISC-V pipeline processor.

**File**: `src/riscv_pipeline.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
parameter IMEM_DEPTH = 1024          // Instruction memory depth (words)
parameter IMEM_ADDR_WIDTH = 10       // Instruction memory address width
parameter DMEM_DEPTH = 1024          // Data memory depth (words)
parameter DMEM_ADDR_WIDTH = 10       // Data memory address width
parameter IMEM_INIT_FILE = "mem/inst_mem.hex"  // Instruction memory init file
parameter DMEM_INIT_FILE = ""        // Data memory init file (empty = no init)
```

**Ports**:
```systemverilog
input  logic clk              // Clock signal
input  logic rst_n            // Active-low reset
input  logic pipeline_stall   // External pipeline stall signal (optional)
input  logic pipeline_flush   // External pipeline flush signal (optional)
```

**Key Functionality**:
- Instantiates all 5 pipeline stages
- Connects pipeline registers between stages
- Integrates forwarding and hazard detection units
- Manages pipeline control signals (stall, flush)
- Connects instruction and data memory

**Usage Example**:
```systemverilog
riscv_pipeline #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32),
    .IMEM_DEPTH(1024),
    .IMEM_ADDR_WIDTH(10),
    .DMEM_DEPTH(1024),
    .DMEM_ADDR_WIDTH(10),
    .IMEM_INIT_FILE("mem/program.hex"),
    .DMEM_INIT_FILE("")
) uut (
    .clk(clk),
    .rst_n(rst_n),
    .pipeline_stall(1'b0),
    .pipeline_flush(1'b0)
);
```

---

## Pipeline Stage Modules

### IF Stage

**Module**: `if_stage`

**Purpose**: Fetches instructions from instruction memory and updates the program counter.

**File**: `src/if_stage.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
parameter IMEM_DEPTH = 1024          // Instruction memory depth (words)
parameter IMEM_ADDR_WIDTH = 10       // Instruction memory address width
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic [ADDR_WIDTH-1:0] branch_target   // Branch/jump target address
input  logic branch_taken           // Branch/jump taken signal
input  logic stall                  // Pipeline stall signal
input  logic flush                  // Pipeline flush signal
input  logic [DATA_WIDTH-1:0] imem_data       // Instruction data from memory
```

**Output Ports**:
```systemverilog
output logic [IMEM_ADDR_WIDTH-1:0] imem_addr  // Instruction memory address
output logic [DATA_WIDTH-1:0] instruction      // Fetched instruction
output logic [ADDR_WIDTH-1:0] PC               // Current program counter
output logic [ADDR_WIDTH-1:0] PC_plus_4       // PC + 4
```

**Key Internal Logic**:
- **PC Register**: Maintains current instruction address
- **PC Update Logic**: Selects between PC+4 (sequential) and branch_target (taken branch/jump)
- **Stall Handling**: PC holds current value when stalled
- **Flush Handling**: Clears fetched instruction

**Usage Example**:
```systemverilog
if_stage #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32),
    .IMEM_DEPTH(1024),
    .IMEM_ADDR_WIDTH(10)
) if_stage_inst (
    .clk(clk),
    .rst_n(rst_n),
    .branch_target(branch_target),
    .branch_taken(branch_taken),
    .stall(stall),
    .flush(flush),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .instruction(instruction),
    .PC(PC),
    .PC_plus_4(PC_plus_4)
);
```

---

### ID Stage

**Module**: `id_stage`

**Purpose**: Decodes instructions, reads register file, generates control signals, and extracts immediate values.

**File**: `src/id_stage.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic [DATA_WIDTH-1:0] instruction      // Instruction from IF stage
input  logic [ADDR_WIDTH-1:0] PC               // PC value from IF stage
input  logic [ADDR_WIDTH-1:0] PC_plus_4        // PC+4 from IF stage
input  logic wb_reg_write_en        // Write enable from WB stage
input  logic [4:0] wb_rd_addr       // Write address from WB stage
input  logic [DATA_WIDTH-1:0] wb_rd_data       // Write data from WB stage
input  logic stall                  // Pipeline stall signal
input  logic flush                  // Pipeline flush signal
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] rs1_data        // Source register 1 data
output logic [DATA_WIDTH-1:0] rs2_data         // Source register 2 data
output logic [DATA_WIDTH-1:0] immediate        // Sign-extended immediate
output logic [ADDR_WIDTH-1:0] PC_out           // PC value (passed through)
output logic [ADDR_WIDTH-1:0] PC_plus_4_out    // PC+4 value (passed through)
output logic RegWrite, MemRead, MemWrite       // Control signals
output logic MemToReg, ALUSrc                  // Control signals
output logic [1:0] ALUOp                       // ALU operation type
output logic Branch, Jump                       // Control signals
output logic [4:0] rs1_addr, rs2_addr, rd_addr // Register addresses
output logic [2:0] funct3                       // Function field
output logic [6:0] funct7, opcode              // Function fields
```

**Key Internal Logic**:
- **Control Unit**: Decodes opcode and generates control signals
- **Register File**: Reads source registers (rs1, rs2)
- **Immediate Generator**: Extracts and sign-extends immediate values
- **Instruction Decoder**: Extracts instruction fields (opcode, funct3, funct7, registers)

**Usage Example**:
```systemverilog
id_stage #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) id_stage_inst (
    .clk(clk),
    .rst_n(rst_n),
    .instruction(instruction),
    .PC(PC),
    .PC_plus_4(PC_plus_4),
    .wb_reg_write_en(wb_reg_write_en),
    .wb_rd_addr(wb_rd_addr),
    .wb_rd_data(wb_rd_data),
    .stall(stall),
    .flush(flush),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data),
    .immediate(immediate),
    // ... other outputs
);
```

---

### EX Stage

**Module**: `ex_stage`

**Purpose**: Performs ALU operations, calculates addresses, evaluates branch conditions, and handles data forwarding.

**File**: `src/ex_stage.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic [DATA_WIDTH-1:0] id_ex_rs1_data   // Source register 1 data
input  logic [DATA_WIDTH-1:0] id_ex_rs2_data   // Source register 2 data
input  logic [DATA_WIDTH-1:0] id_ex_immediate   // Sign-extended immediate
input  logic [ADDR_WIDTH-1:0] id_ex_PC         // PC value
input  logic [ADDR_WIDTH-1:0] id_ex_PC_plus_4   // PC+4 value
input  logic [DATA_WIDTH-1:0] ex_mem_alu_result // Forwarded ALU result
input  logic [DATA_WIDTH-1:0] mem_wb_write_data // Forwarded write-back data
input  logic [1:0] ForwardA, ForwardB          // Forwarding control signals
input  logic ALUSrc                            // ALU source select
input  logic [1:0] ALUOp                       // ALU operation type
input  logic [2:0] funct3                      // Function field
input  logic [6:0] funct7                      // Function field
input  logic [6:0] opcode                      // Instruction opcode
input  logic Branch, Jump                      // Control signals
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] alu_result       // ALU computation result
output logic zero_flag                         // ALU zero flag
output logic branch_taken                      // Branch condition evaluation
output logic [ADDR_WIDTH-1:0] branch_target    // Branch target address
output logic [ADDR_WIDTH-1:0] jump_target      // Jump target address
output logic [DATA_WIDTH-1:0] rs2_data_out     // rs2 data (for stores)
output logic [DATA_WIDTH-1:0] rs1_data_forwarded_out  // Forwarded rs1 data
output logic [DATA_WIDTH-1:0] rs2_data_forwarded_out   // Forwarded rs2 data
```

**Key Internal Logic**:
- **Forwarding Multiplexers**: Select data source for ALU operands (register file, EX/MEM, MEM/WB)
- **ALU Control Unit**: Decodes ALUOp and function fields to generate ALU control
- **ALU**: Performs arithmetic, logic, shift, and comparison operations
- **Branch Target Calculator**: Computes PC + immediate for branches
- **Jump Target Calculator**: Computes target address for jumps

**Usage Example**:
```systemverilog
ex_stage #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) ex_stage_inst (
    .id_ex_rs1_data(rs1_data),
    .id_ex_rs2_data(rs2_data),
    .id_ex_immediate(immediate),
    .id_ex_PC(PC),
    .id_ex_PC_plus_4(PC_plus_4),
    .ex_mem_alu_result(ex_mem_alu_result),
    .mem_wb_write_data(mem_wb_write_data),
    .ForwardA(ForwardA),
    .ForwardB(ForwardB),
    .ALUSrc(ALUSrc),
    .ALUOp(ALUOp),
    .funct3(funct3),
    .funct7(funct7),
    .opcode(opcode),
    .Branch(Branch),
    .Jump(Jump),
    .alu_result(alu_result),
    .zero_flag(zero_flag),
    // ... other outputs
);
```

---

### MEM Stage

**Module**: `mem_stage`

**Purpose**: Accesses data memory for load/store operations, or passes through ALU results for non-memory instructions.

**File**: `src/mem_stage.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
parameter MEM_DEPTH = 1024          // Memory depth in words
parameter MEM_ADDR_WIDTH = 10        // Memory address width
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic [DATA_WIDTH-1:0] ex_mem_alu_result  // ALU result (memory address)
input  logic [DATA_WIDTH-1:0] ex_mem_rs2_data     // Source register 2 data (for stores)
input  logic ex_mem_MemRead         // Memory read enable
input  logic ex_mem_MemWrite        // Memory write enable
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] mem_read_data      // Data read from memory
output logic [DATA_WIDTH-1:0] mem_alu_result      // ALU result (passthrough)
```

**Key Internal Logic**:
- **Data Memory (DMEM)**: Handles read and write operations
- **Address Validation**: Checks address alignment and range
- **ALU Result Passthrough**: Passes ALU result through for non-memory instructions

**Usage Example**:
```systemverilog
mem_stage #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32),
    .MEM_DEPTH(1024),
    .MEM_ADDR_WIDTH(10)
) mem_stage_inst (
    .clk(clk),
    .ex_mem_alu_result(alu_result),
    .ex_mem_rs2_data(rs2_data),
    .ex_mem_MemRead(MemRead),
    .ex_mem_MemWrite(MemWrite),
    .mem_read_data(mem_read_data),
    .mem_alu_result(mem_alu_result)
);
```

---

### WB Stage

**Module**: `wb_stage`

**Purpose**: Selects data source (ALU result or memory data) and writes results back to the register file.

**File**: `src/wb_stage.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
```

**Input Ports**:
```systemverilog
input  logic [DATA_WIDTH-1:0] mem_wb_alu_result      // ALU result from MEM stage
input  logic [DATA_WIDTH-1:0] mem_wb_mem_read_data    // Memory read data from MEM stage
input  logic mem_wb_MemToReg                         // Memory to register select
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] wb_write_data          // Data to write to register file
```

**Key Internal Logic**:
- **Writeback Multiplexer**: Selects between ALU result and memory data based on MemToReg signal
- **Combinational Logic**: No clock required, data available immediately

**Usage Example**:
```systemverilog
wb_stage #(
    .DATA_WIDTH(32)
) wb_stage_inst (
    .mem_wb_alu_result(alu_result),
    .mem_wb_mem_read_data(mem_read_data),
    .mem_wb_MemToReg(MemToReg),
    .wb_write_data(wb_write_data)
);
```

---

## Pipeline Register Modules

### IF/ID Register

**Module**: `if_id_reg`

**Purpose**: Pipeline register between IF and ID stages, stores instruction and PC values.

**File**: `src/if_id_reg.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic enable                 // Enable signal (0 = stall, 1 = normal)
input  logic flush                  // Flush signal (1 = clear register)
input  logic [DATA_WIDTH-1:0] if_instruction  // Instruction from IF stage
input  logic [ADDR_WIDTH-1:0] if_PC           // PC value from IF stage
input  logic [ADDR_WIDTH-1:0] if_PC_plus_4    // PC+4 from IF stage
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] id_instruction   // Instruction to ID stage
output logic [ADDR_WIDTH-1:0] id_PC            // PC value to ID stage
output logic [ADDR_WIDTH-1:0] id_PC_plus_4     // PC+4 to ID stage
```

**Key Internal Logic**:
- **Register Storage**: Stores instruction, PC, and PC+4 values
- **Stall Handling**: Holds current values when enable=0
- **Flush Handling**: Clears instruction register (inserts NOP) when flush=1
- **Reset Handling**: Initializes all registers to zero

**Usage Example**:
```systemverilog
if_id_reg #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) if_id_reg_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(~stall),
    .flush(flush),
    .if_instruction(instruction),
    .if_PC(PC),
    .if_PC_plus_4(PC_plus_4),
    .id_instruction(id_instruction),
    .id_PC(id_PC),
    .id_PC_plus_4(id_PC_plus_4)
);
```

---

### ID/EX Register

**Module**: `id_ex_reg`

**Purpose**: Pipeline register between ID and EX stages, stores register data, immediate values, PC, control signals, and instruction fields.

**File**: `src/id_ex_reg.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic enable                 // Enable signal (0 = stall, 1 = normal)
input  logic flush                  // Flush signal (1 = clear register)
// Data inputs from ID stage
input  logic [DATA_WIDTH-1:0] id_rs1_data, id_rs2_data
input  logic [DATA_WIDTH-1:0] id_immediate
input  logic [ADDR_WIDTH-1:0] id_PC, id_PC_plus_4
input  logic [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr
input  logic [2:0] id_funct3
input  logic [6:0] id_funct7, id_opcode
// Control signal inputs
input  logic id_RegWrite, id_MemRead, id_MemWrite
input  logic id_MemToReg, id_ALUSrc
input  logic [1:0] id_ALUOp
input  logic id_Branch, id_Jump
```

**Output Ports**:
```systemverilog
// Data outputs to EX stage (same structure as inputs, prefixed with 'ex_')
output logic [DATA_WIDTH-1:0] ex_rs1_data, ex_rs2_data
output logic [DATA_WIDTH-1:0] ex_immediate
output logic [ADDR_WIDTH-1:0] ex_PC, ex_PC_plus_4
output logic [4:0] ex_rs1_addr, ex_rs2_addr, ex_rd_addr
output logic [2:0] ex_funct3
output logic [6:0] ex_funct7, ex_opcode
// Control signal outputs
output logic ex_RegWrite, ex_MemRead, ex_MemWrite
output logic ex_MemToReg, ex_ALUSrc
output logic [1:0] ex_ALUOp
output logic ex_Branch, ex_Jump
```

**Key Internal Logic**:
- **Register Storage**: Stores all data and control signals from ID stage
- **Stall Handling**: Holds current values when enable=0
- **Flush Handling**: Clears all registers (inserts NOP) when flush=1
- **Reset Handling**: Initializes all registers to zero

**Usage Example**:
```systemverilog
id_ex_reg #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) id_ex_reg_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(~stall),
    .flush(flush || hazard_id_ex_flush),
    // ... all inputs from ID stage
    // ... all outputs to EX stage
);
```

---

### EX/MEM Register

**Module**: `ex_mem_reg`

**Purpose**: Pipeline register between EX and MEM stages, stores ALU result, rs2 data, control signals, and branch/jump information.

**File**: `src/ex_mem_reg.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic enable                 // Enable signal (0 = stall, 1 = normal)
input  logic flush                  // Flush signal (1 = clear register)
// Computation results from EX stage
input  logic [DATA_WIDTH-1:0] ex_alu_result
input  logic [DATA_WIDTH-1:0] ex_rs2_data
input  logic [4:0] ex_rd_addr
// Control signals
input  logic ex_MemRead, ex_MemWrite
input  logic ex_MemToReg, ex_RegWrite
// Branch/jump information
input  logic ex_Branch, ex_Jump
input  logic ex_branch_taken
input  logic [ADDR_WIDTH-1:0] ex_branch_target, ex_jump_target
input  logic ex_PCSrc, ex_branch_flush
```

**Output Ports**:
```systemverilog
// Computation results to MEM stage
output logic [DATA_WIDTH-1:0] mem_alu_result
output logic [DATA_WIDTH-1:0] mem_rs2_data
output logic [4:0] mem_rd_addr
// Control signals
output logic mem_MemRead, mem_MemWrite
output logic mem_MemToReg, mem_RegWrite
// Branch/jump information
output logic mem_Branch, mem_Jump
output logic mem_branch_taken
output logic [ADDR_WIDTH-1:0] mem_branch_target, mem_jump_target
output logic mem_PCSrc, mem_branch_flush
```

**Key Internal Logic**:
- **Register Storage**: Stores ALU result, rs2 data, control signals, and branch/jump info
- **Stall Handling**: Holds current values when enable=0
- **Flush Handling**: Clears all registers when flush=1
- **Reset Handling**: Initializes all registers to zero

**Usage Example**:
```systemverilog
ex_mem_reg #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) ex_mem_reg_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(~stall),
    .flush(flush),
    // ... all inputs from EX stage
    // ... all outputs to MEM stage
);
```

---

### MEM/WB Register

**Module**: `mem_wb_reg`

**Purpose**: Pipeline register between MEM and WB stages, stores memory read data, ALU result, and control signals.

**File**: `src/mem_wb_reg.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic enable                 // Enable signal (0 = stall, 1 = normal)
input  logic flush                  // Flush signal (1 = clear register)
// Data from MEM stage
input  logic [DATA_WIDTH-1:0] mem_read_data
input  logic [DATA_WIDTH-1:0] mem_alu_result
input  logic [4:0] mem_rd_addr
// Control signals
input  logic mem_MemToReg, mem_RegWrite
```

**Output Ports**:
```systemverilog
// Data to WB stage
output logic [DATA_WIDTH-1:0] wb_mem_read_data
output logic [DATA_WIDTH-1:0] wb_alu_result
output logic [4:0] wb_rd_addr
// Control signals
output logic wb_MemToReg, wb_RegWrite
```

**Key Internal Logic**:
- **Register Storage**: Stores memory read data, ALU result, and control signals
- **Stall Handling**: Holds current values when enable=0
- **Flush Handling**: Clears all registers when flush=1
- **Reset Handling**: Initializes all registers to zero

**Usage Example**:
```systemverilog
mem_wb_reg #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) mem_wb_reg_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(~stall),
    .flush(flush),
    // ... all inputs from MEM stage
    // ... all outputs to WB stage
);
```

---

## Control Modules

### Control Unit

**Module**: `control_unit`

**Purpose**: Decodes instruction opcodes and generates control signals for the datapath.

**File**: `src/control_unit.sv`

**Parameters**: None

**Input Ports**:
```systemverilog
input  logic [6:0] opcode           // Instruction opcode [6:0]
input  logic [2:0] funct3           // Function field [14:12]
input  logic [6:0] funct7            // Function field [31:25]
```

**Output Ports**:
```systemverilog
output logic RegWrite                // Register write enable
output logic MemRead                 // Memory read enable
output logic MemWrite                // Memory write enable
output logic MemToReg                // Memory to register select
output logic ALUSrc                  // ALU source select (0=register, 1=immediate)
output logic [1:0] ALUOp             // ALU operation type
output logic Branch                  // Branch instruction detected
output logic Jump                    // Jump instruction detected
```

**Key Internal Logic**:
- **Opcode Decoding**: Uses case statement on opcode to determine instruction type
- **Control Signal Generation**: Generates control signals based on instruction type
- **Combinational Logic**: All outputs are combinational (no clock)

**Control Signal Encoding**:
- **ALUOp**: `00`=Load/Store, `01`=Branch, `10`=R-type, `11`=I-type immediate
- **ALUSrc**: `0`=use register rs2, `1`=use immediate
- **MemToReg**: `0`=write ALU result, `1`=write memory data

**Usage Example**:
```systemverilog
control_unit control_unit_inst (
    .opcode(instruction[6:0]),
    .funct3(instruction[14:12]),
    .funct7(instruction[31:25]),
    .RegWrite(RegWrite),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .MemToReg(MemToReg),
    .ALUSrc(ALUSrc),
    .ALUOp(ALUOp),
    .Branch(Branch),
    .Jump(Jump)
);
```

---

### Branch/Jump Control Unit

**Module**: `branch_jump_control`

**Purpose**: Evaluates branch conditions and generates PC source selection and flush signals.

**File**: `src/branch_jump_control.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits)
```

**Input Ports**:
```systemverilog
input  logic Branch                  // Branch instruction indicator
input  logic Jump                    // Jump instruction indicator
input  logic [2:0] funct3            // Function field [14:12] (branch type)
input  logic zero_flag               // ALU zero flag (from SUB operation)
input  logic [DATA_WIDTH-1:0] rs1_data  // Source register 1 data (forwarded)
input  logic [DATA_WIDTH-1:0] rs2_data  // Source register 2 data (forwarded)
input  logic [ADDR_WIDTH-1:0] branch_target  // Branch target address
input  logic [ADDR_WIDTH-1:0] jump_target    // Jump target address
```

**Output Ports**:
```systemverilog
output logic PCSrc                   // PC source select (0=PC+4, 1=target)
output logic flush                   // Pipeline flush signal
output logic branch_taken            // Branch condition evaluation result
```

**Key Internal Logic**:
- **Branch Condition Evaluation**: Evaluates branch conditions based on funct3
  - `000` (BEQ): rs1 == rs2 (uses zero_flag)
  - `001` (BNE): rs1 != rs2 (uses !zero_flag)
  - `100` (BLT): rs1 < rs2 (signed comparison)
  - `101` (BGE): rs1 >= rs2 (signed comparison)
  - `110` (BLTU): rs1 < rs2 (unsigned comparison)
  - `111` (BGEU): rs1 >= rs2 (unsigned comparison)
- **Jump Handling**: Jumps are always taken
- **PC Source Selection**: PCSrc=1 when branch/jump taken
- **Flush Generation**: Flush=1 when branch/jump taken

**Usage Example**:
```systemverilog
branch_jump_control #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) branch_jump_control_inst (
    .Branch(Branch),
    .Jump(Jump),
    .funct3(funct3),
    .zero_flag(zero_flag),
    .rs1_data(rs1_data_forwarded),
    .rs2_data(rs2_data_forwarded),
    .branch_target(branch_target),
    .jump_target(jump_target),
    .PCSrc(PCSrc),
    .flush(flush),
    .branch_taken(branch_taken)
);
```

---

### Forwarding Unit

**Module**: `forwarding_unit`

**Purpose**: Detects RAW data hazards and generates forwarding control signals to select correct data source for ALU operands.

**File**: `src/forwarding_unit.sv`

**Parameters**: None

**Input Ports**:
```systemverilog
input  logic [4:0] id_ex_rs1_addr    // Source register 1 address (from ID/EX)
input  logic [4:0] id_ex_rs2_addr    // Source register 2 address (from ID/EX)
input  logic [4:0] ex_mem_rd_addr    // Destination register address (from EX/MEM)
input  logic [4:0] mem_wb_rd_addr    // Destination register address (from MEM/WB)
input  logic ex_mem_reg_write        // Register write enable (from EX/MEM)
input  logic mem_wb_reg_write        // Register write enable (from MEM/WB)
```

**Output Ports**:
```systemverilog
output logic [1:0] ForwardA          // Forwarding control for rs1
output logic [1:0] ForwardB          // Forwarding control for rs2
```

**Key Internal Logic**:
- **EX/MEM Hazard Detection**: Checks if EX/MEM stage writes to register matching rs1/rs2
- **MEM/WB Hazard Detection**: Checks if MEM/WB stage writes to register matching rs1/rs2
- **Priority**: EX/MEM forwarding has priority over MEM/WB forwarding
- **Forwarding Control Encoding**:
  - `00`: No forwarding (use register file data)
  - `01`: Forward from MEM/WB stage
  - `10`: Forward from EX/MEM stage
  - `11`: Reserved

**Usage Example**:
```systemverilog
forwarding_unit forwarding_unit_inst (
    .id_ex_rs1_addr(rs1_addr),
    .id_ex_rs2_addr(rs2_addr),
    .ex_mem_rd_addr(ex_mem_rd_addr),
    .mem_wb_rd_addr(mem_wb_rd_addr),
    .ex_mem_reg_write(ex_mem_reg_write),
    .mem_wb_reg_write(mem_wb_reg_write),
    .ForwardA(ForwardA),
    .ForwardB(ForwardB)
);
```

---

### Hazard Detection Unit

**Module**: `hazard_detection_unit`

**Purpose**: Detects load-use data hazards that cannot be resolved by forwarding and generates stall signals.

**File**: `src/hazard_detection_unit.sv`

**Parameters**: None

**Input Ports**:
```systemverilog
input  logic id_ex_MemRead           // Memory read enable from ID/EX register
input  logic [4:0] id_ex_rd_addr     // Destination register address from ID/EX register
input  logic [4:0] if_id_rs1_addr    // Source register 1 address from IF/ID register
input  logic [4:0] if_id_rs2_addr    // Source register 2 address from IF/ID register
```

**Output Ports**:
```systemverilog
output logic stall                   // Pipeline stall signal
output logic id_ex_flush             // ID/EX register flush signal
```

**Key Internal Logic**:
- **Load-Use Hazard Detection**: Detects when load instruction in EX stage produces data needed by instruction in ID stage
- **Hazard Condition**: `id_ex_MemRead = 1` AND (`id_ex_rd_addr == if_id_rs1_addr` OR `id_ex_rd_addr == if_id_rs2_addr`) AND `id_ex_rd_addr != 0`
- **Stall Generation**: Assert stall signal to prevent PC update and hold IF/ID register
- **Flush Generation**: Assert id_ex_flush to clear ID/EX register (insert NOP)

**Usage Example**:
```systemverilog
hazard_detection_unit hazard_detection_unit_inst (
    .id_ex_MemRead(ex_MemRead),
    .id_ex_rd_addr(ex_rd_addr),
    .if_id_rs1_addr(id_rs1_addr),
    .if_id_rs2_addr(id_rs2_addr),
    .stall(hazard_stall),
    .id_ex_flush(hazard_id_ex_flush)
);
```

---

## Datapath Modules

### ALU

**Module**: `alu`

**Purpose**: Performs arithmetic, logic, shift, and comparison operations.

**File**: `src/alu.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
```

**Input Ports**:
```systemverilog
input  logic [DATA_WIDTH-1:0] operand_a    // First operand (rs1)
input  logic [DATA_WIDTH-1:0] operand_b    // Second operand (rs2 or immediate)
input  logic [3:0] alu_control              // ALU operation control signal
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] result       // ALU computation result
output logic zero_flag                     // Zero flag (1 if result == 0)
```

**Key Internal Logic**:
- **Operation Selection**: Uses case statement on alu_control to select operation
- **ALU Operations**:
  - `0000`: AND (bitwise AND)
  - `0001`: OR (bitwise OR)
  - `0010`: ADD (addition)
  - `0110`: SUB (subtraction)
  - `0111`: SLT (set less than, signed)
  - `1000`: SLL (shift left logical)
  - `1001`: SRL (shift right logical)
  - `1010`: SRA (shift right arithmetic)
  - `1100`: XOR (bitwise XOR)
  - `1101`: SLTU (set less than unsigned)
- **Zero Flag Generation**: Sets zero_flag=1 when result equals zero

**Usage Example**:
```systemverilog
alu #(
    .DATA_WIDTH(32)
) alu_inst (
    .operand_a(alu_operand_a),
    .operand_b(alu_operand_b),
    .alu_control(alu_control),
    .result(alu_result),
    .zero_flag(zero_flag)
);
```

---

### Register File

**Module**: `reg_file`

**Purpose**: Provides 32 general-purpose registers with two read ports and one write port.

**File**: `src/reg_file.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 5             // Address width (5 bits for 32 registers)
parameter NUM_REGS = 32              // Number of registers
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic [ADDR_WIDTH-1:0] rs1_addr  // Read address for rs1
input  logic [ADDR_WIDTH-1:0] rs2_addr  // Read address for rs2
input  logic reg_write_en           // Write enable signal
input  logic [ADDR_WIDTH-1:0] rd_addr   // Write address (destination register)
input  logic [DATA_WIDTH-1:0] rd_data   // Write data
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] rs1_data  // Read data from rs1
output logic [DATA_WIDTH-1:0] rs2_data  // Read data from rs2
```

**Key Internal Logic**:
- **Register Storage**: Array of 32 registers, each 32 bits wide
- **Read Operations**: Combinational (asynchronous) reads from rs1 and rs2
- **Write Operations**: Synchronous writes on clock edge when reg_write_en=1 and rd_addr!=0
- **Register x0**: Hardwired to zero (reads return 0, writes ignored)
- **Reset Behavior**: All registers initialized to zero on reset

**Usage Example**:
```systemverilog
reg_file #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(5),
    .NUM_REGS(32)
) reg_file_inst (
    .clk(clk),
    .rst_n(rst_n),
    .rs1_addr(rs1_addr),
    .rs2_addr(rs2_addr),
    .reg_write_en(reg_write_en),
    .rd_addr(rd_addr),
    .rd_data(rd_data),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data)
);
```

---

### Immediate Generator

**Module**: `imm_gen`

**Purpose**: Extracts and sign-extends immediate values from RISC-V instructions based on instruction type.

**File**: `src/imm_gen.sv`

**Parameters**: None

**Input Ports**:
```systemverilog
input  logic [31:0] instruction      // 32-bit instruction word
```

**Output Ports**:
```systemverilog
output logic [31:0] immediate        // 32-bit sign-extended immediate value
```

**Key Internal Logic**:
- **Instruction Type Detection**: Uses opcode [6:0] to determine instruction type
- **Immediate Extraction**: Extracts immediate bits based on instruction format:
  - **I-type**: `instruction[31:20]` → immediate[11:0], sign-extend
  - **S-type**: `{instruction[31:25], instruction[11:7]}` → immediate[11:0], sign-extend
  - **B-type**: `{instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}` → immediate[12:0], sign-extend
  - **U-type**: `instruction[31:12]` → immediate[31:12], lower 12 bits zero
  - **J-type**: `{instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}` → immediate[20:0], sign-extend
- **Sign Extension**: Replicates MSB to fill upper bits (except U-type which is zero-extended)

**Usage Example**:
```systemverilog
imm_gen imm_gen_inst (
    .instruction(instruction),
    .immediate(immediate)
);
```

---

## Memory Modules

### Instruction Memory (IMEM)

**Module**: `imem`

**Purpose**: Read-only instruction memory that stores program instructions.

**File**: `src/imem.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 10           // Address width (log2 of memory depth)
parameter MEM_DEPTH = 1024          // Memory depth in words (default 1024)
parameter INIT_FILE = "mem/inst_mem.hex"  // Initialization file path
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic [ADDR_WIDTH-1:0] addr  // Read address (word-aligned)
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] data  // Read data (instruction)
output logic addr_valid              // Address valid signal (1 if in range)
```

**Key Internal Logic**:
- **Memory Array**: Array of 32-bit words storing instructions
- **Synchronous Read**: Address registered on clock edge, data available next cycle
- **Memory Initialization**: Loads instructions from hex file using `$readmemh`
- **Address Validation**: Checks if address is within valid range (0 to MEM_DEPTH-1)
- **Default Behavior**: Returns NOP (0x00000013) for invalid addresses

**Usage Example**:
```systemverilog
imem #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(10),
    .MEM_DEPTH(1024),
    .INIT_FILE("mem/program.hex")
) instruction_memory (
    .clk(clk),
    .addr(imem_addr),
    .data(imem_data),
    .addr_valid()
);
```

---

### Data Memory (DMEM)

**Module**: `dmem`

**Purpose**: Read-write data memory for load/store instructions.

**File**: `src/dmem.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter ADDR_WIDTH = 32            // Address width (32 bits for byte addresses)
parameter MEM_DEPTH = 1024          // Memory depth in words (default 1024)
parameter MEM_ADDR_WIDTH = 10        // Memory address width (log2 of MEM_DEPTH)
parameter INIT_FILE = ""             // Initialization file path (empty = no initialization)
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic [ADDR_WIDTH-1:0] addr  // Byte address (word-aligned: addr[1:0] = 00)
input  logic [DATA_WIDTH-1:0] write_data  // Data to write
input  logic MemRead                // Memory read enable
input  logic MemWrite               // Memory write enable
```

**Output Ports**:
```systemverilog
output logic [DATA_WIDTH-1:0] read_data  // Read data
output logic addr_valid                  // Address valid signal (1 if in range and aligned)
```

**Key Internal Logic**:
- **Memory Array**: Array of 32-bit words storing data
- **Word Address Calculation**: Extracts word address from byte address (`addr[MEM_ADDR_WIDTH+1:2]`)
- **Address Validation**: Checks address alignment (lower 2 bits = 00) and range
- **Synchronous Read**: Address registered, data available next cycle
- **Synchronous Write**: Address and data registered, write occurs on clock edge
- **Memory Initialization**: Optionally loads data from hex file using `$readmemh`

**Usage Example**:
```systemverilog
dmem #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32),
    .MEM_DEPTH(1024),
    .MEM_ADDR_WIDTH(10),
    .INIT_FILE("")
) data_memory (
    .clk(clk),
    .addr(mem_addr),
    .write_data(write_data),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .read_data(read_data),
    .addr_valid()
);
```

---

## Performance Monitoring

### Performance Monitor

**Module**: `performance_monitor`

**Purpose**: Monitors pipeline performance metrics including cycles, instructions, stalls, flushes, and CPI.

**File**: `src/performance_monitor.sv`

**Parameters**:
```systemverilog
parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
parameter CPI_WIDTH = 16            // CPI counter width (fixed-point)
parameter CPI_FRAC_BITS = 8         // CPI fractional bits
```

**Input Ports**:
```systemverilog
input  logic clk                    // Clock signal
input  logic rst_n                  // Active-low reset
input  logic enable                 // Enable signal
input  logic instruction_completed  // Instruction completed in WB stage
input  logic pipeline_stall         // Pipeline stall signal
input  logic pipeline_flush         // Pipeline flush signal
input  logic is_load_store          // Load/store instruction indicator
input  logic is_branch              // Branch instruction indicator
```

**Output Ports**:
```systemverilog
output logic [31:0] total_cycles           // Total clock cycles
output logic [31:0] instructions_completed  // Total instructions completed
output logic [31:0] pipeline_stalls         // Total pipeline stalls
output logic [31:0] pipeline_flushes       // Total pipeline flushes
output logic [31:0] load_store_count        // Load/store instruction count
output logic [31:0] branch_count            // Branch instruction count
output logic [CPI_WIDTH-1:0] CPI            // Cycles per instruction (fixed-point)
```

**Key Internal Logic**:
- **Cycle Counter**: Increments every clock cycle when enabled
- **Instruction Counter**: Increments when instruction_completed=1
- **Stall Counter**: Increments when pipeline_stall=1
- **Flush Counter**: Increments when pipeline_flush=1
- **CPI Calculation**: Uses fixed-point arithmetic to calculate CPI = (total_cycles << CPI_FRAC_BITS) / instructions_completed

**Usage Example**:
```systemverilog
performance_monitor #(
    .DATA_WIDTH(32),
    .CPI_WIDTH(16),
    .CPI_FRAC_BITS(8)
) perf_monitor (
    .clk(clk),
    .rst_n(rst_n),
    .enable(1'b1),
    .instruction_completed(wb_RegWrite),
    .pipeline_stall(stall),
    .pipeline_flush(flush),
    .is_load_store(MemRead || MemWrite),
    .is_branch(Branch),
    .total_cycles(total_cycles),
    .instructions_completed(instructions_completed),
    .pipeline_stalls(pipeline_stalls),
    .pipeline_flushes(pipeline_flushes),
    .load_store_count(load_store_count),
    .branch_count(branch_count),
    .CPI(CPI)
);
```

---

## Module Integration Summary

### Pipeline Flow

```
IF Stage → IF/ID Register → ID Stage → ID/EX Register → EX Stage
                                                              ↓
                                                         Forwarding Unit
                                                              ↓
                                                         EX/MEM Register
                                                              ↓
                                                         MEM Stage
                                                              ↓
                                                         MEM/WB Register
                                                              ↓
                                                         WB Stage → Register File
```

### Control Signal Flow

```
Control Unit (ID) → ID/EX Register → EX Stage
                                      ↓
                              Branch/Jump Control
                                      ↓
                              EX/MEM Register → MEM Stage → MEM/WB Register → WB Stage
```

### Hazard Handling Flow

```
Hazard Detection Unit → Stall Signal → IF Stage, IF/ID Register
                              ↓
                         ID/EX Flush → ID/EX Register

Forwarding Unit → ForwardA/ForwardB → EX Stage (Forwarding Muxes)
```

---

## Module Dependencies

### Dependency Graph

```
riscv_pipeline (top-level)
├── if_stage
│   └── imem
├── if_id_reg
├── id_stage
│   ├── control_unit
│   ├── reg_file
│   └── imm_gen
├── id_ex_reg
├── forwarding_unit
├── hazard_detection_unit
├── ex_stage
│   └── alu
├── branch_jump_control
├── ex_mem_reg
├── mem_stage
│   └── dmem
├── mem_wb_reg
└── wb_stage
```

### Key Dependencies

- **IF Stage** depends on: IMEM
- **ID Stage** depends on: Control Unit, Register File, Immediate Generator
- **EX Stage** depends on: ALU, Forwarding Unit, Branch/Jump Control Unit
- **MEM Stage** depends on: DMEM
- **WB Stage**: No dependencies (combinational logic only)
- **Forwarding Unit**: Uses signals from ID/EX, EX/MEM, MEM/WB registers
- **Hazard Detection Unit**: Uses signals from IF/ID and ID/EX registers

---

## Module Design Principles

### 1. Modularity
- Each module has a single, well-defined responsibility
- Modules communicate through well-defined interfaces
- Easy to test and verify independently

### 2. Parameterization
- Key modules are parameterized for flexibility
- Allows customization of data widths, memory sizes, etc.
- Facilitates reuse and scaling

### 3. Pipeline Registers
- All pipeline registers follow consistent interface pattern
- Enable and flush signals for pipeline control
- Synchronous updates on clock edge

### 4. Control Signal Propagation
- Control signals generated in ID stage
- Propagate through pipeline registers to downstream stages
- Used only when needed in each stage

### 5. Forwarding and Hazard Handling
- Forwarding Unit handles most data hazards
- Hazard Detection Unit handles load-use hazards
- Both units operate independently and in parallel

---

*This document provides comprehensive documentation for all SystemVerilog modules in the RISC-V 5-stage pipeline processor. For implementation details, refer to the source code in the `src/` directory.*

