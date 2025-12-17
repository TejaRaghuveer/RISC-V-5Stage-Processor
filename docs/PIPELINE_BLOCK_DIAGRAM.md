# RISC-V 5-Stage Pipeline Block Diagram

## Complete Pipeline Architecture

This document provides a detailed ASCII art block diagram of the RISC-V 5-stage pipeline processor, showing all stages, pipeline registers, major components, forwarding paths, hazard detection, and control connections.

---

## Full Pipeline Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    RISC-V 5-STAGE PIPELINE PROCESSOR                                        │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                          STAGE 1: INSTRUCTION FETCH (IF)                                    │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌──────────────┐
                    │   PC (PC)    │◀────────────────────────────────────────────────────────┐
                    │              │                                                         │
                    └──────┬───────┘                                                         │
                           │                                                                 │
                           │ PC[31:0]                                                        │
                           │                                                                 │
                    ┌──────▼───────┐                                                         │
                    │              │                                                         │
                    │   IMEM       │                                                         │
                    │              │                                                         │
                    └──────┬───────┘                                                         │
                           │                                                                 │
                           │ Instruction[31:0]                                              │
                           │                                                                 │
                    ┌──────▼──────────────────────────────────────────────────────────────┐ │
                    │                        IF/ID PIPELINE REGISTER                       │ │
                    │  ┌──────────────────────────────────────────────────────────────┐ │ │
                    │  │ PC_plus_4[31:0]  │  Instruction[31:0]                       │ │ │
                    │  └──────────────────────────────────────────────────────────────┘ │ │
                    └────────────────────────────────────────────────────────────────────┘ │
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                          STAGE 2: INSTRUCTION DECODE (ID)                                    │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                        IF/ID REGISTER OUTPUTS                       │
                    │  PC_plus_4[31:0]  │  Instruction[31:0]                          │
                    └──────────┬────────────────────┬─────────────────────────────────────┘
                               │                    │
                               │                    │ Instruction[31:0]
                               │                    │
                    ┌──────────▼──────────┐        │
                    │                     │        │
                    │  IMMEDIATE          │        │
                    │  GENERATOR          │        │
                    │                     │        │
                    │  ┌────────────────┐ │        │
                    │  │ I-type        │ │        │
                    │  │ S-type        │ │        │
                    │  │ B-type        │ │        │
                    │  │ U-type        │ │        │
                    │  │ J-type        │ │        │
                    │  └────────────────┘ │        │
                    │                     │        │
                    └──────────┬──────────┘        │
                               │                    │
                               │ Imm[31:0]         │
                               │                    │
                    ┌──────────▼────────────────────▼──────────┐
                    │                                           │
                    │         CONTROL UNIT                      │
                    │  ┌────────────────────────────────────┐ │
                    │  │ RegWrite │ MemRead │ MemWrite       │ │
                    │  │ Branch   │ Jump    │ ALUSrc         │ │
                    │  │ MemtoReg │ ALUOp   │ RegDst         │ │
                    │  └────────────────────────────────────┘ │
                    │                                           │
                    └──────────┬───────────────────────────────┘
                               │
                               │ Control Signals
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                                                                      │
                    │                    REGISTER FILE (32 registers)                      │
                    │  ┌──────────────────────────────────────────────────────────────┐   │
                    │  │ Read Port 1: rs1[4:0] → ReadData1[31:0]                    │   │
                    │  │ Read Port 2: rs2[4:0] → ReadData2[31:0]                    │   │
                    │  │ Write Port:  rd[4:0]  ← WriteData[31:0] (from WB stage)   │   │
                    │  │ x0 hardwired to 0                                            │   │
                    │  └──────────────────────────────────────────────────────────────┘   │
                    │                                                                      │
                    └──────────┬──────────────────────────┬───────────────────────────────┘
                               │                          │
                               │ ReadData1[31:0]          │ ReadData2[31:0]
                               │                          │
                    ┌──────────▼──────────────────────────▼───────────────────────────────┐
                    │                    ID/EX PIPELINE REGISTER                          │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ PC_plus_4[31:0] │ ReadData1[31:0] │ ReadData2[31:0]         │  │
                    │  │ Imm[31:0]       │ rs1[4:0]        │ rs2[4:0]                │  │
                    │  │ rd[4:0]         │ Control Signals │ Instruction[31:0]      │  │
                    │  └──────────────────────────────────────────────────────────────┘  │
                    └──────────────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                          STAGE 3: EXECUTE (EX)                            │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    ID/EX REGISTER OUTPUTS                          │
                    │  PC_plus_4 │ ReadData1 │ ReadData2 │ Imm │ rs1 │ rs2 │ rd │ ... │
                    └──────────┬──────────┬──────────┬──────────┬──────┬──────┬──────┘
                               │          │          │          │      │      │
                               │          │          │          │      │      │
                    ┌──────────▼──────────▼──────────▼──────────▼──────▼──────▼──────────┐
                    │                                                                     │
                    │                    FORWARDING UNIT                                  │
                    │  ┌────────────────────────────────────────────────────────────┐  │
                    │  │ Inputs:                                                      │  │
                    │  │   - EX/MEM RegWrite, EX/MEM rd                              │  │
                    │  │   - MEM/WB RegWrite, MEM/WB rd                              │  │
                    │  │   - ID/EX rs1, ID/EX rs2                                    │  │
                    │  │                                                             │  │
                    │  │ Outputs:                                                    │  │
                    │  │   - ForwardA[1:0] (00=RF, 01=MEM/WB, 10=EX/MEM)           │  │
                    │  │   - ForwardB[1:0] (00=RF, 01=MEM/WB, 10=EX/MEM)           │  │
                    │  └────────────────────────────────────────────────────────────┘  │
                    │                                                                     │
                    └──────────┬─────────────────────────────────────────────────────────┘
                               │
                               │ ForwardA[1:0], ForwardB[1:0]
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                                                                     │
                    │                    FORWARDING MUXES                                 │
                    │                                                                     │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ ForwardA Mux:                                               │  │
                    │  │   00: ReadData1 (from ID/EX)                                │  │
                    │  │   01: WriteData (from MEM/WB)                              │  │
                    │  │   10: ALU_Result (from EX/MEM)                             │  │
                    │  │                                                             │  │
                    │  │ ForwardB Mux:                                               │  │
                    │  │   00: ReadData2 (from ID/EX)                                │  │
                    │  │   01: WriteData (from MEM/WB)                              │  │
                    │  │   10: ALU_Result (from EX/MEM)                             │  │
                    │  └────────────────────────────────────────────────────────────┘  │
                    │                                                                     │
                    └──────────┬──────────────────────────┬──────────────────────────────┘
                               │                          │
                               │ ForwardedData1[31:0]     │ ForwardedData2[31:0]
                               │                          │
                    ┌──────────▼──────────────────────────▼───────────────────────────────┐
                    │                                                                     │
                    │                    ALU INPUT MUX                                   │
                    │                                                                     │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ ALUSrc Mux:                                                 │  │
                    │  │   0: ForwardedData2 (register)                              │  │
                    │  │   1: Imm (immediate)                                        │  │
                    │  └────────────────────────────────────────────────────────────┘  │
                    │                                                                     │
                    └──────────┬──────────────────────────┬──────────────────────────────┘
                               │                          │
                               │ ForwardedData1[31:0]     │ ALU_Input2[31:0]
                               │                          │
                    ┌──────────▼──────────────────────────▼───────────────────────────────┐
                    │                                                                     │
                    │                    ARITHMETIC LOGIC UNIT (ALU)                      │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ Inputs: ALU_Input1[31:0], ALU_Input2[31:0]                  │  │
                    │  │ Control: ALUOp[2:0] (from Control Unit)                     │  │
                    │  │                                                              │  │
                    │  │ Operations: ADD, SUB, AND, OR, XOR, SLT, SLTU, SLL, SRL, SRA│  │
                    │  │                                                              │  │
                    │  │ Outputs: ALU_Result[31:0], Zero, LessThan                   │  │
                    │  └────────────────────────────────────────────────────────────┘  │
                    │                                                                     │
                    └──────────┬───────────────────────────────────────────────────────────┘
                               │
                               │ ALU_Result[31:0]
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                                                                     │
                    │                    BRANCH/JUMP CONTROL                              │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ Branch Condition Evaluation:                               │  │
                    │  │   BEQ:  (ForwardedData1 == ForwardedData2)                │  │
                    │  │   BNE:  (ForwardedData1 != ForwardedData2)                │  │
                    │  │   BLT:  (ForwardedData1 < ForwardedData2) signed          │  │
                    │  │   BGE:  (ForwardedData1 >= ForwardedData2) signed         │  │
                    │  │   BLTU: (ForwardedData1 < ForwardedData2) unsigned         │  │
                    │  │   BGEU: (ForwardedData1 >= ForwardedData2) unsigned        │  │
                    │  │                                                             │  │
                    │  │ Branch Target: PC_plus_4 + Imm                             │  │
                    │  │ Jump Target:   PC_plus_4 + Imm (JAL) or rs1 + Imm (JALR)  │  │
                    │  │                                                             │  │
                    │  │ Outputs: branch_taken, jump_taken, branch_target[31:0]    │  │
                    │  └────────────────────────────────────────────────────────────┘  │
                    │                                                                     │
                    └──────────┬───────────────────────────────────────────────────────────┘
                               │
                               │ branch_taken, jump_taken, branch_target[31:0]
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                    EX/MEM PIPELINE REGISTER                         │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ ALU_Result[31:0] │ ForwardedData2[31:0] │ rd[4:0]         │  │
                    │  │ PC_plus_4[31:0]  │ Control Signals      │ branch_taken    │  │
                    │  │ branch_target    │ jump_taken          │ ...             │  │
                    │  └──────────────────────────────────────────────────────────────┘  │
                    └──────────────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                          STAGE 4: MEMORY ACCESS (MEM)                      │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    EX/MEM REGISTER OUTPUTS                         │
                    │  ALU_Result │ ForwardedData2 │ rd │ Control Signals │ ...       │
                    └──────────┬──────────┬──────────────────────────────────────────────┘
                               │          │
                               │          │ ForwardedData2[31:0] (store data)
                               │          │
                    ┌──────────▼──────────▼───────────────────────────────────────────────┐
                    │                                                                   │
                    │                    DATA MEMORY (DMEM)                             │
                    │  ┌──────────────────────────────────────────────────────────┐  │
                    │  │ Address: ALU_Result[31:0]                                  │  │
                    │  │ WriteData: ForwardedData2[31:0]                            │  │
                    │  │ MemRead: Control Signal                                    │  │
                    │  │ MemWrite: Control Signal                                   │  │
                    │  │                                                             │  │
                    │  │ Operations:                                                │  │
                    │  │   - LW: Load Word                                           │  │
                    │  │   - SW: Store Word                                          │  │
                    │  │   - LB/LH/LBU/LHU: Load Byte/Halfword                       │  │
                    │  │   - SB/SH: Store Byte/Halfword                             │  │
                    │  │                                                             │  │
                    │  │ Output: ReadData[31:0]                                      │  │
                    │  └──────────────────────────────────────────────────────────┘  │
                    │                                                                   │
                    └──────────┬─────────────────────────────────────────────────────────┘
                               │
                               │ ReadData[31:0]
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                    MEM/WB PIPELINE REGISTER                        │
                    │  ┌──────────────────────────────────────────────────────────────┐  │
                    │  │ ALU_Result[31:0] │ ReadData[31:0] │ rd[4:0] │ Control      │  │
                    │  │ MemtoReg signal                                               │  │
                    │  └──────────────────────────────────────────────────────────────┘  │
                    └──────────────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                          STAGE 5: WRITE BACK (WB)                          │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    MEM/WB REGISTER OUTPUTS                         │
                    │  ALU_Result[31:0] │ ReadData[31:0] │ rd[4:0] │ MemtoReg         │
                    └──────────┬──────────┬──────────────────────────────────────────────┘
                               │          │
                    ┌──────────▼──────────▼───────────────────────────────────────────────┐
                    │                                                                     │
                    │                    WRITE-BACK MUX                                  │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ MemtoReg Mux:                                                │ │
                    │  │   0: ALU_Result (from ALU)                                   │ │
                    │  │   1: ReadData (from DMEM)                                    │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └──────────┬──────────────────────────────────────────────────────────┘
                               │
                               │ WriteData[31:0]
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                                                                     │
                    │                    REGISTER FILE WRITE PORT                         │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ WriteData[31:0] → Register[rd]                              │ │
                    │  │ RegWrite signal enables write                                │ │
                    │  │ x0 is hardwired to 0 (writes ignored)                       │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └─────────────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                    HAZARD DETECTION UNIT                                   │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    HAZARD DETECTION UNIT                          │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ Inputs:                                                      │ │
                    │  │   - ID/EX MemRead (load instruction)                        │ │
                    │  │   - ID/EX rd (destination register)                        │ │
                    │  │   - IF/ID rs1, rs2 (source registers)                      │ │
                    │  │   - EX/MEM Branch, Jump (control hazards)                   │ │
                    │  │                                                             │ │
                    │  │ Load-Use Hazard Detection:                                  │ │
                    │  │   IF (ID/EX.MemRead) AND                                    │ │
                    │  │   ((ID/EX.rd == IF/ID.rs1) OR (ID/EX.rd == IF/ID.rs2))     │ │
                    │  │                                                             │ │
                    │  │ Outputs:                                                    │ │
                    │  │   - Stall: Pipeline stall signal                            │ │
                    │  │   - PCWrite: Enable PC update (0 = stall)                  │ │
                    │  │   - IF/ID_Enable: Enable IF/ID update (0 = stall)         │ │
                    │  │   - ID/EX_Flush: Flush ID/EX (insert bubble)               │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └──────────┬──────────────────────────────────────────────────────────┘
                               │
                               │ Stall, PCWrite, IF/ID_Enable, ID/EX_Flush
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                    PIPELINE CONTROL                                 │
                    │  - PC: Hold during stall                                           │
                    │  - IF/ID: Hold during stall                                       │
                    │  - ID/EX: Flush during stall (insert NOP)                         │
                    └──────────────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                    BRANCH/JUMP CONTROL                                     │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    BRANCH/JUMP FLUSH CONTROL                      │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ When branch_taken or jump_taken:                             │ │
                    │  │   - PCSrc: Select branch/jump target (not PC+4)              │ │
                    │  │   - Flush IF/ID: Clear instruction in IF/ID                 │ │
                    │  │   - Flush ID/EX: Clear instruction in ID/EX                 │ │
                    │  │                                                             │ │
                    │  │ PC Update:                                                  │ │
                    │  │   - Sequential: PC = PC + 4                                 │ │
                    │  │   - Branch:     PC = branch_target                           │ │
                    │  │   - Jump:       PC = jump_target                             │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └──────────┬───────────────────────────────────────────────────────────┘
                               │
                               │ PCSrc, Flush_IF_ID, Flush_ID_EX, branch_target, jump_target
                               │
                    ┌──────────▼───────────────────────────────────────────────────────────┐
                    │                    PC UPDATE LOGIC                                  │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ PC_Mux:                                                      │ │
                    │  │   00: PC + 4 (sequential)                                   │ │
                    │  │   01: branch_target (branch taken)                          │ │
                    │  │   10: jump_target (jump)                                    │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    └──────────────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
│                                    FORWARDING PATHS (DETAILED)                            │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    EX/MEM → EX FORWARDING                         │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ Path: EX/MEM.ALU_Result → EX.ForwardA/ForwardB Mux         │ │
                    │  │                                                              │ │
                    │  │ Condition:                                                   │ │
                    │  │   (EX/MEM.RegWrite) AND                                      │ │
                    │  │   (EX/MEM.rd != 0) AND                                       │ │
                    │  │   (EX/MEM.rd == ID/EX.rs1) → ForwardA = 10                  │ │
                    │  │   (EX/MEM.rd == ID/EX.rs2) → ForwardB = 10                  │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └──────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    MEM/WB → EX FORWARDING                         │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ Path: MEM/WB.WriteData → EX.ForwardA/ForwardB Mux          │ │
                    │  │                                                              │ │
                    │  │ Condition:                                                   │ │
                    │  │   (MEM/WB.RegWrite) AND                                      │ │
                    │  │   (MEM/WB.rd != 0) AND                                       │ │
                    │  │   NOT (EX/MEM forwarding) AND                                │ │
                    │  │   (MEM/WB.rd == ID/EX.rs1) → ForwardA = 01                  │ │
                    │  │   (MEM/WB.rd == ID/EX.rs2) → ForwardB = 01                  │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └──────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────────────────────────────────────┐
                    │                    NO FORWARDING (NORMAL)                          │
                    │  ┌──────────────────────────────────────────────────────────────┐ │
                    │  │ Path: Register File → EX Stage                              │ │
                    │  │                                                              │ │
                    │  │ Condition:                                                   │ │
                    │  │   No hazard detected → ForwardA = 00, ForwardB = 00        │ │
                    │  │   Use ReadData1/ReadData2 from Register File               │ │
                    │  └──────────────────────────────────────────────────────────────┘ │
                    │                                                                     │
                    └──────────────────────────────────────────────────────────────────────┘

```

---

## Key Components Summary

### Pipeline Stages

1. **IF (Instruction Fetch)**
   - Program Counter (PC)
   - Instruction Memory (IMEM)
   - PC increment logic

2. **ID (Instruction Decode)**
   - Register File (32 registers)
   - Immediate Generator
   - Control Unit
   - Hazard Detection Unit (load-use detection)

3. **EX (Execute)**
   - ALU (Arithmetic Logic Unit)
   - Forwarding Unit
   - Forwarding Muxes
   - Branch/Jump Control

4. **MEM (Memory Access)**
   - Data Memory (DMEM)
   - Load/Store operations

5. **WB (Write Back)**
   - Write-back Mux (ALU result vs. memory data)
   - Register File write port

### Pipeline Registers

- **IF/ID**: PC+4, Instruction
- **ID/EX**: PC+4, ReadData1, ReadData2, Immediate, rs1, rs2, rd, Control Signals
- **EX/MEM**: ALU_Result, ForwardedData2, rd, Control Signals, branch_taken, jump_taken
- **MEM/WB**: ALU_Result, ReadData, rd, Control Signals

### Forwarding Paths

1. **EX/MEM → EX**: Forward ALU result from EX stage to resolve RAW hazards
2. **MEM/WB → EX**: Forward write-back data from MEM/WB stages to EX stage
3. **Priority**: EX/MEM forwarding takes precedence over MEM/WB forwarding

### Hazard Detection

- **Load-Use Hazard**: Detected in ID stage, causes 1-cycle stall
- **Control Hazard**: Branch/jump taken causes pipeline flush

### Control Signals

- **RegWrite**: Enable register write
- **MemRead**: Enable memory read
- **MemWrite**: Enable memory write
- **Branch**: Branch instruction indicator
- **Jump**: Jump instruction indicator
- **ALUSrc**: ALU input source (register vs. immediate)
- **MemtoReg**: Write-back source (ALU vs. memory)
- **ALUOp**: ALU operation control

---

## Data Flow Example

### Example: ADD instruction with forwarding

```
Cycle 1: IF  - Fetch ADD x3, x1, x2
Cycle 2: ID  - Decode, read x1, x2 from register file
Cycle 3: EX  - ALU computes x1 + x2, forwarding used if needed
Cycle 4: MEM - Pass ALU result through (no memory access)
Cycle 5: WB  - Write result to x3
```

### Example: Load-Use Hazard

```
Cycle 1: IF  - Fetch LW x5, 0(x1)
Cycle 2: ID  - Decode LW
Cycle 3: EX  - Compute address, read memory
Cycle 4: MEM - Load data from memory
Cycle 5: WB  - Write data to x5
         IF  - Fetch ADD x6, x5, x2 (STALL - x5 not ready)
Cycle 6: ID  - Decode ADD (held in ID stage)
Cycle 7: EX  - ALU computes x5 + x2 (x5 now available from MEM/WB)
```

---

## Legend

- **Solid lines**: Data paths
- **Dashed lines**: Control signals
- **Boxes**: Components/modules
- **Arrows**: Data/control flow direction
- **Pipeline Registers**: Shown as boxes between stages

---

**Note**: This diagram shows the complete data path and control flow of the RISC-V 5-stage pipeline processor. All forwarding paths, hazard detection, and control mechanisms are illustrated.

