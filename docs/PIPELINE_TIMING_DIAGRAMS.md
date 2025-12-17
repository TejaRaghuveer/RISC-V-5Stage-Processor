# RISC-V 5-Stage Pipeline Timing Diagrams

This document provides detailed pipeline timing diagrams showing how instructions progress through the pipeline, including hazard scenarios, forwarding, stalls, and flushes.

---

## Table of Contents

1. [Basic Pipeline Flow (No Hazards)](#basic-pipeline-flow-no-hazards)
2. [Data Forwarding (EX/MEM → EX)](#data-forwarding-exmem--ex)
3. [Data Forwarding (MEM/WB → EX)](#data-forwarding-memwb--ex)
4. [Load-Use Hazard (Pipeline Stall)](#load-use-hazard-pipeline-stall)
5. [Branch Taken (Pipeline Flush)](#branch-taken-pipeline-flush)
6. [Complex Scenario (Mixed Hazards)](#complex-scenario-mixed-hazards)

---

## Basic Pipeline Flow (No Hazards)

### Instruction Sequence
```assembly
ADDI x1, x0, 5      # x1 = 5
ADDI x2, x0, 10     # x2 = 10
ADD  x3, x1, x2     # x3 = x1 + x2 = 15
SW   x3, 0(x0)      # Store x3 to memory[0]
LW   x4, 0(x0)      # Load memory[0] to x4
```

### Pipeline Timing Diagram

| Instruction | Cycle 1 | Cycle 2 | Cycle 3 | Cycle 4 | Cycle 5 | Cycle 6 | Cycle 7 | Cycle 8 | Cycle 9 |
|-------------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
| **ADDI x1** | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |         |
| **ADDI x2** |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |
| **ADD x3**  |         |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |
| **SW x3**   |         |         |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |
| **LW x4**   |         |         |         |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |

**Notes**: 
- Ideal pipeline flow with no hazards
- One instruction completes per cycle after initial 4-cycle fill
- CPI = 1.0 (ideal performance)

---

## Data Forwarding (EX/MEM → EX)

### Instruction Sequence
```assembly
ADDI x5, x0, 5      # x5 = 5
ADD  x6, x5, x2     # x6 = x5 + x2 (uses x5 from previous instruction)
SUB  x7, x6, x1     # x7 = x6 - x1 (uses x6 from previous instruction)
```

### Pipeline Timing Diagram

| Instruction | Cycle 1 | Cycle 2 | Cycle 3 | Cycle 4 | Cycle 5 | Cycle 6 | Cycle 7 |
|-------------|---------|---------|---------|---------|---------|---------|---------|
| **ADDI x5** | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |
| **ADD x6**  |         | **IF**  | **ID**  | **EX** ⚡| **MEM** | **WB**  |         |
| **SUB x7**  |         |         | **IF**  | **ID**  | **EX** ⚡| **MEM** | **WB**  |

**Legend**:
- ⚡ = **Forwarding from EX/MEM**: x5 forwarded to ADD x6 in cycle 4, x6 forwarded to SUB x7 in cycle 5
- No stall required - forwarding resolves RAW hazard

**Forwarding Details**:
- **Cycle 4**: ADD x6 uses x5 from EX/MEM stage (ForwardA = 10)
- **Cycle 5**: SUB x7 uses x6 from EX/MEM stage (ForwardA = 10)

---

## Data Forwarding (MEM/WB → EX)

### Instruction Sequence
```assembly
ADDI x10, x0, 10    # x10 = 10
ADDI x11, x0, 20    # x11 = 20
ADD  x12, x10, x11  # x12 = x10 + x11 (uses x10 from MEM/WB)
```

### Pipeline Timing Diagram

| Instruction | Cycle 1 | Cycle 2 | Cycle 3 | Cycle 4 | Cycle 5 | Cycle 6 | Cycle 7 |
|-------------|---------|---------|---------|---------|---------|---------|---------|
| **ADDI x10**| **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |
| **ADDI x11**|         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |
| **ADD x12** |         |         | **IF**  | **ID**  | **EX** ⚡| **MEM** | **WB**  |

**Legend**:
- ⚡ = **Forwarding from MEM/WB**: x10 forwarded to ADD x12 in cycle 5
- No EX/MEM forwarding available, so MEM/WB forwarding used

**Forwarding Details**:
- **Cycle 5**: ADD x12 uses x10 from MEM/WB stage (ForwardA = 01)
- x11 is also available from EX/MEM (ForwardB = 10)

---

## Load-Use Hazard (Pipeline Stall)

### Instruction Sequence
```assembly
LW   x20, 0(x1)     # Load memory[x1] to x20
ADD  x21, x20, x2   # x21 = x20 + x2 (uses x20 immediately - HAZARD!)
ADDI x22, x0, 5     # x22 = 5
```

### Pipeline Timing Diagram

| Instruction | Cycle 1 | Cycle 2 | Cycle 3 | Cycle 4 | Cycle 5 | Cycle 6 | Cycle 7 | Cycle 8 | Cycle 9 |
|-------------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
| **LW x20**  | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |         |
| **ADD x21** |         | **IF**  | **ID** ⏸️| **ID** ⏸️| **EX**  | **MEM** | **WB**  |         |         |
| **ADDI x22**|         |         | **IF**  | **IF** ⏸️| **ID**  | **EX**  | **MEM** | **WB**  |         |

**Legend**:
- ⏸️ = **Pipeline Stall**: ADD x21 held in ID stage, bubble inserted in ID/EX
- **Cycle 3**: Stall detected (load-use hazard)
- **Cycle 4**: Stall continues, ADD x21 still in ID stage
- **Cycle 5**: Stall released, ADD x21 moves to EX stage (x20 now available from MEM/WB)

**Stall Details**:
- **Hazard Detection**: ID/EX.MemRead AND (ID/EX.rd == IF/ID.rs1)
- **Stall Signals**: PCWrite = 0, IF/ID_Enable = 0, ID/EX_Flush = 1
- **Penalty**: 1 cycle stall

---

## Branch Taken (Pipeline Flush)

### Instruction Sequence
```assembly
ADDI x1, x0, 5      # x1 = 5
ADDI x2, x0, 5      # x2 = 5
BEQ  x1, x2, 8      # Branch if x1 == x2 (TAKEN!)
ADDI x3, x0, 10     # This instruction is FLUSHED
ADDI x4, x0, 20     # This instruction is FLUSHED
ADDI x5, x0, 30     # Target instruction (after branch)
```

### Pipeline Timing Diagram

| Instruction | Cycle 1 | Cycle 2 | Cycle 3 | Cycle 4 | Cycle 5 | Cycle 6 | Cycle 7 | Cycle 8 | Cycle 9 |
|-------------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
| **ADDI x1** | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |         |
| **ADDI x2** |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |
| **BEQ x1,x2**|        |         | **IF**  | **ID**  | **EX** 🔄| **MEM** | **WB**  |         |         |
| **ADDI x3** |         |         |         | **IF**  | **ID** 💥|         |         |         |         |
| **ADDI x4** |         |         |         |         | **IF** 💥|         |         |         |         |
| **ADDI x5** |         |         |         |         |         | **IF**  | **ID**  | **EX**  | **MEM** |

**Legend**:
- 🔄 = **Branch Evaluation**: Branch condition evaluated in EX stage
- 💥 = **Pipeline Flush**: Instructions flushed (IF/ID and ID/EX cleared)
- **Cycle 5**: Branch taken, PC jumps to target, IF/ID and ID/EX flushed
- **Cycle 6**: Fetch resumes at branch target (ADDI x5)

**Flush Details**:
- **Branch Evaluation**: BEQ condition evaluated in EX stage (cycle 5)
- **Flush Signals**: Flush_IF_ID = 1, Flush_ID_EX = 1
- **PC Update**: PC = branch_target (not PC+4)
- **Penalty**: 1 cycle flush (2 instructions discarded)

---

## Complex Scenario (Mixed Hazards)

### Instruction Sequence
```assembly
ADDI x10, x0, 10    # x10 = 10
ADDI x11, x0, 20    # x11 = 20
ADD  x12, x10, x11  # x12 = x10 + x11 (forwarding from MEM/WB)
LW   x13, 0(x12)    # Load memory[x12] to x13
ADD  x14, x13, x10  # x14 = x13 + x10 (load-use hazard - STALL!)
BEQ  x14, x0, 8     # Branch if x14 == 0 (not taken)
ADDI x15, x0, 100   # x15 = 100
```

### Pipeline Timing Diagram

| Instruction | Cycle 1 | Cycle 2 | Cycle 3 | Cycle 4 | Cycle 5 | Cycle 6 | Cycle 7 | Cycle 8 | Cycle 9 | Cycle 10| Cycle 11| Cycle 12|
|-------------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
| **ADDI x10**| **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |         |         |         |         |
| **ADDI x11**|         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |         |         |         |
| **ADD x12** |         |         | **IF**  | **ID**  | **EX** ⚡| **MEM** | **WB**  |         |         |         |         |         |
| **LW x13**  |         |         |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |         |         |         |         |
| **ADD x14** |         |         |         |         | **IF**  | **ID** ⏸️| **ID** ⏸️| **EX**  | **MEM** | **WB**  |         |         |
| **BEQ x14** |         |         |         |         |         | **IF** ⏸️| **IF** ⏸️| **ID**  | **EX** 🔄| **MEM** | **WB**  |         |
| **ADDI x15**|         |         |         |         |         |         |         | **IF**  | **ID**  | **EX**  | **MEM** | **WB**  |

**Legend**:
- ⚡ = **Forwarding**: x10 forwarded from MEM/WB to ADD x12 (cycle 5)
- ⏸️ = **Pipeline Stall**: Load-use hazard causes stall (cycles 6-7)
- 🔄 = **Branch Evaluation**: BEQ evaluated in EX stage (cycle 9, not taken)

**Hazard Details**:
- **Cycle 5**: ADD x12 uses x10 from MEM/WB (ForwardA = 01), x11 from EX/MEM (ForwardB = 10)
- **Cycle 6**: Load-use hazard detected (LW x13 → ADD x14), stall begins
- **Cycle 7**: Stall continues, ADD x14 held in ID stage
- **Cycle 8**: Stall released, ADD x14 moves to EX (x13 available from MEM/WB)
- **Cycle 9**: BEQ evaluated, not taken (no flush)

---

## Summary of Pipeline Behaviors

### Forwarding Scenarios

| Scenario | Forwarding Path | Cycle Penalty | Example |
|----------|----------------|---------------|---------|
| **EX/MEM → EX** | EX/MEM.ALU_Result → EX stage | 0 cycles | ADD x6, x5, x2 (x5 from previous ADD) |
| **MEM/WB → EX** | MEM/WB.WriteData → EX stage | 0 cycles | ADD x12, x10, x11 (x10 from 2 cycles ago) |
| **No Forwarding** | Register File → EX stage | 0 cycles | Normal operation |

### Stall Scenarios

| Scenario | Detection | Stall Cycles | Example |
|----------|-----------|--------------|---------|
| **Load-Use Hazard** | ID/EX.MemRead AND (ID/EX.rd == IF/ID.rs1/rs2) | 1 cycle | LW x20 → ADD x21, x20, x2 |

### Flush Scenarios

| Scenario | Detection | Flush Cycles | Instructions Flushed | Example |
|----------|-----------|--------------|----------------------|---------|
| **Branch Taken** | EX stage branch evaluation | 1 cycle | 2 instructions (IF/ID, ID/EX) | BEQ x1, x2, target |
| **Jump (JAL/JALR)** | ID/EX stage jump detection | 1 cycle | 2 instructions (IF/ID, ID/EX) | JAL x1, target |

---

## Performance Impact

### CPI Calculation

For the **Complex Scenario** example:
- **Total Cycles**: 12 cycles
- **Instructions Completed**: 7 instructions
- **CPI**: 12/7 = 1.71

**Breakdown**:
- Base CPI: 1.0 (ideal)
- Stall penalty: +0.14 (1 stall cycle / 7 instructions)
- Flush penalty: 0 (branch not taken)
- **Total CPI**: 1.14

### Optimization Opportunities

1. **Forwarding**: Eliminates most RAW hazards (0 cycle penalty)
2. **Load-Use Stalls**: Unavoidable, but can be minimized with instruction scheduling
3. **Branch Prediction**: Could reduce flush penalty for taken branches

---

## Key Takeaways

1. **Forwarding** resolves most data hazards without performance penalty
2. **Load-Use Hazards** require 1-cycle stalls (unavoidable)
3. **Branch Taken** causes 1-cycle flush (2 instructions discarded)
4. **Pipeline Efficiency** depends on instruction mix and hazard frequency
5. **CPI** ranges from 1.0 (ideal) to ~1.3-1.5 (typical with hazards)

---

**Note**: These timing diagrams assume:
- Single-cycle memory access
- No cache misses
- Branch prediction: always-not-taken
- Perfect forwarding (no timing violations)

For real-world performance, additional factors like cache misses, memory latency, and branch misprediction penalties would increase CPI further.

