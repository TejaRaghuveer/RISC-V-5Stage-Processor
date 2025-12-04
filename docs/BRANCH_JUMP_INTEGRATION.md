# Branch/Jump Control Integration Guide

This document explains how the branch/jump control unit is integrated into the RISC-V 5-stage pipeline processor top-level module.

## Overview

The branch/jump control unit evaluates branch conditions, generates PC source selection signals, and produces pipeline flush signals. It is integrated into the pipeline after the EX stage, with signals registered in the EX/MEM register for proper timing.

## Integration Points

### 1. Signal Declarations

```systemverilog
/**
 * Branch/Jump Control Signals
 */
logic                              ex_PCSrc;          // PC source select from EX stage (combinational)
logic                              ex_branch_flush;   // Branch/jump flush from EX stage (combinational)
logic                              mem_PCSrc;         // PC source select from EX/MEM register (registered)
logic                              mem_branch_flush;  // Branch/jump flush from EX/MEM register (registered)

/**
 * Signals for Branch/Jump Control Unit
 */
logic [DATA_WIDTH-1:0]             ex_rs1_data_forwarded; // Forwarded rs1 data for branch control
logic [DATA_WIDTH-1:0]             ex_rs2_data_forwarded; // Forwarded rs2 data for branch control
```

### 2. EX Stage Modifications

Added forwarded data outputs to EX stage for branch condition evaluation:

```systemverilog
// In ex_stage.sv
output logic [DATA_WIDTH-1:0]       rs1_data_forwarded_out, // Forwarded rs1 data
output logic [DATA_WIDTH-1:0]       rs2_data_forwarded_out  // Forwarded rs2 data

// Assignment
assign rs1_data_forwarded_out = rs1_data_forwarded;
assign rs2_data_forwarded_out = rs2_data_forwarded;
```

### 3. Branch/Jump Control Unit Instantiation

```systemverilog
/**
 * Branch/Jump Control Unit Module
 * 
 * Evaluates branch conditions and generates PC source selection and flush signals.
 * Uses combinational signals from EX stage for immediate evaluation.
 */
branch_jump_control #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) branch_jump_control_inst (
    // Control Signals from ID/EX Register
    .Branch(ex_Branch),                  // Branch instruction indicator
    .Jump(ex_Jump),                      // Jump instruction indicator
    .funct3(ex_funct3),                  // Function field [14:12] (branch type)
    
    // ALU Results for Branch Condition Evaluation
    .zero_flag(ex_zero_flag),            // ALU zero flag (from SUB operation)
    .rs1_data(ex_rs1_data_forwarded),   // Forwarded rs1 data (for comparison)
    .rs2_data(ex_rs2_data_forwarded),   // Forwarded rs2 data (for comparison)
    
    // Target Addresses from EX Stage
    .branch_target(ex_branch_target),    // Branch target address (PC + immediate)
    .jump_target(ex_jump_target),        // Jump target address (PC + immediate or rs1 + immediate)
    
    // Outputs for Pipeline Control
    .PCSrc(ex_PCSrc),                    // PC source select (0 = PC+4, 1 = target)
    .flush(ex_branch_flush),             // Pipeline flush signal (active high)
    .branch_taken()                      // Branch condition evaluation (not used, ex_branch_taken used instead)
);
```

**Key Connections:**
- **Inputs**: Control signals, ALU results, and target addresses from EX stage
- **Outputs**: `PCSrc` (PC source selection) and `flush` (pipeline flush)

### 4. EX/MEM Register Updates

Added PCSrc and flush signals to EX/MEM register:

```systemverilog
// Inputs
input  logic                        ex_PCSrc,         // PC source select from branch/jump control
input  logic                        ex_branch_flush   // Branch/jump flush signal

// Outputs
output logic                        mem_PCSrc,        // PC source select (registered)
output logic                        mem_branch_flush  // Branch/jump flush (registered)

// Register storage
logic PCSrc_reg;
logic branch_flush_reg;

// Register update
PCSrc_reg <= ex_PCSrc;
branch_flush_reg <= ex_branch_flush;

// Output assignment
assign mem_PCSrc = PCSrc_reg;
assign mem_branch_flush = branch_flush_reg;
```

**Purpose**: Register PCSrc and flush signals for proper timing alignment with PC update.

### 5. PC Source Selection Logic

```systemverilog
/**
 * Branch/Jump Target Multiplexer and PC Source Selection
 * 
 * Selects target address for PC update using REGISTERED signals from EX/MEM.
 */
always_comb begin
    // Use registered signals from EX/MEM register for proper timing
    if (mem_PCSrc) begin
        // Branch/Jump taken: Select target address
        if (mem_Jump) begin
            // Jump instruction: Use jump target (always taken)
            branch_target = mem_jump_target;
        end else begin
            // Branch instruction: Use branch target (if taken)
            branch_target = mem_branch_target;
        end
        branch_taken = 1'b1;
    end else begin
        // No branch/jump or not taken: Sequential execution (IF stage handles PC+4)
        branch_target = {ADDR_WIDTH{1'b0}};  // Don't care
        branch_taken = 1'b0;
    end
end
```

**Behavior:**
- `mem_PCSrc = 1`: Branch/jump taken → use target address
- `mem_PCSrc = 0`: Sequential execution → use PC+4 (handled in IF stage)

### 6. IF Stage Connection

```systemverilog
if_stage if_stage_inst (
    // ...
    .branch_target(branch_target),      // Branch/jump target address (selected)
    .branch_taken(mem_PCSrc),           // PC source select (1 = branch/jump taken, 0 = sequential)
    // ...
);
```

**PC Update Logic in IF Stage:**
```systemverilog
always_comb begin
    if (stall) begin
        PC_next = PC_reg;              // Stall: Hold PC
    end else if (branch_taken) begin
        PC_next = branch_target;       // Branch/Jump: Use target
    end else begin
        PC_next = PC_inc;              // Sequential: PC + 4
    end
end
```

### 7. Pipeline Flush Signal Generation

```systemverilog
/**
 * Pipeline Flush Signal Generation
 * 
 * Generate flush signal when branch/jump is taken to flush incorrect
 * instructions that were fetched speculatively.
 */
assign pipeline_flush_internal = mem_branch_flush || pipeline_flush;
```

**Flush Sources:**
- `mem_branch_flush`: Branch/jump taken (from branch/jump control, registered)
- `pipeline_flush`: External flush (for exceptions, interrupts)

### 8. Pipeline Register Flush Connections

```systemverilog
// IF/ID Register
if_id_reg if_id_reg_inst (
    // ...
    .flush(pipeline_flush_internal),    // Flush on branch/jump taken or external flush
    // ...
);

// ID/EX Register
id_ex_reg id_ex_reg_inst (
    // ...
    .flush(pipeline_flush_internal || hazard_id_ex_flush), // Flush: branch/jump + external + hazard
    // ...
);
```

**Flush Behavior:**
- **IF/ID Register**: Cleared when branch/jump taken or external flush
- **ID/EX Register**: Cleared when branch/jump taken, external flush, or hazard NOP insertion

## Signal Flow Diagram

```
EX Stage (Combinational):
┌─────────────────────────────────────────┐
│ Branch/Jump Control Unit                │
│ - Evaluates branch conditions           │
│ - Generates ex_PCSrc                    │
│ - Generates ex_branch_flush             │
└──────────────┬──────────────────────────┘
               │
               ↓
EX/MEM Register (Registered):
┌─────────────────────────────────────────┐
│ - Stores ex_PCSrc → mem_PCSrc           │
│ - Stores ex_branch_flush → mem_branch_flush │
│ - Stores branch_target, jump_target     │
└──────────────┬──────────────────────────┘
               │
               ↓
PC Selection Logic:
┌─────────────────────────────────────────┐
│ - If mem_PCSrc = 1: Use target address  │
│ - If mem_PCSrc = 0: Use PC+4           │
└──────────────┬──────────────────────────┘
               │
               ↓
IF Stage:
┌─────────────────────────────────────────┐
│ - Updates PC based on branch_taken      │
│ - Fetches new instruction                │
└─────────────────────────────────────────┘
```

## Timing Considerations

### Pipeline Timing:

1. **Cycle N (EX Stage)**:
   - Branch/jump control evaluates condition (combinational)
   - Generates `ex_PCSrc` and `ex_branch_flush`
   - Signals available immediately

2. **Cycle N+1 (EX/MEM Register)**:
   - `ex_PCSrc` → `mem_PCSrc` (registered)
   - `ex_branch_flush` → `mem_branch_flush` (registered)
   - Target addresses stored

3. **Cycle N+1 (PC Update)**:
   - IF stage uses `mem_PCSrc` for PC selection
   - PC updated to target address if `mem_PCSrc = 1`
   - Pipeline flush clears IF/ID and ID/EX registers

### Why Register PCSrc and Flush?

- **Timing Alignment**: PC update occurs in IF stage, which needs registered signals
- **Pipeline Consistency**: All control signals should be registered for proper pipeline operation
- **Hazard Avoidance**: Prevents combinational paths from EX stage directly to IF stage

## Key Design Decisions

### 1. Combinational Evaluation, Registered Control

- **Branch condition evaluation**: Combinational (immediate)
- **Control signals**: Registered (proper timing)
- **Result**: Fast evaluation with correct pipeline timing

### 2. Forwarded Data Usage

- **rs1_data_forwarded**: Used for branch condition evaluation
- **rs2_data_forwarded**: Used for branch condition evaluation
- **Purpose**: Ensures correct data used even with data hazards

### 3. Separate Flush Signals

- **mem_branch_flush**: Branch/jump flush (registered)
- **hazard_id_ex_flush**: Hazard flush (combinational)
- **pipeline_flush**: External flush
- **Result**: Modular flush control, each source independent

### 4. PCSrc Signal

- **Single signal**: Combines branch and jump decisions
- **Registered**: Proper timing with PC update
- **Simple logic**: Easy to understand and verify

## Testing Considerations

### Test Cases:

1. **BEQ Branch Taken**:
   ```assembly
   beq x1, x2, label  ; x1 == x2
   ```
   - Verify: `ex_PCSrc = 1`, `mem_PCSrc = 1`, PC updates to target

2. **BNE Branch Not Taken**:
   ```assembly
   bne x1, x2, label  ; x1 != x2
   ```
   - Verify: `ex_PCSrc = 0`, `mem_PCSrc = 0`, PC increments normally

3. **JAL Jump**:
   ```assembly
   jal x1, label       ; Always taken
   ```
   - Verify: `ex_PCSrc = 1`, `mem_PCSrc = 1`, PC updates to target

4. **Pipeline Flush**:
   - Verify: IF/ID and ID/EX registers cleared when branch/jump taken
   - Verify: NOP instructions inserted in pipeline

## Summary

The branch/jump control integration provides:
- **Centralized Control**: All branch/jump logic in one module
- **Proper Timing**: Registered signals for correct pipeline operation
- **Forwarding Support**: Uses forwarded data for accurate condition evaluation
- **Modular Design**: Clean separation of concerns
- **Pipeline Flush**: Automatic flush on branch/jump taken

The integration ensures correct branch and jump behavior while maintaining pipeline performance and timing alignment with PC updates.

