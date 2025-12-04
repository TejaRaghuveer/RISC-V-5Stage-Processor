# Branch and Jump Control Unit

This document explains the RISC-V branch and jump control unit implementation and integration.

## Overview

The `branch_jump_control` module evaluates branch conditions, generates PC source selection signals, and produces pipeline flush signals for the RISC-V 5-stage pipeline processor.

## Module Interface

### Inputs

- **Control Signals:**
  - `Branch`: Branch instruction indicator (from control unit)
  - `Jump`: Jump instruction indicator (from control unit)
  - `funct3`: Function field [14:12] (branch type encoding)

- **ALU Results:**
  - `zero_flag`: ALU zero flag (from SUB operation for comparison)
  - `rs1_data`: Source register 1 data (forwarded)
  - `rs2_data`: Source register 2 data (forwarded)

- **Target Addresses:**
  - `branch_target`: Branch target address (PC + immediate)
  - `jump_target`: Jump target address (PC + immediate or rs1 + immediate)

### Outputs

- **`PCSrc`**: PC source select signal
  - `0`: Use PC + 4 (sequential execution)
  - `1`: Use branch/jump target (taken branch or jump)

- **`flush`**: Pipeline flush signal (active high)
  - `1`: Clear IF/ID and ID/EX registers (insert NOPs)
  - `0`: Normal pipeline operation

- **`branch_taken`**: Branch condition evaluation result
  - `1`: Branch condition is true
  - `0`: Branch condition is false or not a branch

## Branch Condition Evaluation

### Supported Branch Instructions

| Instruction | funct3 | Condition | Evaluation Method |
|------------|--------|-----------|-------------------|
| BEQ | 000 | rs1 == rs2 | `zero_flag == 1` |
| BNE | 001 | rs1 != rs2 | `zero_flag == 0` |
| BLT | 100 | rs1 < rs2 (signed) | `$signed(rs1) < $signed(rs2)` |
| BGE | 101 | rs1 >= rs2 (signed) | `$signed(rs1) >= $signed(rs2)` |
| BLTU | 110 | rs1 < rs2 (unsigned) | `rs1 < rs2` (unsigned) |
| BGEU | 111 | rs1 >= rs2 (unsigned) | `rs1 >= rs2` (unsigned) |

### Evaluation Logic

```systemverilog
// Signed comparison for BLT/BGE
logic signed_less_than;
assign signed_less_than = ($signed(rs1_data) < $signed(rs2_data));

// Unsigned comparison for BLTU/BGEU
logic unsigned_less_than;
assign unsigned_less_than = (rs1_data < rs2_data);

// Branch condition evaluation
case (funct3)
    3'b000: branch_taken = zero_flag;              // BEQ
    3'b001: branch_taken = !zero_flag;            // BNE
    3'b100: branch_taken = signed_less_than;      // BLT
    3'b101: branch_taken = !signed_less_than;    // BGE
    3'b110: branch_taken = unsigned_less_than;    // BLTU
    3'b111: branch_taken = !unsigned_less_than;   // BGEU
endcase
```

## Integration Example

### Integration with EX Stage

The branch/jump control unit can be integrated into the EX stage or used as a separate module:

```systemverilog
// In EX stage module
branch_jump_control #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(32)
) branch_jump_control_inst (
    // Control signals
    .Branch(Branch),
    .Jump(Jump),
    .funct3(funct3),
    
    // ALU results
    .zero_flag(zero_flag),
    .rs1_data(rs1_data_forwarded),
    .rs2_data(rs2_data_forwarded),
    
    // Target addresses
    .branch_target(branch_target),
    .jump_target(jump_target),
    
    // Outputs
    .PCSrc(pcsrc),
    .flush(flush),
    .branch_taken(branch_taken)
);
```

### Integration with IF Stage (PC Update)

```systemverilog
// In IF stage module
always_comb begin
    if (PCSrc) begin
        // Branch/Jump taken: Use target address
        PC_next = branch_jump_target;
    end else begin
        // Sequential execution: Use PC + 4
        PC_next = PC_reg + 32'd4;
    end
end
```

### Integration with Pipeline Registers (Flush)

```systemverilog
// In IF/ID and ID/EX pipeline registers
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset: Clear registers
        instruction_reg <= {DATA_WIDTH{1'b0}};
        // ... other registers
    end else if (flush) begin
        // Flush: Insert NOP (clear registers)
        instruction_reg <= 32'h00000013;  // NOP = ADDI x0, x0, 0
        // ... other registers cleared
    end else if (enable) begin
        // Normal operation: Update registers
        instruction_reg <= if_instruction;
        // ... other registers updated
    end
end
```

## Timing Diagram

```
Clock Cycle N:
┌─────────────────────────────────────────┐
│ EX Stage:                               │
│   - ALU computes rs1 - rs2              │
│   - zero_flag generated                 │
│   - Branch condition evaluated          │
│   - PCSrc = 1 (branch taken)           │
│   - flush = 1                           │
└─────────────────────────────────────────┘
                    ↓
Clock Cycle N+1:
┌─────────────────────────────────────────┐
│ IF Stage:                               │
│   - PC updated to branch_target         │
│   - New instruction fetched              │
│                                          │
│ IF/ID Register:                         │
│   - Flushed (NOP inserted)              │
│                                          │
│ ID/EX Register:                         │
│   - Flushed (NOP inserted)             │
└─────────────────────────────────────────┘
```

## Key Design Decisions

### 1. Signed vs Unsigned Comparison

- **Signed (BLT/BGE)**: Uses `$signed()` function for proper sign extension
- **Unsigned (BLTU/BGEU)**: Direct comparison without sign extension
- **Critical**: Incorrect comparison leads to wrong branch decisions

### 2. Zero Flag Usage

- **BEQ/BNE**: Uses ALU zero flag from SUB operation
- **ALU computes**: `rs1 - rs2`
- **zero_flag = 1**: Result is zero → `rs1 == rs2`
- **zero_flag = 0**: Result is non-zero → `rs1 != rs2`

### 3. PCSrc Signal

- **Active High**: `1` = use target, `0` = use PC+4
- **Combines**: Branch and jump decisions into single signal
- **Used in**: IF stage PC update logic

### 4. Flush Signal

- **Active High**: `1` = flush, `0` = normal
- **Clears**: IF/ID and ID/EX pipeline registers
- **Prevents**: Incorrect instructions from executing

## Testing Considerations

### Test Cases

1. **BEQ (Branch if Equal)**:
   ```assembly
   add x1, x2, x3    ; x1 = x2 + x3
   beq x1, x2, label ; Branch if x1 == x2
   ```
   - Test: `x1 == x2` → `branch_taken = 1`, `PCSrc = 1`
   - Test: `x1 != x2` → `branch_taken = 0`, `PCSrc = 0`

2. **BLT vs BLTU (Signed vs Unsigned)**:
   ```assembly
   addi x1, x0, -1   ; x1 = -1 (signed) = 0xFFFFFFFF (unsigned)
   addi x2, x0, 1    ; x2 = 1
   blt x1, x2, label ; Signed: -1 < 1 → taken
   bltu x1, x2, label; Unsigned: 0xFFFFFFFF < 1 → not taken
   ```
   - Test: Signed comparison treats -1 < 1
   - Test: Unsigned comparison treats 0xFFFFFFFF > 1

3. **JAL (Jump and Link)**:
   ```assembly
   jal x1, label     ; Always taken
   ```
   - Test: `Jump = 1` → `PCSrc = 1`, `flush = 1`

## Summary

The branch/jump control unit provides:
- **Comprehensive Branch Support**: All 6 RISC-V branch types
- **Correct Signed/Unsigned Handling**: Proper comparison logic
- **Pipeline Control**: PCSrc and flush signal generation
- **Modular Design**: Can be integrated or used standalone
- **Combinational Logic**: Fast evaluation, no clock needed

The module ensures correct branch and jump behavior while maintaining pipeline performance through proper flush control.

