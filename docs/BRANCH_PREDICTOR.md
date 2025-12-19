# Branch Predictor Design and Integration

This document describes the 1-bit branch predictor implementation for the RISC-V 5-stage pipeline processor.

## Overview

The branch predictor reduces pipeline flushes by speculatively predicting branch direction before the branch condition is evaluated. This improves performance by reducing the branch penalty from 1 cycle to 0 cycles for correctly predicted branches.

### Key Features

- **1-Bit Predictor**: Simple, low-overhead design
- **Branch History Table (BHT)**: 256-entry table indexed by PC bits
- **Speculative Fetch**: Fetches from predicted target in IF stage
- **Update on Resolution**: Updates BHT when branch resolves in EX stage
- **Misprediction Detection**: Detects and handles incorrect predictions

## Architecture

### Branch History Table (BHT)

```
┌─────────────────────────────────────┐
│  Branch History Table (256 entries) │
├─────────────────────────────────────┤
│  Index │ Prediction │
├────────┼────────────┤
│   0    │     0      │  Predict Not Taken
│   1    │     1      │  Predict Taken
│   2    │     0      │  Predict Not Taken
│  ...   │    ...     │
│  255   │     1      │  Predict Taken
└─────────────────────────────────────┘
```

**Indexing**:
- Index = `PC[BHT_ADDR_WIDTH+1:2]`
- For 256-entry BHT: Index = `PC[9:2]` (8 bits)
- Uses lower PC bits to capture local branch patterns

**Prediction States**:
- `0`: Predict Not Taken (NT)
- `1`: Predict Taken (T)

### Prediction Flow

```
IF Stage:
  PC → BHT Index → BHT[Index] → Prediction
  If Prediction = Taken:
    Fetch from PC + immediate
  Else:
    Fetch from PC + 4

EX Stage:
  Branch Resolves → Compare Prediction vs Actual
  If Misprediction:
    Flush pipeline, fetch from correct target
  Update BHT[Index] = Actual Outcome
```

## Module Interface

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ADDR_WIDTH` | 32 | Address width (bits) |
| `BHT_SIZE` | 256 | Branch History Table size (entries) |
| `BHT_ADDR_WIDTH` | 8 | BHT address width (log2(BHT_SIZE)) |

### Inputs

**Prediction Interface (IF Stage)**:
- `if_PC`: Current PC (for prediction lookup)
- `is_branch`: Branch instruction indicator (from ID stage)
- `branch_immediate`: Branch immediate value (for target calculation)

**Update Interface (EX Stage)**:
- `update_enable`: Update enable (branch instruction resolved)
- `ex_PC`: PC of branch instruction (for BHT update)
- `actual_taken`: Actual branch outcome (1 = taken, 0 = not taken)
- `actual_target`: Actual branch target address

### Outputs

**Prediction Interface (IF Stage)**:
- `predict_taken`: Prediction (1 = taken, 0 = not taken)
- `predict_target`: Predicted target address (PC + immediate)

**Internal**:
- `misprediction`: Misprediction indicator (for statistics)

## Integration into Processor

### 1. IF Stage Integration

**Modifications**:
- Add branch predictor instance
- Use prediction to speculatively fetch from predicted target
- Update PC based on prediction (if prediction = taken, use predict_target)

**Code Changes**:
```systemverilog
// In if_stage.sv

// Branch predictor instance
branch_predictor #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .BHT_SIZE(256),
    .BHT_ADDR_WIDTH(8)
) branch_predictor_inst (
    .clk(clk),
    .rst_n(rst_n),
    .if_PC(PC_reg),
    .predict_taken(predict_taken),
    .predict_target(predict_target),
    .update_enable(update_enable),
    .ex_PC(ex_PC),
    .actual_taken(actual_taken),
    .actual_target(actual_target),
    .is_branch(is_branch),
    .branch_immediate(branch_immediate)
);

// PC update logic (modified)
always_comb begin
    if (stall) begin
        PC_next = PC_reg;
    end else if (predict_taken && is_branch) begin
        // Speculative fetch from predicted target
        PC_next = predict_target;
    end else if (branch_taken) begin
        // Branch resolved: Use actual target (misprediction correction)
        PC_next = branch_target;
    end else begin
        // Normal operation: Sequential
        PC_next = PC_inc;
    end
end
```

### 2. ID Stage Integration

**Modifications**:
- Detect branch instructions early
- Calculate branch immediate for prediction
- Pass branch information to IF stage

**Code Changes**:
```systemverilog
// In id_stage.sv

// Branch detection
assign is_branch = (opcode == 7'b1100011);  // B-type instruction

// Branch immediate calculation (for prediction)
logic [ADDR_WIDTH-1:0] branch_immediate;
assign branch_immediate = {{20{instruction[31]}}, 
                           instruction[7], 
                           instruction[30:25], 
                           instruction[11:8], 
                           1'b0};  // Sign-extended, left-shifted by 1
```

### 3. EX Stage Integration

**Modifications**:
- Compare prediction with actual outcome
- Generate update_enable signal
- Provide actual branch outcome and target

**Code Changes**:
```systemverilog
// In ex_stage.sv or riscv_pipeline.sv

// Branch resolution signals
logic update_enable;
assign update_enable = ex_Branch;  // Update when branch resolves

// Actual branch outcome (from branch_jump_control)
logic actual_taken;
assign actual_taken = ex_branch_taken;

// Actual branch target
logic [ADDR_WIDTH-1:0] actual_target;
assign actual_target = ex_branch_target;

// PC of branch instruction (from ID/EX register)
logic [ADDR_WIDTH-1:0] ex_PC;
// (Already available from ID/EX register)
```

### 4. Pipeline Flush Control

**Misprediction Handling**:
- When prediction != actual outcome: Flush pipeline
- Fetch from correct target (actual_target)
- Update BHT with actual outcome

**Code Changes**:
```systemverilog
// In riscv_pipeline.sv

// Misprediction detection
logic misprediction;
assign misprediction = (ex_Branch && 
                       (predict_taken_reg != ex_branch_taken));

// Pipeline flush (modified)
assign pipeline_flush_internal = 
    (ex_Branch && ex_branch_taken && !predict_taken_reg) ||  // Mispredicted: predicted NT, actual T
    (ex_Branch && !ex_branch_taken && predict_taken_reg) ||  // Mispredicted: predicted T, actual NT
    (ex_Jump) ||                                              // Jumps always flush
    (pipeline_flush);                                         // External flush
```

## Performance Analysis

### Expected Performance Improvement

**Without Branch Predictor**:
- All taken branches: 1-cycle penalty (flush)
- Branch penalty = 1 cycle per taken branch
- Flush rate = (taken branches) / (total branches)

**With 1-Bit Branch Predictor**:
- Correctly predicted branches: 0-cycle penalty
- Mispredicted branches: 1-cycle penalty (flush)
- Branch penalty = (misprediction rate) × 1 cycle

### Performance Metrics

**Prediction Accuracy**:
- Depends on branch behavior patterns
- Consistent branches (loops): High accuracy (~95%+)
- Alternating branches: Lower accuracy (~50%)
- Overall: Typically 70-85% accuracy for 1-bit predictor

**Flush Rate Reduction**:
- Without predictor: Flush rate = taken branch rate
- With predictor: Flush rate = misprediction rate
- Typical reduction: 50-70% reduction in flush rate

**CPI Improvement**:
- Baseline CPI: ~1.2-1.5 (with branch penalties)
- With predictor: ~1.1-1.3 (reduced branch penalties)
- Improvement: ~0.1-0.2 CPI reduction

### Example Calculation

**Scenario**:
- 1000 instructions executed
- 200 branch instructions (20% branch frequency)
- 120 taken branches (60% taken rate)
- 1-bit predictor: 85% accuracy

**Without Predictor**:
- Flushes = 120 (all taken branches)
- CPI = 1.0 + (120/1000) = 1.12

**With Predictor**:
- Correct predictions = 120 × 0.85 = 102
- Mispredictions = 120 × 0.15 = 18
- Flushes = 18 (only mispredictions)
- CPI = 1.0 + (18/1000) = 1.018

**Improvement**: CPI reduction of 0.102 (9.1% improvement)

## Limitations and Future Improvements

### Current Limitations

1. **1-Bit Predictor**:
   - Mispredicts on alternating patterns (T-NT-T-NT)
   - No hysteresis (immediate update)
   - Solution: Upgrade to 2-bit saturating counter

2. **No Correlation**:
   - Doesn't consider branch history
   - Solution: Add branch history register (BHR)

3. **Aliasing**:
   - Multiple branches may map to same BHT entry
   - Solution: Increase BHT size or use better indexing

4. **No Target Prediction**:
   - Only predicts direction, not target
   - Solution: Add Branch Target Buffer (BTB)

### Future Improvements

1. **2-Bit Saturating Counter**:
   - States: Strongly Not Taken, Weakly Not Taken, Weakly Taken, Strongly Taken
   - Better accuracy for alternating patterns
   - Requires 2 bits per BHT entry

2. **Branch Target Buffer (BTB)**:
   - Stores predicted target addresses
   - Reduces target calculation delay
   - Useful for indirect branches

3. **Return Address Stack (RAS)**:
   - Predicts return addresses for function calls
   - Improves JAL/JALR handling
   - Reduces mispredictions on returns

4. **Hybrid Predictor**:
   - Combines multiple predictors
   - Selects best predictor per branch
   - Higher accuracy but more complex

## Testing

### Test Cases

1. **Loop Branches**:
   - Test backward branches (loops)
   - Should predict taken consistently
   - Verify high accuracy

2. **Conditional Branches**:
   - Test forward branches (if-else)
   - May have alternating patterns
   - Verify reasonable accuracy

3. **Misprediction Handling**:
   - Test misprediction detection
   - Verify pipeline flush on misprediction
   - Verify BHT update after resolution

4. **BHT Aliasing**:
   - Test multiple branches mapping to same entry
   - Verify correct behavior
   - Measure accuracy impact

### Verification

1. **Functional Verification**:
   - Run test programs with branches
   - Compare prediction vs actual outcome
   - Verify BHT updates correctly

2. **Performance Verification**:
   - Measure flush rate reduction
   - Calculate CPI improvement
   - Compare with baseline (no predictor)

3. **Corner Cases**:
   - BHT overflow (wraparound)
   - Concurrent updates
   - Reset behavior

## Conclusion

The 1-bit branch predictor provides a simple, low-overhead solution for reducing branch penalties in the RISC-V processor. While it has limitations, it offers significant performance improvements for typical workloads with consistent branch behavior patterns.

For production use, consider upgrading to a 2-bit saturating counter predictor or adding a Branch Target Buffer (BTB) for even better performance.

