# Branch Predictor Integration Guide

This guide provides step-by-step instructions for integrating the 1-bit branch predictor into the RISC-V processor pipeline.

## Overview

The branch predictor reduces pipeline flushes by predicting branch direction before the branch condition is evaluated. This integration guide shows how to modify the processor to use branch prediction.

## Integration Steps

### Step 1: Add Branch Predictor Instance

Add the branch predictor module to the main pipeline module (`riscv_pipeline.sv`):

```systemverilog
// In riscv_pipeline.sv, add after other module instantiations

// Branch Predictor Instance
branch_predictor #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .BHT_SIZE(256),
    .BHT_ADDR_WIDTH(8)
) branch_predictor_inst (
    .clk(clk),
    .rst_n(rst_n),
    // Prediction Interface (IF Stage)
    .if_PC(if_PC),
    .predict_taken(predict_taken),
    .predict_target(predict_target),
    .is_branch(id_Branch),  // From ID stage (needs to be passed to IF)
    .branch_immediate(id_branch_immediate),  // Calculated in ID stage
    // Update Interface (EX Stage)
    .update_enable(ex_Branch),  // Update when branch resolves
    .ex_PC(ex_PC),
    .actual_taken(ex_branch_taken),
    .actual_target(ex_branch_target)
);
```

### Step 2: Modify IF Stage

Modify `if_stage.sv` to use branch prediction:

```systemverilog
// Add inputs to if_stage module
module if_stage #(...) (
    // ... existing inputs ...
    
    // Branch Prediction Inputs
    input  logic                        predict_taken,     // Prediction from branch predictor
    input  logic [ADDR_WIDTH-1:0]      predict_target,    // Predicted target address
    
    // ... existing outputs ...
);

// Modify PC update logic
always_comb begin
    if (stall) begin
        PC_next = PC_reg;
    end else if (predict_taken) begin
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

### Step 3: Modify ID Stage

Modify `id_stage.sv` to calculate branch immediate early:

```systemverilog
// Add output for branch immediate
output logic [ADDR_WIDTH-1:0] id_branch_immediate,

// Calculate branch immediate (for prediction)
logic [ADDR_WIDTH-1:0] branch_immediate_calc;
always_comb begin
    if (id_Branch) begin
        // B-type immediate: imm[12|10:5|4:1|11]
        branch_immediate_calc = {{20{instruction[31]}}, 
                                 instruction[7], 
                                 instruction[30:25], 
                                 instruction[11:8], 
                                 1'b0};  // Sign-extended, left-shifted by 1
    end else begin
        branch_immediate_calc = {ADDR_WIDTH{1'b0}};
    end
end

assign id_branch_immediate = branch_immediate_calc;
```

### Step 4: Modify Pipeline Flush Logic

Update flush logic in `riscv_pipeline.sv` to handle mispredictions:

```systemverilog
// Add misprediction signal from branch predictor
logic misprediction;
assign misprediction = (ex_Branch && 
                       (predict_taken_reg != ex_branch_taken));

// Store prediction when branch is in IF stage
logic predict_taken_reg;
always_ff @(posedge clk) begin
    if (id_Branch) begin
        predict_taken_reg <= predict_taken;
    end
end

// Modified flush logic
assign pipeline_flush_internal = 
    misprediction ||                    // Branch misprediction
    (ex_Jump) ||                        // Jumps always flush
    (hazard_id_ex_flush) ||             // Hazard flush
    (pipeline_flush);                   // External flush
```

### Step 5: Pass Branch Information Through Pipeline

Add signals to pipeline registers to pass branch information:

**IF/ID Register** (`if_id_reg.sv`):
```systemverilog
// Add to IF/ID register
input  logic                        if_predict_taken,
output logic                        id_predict_taken,
```

**ID/EX Register** (`id_ex_reg.sv`):
```systemverilog
// Add to ID/EX register
input  logic                        id_predict_taken,
output logic                        ex_predict_taken,
```

## Complete Integration Example

Here's a simplified example showing the complete integration:

```systemverilog
module riscv_pipeline #(...) (
    // ... ports ...
);

    // Branch Predictor Signals
    logic predict_taken;
    logic [ADDR_WIDTH-1:0] predict_target;
    logic [ADDR_WIDTH-1:0] id_branch_immediate;
    logic predict_taken_reg;
    
    // Branch Predictor Instance
    branch_predictor #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .BHT_SIZE(256),
        .BHT_ADDR_WIDTH(8)
    ) branch_predictor_inst (
        .clk(clk),
        .rst_n(rst_n),
        .if_PC(if_PC),
        .predict_taken(predict_taken),
        .predict_target(predict_target),
        .is_branch(id_Branch),
        .branch_immediate(id_branch_immediate),
        .update_enable(ex_Branch),
        .ex_PC(ex_PC),
        .actual_taken(ex_branch_taken),
        .actual_target(ex_branch_target)
    );
    
    // IF Stage (modified)
    if_stage #(...) if_stage_inst (
        .clk(clk),
        .rst_n(rst_n),
        .branch_target(branch_target),
        .branch_taken(mem_PCSrc),
        .predict_taken(predict_taken),
        .predict_target(predict_target),
        .stall(pipeline_stall_internal),
        .flush(pipeline_flush_internal),
        // ... other ports ...
    );
    
    // ID Stage (modified)
    id_stage #(...) id_stage_inst (
        // ... existing ports ...
        .branch_immediate(id_branch_immediate),
        // ... other ports ...
    );
    
    // Store prediction for misprediction detection
    always_ff @(posedge clk) begin
        if (id_Branch) begin
            predict_taken_reg <= predict_taken;
        end
    end
    
    // Misprediction detection
    logic misprediction;
    assign misprediction = (ex_Branch && 
                           (predict_taken_reg != ex_branch_taken));
    
    // Pipeline flush (modified)
    assign pipeline_flush_internal = 
        misprediction ||
        (ex_Jump) ||
        (hazard_id_ex_flush) ||
        (pipeline_flush);
    
endmodule
```

## Performance Impact

### Expected Improvements

**Baseline (No Predictor)**:
- All taken branches: 1-cycle penalty
- Flush rate = taken branch rate
- CPI ≈ 1.0 + (branch_frequency × taken_rate)

**With 1-Bit Predictor**:
- Correctly predicted branches: 0-cycle penalty
- Mispredicted branches: 1-cycle penalty
- Flush rate = misprediction rate
- CPI ≈ 1.0 + (branch_frequency × misprediction_rate)

### Example Calculation

**Scenario**:
- 1000 instructions
- 200 branches (20% frequency)
- 120 taken (60% taken rate)
- 85% prediction accuracy

**Without Predictor**:
- Flushes = 120
- CPI = 1.0 + (120/1000) = 1.12

**With Predictor**:
- Correct predictions = 102
- Mispredictions = 18
- Flushes = 18
- CPI = 1.0 + (18/1000) = 1.018

**Improvement**: 9.1% CPI reduction

## Testing

### Test Cases

1. **Loop Branches**:
   ```assembly
   loop:
       ADD x1, x1, x2
       BNE x1, x3, loop  # Should predict taken consistently
   ```

2. **Conditional Branches**:
   ```assembly
       BEQ x1, x2, label1
       # ... code ...
   label1:
   ```

3. **Misprediction Handling**:
   - Verify pipeline flushes on misprediction
   - Verify correct target is fetched
   - Verify BHT is updated correctly

### Verification Checklist

- [ ] Predictor makes predictions in IF stage
- [ ] Predictions are used for speculative fetch
- [ ] BHT updates correctly on branch resolution
- [ ] Mispredictions are detected correctly
- [ ] Pipeline flushes on misprediction
- [ ] Correct target is fetched after misprediction
- [ ] Performance improvement is measurable

## Troubleshooting

### Common Issues

1. **Prediction Not Used**:
   - Check if `predict_taken` is connected correctly
   - Verify `is_branch` signal is set correctly
   - Check PC indexing for BHT

2. **BHT Not Updating**:
   - Verify `update_enable` is asserted when branch resolves
   - Check `ex_PC` matches branch PC
   - Verify reset is not active

3. **Misprediction Not Detected**:
   - Check prediction storage logic
   - Verify PC comparison logic
   - Ensure timing is correct

4. **Performance Not Improving**:
   - Check prediction accuracy
   - Verify flush rate reduction
   - Measure CPI improvement

## Conclusion

The branch predictor integration provides significant performance improvements for workloads with consistent branch behavior. Follow the integration steps carefully and verify each component works correctly before moving to the next step.

For production use, consider upgrading to a 2-bit saturating counter predictor for even better accuracy.

