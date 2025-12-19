/**
 * @file branch_predictor.sv
 * @brief 1-Bit Branch Predictor with Branch History Table (BHT)
 * 
 * This module implements a simple 1-bit branch predictor for the RISC-V processor.
 * It uses a Branch History Table (BHT) indexed by PC bits to predict branch direction.
 * 
 * @details
 * Branch Prediction Strategy:
 * - 1-bit predictor: Stores last outcome (taken/not taken) for each branch
 * - BHT indexed by lower bits of PC (configurable size)
 * - Prediction made in IF stage (speculative)
 * - Update on branch resolution in EX stage
 * 
 * Prediction States:
 * - 0: Predict Not Taken (NT)
 * - 1: Predict Taken (T)
 * 
 * Update Policy:
 * - On branch resolution: Update BHT entry with actual outcome
 * - Simple update: BHT[PC_index] = actual_outcome
 * 
 * Performance:
 * - Reduces flush rate for branches with consistent behavior
 * - Works well for loops and frequently executed branches
 * - Simple implementation with minimal hardware overhead
 * 
 * Limitations:
 * - 1-bit predictor: Mispredicts on alternating patterns (T-NT-T-NT)
 * - No correlation: Doesn't consider branch history
 * - Aliasing: Multiple branches may map to same BHT entry
 */

module branch_predictor #(
    parameter ADDR_WIDTH = 32,            // Address width (32 bits)
    parameter BHT_SIZE = 256,             // Branch History Table size (entries)
    parameter BHT_ADDR_WIDTH = 8          // BHT address width (log2(BHT_SIZE))
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // Prediction Interface (IF Stage)
    input  logic [ADDR_WIDTH-1:0]       if_PC,            // Current PC (for prediction lookup)
    output logic                        predict_taken,     // Prediction: 1 = taken, 0 = not taken
    output logic [ADDR_WIDTH-1:0]      predict_target,    // Predicted target address (PC + immediate)
    
    // Update Interface (EX Stage)
    input  logic                        update_enable,     // Update enable (branch instruction resolved)
    input  logic [ADDR_WIDTH-1:0]       ex_PC,             // PC of branch instruction (for BHT update)
    input  logic                        actual_taken,      // Actual branch outcome (1 = taken, 0 = not taken)
    input  logic [ADDR_WIDTH-1:0]      actual_target,     // Actual branch target address
    
    // Branch Instruction Information (from ID stage)
    input  logic                        is_branch,         // Is this a branch instruction?
    input  logic [ADDR_WIDTH-1:0]       branch_immediate   // Branch immediate (for target calculation)
);

    // ============================================
    // Branch History Table (BHT)
    // ============================================
    
    /**
     * Branch History Table
     * 
     * Stores 1-bit prediction for each branch:
     * - 0: Predict Not Taken
     * - 1: Predict Taken
     * 
     * Indexing:
     * - Indexed by lower BHT_ADDR_WIDTH bits of PC
     * - BHT[index] = prediction for branches at PC[31:2] & mask
     * 
     * Size:
     * - Default: 256 entries (8-bit index)
     * - Can be parameterized for different sizes
     * - Larger BHT = better accuracy but more hardware
     */
    logic [BHT_SIZE-1:0] bht;  // Branch History Table (1 bit per entry)
    
    /**
     * Prediction Storage for Misprediction Detection
     * 
     * Stores prediction made in IF stage for comparison when branch resolves.
     * This allows detection of mispredictions.
     */
    logic predicted_taken_stored;  // Stored prediction from IF stage
    logic [ADDR_WIDTH-1:0] predicted_PC_stored;  // Stored PC for comparison
    
    /**
     * BHT Index Calculation
     * 
     * Uses lower BHT_ADDR_WIDTH bits of PC to index into BHT.
     * PC is word-aligned (lower 2 bits are 00), so we use PC[BHT_ADDR_WIDTH+1:2].
     * 
     * Example (BHT_ADDR_WIDTH = 8):
     * - PC = 0x00001004
     * - Index = PC[9:2] = 0x04 = 4
     * - BHT[4] = prediction for this branch
     */
    logic [BHT_ADDR_WIDTH-1:0] bht_index_predict;  // BHT index for prediction (IF stage)
    logic [BHT_ADDR_WIDTH-1:0] bht_index_update;    // BHT index for update (EX stage)
    
    // Calculate BHT indices
    assign bht_index_predict = if_PC[BHT_ADDR_WIDTH+1:2];
    assign bht_index_update = ex_PC[BHT_ADDR_WIDTH+1:2];
    
    // ============================================
    // Prediction Logic (IF Stage)
    // ============================================
    
    /**
     * Branch Prediction
     * 
     * Predicts branch direction based on BHT entry:
     * - Read BHT[PC_index] to get prediction
     * - If BHT entry = 1: Predict Taken
     * - If BHT entry = 0: Predict Not Taken
     * 
     * Timing:
     * - Combinational: Prediction available in same cycle as PC
     * - Used to speculatively fetch from predicted target
     * - If prediction is wrong, pipeline will be flushed
     */
    always_comb begin
        if (is_branch) begin
            // Branch instruction: Use BHT prediction
            predict_taken = bht[bht_index_predict];
        end else begin
            // Not a branch: Predict not taken (sequential execution)
            predict_taken = 1'b0;
        end
    end
    
    /**
     * Predicted Target Address Calculation
     * 
     * Calculates predicted branch target: PC + immediate
     * This is used for speculative instruction fetch when prediction = taken.
     * 
     * Note: Immediate is sign-extended and left-shifted by 1 bit for branches.
     * For now, we assume branch_immediate is already calculated correctly.
     */
    always_comb begin
        if (predict_taken && is_branch) begin
            // Prediction = taken: Use branch target
            predict_target = if_PC + branch_immediate;
        end else begin
            // Prediction = not taken: Sequential (PC + 4, handled in IF stage)
            predict_target = {ADDR_WIDTH{1'b0}};  // Don't care
        end
    end
    
    // ============================================
    // Update Logic (EX Stage)
    // ============================================
    
    /**
     * BHT Update on Branch Resolution
     * 
     * Updates BHT entry when branch resolves in EX stage:
     * - Write actual outcome to BHT[PC_index]
     * - Simple 1-bit update: BHT[index] = actual_taken
     * 
     * Update Timing:
     * - Synchronous: Updates on clock edge
     * - Only updates when update_enable = 1 (branch instruction resolved)
     * 
     * Update Policy:
     * - Always update with actual outcome
     * - No hysteresis (1-bit predictor)
     * - Works well for branches with consistent behavior
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Initialize all BHT entries to 0 (predict not taken)
            // This is a safe default (most branches are not taken)
            bht <= {BHT_SIZE{1'b0}};
        end else if (update_enable) begin
            // Update BHT entry with actual branch outcome
            bht[bht_index_update] <= actual_taken;
        end
    end
    
    // ============================================
    // Misprediction Detection (for statistics)
    // ============================================
    
    /**
     * Misprediction Signal
     * 
     * Indicates if prediction was incorrect:
     * - 1: Prediction != actual outcome (misprediction)
     * - 0: Prediction == actual outcome (correct prediction)
     * 
     * This signal can be used for:
     * - Performance monitoring
     * - Pipeline flush control
     * - Statistics collection
     * 
     * Note: Prediction is stored when branch is in IF stage,
     * and compared when branch resolves in EX stage.
     */
    logic misprediction;
    
    // Store prediction when branch is in IF stage
    // This allows comparison when branch resolves in EX stage
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            predicted_taken_stored <= 1'b0;
            predicted_PC_stored <= {ADDR_WIDTH{1'b0}};
        end else if (is_branch) begin
            // Store prediction and PC when branch is predicted
            predicted_taken_stored <= predict_taken;
            predicted_PC_stored <= if_PC;
        end
    end
    
    // Compare prediction with actual outcome
    // Only compare if the stored PC matches the branch PC being resolved
    assign misprediction = update_enable && 
                          (predicted_PC_stored == ex_PC) &&
                          (predicted_taken_stored != actual_taken);
    
    // ============================================
    // Implementation Notes
    // ============================================
    
    /**
     * Design Trade-offs:
     * 
     * 1. BHT Size:
     *    - Larger BHT: Better accuracy, more hardware
     *    - Smaller BHT: Less hardware, more aliasing
     *    - Default: 256 entries (good balance)
     * 
     * 2. Indexing:
     *    - PC[9:2] for 256-entry BHT (8-bit index)
     *    - Uses lower bits: Captures local branch patterns
     *    - Higher bits could be used for better distribution
     * 
     * 3. Update Policy:
     *    - 1-bit: Simple, updates immediately
     *    - 2-bit: Better for alternating patterns (not implemented)
     *    - Always update: Learns quickly, adapts to changes
     * 
     * 4. Initialization:
     *    - Reset to 0 (predict not taken)
     *    - Safe default (most branches are not taken)
     *    - Learns correct behavior quickly
     * 
     * 5. Performance:
     *    - Reduces flush rate for consistent branches
     *    - Works well for loops (always taken)
     *    - Works well for error checks (usually not taken)
     *    - Mispredicts on alternating patterns
     */
    
    /**
     * Integration Points:
     * 
     * 1. IF Stage:
     *    - Input: if_PC (current PC)
     *    - Output: predict_taken, predict_target
     *    - Use prediction to speculatively fetch from target
     * 
     * 2. ID Stage:
     *    - Input: is_branch (from instruction decode)
     *    - Input: branch_immediate (calculated immediate)
     *    - Pass through to IF stage for prediction
     * 
     * 3. EX Stage:
     *    - Input: ex_PC (PC of branch instruction)
     *    - Input: actual_taken (branch resolution result)
     *    - Input: actual_target (calculated branch target)
     *    - Output: update_enable (when branch resolves)
     *    - Update BHT with actual outcome
     * 
     * 4. Pipeline Control:
     *    - Misprediction: Flush pipeline, fetch from correct target
     *    - Correct prediction: Continue execution normally
     */
    
endmodule

