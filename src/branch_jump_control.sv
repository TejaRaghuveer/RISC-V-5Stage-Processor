/**
 * @file branch_jump_control.sv
 * @brief RISC-V Branch and Jump Control Unit Module
 * 
 * This module implements branch condition evaluation and jump control for the
 * RISC-V 5-stage pipeline processor. It evaluates branch conditions, generates
 * PC source selection signals, and produces pipeline flush signals.
 * 
 * @details
 * Branch and Jump Control Responsibilities:
 * - Evaluate branch conditions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
 * - Determine if branch/jump should be taken
 * - Generate PCSrc signal for PC update selection
 * - Generate flush signal for pipeline register clearing
 * 
 * Branch Instructions (B-type):
 * - BEQ (000): Branch if rs1 == rs2
 * - BNE (001): Branch if rs1 != rs2
 * - BLT (100): Branch if rs1 < rs2 (signed)
 * - BGE (101): Branch if rs1 >= rs2 (signed)
 * - BLTU (110): Branch if rs1 < rs2 (unsigned)
 * - BGEU (111): Branch if rs1 >= rs2 (unsigned)
 * 
 * Jump Instructions:
 * - JAL: Always taken (PC-relative jump)
 * - JALR: Always taken (register + immediate jump)
 * 
 * Pipeline Control:
 * - PCSrc: Selects PC source (0 = PC+4, 1 = branch/jump target)
 * - Flush: Clears IF/ID and ID/EX registers when branch/jump taken
 */

module branch_jump_control #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32            // Address width (32 bits)
) (
    // ============================================
    // Inputs from EX Stage
    // ============================================
    
    /**
     * Control Signals
     */
    input  logic                        Branch,           // Branch instruction indicator
    input  logic                        Jump,             // Jump instruction indicator
    input  logic [2:0]                  funct3,           // Function field [14:12] (branch type)
    
    /**
     * ALU Results for Branch Condition Evaluation
     */
    input  logic                        zero_flag,        // ALU zero flag (from SUB operation)
    input  logic [DATA_WIDTH-1:0]       rs1_data,         // Source register 1 data (forwarded)
    input  logic [DATA_WIDTH-1:0]       rs2_data,         // Source register 2 data (forwarded)
    
    /**
     * Target Addresses
     */
    input  logic [ADDR_WIDTH-1:0]       branch_target,    // Branch target address (PC + immediate)
    input  logic [ADDR_WIDTH-1:0]       jump_target,     // Jump target address (PC + immediate or rs1 + immediate)
    
    // ============================================
    // Outputs for Pipeline Control
    // ============================================
    
    /**
     * PC Source Selection Signal
     * 
     * PCSrc selects the next PC value:
     * - 0: PC + 4 (sequential execution)
     * - 1: Branch/Jump target (taken branch or jump)
     * 
     * This signal is used in the IF stage to update the PC register.
     */
    output logic                        PCSrc,            // PC source select (0 = PC+4, 1 = target)
    
    /**
     * Pipeline Flush Signal
     * 
     * Flush signal clears pipeline registers when branch/jump is taken:
     * - Clears IF/ID register (inserts NOP)
     * - Clears ID/EX register (inserts NOP)
     * - Prevents incorrect instructions from executing
     * 
     * Flush is asserted when:
     * - Branch is taken (branch_taken = 1)
     * - Jump is taken (Jump = 1, always taken)
     */
    output logic                        flush,            // Pipeline flush signal (active high)
    
    /**
     * Branch Taken Signal
     * 
     * Indicates whether a branch condition evaluated to true.
     * Used for debugging and pipeline control.
     */
    output logic                        branch_taken      // Branch condition evaluation result
);

    /**
     * Branch Condition Evaluation
     * 
     * Evaluates branch condition based on funct3 field and ALU results.
     * The ALU performs SUB operation (rs1 - rs2) for comparison.
     * 
     * Branch Condition Logic:
     * - BEQ (000): rs1 == rs2 → zero_flag == 1
     * - BNE (001): rs1 != rs2 → zero_flag == 0
     * - BLT (100): rs1 < rs2 (signed) → (rs1 - rs2) < 0 → sign bit of result
     * - BGE (101): rs1 >= rs2 (signed) → (rs1 - rs2) >= 0 → sign bit == 0
     * - BLTU (110): rs1 < rs2 (unsigned) → direct unsigned comparison
     * - BGEU (111): rs1 >= rs2 (unsigned) → direct unsigned comparison
     * 
     * For signed comparisons (BLT, BGE), we use the sign bit of (rs1 - rs2).
     * The ALU performs SUB, so we can check the sign of the result.
     * However, we need to compute this separately for signed vs unsigned.
     */
    
    /**
     * Signed Less-Than Flag
     * 
     * Computes rs1 < rs2 using signed comparison.
     * Uses SystemVerilog $signed() function for proper sign extension.
     */
    logic signed_less_than;
    assign signed_less_than = ($signed(rs1_data) < $signed(rs2_data));
    
    /**
     * Unsigned Less-Than Flag
     * 
     * Computes rs1 < rs2 using unsigned comparison.
     * Direct comparison without sign extension.
     */
    logic unsigned_less_than;
    assign unsigned_less_than = (rs1_data < rs2_data);
    
    /**
     * Branch Condition Evaluation Logic
     * 
     * Evaluates branch condition based on funct3 field.
     * Only evaluates if Branch signal is asserted.
     */
    always_comb begin
        if (Branch) begin
            case (funct3)
                3'b000: begin
                    // BEQ (Branch if Equal)
                    // Condition: rs1 == rs2
                    // Evaluation: zero_flag == 1 (from ALU SUB operation)
                    // ALU computes: rs1 - rs2, zero_flag = 1 if result == 0
                    branch_taken = zero_flag;
                end
                
                3'b001: begin
                    // BNE (Branch if Not Equal)
                    // Condition: rs1 != rs2
                    // Evaluation: zero_flag == 0 (from ALU SUB operation)
                    // ALU computes: rs1 - rs2, zero_flag = 0 if result != 0
                    branch_taken = !zero_flag;
                end
                
                3'b100: begin
                    // BLT (Branch if Less Than, signed)
                    // Condition: rs1 < rs2 (signed comparison)
                    // Evaluation: Check if rs1 < rs2 using signed comparison
                    // Note: ALU performs SUB, but we need signed comparison result
                    branch_taken = signed_less_than;
                end
                
                3'b101: begin
                    // BGE (Branch if Greater than or Equal, signed)
                    // Condition: rs1 >= rs2 (signed comparison)
                    // Evaluation: Check if rs1 >= rs2 using signed comparison
                    // Equivalent to: !(rs1 < rs2)
                    branch_taken = !signed_less_than;
                end
                
                3'b110: begin
                    // BLTU (Branch if Less Than, unsigned)
                    // Condition: rs1 < rs2 (unsigned comparison)
                    // Evaluation: Direct unsigned comparison
                    // No sign extension, treat as unsigned integers
                    branch_taken = unsigned_less_than;
                end
                
                3'b111: begin
                    // BGEU (Branch if Greater than or Equal, unsigned)
                    // Condition: rs1 >= rs2 (unsigned comparison)
                    // Evaluation: Direct unsigned comparison
                    // Equivalent to: !(rs1 < rs2)
                    branch_taken = !unsigned_less_than;
                end
                
                default: begin
                    // Invalid funct3: Branch not taken
                    branch_taken = 1'b0;
                end
            endcase
        end else begin
            // Not a branch instruction: Branch not taken
            branch_taken = 1'b0;
        end
    end
    
    /**
     * Jump Control Logic
     * 
     * Jump instructions (JAL, JALR) are always taken.
     * No condition evaluation needed.
     */
    logic jump_taken;
    assign jump_taken = Jump;  // Jumps are always taken
    
    /**
     * PC Source Selection (PCSrc)
     * 
     * Selects the next PC value:
     * - 0: PC + 4 (sequential execution, no branch/jump)
     * - 1: Branch/Jump target (branch taken or jump)
     * 
     * Priority:
     * 1. Jump: Always taken, use jump_target
     * 2. Branch: If taken, use branch_target
     * 3. Otherwise: Sequential (PC + 4, handled in IF stage)
     * 
     * Note: PCSrc = 1 means PC should be updated to target address.
     * The actual PC update is handled in the IF stage.
     */
    always_comb begin
        if (Jump) begin
            // Jump instruction: Always taken
            // Use jump_target (PC + immediate for JAL, rs1 + immediate for JALR)
            PCSrc = 1'b1;
        end else if (Branch && branch_taken) begin
            // Branch instruction: Taken if condition is true
            // Use branch_target (PC + immediate)
            PCSrc = 1'b1;
        end else begin
            // No branch/jump or branch not taken: Sequential execution
            // Use PC + 4 (handled in IF stage, PCSrc = 0)
            PCSrc = 1'b0;
        end
    end
    
    /**
     * Pipeline Flush Signal Generation
     * 
     * Flush signal clears pipeline registers when branch/jump is taken.
     * This prevents incorrect instructions (fetched speculatively) from executing.
     * 
     * Flush is asserted when:
     * - Branch is taken (branch_taken = 1)
     * - Jump is taken (Jump = 1, always taken)
     * 
     * Flush Behavior:
     * - Clears IF/ID register: Inserts NOP instruction
     * - Clears ID/EX register: Inserts NOP instruction
     * - Prevents incorrect instructions from propagating through pipeline
     * 
     * Timing:
     * - Flush is combinational (based on current branch/jump decision)
     * - Takes effect on next clock cycle
     * - Pipeline registers are cleared, inserting bubbles
     */
    assign flush = branch_taken || jump_taken;
    
    /**
     * Branch and Jump Control Summary:
     * 
     * 1. Branch Condition Evaluation:
     *    - BEQ: rs1 == rs2 → zero_flag == 1
     *    - BNE: rs1 != rs2 → zero_flag == 0
     *    - BLT: rs1 < rs2 (signed) → signed_less_than == 1
     *    - BGE: rs1 >= rs2 (signed) → signed_less_than == 0
     *    - BLTU: rs1 < rs2 (unsigned) → unsigned_less_than == 1
     *    - BGEU: rs1 >= rs2 (unsigned) → unsigned_less_than == 0
     * 
     * 2. Jump Control:
     *    - JAL: Always taken (PC-relative)
     *    - JALR: Always taken (register + immediate)
     * 
     * 3. PC Source Selection:
     *    - PCSrc = 0: Sequential execution (PC + 4)
     *    - PCSrc = 1: Branch/Jump taken (use target address)
     * 
     * 4. Pipeline Flush:
     *    - Flush = 1: Branch/Jump taken, clear pipeline registers
     *    - Flush = 0: No branch/jump or not taken, normal execution
     * 
     * 5. Timing:
     *    - All logic is combinational
     *    - PCSrc and flush signals available immediately
     *    - Used in IF stage for PC update and pipeline control
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Signed vs Unsigned Comparison:
     *    - Signed: Uses $signed() function for proper sign extension
     *    - Unsigned: Direct comparison without sign extension
     *    - Critical for correct BLT/BGE vs BLTU/BGEU behavior
     * 
     * 2. ALU Zero Flag:
     *    - Used for BEQ/BNE comparisons
     *    - ALU performs SUB operation (rs1 - rs2)
     *    - zero_flag = 1 if result == 0 (rs1 == rs2)
     *    - zero_flag = 0 if result != 0 (rs1 != rs2)
     * 
     * 3. Less-Than Flags:
     *    - signed_less_than: Uses $signed() for BLT/BGE
     *    - unsigned_less_than: Direct comparison for BLTU/BGEU
     *    - Computed separately for signed and unsigned cases
     * 
     * 4. PCSrc Signal:
     *    - Active high: 1 = use target, 0 = use PC+4
     *    - Used in IF stage PC update logic
     *    - Combines branch and jump decisions
     * 
     * 5. Flush Signal:
     *    - Active high: 1 = flush, 0 = normal
     *    - Clears IF/ID and ID/EX registers
     *    - Prevents incorrect instructions from executing
     * 
     * 6. Synthesis:
     *    - All logic is combinational
     *    - Synthesizes to gates and multiplexers
     *    - Critical path: Branch condition evaluation → PCSrc/Flush
     */

endmodule

