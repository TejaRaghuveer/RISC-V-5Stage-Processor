/**
 * @file imm_gen.sv
 * @brief RISC-V Immediate Generator Module
 * 
 * This module extracts and sign-extends immediate values from RISC-V instructions
 * based on the instruction type. Different instruction formats have different
 * immediate field layouts, and this module handles all RV32I formats.
 * 
 * @details
 * Immediate Generation:
 * - Extracts immediate bits from instruction word based on instruction type
 * - Sign-extends immediate to 32 bits
 * - Handles all RV32I instruction formats (I, S, B, U, J)
 * 
 * Instruction Formats Supported:
 * - I-type: 12-bit immediate (ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU, LW, LH, LB, LHU, LBU, JALR)
 * - S-type: 12-bit immediate (SW, SH, SB)
 * - B-type: 13-bit immediate (BEQ, BNE, BLT, BGE, BLTU, BGEU)
 * - U-type: 20-bit immediate (LUI, AUIPC)
 * - J-type: 21-bit immediate (JAL)
 * 
 * Sign Extension:
 * - I-type, S-type, B-type, J-type: Sign-extend from MSB
 * - U-type: Zero-extend (upper 20 bits, lower 12 bits are zero)
 */

module imm_gen (
    // Input
    input  logic [31:0] instruction,     // 32-bit instruction word
    
    // Output
    output logic [31:0] immediate        // 32-bit sign-extended immediate value
);

    /**
     * Instruction Field Extraction
     * 
     * RISC-V instruction format:
     *   [31:25] [24:20] [19:15] [14:12] [11:7]  [6:0]
     *   funct7  rs2     rs1     funct3  rd      opcode
     * 
     * Immediate bits are scattered across different positions depending on type.
     */
    
    /**
     * Immediate Generation Based on Instruction Type
     * 
     * The immediate value is extracted from the instruction word based on the
     * opcode field [6:0]. Each instruction type has a different immediate layout.
     */
    always_comb begin
        case (instruction[6:0])
            // ============================================
            // I-Type Instructions
            // Opcode: 7'b0010011 (immediate ops), 7'b0000011 (loads), 7'b1100111 (JALR)
            // Immediate Format: [31:20] -> imm[11:0]
            // Sign-extend: imm[31:12] = imm[11]
            // ============================================
            7'b0010011,  // ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
            7'b0000011,  // LB, LH, LW, LBU, LHU
            7'b1100111:  // JALR
            begin
                // Extract immediate[11:0] from instruction[31:20]
                // Sign-extend to 32 bits: imm[31:12] = imm[11]
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            end
            
            // ============================================
            // S-Type Instructions (Store)
            // Opcode: 7'b0100011
            // Immediate Format: [31:25] -> imm[11:5], [11:7] -> imm[4:0]
            // Sign-extend: imm[31:12] = imm[11]
            // ============================================
            7'b0100011:  // SB, SH, SW
            begin
                // Extract immediate[11:5] from instruction[31:25]
                // Extract immediate[4:0] from instruction[11:7]
                // Combine: imm[11:0] = {instruction[31:25], instruction[11:7]}
                // Sign-extend to 32 bits: imm[31:12] = imm[11]
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            end
            
            // ============================================
            // B-Type Instructions (Branch)
            // Opcode: 7'b1100011
            // Immediate Format: 
            //   [31] -> imm[12]
            //   [30:25] -> imm[10:5]
            //   [11:8] -> imm[4:1]
            //   [7] -> imm[11]
            //   imm[0] = 0 (implicit)
            // Sign-extend: imm[31:13] = imm[12]
            // ============================================
            7'b1100011:  // BEQ, BNE, BLT, BGE, BLTU, BGEU
            begin
                // Extract and rearrange bits:
                // imm[12] = instruction[31]
                // imm[10:5] = instruction[30:25]
                // imm[4:1] = instruction[11:8]
                // imm[11] = instruction[7]
                // imm[0] = 0 (implicit, not stored in instruction)
                // Combine: imm[12:0] = {instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}
                // Sign-extend to 32 bits: imm[31:13] = imm[12]
                immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            end
            
            // ============================================
            // U-Type Instructions (Upper Immediate)
            // Opcode: 7'b0110111 (LUI), 7'b0010111 (AUIPC)
            // Immediate Format: [31:12] -> imm[31:12]
            // Lower 12 bits are zero: imm[11:0] = 0
            // Note: U-type is NOT sign-extended, it's zero-extended
            // ============================================
            7'b0110111,  // LUI
            7'b0010111:  // AUIPC
            begin
                // Extract immediate[31:12] from instruction[31:12]
                // Lower 12 bits are zero: imm[11:0] = 0
                // This is zero-extended, not sign-extended
                immediate = {instruction[31:12], 12'b0};
            end
            
            // ============================================
            // J-Type Instructions (Jump)
            // Opcode: 7'b1101111
            // Immediate Format:
            //   [31] -> imm[20]
            //   [30:21] -> imm[10:1]
            //   [20] -> imm[11]
            //   [19:12] -> imm[19:12]
            //   imm[0] = 0 (implicit)
            // Sign-extend: imm[31:21] = imm[20]
            // ============================================
            7'b1101111:  // JAL
            begin
                // Extract and rearrange bits:
                // imm[20] = instruction[31]
                // imm[10:1] = instruction[30:21]
                // imm[11] = instruction[20]
                // imm[19:12] = instruction[19:12]
                // imm[0] = 0 (implicit, not stored in instruction)
                // Combine: imm[20:0] = {instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}
                // Sign-extend to 32 bits: imm[31:21] = imm[20]
                immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            end
            
            // ============================================
            // Default Case (R-type or invalid opcode)
            // R-type instructions have no immediate (use funct3/funct7)
            // ============================================
            default: begin
                // R-type instructions (7'b0110011) have no immediate
                // Return zero for R-type or invalid opcodes
                immediate = 32'b0;
            end
        endcase
    end
    
    /**
     * Immediate Format Details:
     * 
     * 1. I-Type Immediate (12 bits):
     *    - Bits: instruction[31:20] -> immediate[11:0]
     *    - Sign-extend: immediate[31:12] = immediate[11]
     *    - Used by: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
     *               LW, LH, LB, LHU, LBU, JALR
     *    - Example: instruction = 0x00100093 (ADDI x1, x0, 1)
     *               immediate = 0x00000001
     * 
     * 2. S-Type Immediate (12 bits):
     *    - Bits: instruction[31:25] -> immediate[11:5]
     *            instruction[11:7] -> immediate[4:0]
     *    - Sign-extend: immediate[31:12] = immediate[11]
     *    - Used by: SW, SH, SB
     *    - Example: instruction = 0x0030A023 (SW x3, 0(x1))
     *               immediate = 0x00000000
     * 
     * 3. B-Type Immediate (13 bits, but stored as 12 bits + implicit 0):
     *    - Bits: instruction[31] -> immediate[12]
     *            instruction[7] -> immediate[11]
     *            instruction[30:25] -> immediate[10:5]
     *            instruction[11:8] -> immediate[4:1]
     *            immediate[0] = 0 (implicit)
     *    - Sign-extend: immediate[31:13] = immediate[12]
     *    - Used by: BEQ, BNE, BLT, BGE, BLTU, BGEU
     *    - Note: Lower bit is always 0 (word-aligned branches)
     *    - Example: instruction = 0x00208163 (BNE x1, x2, 4)
     *               immediate = 0x00000004
     * 
     * 4. U-Type Immediate (20 bits):
     *    - Bits: instruction[31:12] -> immediate[31:12]
     *            immediate[11:0] = 0 (zero)
     *    - Zero-extended: Lower 12 bits are always zero
     *    - Used by: LUI, AUIPC
     *    - Example: instruction = 0x00001037 (LUI x0, 1)
     *               immediate = 0x00001000
     * 
     * 5. J-Type Immediate (21 bits, but stored as 20 bits + implicit 0):
     *    - Bits: instruction[31] -> immediate[20]
     *            instruction[19:12] -> immediate[19:12]
     *            instruction[20] -> immediate[11]
     *            instruction[30:21] -> immediate[10:1]
     *            immediate[0] = 0 (implicit)
     *    - Sign-extend: immediate[31:21] = immediate[20]
     *    - Used by: JAL
     *    - Note: Lower bit is always 0 (word-aligned jumps)
     *    - Example: instruction = 0x004000EF (JAL x1, 4)
     *               immediate = 0x00000004
     * 
     * 6. R-Type Instructions:
     *    - No immediate field
     *    - Return zero
     *    - Used by: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
     */
    
    /**
     * Sign Extension Explanation:
     * 
     * Sign extension replicates the most significant bit (MSB) of the immediate
     * to fill the upper bits. This preserves the signed value when extending
     * from a smaller bit width to a larger bit width.
     * 
     * Examples:
     * - 12-bit immediate: 0xFFF (-1) -> 0xFFFFFFFF (-1 in 32-bit)
     * - 12-bit immediate: 0x001 (+1) -> 0x00000001 (+1 in 32-bit)
     * - 13-bit immediate: 0x1FFF (-1) -> 0xFFFFFFFF (-1 in 32-bit)
     * - 21-bit immediate: 0x1FFFFF (-1) -> 0xFFFFFFFF (-1 in 32-bit)
     * 
     * U-type is special: It's zero-extended (not sign-extended) because
     * it represents an upper 20-bit value with lower 12 bits zero.
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Combinational Logic:
     *    - Immediate generation is combinational (no clock)
     *    - Output available in same cycle as instruction decode
     * 
     * 2. Bit Extraction:
     *    - Uses SystemVerilog bit selection and concatenation
     *    - Bit positions match RISC-V specification exactly
     * 
     * 3. Sign Extension:
     *    - Uses replication operator: {N{bit}} replicates bit N times
     *    - Example: {20{instruction[31]}} replicates MSB 20 times
     * 
     * 4. U-Type Special Case:
     *    - Not sign-extended, lower 12 bits are explicitly zero
     *    - Used for LUI (load upper immediate) and AUIPC
     * 
     * 5. Implicit Zero Bits:
     *    - B-type and J-type have implicit zero in bit[0]
     *    - This ensures word-aligned branch/jump targets
     *    - Explicitly set to 0 in the concatenation
     * 
     * 6. Default Case:
     *    - R-type instructions return zero (no immediate)
     *    - Invalid opcodes also return zero (safe default)
     */

endmodule

