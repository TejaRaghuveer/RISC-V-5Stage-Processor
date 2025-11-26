/**
 * @file alu.sv
 * @brief Parameterized 32-bit Arithmetic Logic Unit (ALU) for RISC-V RV32I Processor
 * 
 * This module implements a complete ALU supporting all arithmetic, logic, shift,
 * and comparison operations required by the RISC-V RV32I base instruction set.
 * 
 * @details
 * The ALU performs operations based on the ALU control signal (alu_control).
 * Supported operations include:
 * - Arithmetic: ADD, SUB
 * - Logic: AND, OR, XOR
 * - Shift: SLL, SRL, SRA
 * - Comparison: SLT, SLTU
 * 
 * The module also generates a zero flag indicating when the result is zero,
 * which is useful for branch instructions (BEQ, BNE).
 */

module alu #(
    parameter DATA_WIDTH = 32        // Data width (32 bits for RV32I)
) (
    // Inputs
    input  logic [DATA_WIDTH-1:0] operand_a,    // First operand (rs1)
    input  logic [DATA_WIDTH-1:0] operand_b,    // Second operand (rs2 or immediate)
    input  logic [3:0]             alu_control,  // ALU operation control signal
    
    // Outputs
    output logic [DATA_WIDTH-1:0] result,       // ALU computation result
    output logic                   zero_flag     // Zero flag (1 if result == 0)
);

    // Internal signal for ALU result
    logic [DATA_WIDTH-1:0] alu_result;
    
    // Shift amount (lower 5 bits of operand_b for shift operations)
    logic [4:0] shift_amount;
    assign shift_amount = operand_b[4:0];
    
    /**
     * ALU Operation Selection
     * 
     * The ALU control signal (alu_control) determines which operation to perform.
     * Control encoding:
     * - 4'b0000: AND  - Bitwise AND
     * - 4'b0001: OR   - Bitwise OR
     * - 4'b0010: ADD  - Addition
     * - 4'b0110: SUB  - Subtraction
     * - 4'b0111: SLT  - Set Less Than (signed)
     * - 4'b1000: SLL  - Shift Left Logical
     * - 4'b1001: SRL  - Shift Right Logical
     * - 4'b1010: SRA  - Shift Right Arithmetic
     * - 4'b1100: XOR  - Bitwise XOR
     * - 4'b1101: SLTU - Set Less Than Unsigned
     */
    always_comb begin
        case (alu_control)
            // ============================================
            // Logic Operations
            // ============================================
            
            // AND: Bitwise AND operation
            // Used by: AND, ANDI instructions
            // Result: operand_a & operand_b
            4'b0000: begin
                alu_result = operand_a & operand_b;
            end
            
            // OR: Bitwise OR operation
            // Used by: OR, ORI instructions
            // Result: operand_a | operand_b
            4'b0001: begin
                alu_result = operand_a | operand_b;
            end
            
            // ============================================
            // Arithmetic Operations
            // ============================================
            
            // ADD: Addition operation
            // Used by: ADD, ADDI, LW, SW, JALR instructions
            // Result: operand_a + operand_b
            // Note: For address calculation in load/store instructions
            4'b0010: begin
                alu_result = operand_a + operand_b;
            end
            
            // SUB: Subtraction operation
            // Used by: SUB instruction
            // Result: operand_a - operand_b
            // Note: Also used for comparison in branch instructions
            4'b0110: begin
                alu_result = operand_a - operand_b;
            end
            
            // ============================================
            // Comparison Operations
            // ============================================
            
            // SLT: Set Less Than (signed comparison)
            // Used by: SLT, SLTI instructions
            // Result: 1 if operand_a < operand_b (signed), else 0
            // Comparison: Treats operands as signed two's complement numbers
            4'b0111: begin
                // Signed comparison: if (operand_a < operand_b) then 1, else 0
                alu_result = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0;
            end
            
            // SLTU: Set Less Than Unsigned
            // Used by: SLTU, SLTIU instructions
            // Result: 1 if operand_a < operand_b (unsigned), else 0
            // Comparison: Treats operands as unsigned numbers
            4'b1101: begin
                // Unsigned comparison: if (operand_a < operand_b) then 1, else 0
                alu_result = (operand_a < operand_b) ? 32'd1 : 32'd0;
            end
            
            // ============================================
            // Shift Operations
            // ============================================
            
            // SLL: Shift Left Logical
            // Used by: SLL, SLLI instructions
            // Result: operand_a << shift_amount
            // Shifts left, fills right with zeros
            // Shift amount: lower 5 bits of operand_b (0-31 positions)
            4'b1000: begin
                alu_result = operand_a << shift_amount;
            end
            
            // SRL: Shift Right Logical
            // Used by: SRL, SRLI instructions
            // Result: operand_a >> shift_amount
            // Shifts right, fills left with zeros
            // Shift amount: lower 5 bits of operand_b (0-31 positions)
            4'b1001: begin
                alu_result = operand_a >> shift_amount;
            end
            
            // SRA: Shift Right Arithmetic
            // Used by: SRA, SRAI instructions
            // Result: operand_a >>> shift_amount (arithmetic right shift)
            // Shifts right, fills left with sign bit (MSB)
            // Preserves sign for signed numbers
            // Shift amount: lower 5 bits of operand_b (0-31 positions)
            4'b1010: begin
                // Arithmetic right shift: sign-extends the MSB
                alu_result = $signed(operand_a) >>> shift_amount;
            end
            
            // ============================================
            // Additional Logic Operations
            // ============================================
            
            // XOR: Bitwise XOR operation
            // Used by: XOR, XORI instructions
            // Result: operand_a ^ operand_b
            4'b1100: begin
                alu_result = operand_a ^ operand_b;
            end
            
            // ============================================
            // Default Case (should not occur in normal operation)
            // ============================================
            default: begin
                // Default to addition for safety
                // In a real design, this might be an error condition
                alu_result = operand_a + operand_b;
            end
        endcase
    end
    
    // Assign ALU result to output
    assign result = alu_result;
    
    /**
     * Zero Flag Generation
     * 
     * The zero flag is set to 1 when the ALU result equals zero.
     * This flag is used by branch instructions (BEQ, BNE) to determine
     * whether to take a branch.
     * 
     * For BEQ: branch if zero_flag == 1 (operands are equal)
     * For BNE: branch if zero_flag == 0 (operands are not equal)
     * 
     * Note: For branch instructions, operand_a and operand_b are compared
     * using subtraction (SUB operation), and the zero flag indicates equality.
     */
    assign zero_flag = (alu_result == 32'd0) ? 1'b1 : 1'b0;
    
    /**
     * Operation Summary:
     * 
     * ADD  (alu_control = 4'b0010): result = operand_a + operand_b
     * SUB  (alu_control = 4'b0110): result = operand_a - operand_b
     * AND  (alu_control = 4'b0000): result = operand_a & operand_b
     * OR   (alu_control = 4'b0001): result = operand_a | operand_b
     * XOR  (alu_control = 4'b1100): result = operand_a ^ operand_b
     * SLL  (alu_control = 4'b1000): result = operand_a << operand_b[4:0]
     * SRL  (alu_control = 4'b1001): result = operand_a >> operand_b[4:0]
     * SRA  (alu_control = 4'b1010): result = operand_a >>> operand_b[4:0] (arithmetic)
     * SLT  (alu_control = 4'b0111): result = (operand_a < operand_b) ? 1 : 0 (signed)
     * SLTU (alu_control = 4'b1101): result = (operand_a < operand_b) ? 1 : 0 (unsigned)
     * 
     * Zero Flag: zero_flag = (result == 0) ? 1 : 0
     */

endmodule

