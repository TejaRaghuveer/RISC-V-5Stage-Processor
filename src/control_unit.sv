/**
 * @file control_unit.sv
 * @brief RISC-V RV32I Control Unit Module
 * 
 * This module implements the main control unit for the RISC-V RV32I processor.
 * It decodes instruction opcodes and function fields to generate control signals
 * that control the datapath components (ALU, register file, memory, etc.).
 * 
 * @details
 * Control Unit Responsibilities:
 * - Decode instruction opcode and function fields
 * - Generate control signals for datapath components
 * - Support all RV32I instruction types (R, I, S, B, U, J)
 * - Provide control signals for pipeline stages
 * 
 * Instruction Types Supported:
 * - R-type: Register-register operations (ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU)
 * - I-type: Immediate operations (ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU)
 * - I-type Load: Load instructions (LB, LH, LW, LBU, LHU)
 * - I-type JALR: Jump and link register
 * - S-type: Store instructions (SB, SH, SW)
 * - B-type: Branch instructions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
 * - U-type: Upper immediate (LUI, AUIPC)
 * - J-type: Jump and link (JAL)
 * 
 * Control Signals Generated:
 * - RegWrite: Enable register file write
 * - MemRead: Enable memory read
 * - MemWrite: Enable memory write
 * - MemToReg: Select memory data vs ALU result for writeback
 * - ALUSrc: Select immediate vs register for ALU operand B
 * - ALUOp: ALU operation type (for ALU control unit)
 * - Branch: Branch instruction detected
 * - Jump: Jump instruction detected (JAL, JALR)
 */

module control_unit (
    // Instruction Fields
    input  logic [6:0]  opcode,        // Instruction opcode [6:0]
    input  logic [2:0]  funct3,        // Function field [14:12]
    input  logic [6:0]  funct7,        // Function field [31:25] (for R-type)
    
    // Control Signals Output
    output logic        RegWrite,      // Register write enable
    output logic        MemRead,       // Memory read enable
    output logic        MemWrite,      // Memory write enable
    output logic        MemToReg,      // Memory to register (1 = memory, 0 = ALU)
    output logic        ALUSrc,        // ALU source (1 = immediate, 0 = register)
    output logic [1:0]  ALUOp,         // ALU operation type
    output logic        Branch,         // Branch instruction
    output logic        Jump           // Jump instruction (JAL, JALR)
);

    /**
     * Control Signal Encoding:
     * 
     * ALUOp Encoding:
     * - 2'b00: Load/Store (ADD for address calculation)
     * - 2'b01: Branch (SUB for comparison)
     * - 2'b10: R-type (ALU operation from funct3/funct7)
     * - 2'b11: I-type immediate (ALU operation from funct3)
     * 
     * Control Signal Meanings:
     * - RegWrite: 1 = write result to register file, 0 = no write
     * - MemRead: 1 = read from memory, 0 = no read
     * - MemWrite: 1 = write to memory, 0 = no write
     * - MemToReg: 1 = write memory data, 0 = write ALU result
     * - ALUSrc: 1 = use immediate, 0 = use register rs2
     * - Branch: 1 = branch instruction, 0 = not branch
     * - Jump: 1 = jump instruction (JAL/JALR), 0 = not jump
     */
    
    /**
     * Main Control Logic
     * 
     * Decodes instruction opcode to generate control signals.
     * Opcode field determines instruction type and basic control signals.
     */
    always_comb begin
        // Default values (most common case: no operation)
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        MemToReg = 1'b0;
        ALUSrc = 1'b0;
        ALUOp = 2'b00;
        Branch = 1'b0;
        Jump = 1'b0;
        
        case (opcode)
            // ============================================
            // R-Type Instructions (Register-Register)
            // Opcode: 7'b0110011
            // Format: ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
            // ============================================
            7'b0110011: begin
                RegWrite = 1'b1;      // Write result to register
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Write ALU result (not memory)
                ALUSrc = 1'b0;        // Use register rs2 (not immediate)
                ALUOp = 2'b10;        // R-type: ALU operation from funct3/funct7
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump
            end
            
            // ============================================
            // I-Type Instructions (Immediate)
            // Opcode: 7'b0010011
            // Format: ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
            // ============================================
            7'b0010011: begin
                RegWrite = 1'b1;      // Write result to register
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Write ALU result (not memory)
                ALUSrc = 1'b1;        // Use immediate (not register rs2)
                ALUOp = 2'b11;        // I-type: ALU operation from funct3
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump
            end
            
            // ============================================
            // I-Type Load Instructions
            // Opcode: 7'b0000011
            // Format: LB, LH, LW, LBU, LHU
            // ============================================
            7'b0000011: begin
                RegWrite = 1'b1;      // Write loaded data to register
                MemRead = 1'b1;       // Read from memory
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b1;      // Write memory data (not ALU result)
                ALUSrc = 1'b1;        // Use immediate for address calculation
                ALUOp = 2'b00;        // ADD for address calculation (rs1 + immediate)
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump
            end
            
            // ============================================
            // S-Type Store Instructions
            // Opcode: 7'b0100011
            // Format: SB, SH, SW
            // ============================================
            7'b0100011: begin
                RegWrite = 1'b0;      // No register write (store only)
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b1;      // Write to memory
                MemToReg = 1'b0;      // Don't care (no register write)
                ALUSrc = 1'b1;        // Use immediate for address calculation
                ALUOp = 2'b00;        // ADD for address calculation (rs1 + immediate)
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump
            end
            
            // ============================================
            // B-Type Branch Instructions
            // Opcode: 7'b1100011
            // Format: BEQ, BNE, BLT, BGE, BLTU, BGEU
            // ============================================
            7'b1100011: begin
                RegWrite = 1'b0;      // No register write
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Don't care (no register write)
                ALUSrc = 1'b0;        // Use register rs2 for comparison
                ALUOp = 2'b01;        // SUB for comparison (rs1 - rs2)
                Branch = 1'b1;        // Branch instruction
                Jump = 1'b0;          // Not a jump (branch uses PC-relative)
            end
            
            // ============================================
            // U-Type LUI (Load Upper Immediate)
            // Opcode: 7'b0110111
            // Format: LUI
            // ============================================
            7'b0110111: begin
                RegWrite = 1'b1;      // Write immediate to register
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Write ALU result (immediate)
                ALUSrc = 1'b1;        // Use immediate
                ALUOp = 2'b11;        // Pass immediate through ALU
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump
            end
            
            // ============================================
            // U-Type AUIPC (Add Upper Immediate to PC)
            // Opcode: 7'b0010111
            // Format: AUIPC
            // ============================================
            7'b0010111: begin
                RegWrite = 1'b1;      // Write result to register
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Write ALU result (PC + immediate)
                ALUSrc = 1'b1;        // Use immediate
                ALUOp = 2'b00;        // ADD (PC + immediate)
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump
            end
            
            // ============================================
            // J-Type JAL (Jump and Link)
            // Opcode: 7'b1101111
            // Format: JAL
            // ============================================
            7'b1101111: begin
                RegWrite = 1'b1;      // Write return address (PC+4) to register
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Write PC+4 (not memory)
                ALUSrc = 1'b0;        // Don't care (PC update handled separately)
                ALUOp = 2'b00;        // Don't care
                Branch = 1'b0;        // Not a branch (it's a jump)
                Jump = 1'b1;          // Jump instruction
            end
            
            // ============================================
            // I-Type JALR (Jump and Link Register)
            // Opcode: 7'b1100111
            // Format: JALR
            // ============================================
            7'b1100111: begin
                RegWrite = 1'b1;      // Write return address (PC+4) to register
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Write PC+4 (not memory)
                ALUSrc = 1'b1;        // Use immediate for address calculation
                ALUOp = 2'b00;        // ADD for address calculation (rs1 + immediate)
                Branch = 1'b0;        // Not a branch (it's a jump)
                Jump = 1'b1;          // Jump instruction
            end
            
            // ============================================
            // System Instructions (ECALL, EBREAK)
            // Opcode: 7'b1110011
            // Format: ECALL, EBREAK
            // Note: These are handled separately, but included for completeness
            // ============================================
            7'b1110011: begin
                // System instructions are typically handled by exception handler
                // Control signals depend on specific implementation
                RegWrite = 1'b0;      // Typically no register write
                MemRead = 1'b0;       // No memory read
                MemWrite = 1'b0;      // No memory write
                MemToReg = 1'b0;      // Don't care
                ALUSrc = 1'b0;        // Don't care
                ALUOp = 2'b00;        // Don't care
                Branch = 1'b0;        // Not a branch
                Jump = 1'b0;          // Not a jump (handled by exception)
            end
            
            // ============================================
            // Default Case (Invalid Opcode)
            // ============================================
            default: begin
                // Invalid opcode: Set all control signals to safe defaults
                RegWrite = 1'b0;
                MemRead = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'b0;
                ALUSrc = 1'b0;
                ALUOp = 2'b00;
                Branch = 1'b0;
                Jump = 1'b0;
            end
        endcase
    end
    
    /**
     * Control Signal Summary by Instruction Type:
     * 
     * R-type (ADD, SUB, etc.):
     *   RegWrite=1, MemRead=0, MemWrite=0, MemToReg=0, ALUSrc=0, ALUOp=10, Branch=0, Jump=0
     * 
     * I-type immediate (ADDI, etc.):
     *   RegWrite=1, MemRead=0, MemWrite=0, MemToReg=0, ALUSrc=1, ALUOp=11, Branch=0, Jump=0
     * 
     * I-type load (LW, etc.):
     *   RegWrite=1, MemRead=1, MemWrite=0, MemToReg=1, ALUSrc=1, ALUOp=00, Branch=0, Jump=0
     * 
     * S-type store (SW, etc.):
     *   RegWrite=0, MemRead=0, MemWrite=1, MemToReg=X, ALUSrc=1, ALUOp=00, Branch=0, Jump=0
     * 
     * B-type branch (BEQ, etc.):
     *   RegWrite=0, MemRead=0, MemWrite=0, MemToReg=X, ALUSrc=0, ALUOp=01, Branch=1, Jump=0
     * 
     * U-type LUI:
     *   RegWrite=1, MemRead=0, MemWrite=0, MemToReg=0, ALUSrc=1, ALUOp=11, Branch=0, Jump=0
     * 
     * U-type AUIPC:
     *   RegWrite=1, MemRead=0, MemWrite=0, MemToReg=0, ALUSrc=1, ALUOp=00, Branch=0, Jump=0
     * 
     * J-type JAL:
     *   RegWrite=1, MemRead=0, MemWrite=0, MemToReg=0, ALUSrc=X, ALUOp=XX, Branch=0, Jump=1
     * 
     * I-type JALR:
     *   RegWrite=1, MemRead=0, MemWrite=0, MemToReg=0, ALUSrc=1, ALUOp=00, Branch=0, Jump=1
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Opcode Decoding:
     *    - Primary decoding based on opcode field [6:0]
     *    - Function fields (funct3, funct7) used by ALU control unit
     * 
     * 2. Control Signal Generation:
     *    - All signals generated combinational (no clock)
     *    - Available in same cycle as instruction decode
     * 
     * 3. Pipeline Considerations:
     *    - Control signals propagate through pipeline registers
     *    - Some signals only needed in specific stages:
     *      * EX stage: ALUSrc, ALUOp
     *      * MEM stage: MemRead, MemWrite, Branch
     *      * WB stage: RegWrite, MemToReg
     * 
     * 4. ALUOp Field:
     *    - Used by ALU control unit to determine specific ALU operation
     *    - ALU control unit uses ALUOp + funct3 + funct7
     * 
     * 5. Special Cases:
     *    - LUI: Immediate shifted left 12 bits, lower 12 bits zero
     *    - AUIPC: PC + (immediate << 12)
     *    - JAL/JALR: PC update handled separately, return address = PC+4
     * 
     * 6. System Instructions:
     *    - ECALL/EBREAK typically trigger exceptions
     *    - Control signals may need special handling
     */

endmodule

