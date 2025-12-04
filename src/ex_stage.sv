/**
 * @file ex_stage.sv
 * @brief RISC-V Execute (EX) Stage Module
 * 
 * This module implements the Execute stage of the RISC-V 5-stage pipeline.
 * The EX stage performs arithmetic/logic operations, calculates addresses,
 * evaluates branch conditions, and handles data forwarding.
 * 
 * @details
 * Execute Stage Responsibilities:
 * - Execute ALU operations (arithmetic, logic, shift, compare)
 * - Calculate memory addresses (for load/store)
 * - Calculate branch/jump target addresses
 * - Evaluate branch conditions
 * - Handle data forwarding from later pipeline stages
 * 
 * Components:
 * - ALU: Performs arithmetic/logic operations
 * - Forwarding Multiplexers: Select correct data source for ALU operands
 * - Branch Target Calculator: Computes PC + immediate for branches
 * - Jump Target Calculator: Computes target address for jumps
 * 
 * Forwarding Paths:
 * - ForwardA: Selects rs1_data source (register file, EX/MEM, or MEM/WB)
 * - ForwardB: Selects rs2_data source (register file, EX/MEM, or MEM/WB)
 * - ALUSrc: Selects ALU operand B source (register or immediate)
 */

module ex_stage #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32            // Address width (32 bits)
) (
    // Inputs from ID/EX Pipeline Register
    input  logic [DATA_WIDTH-1:0]       id_ex_rs1_data,    // Source register 1 data
    input  logic [DATA_WIDTH-1:0]       id_ex_rs2_data,    // Source register 2 data
    input  logic [DATA_WIDTH-1:0]       id_ex_immediate,   // Sign-extended immediate
    input  logic [ADDR_WIDTH-1:0]       id_ex_PC,          // PC value
    input  logic [ADDR_WIDTH-1:0]       id_ex_PC_plus_4,   // PC+4 value
    
    // Forwarded Data from Later Stages
    input  logic [DATA_WIDTH-1:0]       ex_mem_alu_result, // ALU result from EX/MEM
    input  logic [DATA_WIDTH-1:0]       mem_wb_write_data, // Write data from MEM/WB
    
    // Forwarding Control Signals
    input  logic [1:0]                   ForwardA,          // Forwarding control for rs1
    input  logic [1:0]                   ForwardB,          // Forwarding control for rs2
    
    // Control Signals from ID/EX Register
    input  logic                        ALUSrc,            // ALU source select (0=register, 1=immediate)
    input  logic [1:0]                  ALUOp,             // ALU operation type
    input  logic [2:0]                  funct3,             // Function field (for ALU control)
    input  logic [6:0]                  funct7,             // Function field (for ALU control)
    input  logic [6:0]                  opcode,              // Instruction opcode [6:0]
    input  logic                        Branch,             // Branch instruction
    input  logic                        Jump,               // Jump instruction
    
    // Outputs to EX/MEM Pipeline Register
    output logic [DATA_WIDTH-1:0]       alu_result,        // ALU computation result
    output logic                        zero_flag,         // ALU zero flag (for branches)
    output logic                        branch_taken,       // Branch condition evaluation
    output logic [ADDR_WIDTH-1:0]       branch_target,     // Branch target address (PC + immediate)
    output logic [ADDR_WIDTH-1:0]       jump_target,       // Jump target address
    output logic [DATA_WIDTH-1:0]       rs2_data_out,      // rs2 data (for store instructions)
    
    // Forwarded Data Outputs (for branch/jump control unit)
    output logic [DATA_WIDTH-1:0]       rs1_data_forwarded_out, // Forwarded rs1 data
    output logic [DATA_WIDTH-1:0]       rs2_data_forwarded_out  // Forwarded rs2 data
);

    /**
     * Forwarded Operand Signals
     * 
     * These signals hold the selected data sources after forwarding muxes.
     * They are used as ALU operands.
     */
    logic [DATA_WIDTH-1:0] rs1_data_forwarded;  // Forwarded rs1 data
    logic [DATA_WIDTH-1:0] rs2_data_forwarded;  // Forwarded rs2 data
    
    /**
     * ALU Operand Signals
     * 
     * Final operands to ALU after forwarding and immediate selection.
     */
    logic [DATA_WIDTH-1:0] alu_operand_a;       // ALU operand A (always rs1)
    logic [DATA_WIDTH-1:0] alu_operand_b;        // ALU operand B (rs2 or immediate)
    
    /**
     * ALU Control Signal
     * 
     * Generated from ALUOp, funct3, and funct7 to select specific ALU operation.
     */
    logic [3:0] alu_control;
    
    /**
     * Forwarding Multiplexer for rs1 (ForwardA)
     * 
     * Selects the correct source for ALU operand A (rs1):
     * - 2'b00: Use register file data from ID/EX (id_ex_rs1_data)
     * - 2'b01: Forward from MEM/WB stage (mem_wb_write_data)
     * - 2'b10: Forward from EX/MEM stage (ex_mem_alu_result)
     * 
     * Forwarding Paths:
     * - Normal: Register file → ID/EX → ALU
     * - MEM/WB Forward: MEM/WB → ALU (bypasses register file)
     * - EX/MEM Forward: EX/MEM → ALU (bypasses register file and MEM/WB)
     */
    always_comb begin
        case (ForwardA)
            2'b00: begin
                // No forwarding: Use register file data from ID/EX register
                rs1_data_forwarded = id_ex_rs1_data;
            end
            2'b01: begin
                // Forward from MEM/WB stage
                // Used when: Instruction in WB stage writes to register read by current instruction
                rs1_data_forwarded = mem_wb_write_data;
            end
            2'b10: begin
                // Forward from EX/MEM stage (highest priority)
                // Used when: Instruction in MEM stage writes to register read by current instruction
                rs1_data_forwarded = ex_mem_alu_result;
            end
            default: begin
                // Default to register file data
                rs1_data_forwarded = id_ex_rs1_data;
            end
        endcase
    end
    
    /**
     * Forwarding Multiplexer for rs2 (ForwardB)
     * 
     * Selects the correct source for ALU operand B (rs2):
     * - 2'b00: Use register file data from ID/EX (id_ex_rs2_data)
     * - 2'b01: Forward from MEM/WB stage (mem_wb_write_data)
     * - 2'b10: Forward from EX/MEM stage (ex_mem_alu_result)
     * 
     * Forwarding Paths:
     * - Normal: Register file → ID/EX → ALU
     * - MEM/WB Forward: MEM/WB → ALU (bypasses register file)
     * - EX/MEM Forward: EX/MEM → ALU (bypasses register file and MEM/WB)
     * 
     * Note: rs2 is also used for store instructions (data to write to memory)
     */
    always_comb begin
        case (ForwardB)
            2'b00: begin
                // No forwarding: Use register file data from ID/EX register
                rs2_data_forwarded = id_ex_rs2_data;
            end
            2'b01: begin
                // Forward from MEM/WB stage
                // Used when: Instruction in WB stage writes to register read by current instruction
                rs2_data_forwarded = mem_wb_write_data;
            end
            2'b10: begin
                // Forward from EX/MEM stage (highest priority)
                // Used when: Instruction in MEM stage writes to register read by current instruction
                rs2_data_forwarded = ex_mem_alu_result;
            end
            default: begin
                // Default to register file data
                rs2_data_forwarded = id_ex_rs2_data;
            end
        endcase
    end
    
    /**
     * ALU Operand A Selection
     * 
     * ALU Operand A: Selects between forwarded rs1_data and PC
     * - Normal instructions: Use forwarded rs1_data
     * - AUIPC instruction: Use PC (opcode = 7'b0010111)
     * 
     * AUIPC (Add Upper Immediate to PC) requires PC + immediate,
     * so PC must be selected as operand A instead of rs1_data.
     */
    assign alu_operand_a = (opcode == 7'b0010111) ? id_ex_PC : rs1_data_forwarded;
    
    /**
     * ALU Operand B Multiplexer (ALUSrc)
     * 
     * Selects between register (rs2) and immediate:
     * - ALUSrc = 0: Use register rs2 (for R-type instructions)
     * - ALUSrc = 1: Use immediate (for I-type, S-type, load/store, etc.)
     */
    assign alu_operand_b = ALUSrc ? id_ex_immediate : rs2_data_forwarded;
    
    /**
     * ALU Control Unit
     * 
     * Generates ALU control signal from ALUOp, funct3, and funct7.
     * This determines the specific ALU operation to perform.
     */
    always_comb begin
        case (ALUOp)
            2'b00: begin
                // Load/Store: ADD operation (for address calculation)
                alu_control = 4'b0010;  // ADD
            end
            2'b01: begin
                // Branch: SUB operation (for comparison)
                alu_control = 4'b0110;  // SUB
            end
            2'b10: begin
                // R-type: Operation determined by funct3 and funct7
                case ({funct7[5], funct3})
                    // ADD/SUB
                    4'b0_000: alu_control = 4'b0010;  // ADD
                    4'b1_000: alu_control = 4'b0110;  // SUB
                    // Shift
                    4'b0_001: alu_control = 4'b1000;  // SLL
                    4'b0_101: alu_control = 4'b1001;  // SRL
                    4'b1_101: alu_control = 4'b1010;  // SRA
                    // Compare
                    4'b0_010: alu_control = 4'b0111;  // SLT
                    4'b0_011: alu_control = 4'b1101;  // SLTU
                    // Logic
                    4'b0_100: alu_control = 4'b1100;  // XOR
                    4'b0_110: alu_control = 4'b0001;  // OR
                    4'b0_111: alu_control = 4'b0000;  // AND
                    default:  alu_control = 4'b0010;  // Default to ADD
                endcase
            end
            2'b11: begin
                // I-type immediate: Operation determined by funct3
                case (funct3)
                    3'b000: alu_control = 4'b0010;  // ADDI
                    3'b010: alu_control = 4'b0111;  // SLTI
                    3'b011: alu_control = 4'b1101;  // SLTIU
                    3'b100: alu_control = 4'b1100;  // XORI
                    3'b110: alu_control = 4'b0001;  // ORI
                    3'b111: alu_control = 4'b0000;  // ANDI
                    3'b001: alu_control = 4'b1000;  // SLLI
                    3'b101: begin
                        // SRLI or SRAI (determined by funct7[5])
                        if (funct7[5])
                            alu_control = 4'b1010;  // SRAI
                        else
                            alu_control = 4'b1001;  // SRLI
                    end
                    default: alu_control = 4'b0010;  // Default to ADD
                endcase
            end
            default: begin
                alu_control = 4'b0010;  // Default to ADD
            end
        endcase
    end
    
    /**
     * ALU Instantiation
     * 
     * Performs arithmetic/logic operations based on alu_control signal.
     * Outputs result and zero flag for branch condition evaluation.
     */
    alu alu_inst (
        .operand_a(alu_operand_a),
        .operand_b(alu_operand_b),
        .alu_control(alu_control),
        .result(alu_result),
        .zero_flag(zero_flag)
    );
    
    /**
     * Branch Target Address Calculation
     * 
     * Calculates branch target address: PC + immediate
     * Used for B-type branch instructions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
     * 
     * Note: Immediate is already sign-extended and includes implicit zero bit[0]
     */
    assign branch_target = id_ex_PC + id_ex_immediate;
    
    /**
     * Jump Target Address Calculation
     * 
     * Calculates jump target address for JAL and JALR:
     * - JAL (opcode = 7'b1101111): PC + immediate (PC-relative)
     * - JALR (opcode = 7'b1100111): rs1 + immediate (register + immediate)
     * 
     * Note: JAL and JALR are differentiated by opcode, not funct3.
     * For JAL, funct3 bits [14:12] are part of the immediate field and can have any value.
     * For JALR, funct3 = 3'b000 is required, but we use opcode for differentiation.
     * 
     * For JALR, rs1_data_forwarded is used (with forwarding support)
     * Lower bit is cleared (word-aligned)
     */
    always_comb begin
        if (Jump && opcode == 7'b1100111) begin
            // JALR: rs1 + immediate (clear lower bit for word alignment)
            jump_target = (rs1_data_forwarded + id_ex_immediate) & 32'hFFFFFFFE;
        end else if (Jump && opcode == 7'b1101111) begin
            // JAL: PC + immediate (PC-relative)
            jump_target = id_ex_PC + id_ex_immediate;
        end else begin
            // Default: PC + immediate (for safety, though Jump should be set)
            jump_target = id_ex_PC + id_ex_immediate;
        end
    end
    
    /**
     * Branch Condition Evaluation
     * 
     * Evaluates branch condition based on funct3 field:
     * - BEQ (000): rs1 == rs2 (zero_flag from SUB)
     * - BNE (001): rs1 != rs2 (!zero_flag)
     * - BLT (100): rs1 < rs2 (signed, less_than flag)
     * - BGE (101): rs1 >= rs2 (signed, !less_than flag)
     * - BLTU (110): rs1 < rs2 (unsigned)
     * - BGEU (111): rs1 >= rs2 (unsigned)
     * 
     * Note: ALU performs SUB operation (rs1 - rs2) for comparison
     */
    logic less_than_flag;
    assign less_than_flag = ($signed(rs1_data_forwarded) < $signed(rs2_data_forwarded));
    
    always_comb begin
        if (Branch) begin
            case (funct3)
                3'b000: branch_taken = zero_flag;           // BEQ: rs1 == rs2
                3'b001: branch_taken = !zero_flag;          // BNE: rs1 != rs2
                3'b100: branch_taken = less_than_flag;      // BLT: rs1 < rs2 (signed)
                3'b101: branch_taken = !less_than_flag;     // BGE: rs1 >= rs2 (signed)
                3'b110: branch_taken = (rs1_data_forwarded < rs2_data_forwarded);  // BLTU: rs1 < rs2 (unsigned)
                3'b111: branch_taken = (rs1_data_forwarded >= rs2_data_forwarded); // BGEU: rs1 >= rs2 (unsigned)
                default: branch_taken = 1'b0;
            endcase
        end else begin
            branch_taken = 1'b0;
        end
    end
    
    /**
     * rs2 Data Output
     * 
     * Passes forwarded rs2_data to EX/MEM register.
     * Used for store instructions (data to write to memory).
     */
    assign rs2_data_out = rs2_data_forwarded;
    
    /**
     * Forwarded Data Outputs
     * 
     * Output forwarded rs1 and rs2 data for branch/jump control unit.
     * These are used for branch condition evaluation.
     */
    assign rs1_data_forwarded_out = rs1_data_forwarded;
    assign rs2_data_forwarded_out = rs2_data_forwarded;
    
    /**
     * Forwarding Path Summary:
     * 
     * ForwardA Paths (for rs1):
     * 1. Register File → ID/EX → ForwardA Mux → ALU Operand A
     * 2. MEM/WB → ForwardA Mux → ALU Operand A (bypasses register file)
     * 3. EX/MEM → ForwardA Mux → ALU Operand A (bypasses register file and MEM/WB)
     * 
     * ForwardB Paths (for rs2):
     * 1. Register File → ID/EX → ForwardB Mux → ALUSrc Mux → ALU Operand B
     * 2. MEM/WB → ForwardB Mux → ALUSrc Mux → ALU Operand B
     * 3. EX/MEM → ForwardB Mux → ALUSrc Mux → ALU Operand B
     * 
     * ALUSrc Mux:
     * - Selects between forwarded rs2_data and immediate
     * - Used for I-type, S-type, load/store instructions
     */
    
    /**
     * Stage Behavior Summary:
     * 
     * 1. Forwarding:
     *    - Selects correct data source for ALU operands
     *    - Resolves RAW data hazards
     *    - No pipeline stall needed for most cases
     * 
     * 2. ALU Operation:
     *    - Performs arithmetic/logic operations
     *    - Generates result and zero flag
     *    - Used for computation and comparison
     * 
     * 3. Address Calculation:
     *    - Calculates memory addresses (for load/store)
     *    - Calculates branch/jump targets
     *    - Uses PC and immediate values
     * 
     * 4. Branch Evaluation:
     *    - Evaluates branch conditions
     *    - Determines if branch should be taken
     *    - Uses ALU result and flags
     * 
     * 5. Output Generation:
     *    - ALU result for computation or memory address
     *    - Branch target for PC update
     *    - Jump target for PC update
     *    - rs2_data for store instructions
     */

endmodule

