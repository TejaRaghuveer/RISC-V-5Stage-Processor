/**
 * @file id_stage.sv
 * @brief RISC-V Instruction Decode (ID) Stage Module
 * 
 * This module implements the Instruction Decode stage of the RISC-V 5-stage pipeline.
 * The ID stage decodes instructions, reads operands from the register file, generates
 * control signals, and extracts immediate values.
 * 
 * @details
 * Instruction Decode Stage Responsibilities:
 * - Decode instruction opcode and function fields
 * - Read source operands (rs1, rs2) from register file
 * - Generate control signals for downstream stages
 * - Extract and sign-extend immediate values
 * - Provide register addresses for hazard detection
 * 
 * Components Integrated:
 * - Register File: Two read ports for rs1 and rs2
 * - Control Unit: Generates control signals from opcode
 * - Immediate Generator: Extracts immediate values from instruction
 * 
 * Pipeline Interface:
 * - Inputs: Instruction, PC, PC+4 from IF/ID register
 * - Outputs: Register data, immediate, control signals to ID/EX register
 * - Write-back: Receives write data and address from WB stage
 * 
 * Hazard Detection:
 * - Provides rs1_addr and rs2_addr for forwarding unit
 * - Provides rd_addr for hazard detection unit
 * - Receives hazard control signals (stall, flush)
 */

module id_stage #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32            // Address width (32 bits)
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // Inputs from IF/ID Pipeline Register
    input  logic [DATA_WIDTH-1:0]       instruction,      // Instruction from IF stage
    input  logic [ADDR_WIDTH-1:0]      PC,               // PC value from IF stage
    input  logic [ADDR_WIDTH-1:0]      PC_plus_4,        // PC+4 from IF stage
    
    // Write-back Interface (from WB stage)
    input  logic                        wb_reg_write_en,  // Write enable from WB stage
    input  logic [4:0]                  wb_rd_addr,      // Write address from WB stage
    input  logic [DATA_WIDTH-1:0]      wb_rd_data,       // Write data from WB stage
    
    // Hazard Detection Interface
    input  logic                        stall,            // Pipeline stall signal
    input  logic                        flush,            // Pipeline flush signal
    
    // Outputs to ID/EX Pipeline Register
    output logic [DATA_WIDTH-1:0]      rs1_data,         // Source register 1 data
    output logic [DATA_WIDTH-1:0]      rs2_data,         // Source register 2 data
    output logic [DATA_WIDTH-1:0]      immediate,        // Sign-extended immediate value
    output logic [ADDR_WIDTH-1:0]      PC_out,           // PC value (passed through)
    output logic [ADDR_WIDTH-1:0]      PC_plus_4_out,    // PC+4 value (passed through)
    
    // Control Signals (to ID/EX register)
    output logic                        RegWrite,         // Register write enable
    output logic                        MemRead,          // Memory read enable
    output logic                        MemWrite,         // Memory write enable
    output logic                        MemToReg,         // Memory to register select
    output logic                        ALUSrc,           // ALU source select
    output logic [1:0]                  ALUOp,            // ALU operation type
    output logic                        Branch,            // Branch instruction
    output logic                        Jump,              // Jump instruction
    
    // Register Addresses (for forwarding and hazard detection)
    output logic [4:0]                  rs1_addr,         // Source register 1 address
    output logic [4:0]                  rs2_addr,         // Source register 2 address
    output logic [4:0]                  rd_addr            // Destination register address
);

    /**
     * Instruction Field Extraction
     * 
     * RISC-V instruction format:
     *   [31:25] [24:20] [19:15] [14:12] [11:7]  [6:0]
     *   funct7  rs2     rs1     funct3  rd      opcode
     */
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    
    assign opcode = instruction[6:0];
    assign funct3 = instruction[14:12];
    assign funct7 = instruction[31:25];
    
    /**
     * Register Address Extraction
     * 
     * Extract register addresses from instruction fields.
     * These are used for register file reads and forwarding.
     */
    assign rs1_addr = instruction[19:15];  // Source register 1 address
    assign rs2_addr = instruction[24:20]; // Source register 2 address
    assign rd_addr = instruction[11:7];   // Destination register address
    
    /**
     * Register File Instantiation
     * 
     * Two read ports (rs1, rs2) and one write port (rd).
     * Reads are asynchronous (combinational).
     * Writes are synchronous (clocked).
     */
    reg_file reg_file_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rs1_addr(rs1_addr),
        .rs1_data(rs1_data),
        .rs2_addr(rs2_addr),
        .rs2_data(rs2_data),
        .reg_write_en(wb_reg_write_en),
        .rd_addr(wb_rd_addr),
        .rd_data(wb_rd_data)
    );
    
    /**
     * Control Unit Instantiation
     * 
     * Decodes instruction opcode and function fields to generate
     * control signals for the datapath.
     */
    control_unit control_unit_inst (
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .ALUSrc(ALUSrc),
        .ALUOp(ALUOp),
        .Branch(Branch),
        .Jump(Jump)
    );
    
    /**
     * Immediate Generator Instantiation
     * 
     * Extracts and sign-extends immediate values from instruction
     * based on instruction type (I, S, B, U, J).
     */
    imm_gen imm_gen_inst (
        .instruction(instruction),
        .immediate(immediate)
    );
    
    /**
     * Pipeline Register Pass-through
     * 
     * PC and PC+4 values are passed through to ID/EX register.
     * These are used for branch target calculation and return addresses.
     */
    assign PC_out = PC;
    assign PC_plus_4_out = PC_plus_4;
    
    /**
     * Hazard Detection Interface
     * 
     * The ID stage provides register addresses for hazard detection:
     * - rs1_addr, rs2_addr: Used by forwarding unit
     * - rd_addr: Used by hazard detection unit
     * 
     * Hazard control signals (stall, flush) are received but handled
     * at the pipeline register level (ID/EX register).
     * 
     * Note: When stalled, the ID stage continues to decode the same
     * instruction. The ID/EX register holds the values when stalled.
     */
    
    /**
     * Stage Behavior Summary:
     * 
     * 1. Instruction Decode:
     *    - Extract opcode, funct3, funct7 from instruction
     *    - Extract register addresses (rs1, rs2, rd)
     * 
     * 2. Register File Read:
     *    - Read rs1_data from register file (asynchronous)
     *    - Read rs2_data from register file (asynchronous)
     *    - x0 always reads as zero (handled by register file)
     * 
     * 3. Control Signal Generation:
     *    - Control unit generates signals based on opcode
     *    - Signals propagate to ID/EX register
     * 
     * 4. Immediate Generation:
     *    - Immediate generator extracts immediate value
     *    - Sign-extends based on instruction type
     * 
     * 5. Pipeline Control:
     *    - Stall: ID stage continues decoding same instruction
     *    - Flush: ID stage outputs cleared (handled by ID/EX register)
     * 
     * 6. Write-back Interface:
     *    - Receives write data and address from WB stage
     *    - Passes to register file for write operation
     *    - Write occurs in WB stage (one cycle after ID stage)
     */
    
    /**
     * Timing Considerations:
     * 
     * 1. Combinational Paths:
     *    - Instruction decode: Combinational (opcode extraction)
     *    - Register file read: Combinational (asynchronous)
     *    - Control signal generation: Combinational
     *    - Immediate generation: Combinational
     * 
     * 2. Critical Path:
     *    - Instruction → Control Unit → Control Signals
     *    - Instruction → Immediate Generator → Immediate
     *    - rs1_addr/rs2_addr → Register File → rs1_data/rs2_data
     * 
     * 3. Pipeline Register:
     *    - All outputs registered in ID/EX pipeline register
     *    - Register updates on clock edge
     *    - Stall/flush handled at pipeline register level
     * 
     * 4. Write-back Timing:
     *    - Write occurs in WB stage (later in pipeline)
     *    - Write data available to ID stage in same cycle
     *    - Forwarding may be needed for data hazards
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Register File:
     *    - Two read ports allow simultaneous rs1 and rs2 reads
     *    - Write port receives data from WB stage
     *    - x0 hardwired to zero (handled internally)
     * 
     * 2. Control Unit:
     *    - Generates all control signals combinational
     *    - Signals propagate through ID/EX register
     *    - Some signals only used in specific stages
     * 
     * 3. Immediate Generator:
     *    - Handles all instruction formats (I, S, B, U, J)
     *    - Sign-extends to 32 bits
     *    - R-type returns zero (no immediate)
     * 
     * 4. Hazard Detection:
     *    - Register addresses provided for forwarding
     *    - Hazard unit compares addresses to detect hazards
     *    - Stall signal prevents new instruction from entering EX stage
     * 
     * 5. Pipeline Control:
     *    - Stall: ID stage continues, ID/EX register holds values
     *    - Flush: ID/EX register cleared (handled externally)
     *    - Reset: All components reset to known state
     */

endmodule

