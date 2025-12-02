/**
 * @file id_ex_reg.sv
 * @brief ID/EX Pipeline Register Module
 * 
 * This module implements the pipeline register between the Instruction Decode (ID)
 * and Execute (EX) stages of the RISC-V 5-stage pipeline.
 * 
 * @details
 * Pipeline Register Purpose:
 * - Stores instruction data and control signals from ID stage
 * - Passes data to EX stage on next clock cycle
 * - Enables pipeline operation by breaking combinational paths
 * - Provides pipeline control through enable and flush signals
 * 
 * Pipeline Register Contents:
 * - Register data: rs1_data, rs2_data
 * - Register addresses: rs1_addr, rs2_addr, rd_addr
 * - Immediate value: Sign-extended immediate from instruction
 * - PC values: PC and PC+4 (for branch/jump target calculation)
 * - Control signals: All control signals from control unit
 * 
 * Pipeline Control:
 * - Enable (stall): When low, holds current values (prevents new data)
 * - Flush: Clears register (inserts NOP/bubble in pipeline)
 * - Reset: Initializes register to zero
 */

module id_ex_reg #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32            // Address width (32 bits)
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // Pipeline Control Signals
    input  logic                        enable,           // Enable signal (0 = stall, 1 = normal)
    input  logic                        flush,            // Flush signal (1 = clear register)
    
    // ============================================
    // Inputs from ID Stage
    // ============================================
    
    // Register Data
    input  logic [DATA_WIDTH-1:0]       id_rs1_data,      // Source register 1 data
    input  logic [DATA_WIDTH-1:0]       id_rs2_data,     // Source register 2 data
    
    // Register Addresses
    input  logic [4:0]                  id_rs1_addr,      // Source register 1 address
    input  logic [4:0]                  id_rs2_addr,     // Source register 2 address
    input  logic [4:0]                  id_rd_addr,      // Destination register address
    
    // Immediate Value
    input  logic [DATA_WIDTH-1:0]       id_immediate,     // Sign-extended immediate value
    
    // PC Values
    input  logic [ADDR_WIDTH-1:0]       id_PC,            // PC value from IF stage
    input  logic [ADDR_WIDTH-1:0]       id_PC_plus_4,     // PC+4 value from IF stage
    
    // Control Signals from Control Unit
    input  logic                        id_RegWrite,      // Register write enable
    input  logic                        id_MemRead,       // Memory read enable
    input  logic                        id_MemWrite,      // Memory write enable
    input  logic                        id_MemToReg,      // Memory to register select
    input  logic                        id_ALUSrc,        // ALU source select
    input  logic [1:0]                  id_ALUOp,         // ALU operation type
    input  logic                        id_Branch,        // Branch instruction
    input  logic                        id_Jump,          // Jump instruction
    
    // Instruction Fields (for ALU control)
    input  logic [2:0]                  id_funct3,        // Function field [14:12]
    input  logic [6:0]                  id_funct7,        // Function field [31:25]
    input  logic [6:0]                  id_opcode,        // Instruction opcode [6:0]
    
    // ============================================
    // Outputs to EX Stage
    // ============================================
    
    // Register Data
    output logic [DATA_WIDTH-1:0]      ex_rs1_data,      // Source register 1 data
    output logic [DATA_WIDTH-1:0]      ex_rs2_data,      // Source register 2 data
    
    // Register Addresses
    output logic [4:0]                  ex_rs1_addr,      // Source register 1 address
    output logic [4:0]                  ex_rs2_addr,     // Source register 2 address
    output logic [4:0]                  ex_rd_addr,      // Destination register address
    
    // Immediate Value
    output logic [DATA_WIDTH-1:0]      ex_immediate,     // Sign-extended immediate value
    
    // PC Values
    output logic [ADDR_WIDTH-1:0]      ex_PC,            // PC value
    output logic [ADDR_WIDTH-1:0]      ex_PC_plus_4,     // PC+4 value
    
    // Control Signals
    output logic                        ex_RegWrite,      // Register write enable
    output logic                        ex_MemRead,       // Memory read enable
    output logic                        ex_MemWrite,      // Memory write enable
    output logic                        ex_MemToReg,      // Memory to register select
    output logic                        ex_ALUSrc,        // ALU source select
    output logic [1:0]                  ex_ALUOp,         // ALU operation type
    output logic                        ex_Branch,        // Branch instruction
    output logic                        ex_Jump,          // Jump instruction
    
    // Instruction Fields
    output logic [2:0]                  ex_funct3,        // Function field [14:12]
    output logic [6:0]                  ex_funct7,        // Function field [31:25]
    output logic [6:0]                  ex_opcode         // Instruction opcode [6:0]
);

    /**
     * Pipeline Register Storage
     * 
     * These registers store the pipeline data between ID and EX stages.
     * They are updated synchronously on the positive edge of the clock.
     */
    
    // Register Data Registers
    logic [DATA_WIDTH-1:0] rs1_data_reg;
    logic [DATA_WIDTH-1:0] rs2_data_reg;
    
    // Register Address Registers
    logic [4:0] rs1_addr_reg;
    logic [4:0] rs2_addr_reg;
    logic [4:0] rd_addr_reg;
    
    // Immediate Value Register
    logic [DATA_WIDTH-1:0] immediate_reg;
    
    // PC Value Registers
    logic [ADDR_WIDTH-1:0] PC_reg;
    logic [ADDR_WIDTH-1:0] PC_plus_4_reg;
    
    // Control Signal Registers
    logic RegWrite_reg;
    logic MemRead_reg;
    logic MemWrite_reg;
    logic MemToReg_reg;
    logic ALUSrc_reg;
    logic [1:0] ALUOp_reg;
    logic Branch_reg;
    logic Jump_reg;
    
    // Instruction Field Registers
    logic [2:0] funct3_reg;
    logic [6:0] funct7_reg;
    logic [6:0] opcode_reg;
    
    /**
     * Pipeline Register Update Logic
     * 
     * The register updates based on control signals with the following priority:
     * 1. Reset (rst_n = 0): Initialize to zero
     * 2. Flush (flush = 1): Clear register (insert NOP)
     * 3. Stall (enable = 0): Hold current values
     * 4. Normal (enable = 1, flush = 0): Update with new data
     * 
     * NOP Instruction Handling:
     * - When flushed, control signals set to safe defaults (no operation)
     * - Register addresses set to zero (x0)
     * - Data values cleared
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Initialize all registers to zero
            rs1_data_reg <= {DATA_WIDTH{1'b0}};
            rs2_data_reg <= {DATA_WIDTH{1'b0}};
            rs1_addr_reg <= 5'b00000;
            rs2_addr_reg <= 5'b00000;
            rd_addr_reg <= 5'b00000;
            immediate_reg <= {DATA_WIDTH{1'b0}};
            PC_reg <= {ADDR_WIDTH{1'b0}};
            PC_plus_4_reg <= {ADDR_WIDTH{1'b0}};
            RegWrite_reg <= 1'b0;
            MemRead_reg <= 1'b0;
            MemWrite_reg <= 1'b0;
            MemToReg_reg <= 1'b0;
            ALUSrc_reg <= 1'b0;
            ALUOp_reg <= 2'b00;
            Branch_reg <= 1'b0;
            Jump_reg <= 1'b0;
            funct3_reg <= 3'b000;
            funct7_reg <= 7'b0000000;
            opcode_reg <= 7'b0000000;
        end else if (flush) begin
            // Flush: Clear register (insert NOP/bubble)
            // Set control signals to safe defaults (no operation)
            rs1_data_reg <= {DATA_WIDTH{1'b0}};
            rs2_data_reg <= {DATA_WIDTH{1'b0}};
            rs1_addr_reg <= 5'b00000;
            rs2_addr_reg <= 5'b00000;
            rd_addr_reg <= 5'b00000;
            immediate_reg <= {DATA_WIDTH{1'b0}};
            PC_reg <= id_PC;              // Preserve PC for debugging
            PC_plus_4_reg <= id_PC_plus_4; // Preserve PC+4 for debugging
            RegWrite_reg <= 1'b0;         // No register write
            MemRead_reg <= 1'b0;          // No memory read
            MemWrite_reg <= 1'b0;         // No memory write
            MemToReg_reg <= 1'b0;         // Don't care
            ALUSrc_reg <= 1'b0;           // Don't care
            ALUOp_reg <= 2'b00;           // ADD (safe default)
            Branch_reg <= 1'b0;           // No branch
            Jump_reg <= 1'b0;             // No jump
            funct3_reg <= 3'b000;         // Don't care
            funct7_reg <= 7'b0000000;     // Don't care
        end else if (enable) begin
            // Normal operation: Update register with new ID stage data
            rs1_data_reg <= id_rs1_data;
            rs2_data_reg <= id_rs2_data;
            rs1_addr_reg <= id_rs1_addr;
            rs2_addr_reg <= id_rs2_addr;
            rd_addr_reg <= id_rd_addr;
            immediate_reg <= id_immediate;
            PC_reg <= id_PC;
            PC_plus_4_reg <= id_PC_plus_4;
            RegWrite_reg <= id_RegWrite;
            MemRead_reg <= id_MemRead;
            MemWrite_reg <= id_MemWrite;
            MemToReg_reg <= id_MemToReg;
            ALUSrc_reg <= id_ALUSrc;
            ALUOp_reg <= id_ALUOp;
            Branch_reg <= id_Branch;
            Jump_reg <= id_Jump;
            funct3_reg <= id_funct3;
            funct7_reg <= id_funct7;
            opcode_reg <= id_opcode;
        end
        // If enable = 0 (stall), register holds current values (no change)
    end
    
    /**
     * Output Assignments
     * 
     * Connect internal registers to output ports.
     * Outputs are always available (combinational from registers).
     */
    assign ex_rs1_data = rs1_data_reg;
    assign ex_rs2_data = rs2_data_reg;
    assign ex_rs1_addr = rs1_addr_reg;
    assign ex_rs2_addr = rs2_addr_reg;
    assign ex_rd_addr = rd_addr_reg;
    assign ex_immediate = immediate_reg;
    assign ex_PC = PC_reg;
    assign ex_PC_plus_4 = PC_plus_4_reg;
    assign ex_RegWrite = RegWrite_reg;
    assign ex_MemRead = MemRead_reg;
    assign ex_MemWrite = MemWrite_reg;
    assign ex_MemToReg = MemToReg_reg;
    assign ex_ALUSrc = ALUSrc_reg;
    assign ex_ALUOp = ALUOp_reg;
    assign ex_Branch = Branch_reg;
    assign ex_Jump = Jump_reg;
    assign ex_funct3 = funct3_reg;
    assign ex_funct7 = funct7_reg;
    assign ex_opcode = opcode_reg;
    
    /**
     * Pipeline Register Contents Summary:
     * 
     * 1. Register Data (rs1_data, rs2_data):
     *    - Data read from register file in ID stage
     *    - Used as ALU operands in EX stage
     *    - May be replaced by forwarded data in EX stage
     * 
     * 2. Register Addresses (rs1_addr, rs2_addr, rd_addr):
     *    - Used for forwarding unit (rs1_addr, rs2_addr)
     *    - Used for hazard detection (rd_addr)
     *    - rd_addr used in EX/MEM and MEM/WB stages
     * 
     * 3. Immediate Value:
     *    - Sign-extended immediate from instruction
     *    - Used for ALU operations (I-type, S-type)
     *    - Used for address calculation (load/store)
     *    - Used for branch/jump target calculation
     * 
     * 4. PC Values (PC, PC+4):
     *    - PC: Used for branch/jump target calculation
     *    - PC+4: Used for return address (JAL/JALR)
     *    - Passed through to EX stage
     * 
     * 5. Control Signals:
     *    - RegWrite: Enable register write in WB stage
     *    - MemRead: Enable memory read in MEM stage
     *    - MemWrite: Enable memory write in MEM stage
     *    - MemToReg: Select memory data vs ALU result in WB stage
     *    - ALUSrc: Select immediate vs register for ALU operand B
     *    - ALUOp: ALU operation type (for ALU control unit)
     *    - Branch: Branch instruction (for branch control)
     *    - Jump: Jump instruction (for jump control)
     * 
     * 6. Instruction Fields (funct3, funct7):
     *    - Used by ALU control unit to determine specific operation
     *    - funct3: Function field for R-type, I-type, S-type, B-type
     *    - funct7: Function field for R-type (distinguishes ADD/SUB, SRL/SRA)
     */
    
    /**
     * Pipeline Register Behavior Summary:
     * 
     * 1. Normal Operation (enable=1, flush=0):
     *    - Register updates with new ID stage outputs
     *    - Data flows through pipeline normally
     *    - One instruction per clock cycle
     * 
     * 2. Stall Operation (enable=0):
     *    - Register holds current values
     *    - Prevents new instruction from entering EX stage
     *    - Used for data hazards (e.g., load-use hazard)
     *    - Pipeline bubble inserted
     * 
     * 3. Flush Operation (flush=1):
     *    - Register cleared (NOP inserted)
     *    - Control signals set to safe defaults
     *    - Used on branch misprediction or exception
     *    - Discards incorrect instruction
     * 
     * 4. Reset Operation (rst_n=0):
     *    - All registers initialized to zero
     *    - Pipeline starts in known state
     *    - First instruction fetched after reset release
     */
    
    /**
     * Pipeline Control Signal Usage:
     * 
     * Enable Signal (stall control):
     * - Source: Hazard Detection Unit
     * - Purpose: Prevent new instruction from entering EX stage
     * - When asserted (low): Register holds current values
     * - Used for: Data hazards, structural hazards
     * 
     * Flush Signal:
     * - Source: Control Unit (on branch taken, exception, etc.)
     * - Purpose: Clear incorrect instruction from pipeline
     * - When asserted (high): Register cleared, NOP inserted
     * - Used for: Branch misprediction, exceptions, interrupts
     * 
     * Reset Signal:
     * - Source: System reset
     * - Purpose: Initialize pipeline to known state
     * - When asserted (low): All registers cleared
     * - Used for: System initialization, error recovery
     */
    
    /**
     * Timing Considerations:
     * 
     * 1. Register Update:
     *    - Updates synchronously on clock edge
     *    - Requires setup time for inputs
     *    - Outputs stable after hold time
     * 
     * 2. Stall Behavior:
     *    - When stalled, register holds previous instruction
     *    - EX stage sees same instruction multiple cycles
     *    - Must be handled properly in EX stage
     * 
     * 3. Flush Behavior:
     *    - NOP inserted (control signals = 0)
     *    - EX stage should recognize and handle NOP
     *    - No operation performed in EX stage
     * 
     * 4. Reset Behavior:
     *    - All registers cleared to zero
     *    - Zero = safe default (no operation)
     *    - Pipeline starts in safe state
     * 
     * 5. Critical Path:
     *    - ID stage → ID/EX register → EX stage
     *    - Register adds one cycle latency
     *    - Enables pipelined operation
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Register Width:
     *    - All data registers: 32 bits (DATA_WIDTH)
     *    - Address registers: 5 bits (for 32 registers)
     *    - Control signals: 1 bit each (except ALUOp: 2 bits)
     * 
     * 2. Forwarding Support:
     *    - rs1_addr and rs2_addr provided for forwarding unit
     *    - Forwarding unit compares addresses to detect hazards
     *    - Forwarded data replaces register data in EX stage
     * 
     * 3. Hazard Detection Support:
     *    - rd_addr provided for hazard detection unit
     *    - Hazard unit compares rd_addr with future instruction rs1/rs2
     *    - Generates stall signal when hazard detected
     * 
     * 4. Control Signal Propagation:
     *    - Control signals propagate through pipeline registers
     *    - Some signals only used in specific stages:
     *      * EX stage: ALUSrc, ALUOp, Branch, Jump, funct3, funct7
     *      * MEM stage: MemRead, MemWrite, Branch
     *      * WB stage: RegWrite, MemToReg
     * 
     * 5. Synthesis:
     *    - Registers synthesize to flip-flops
     *    - Enable signal gates clock or data
     *    - Flush can be implemented as data mux
     */

endmodule

