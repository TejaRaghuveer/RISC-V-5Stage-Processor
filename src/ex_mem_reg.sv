/**
 * @file ex_mem_reg.sv
 * @brief EX/MEM Pipeline Register Module
 * 
 * This module implements the pipeline register between the Execute (EX)
 * and Memory Access (MEM) stages of the RISC-V 5-stage pipeline.
 * 
 * @details
 * Pipeline Register Purpose:
 * - Stores computation results and control signals from EX stage
 * - Passes data to MEM stage on next clock cycle
 * - Enables pipeline operation by breaking combinational paths
 * - Provides pipeline control through enable and flush signals
 * 
 * Pipeline Register Contents:
 * - ALU result: Computation result or memory address
 * - rs2_data: Source register 2 data (for store instructions)
 * - rd_addr: Destination register address
 * - Control signals: MemRead, MemWrite, MemToReg, RegWrite
 * - Branch/jump information: branch_taken, branch_target, jump_target
 * 
 * Pipeline Control:
 * - Enable (stall): When low, holds current values (prevents new data)
 * - Flush: Clears register (inserts NOP/bubble in pipeline)
 * - Reset: Initializes register to zero
 */

module ex_mem_reg #(
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
    // Inputs from EX Stage
    // ============================================
    
    // Computation Results
    input  logic [DATA_WIDTH-1:0]       ex_alu_result,    // ALU computation result
    input  logic [DATA_WIDTH-1:0]       ex_rs2_data,      // Source register 2 data (for stores)
    
    // Register Address
    input  logic [4:0]                  ex_rd_addr,       // Destination register address
    
    // Control Signals from EX Stage
    input  logic                        ex_MemRead,       // Memory read enable
    input  logic                        ex_MemWrite,      // Memory write enable
    input  logic                        ex_MemToReg,      // Memory to register select
    input  logic                        ex_RegWrite,      // Register write enable
    
    // Branch/Jump Information
    input  logic                        ex_branch_taken,  // Branch condition evaluation
    input  logic [ADDR_WIDTH-1:0]       ex_branch_target, // Branch target address
    input  logic [ADDR_WIDTH-1:0]       ex_jump_target,   // Jump target address
    
    // ============================================
    // Outputs to MEM Stage
    // ============================================
    
    // Computation Results
    output logic [DATA_WIDTH-1:0]      mem_alu_result,   // ALU result (memory address)
    output logic [DATA_WIDTH-1:0]      mem_rs2_data,      // Source register 2 data (for stores)
    
    // Register Address
    output logic [4:0]                  mem_rd_addr,      // Destination register address
    
    // Control Signals
    output logic                        mem_MemRead,      // Memory read enable
    output logic                        mem_MemWrite,     // Memory write enable
    output logic                        mem_MemToReg,     // Memory to register select
    output logic                        mem_RegWrite,     // Register write enable
    
    // Branch/Jump Information
    output logic                        mem_branch_taken, // Branch condition evaluation
    output logic [ADDR_WIDTH-1:0]       mem_branch_target, // Branch target address
    output logic [ADDR_WIDTH-1:0]       mem_jump_target   // Jump target address
);

    /**
     * Pipeline Register Storage
     * 
     * These registers store the pipeline data between EX and MEM stages.
     * They are updated synchronously on the positive edge of the clock.
     */
    
    // Computation Result Registers
    logic [DATA_WIDTH-1:0] alu_result_reg;
    logic [DATA_WIDTH-1:0] rs2_data_reg;
    
    // Register Address Register
    logic [4:0] rd_addr_reg;
    
    // Control Signal Registers
    logic MemRead_reg;
    logic MemWrite_reg;
    logic MemToReg_reg;
    logic RegWrite_reg;
    
    // Branch/Jump Registers
    logic branch_taken_reg;
    logic [ADDR_WIDTH-1:0] branch_target_reg;
    logic [ADDR_WIDTH-1:0] jump_target_reg;
    
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
            alu_result_reg <= {DATA_WIDTH{1'b0}};
            rs2_data_reg <= {DATA_WIDTH{1'b0}};
            rd_addr_reg <= 5'b00000;
            MemRead_reg <= 1'b0;
            MemWrite_reg <= 1'b0;
            MemToReg_reg <= 1'b0;
            RegWrite_reg <= 1'b0;
            branch_taken_reg <= 1'b0;
            branch_target_reg <= {ADDR_WIDTH{1'b0}};
            jump_target_reg <= {ADDR_WIDTH{1'b0}};
        end else if (flush) begin
            // Flush: Clear register (insert NOP/bubble)
            // Set control signals to safe defaults (no operation)
            alu_result_reg <= {DATA_WIDTH{1'b0}};
            rs2_data_reg <= {DATA_WIDTH{1'b0}};
            rd_addr_reg <= 5'b00000;
            MemRead_reg <= 1'b0;         // No memory read
            MemWrite_reg <= 1'b0;        // No memory write
            MemToReg_reg <= 1'b0;        // Don't care
            RegWrite_reg <= 1'b0;        // No register write
            branch_taken_reg <= 1'b0;    // No branch
            branch_target_reg <= {ADDR_WIDTH{1'b0}};
            jump_target_reg <= {ADDR_WIDTH{1'b0}};
        end else if (enable) begin
            // Normal operation: Update register with new EX stage data
            alu_result_reg <= ex_alu_result;
            rs2_data_reg <= ex_rs2_data;
            rd_addr_reg <= ex_rd_addr;
            MemRead_reg <= ex_MemRead;
            MemWrite_reg <= ex_MemWrite;
            MemToReg_reg <= ex_MemToReg;
            RegWrite_reg <= ex_RegWrite;
            branch_taken_reg <= ex_branch_taken;
            branch_target_reg <= ex_branch_target;
            jump_target_reg <= ex_jump_target;
        end
        // If enable = 0 (stall), register holds current values (no change)
    end
    
    /**
     * Output Assignments
     * 
     * Connect internal registers to output ports.
     * Outputs are always available (combinational from registers).
     */
    assign mem_alu_result = alu_result_reg;
    assign mem_rs2_data = rs2_data_reg;
    assign mem_rd_addr = rd_addr_reg;
    assign mem_MemRead = MemRead_reg;
    assign mem_MemWrite = MemWrite_reg;
    assign mem_MemToReg = MemToReg_reg;
    assign mem_RegWrite = RegWrite_reg;
    assign mem_branch_taken = branch_taken_reg;
    assign mem_branch_target = branch_target_reg;
    assign mem_jump_target = jump_target_reg;
    
    /**
     * Pipeline Register Contents Summary:
     * 
     * 1. ALU Result (alu_result):
     *    - Computation result for arithmetic/logic instructions
     *    - Memory address for load/store instructions
     *    - Used by MEM stage for memory access
     *    - Passed through to MEM/WB register
     * 
     * 2. rs2 Data (rs2_data):
     *    - Source register 2 data (for store instructions)
     *    - Written to memory when MemWrite = 1
     *    - May be forwarded from earlier stages
     * 
     * 3. Register Address (rd_addr):
     *    - Destination register address
     *    - Used by forwarding unit (MEM→EX forwarding)
     *    - Used by hazard detection unit
     *    - Passed to MEM/WB register for writeback
     * 
     * 4. Control Signals:
     *    - MemRead: Enable memory read in MEM stage
     *    - MemWrite: Enable memory write in MEM stage
     *    - MemToReg: Select memory data vs ALU result in WB stage
     *    - RegWrite: Enable register write in WB stage
     * 
     * 5. Branch/Jump Information:
     *    - branch_taken: Branch condition evaluation result
     *    - branch_target: Branch target address (for PC update)
     *    - jump_target: Jump target address (for PC update)
     *    - Used by IF stage for PC update
     */
    
    /**
     * Pipeline Register Behavior Summary:
     * 
     * 1. Normal Operation (enable=1, flush=0):
     *    - Register updates with new EX stage outputs
     *    - Data flows through pipeline normally
     *    - One instruction per clock cycle
     * 
     * 2. Stall Operation (enable=0):
     *    - Register holds current values
     *    - Prevents new instruction from entering MEM stage
     *    - Used for data hazards
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

endmodule

