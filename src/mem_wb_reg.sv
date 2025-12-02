/**
 * @file mem_wb_reg.sv
 * @brief MEM/WB Pipeline Register Module
 * 
 * This module implements the pipeline register between the Memory Access (MEM)
 * and Writeback (WB) stages of the RISC-V 5-stage pipeline.
 * 
 * @details
 * Pipeline Register Purpose:
 * - Stores memory access results and control signals from MEM stage
 * - Passes data to WB stage on next clock cycle
 * - Enables pipeline operation by breaking combinational paths
 * - Provides pipeline control through enable and flush signals
 * 
 * Pipeline Register Contents:
 * - Memory read data: Data read from memory (for load instructions)
 * - ALU result: ALU computation result (passthrough from EX stage)
 * - rd_addr: Destination register address
 * - Control signals: MemToReg, RegWrite
 * 
 * Pipeline Control:
 * - Enable (stall): When low, holds current values (prevents new data)
 * - Flush: Clears register (inserts NOP/bubble in pipeline)
 * - Reset: Initializes register to zero
 */

module mem_wb_reg #(
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
    // Inputs from MEM Stage
    // ============================================
    
    // Data from Memory Stage
    input  logic [DATA_WIDTH-1:0]       mem_read_data,    // Data read from memory
    input  logic [DATA_WIDTH-1:0]       mem_alu_result,    // ALU result (passthrough)
    
    // Register Address
    input  logic [4:0]                  mem_rd_addr,     // Destination register address
    
    // Control Signals from MEM Stage
    input  logic                        mem_MemToReg,      // Memory to register select
    input  logic                        mem_RegWrite,      // Register write enable
    
    // ============================================
    // Outputs to WB Stage
    // ============================================
    
    // Data for Writeback
    output logic [DATA_WIDTH-1:0]      wb_mem_read_data, // Memory read data
    output logic [DATA_WIDTH-1:0]      wb_alu_result,     // ALU result
    output logic [4:0]                  wb_rd_addr,       // Destination register address
    
    // Control Signals
    output logic                        wb_MemToReg,      // Memory to register select
    output logic                        wb_RegWrite        // Register write enable
);

    /**
     * Pipeline Register Storage
     * 
     * These registers store the pipeline data between MEM and WB stages.
     * They are updated synchronously on the positive edge of the clock.
     */
    
    // Data Registers
    logic [DATA_WIDTH-1:0] mem_read_data_reg;
    logic [DATA_WIDTH-1:0] alu_result_reg;
    
    // Register Address Register
    logic [4:0] rd_addr_reg;
    
    // Control Signal Registers
    logic MemToReg_reg;
    logic RegWrite_reg;
    
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
            mem_read_data_reg <= {DATA_WIDTH{1'b0}};
            alu_result_reg <= {DATA_WIDTH{1'b0}};
            rd_addr_reg <= 5'b00000;
            MemToReg_reg <= 1'b0;
            RegWrite_reg <= 1'b0;
        end else if (flush) begin
            // Flush: Clear register (insert NOP/bubble)
            // Set control signals to safe defaults (no operation)
            mem_read_data_reg <= {DATA_WIDTH{1'b0}};
            alu_result_reg <= {DATA_WIDTH{1'b0}};
            rd_addr_reg <= 5'b00000;
            MemToReg_reg <= 1'b0;        // Don't care
            RegWrite_reg <= 1'b0;        // No register write
        end else if (enable) begin
            // Normal operation: Update register with new MEM stage data
            mem_read_data_reg <= mem_read_data;
            alu_result_reg <= mem_alu_result;
            rd_addr_reg <= mem_rd_addr;
            MemToReg_reg <= mem_MemToReg;
            RegWrite_reg <= mem_RegWrite;
        end
        // If enable = 0 (stall), register holds current values (no change)
    end
    
    /**
     * Output Assignments
     * 
     * Connect internal registers to output ports.
     * Outputs are always available (combinational from registers).
     */
    assign wb_mem_read_data = mem_read_data_reg;
    assign wb_alu_result = alu_result_reg;
    assign wb_rd_addr = rd_addr_reg;
    assign wb_MemToReg = MemToReg_reg;
    assign wb_RegWrite = RegWrite_reg;
    
    /**
     * Pipeline Register Contents Summary:
     * 
     * 1. Memory Read Data (mem_read_data):
     *    - Data read from memory (for load instructions)
     *    - Used when MemToReg = 1
     *    - Written to register file in WB stage
     *    - Used by forwarding unit (MEM/WB → EX forwarding)
     * 
     * 2. ALU Result (alu_result):
     *    - ALU computation result (for arithmetic/logic instructions)
     *    - Used when MemToReg = 0
     *    - Written to register file in WB stage
     *    - Used by forwarding unit (MEM/WB → EX forwarding)
     * 
     * 3. Register Address (rd_addr):
     *    - Destination register address
     *    - Used by forwarding unit (MEM/WB → EX forwarding)
     *    - Used by hazard detection unit
     *    - Used by register file for write operation
     * 
     * 4. Control Signals:
     *    - MemToReg: Selects memory data vs ALU result for writeback
     *    - RegWrite: Enables register file write operation
     */
    
    /**
     * Pipeline Register Behavior Summary:
     * 
     * 1. Normal Operation (enable=1, flush=0):
     *    - Register updates with new MEM stage outputs
     *    - Data flows through pipeline normally
     *    - One instruction per clock cycle
     * 
     * 2. Stall Operation (enable=0):
     *    - Register holds current values
     *    - Prevents new instruction from entering WB stage
     *    - Used for data hazards (rare, usually handled by forwarding)
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
     * Forwarding Support:
     * 
     * The MEM/WB register provides data for forwarding:
     * - wb_alu_result: Used for MEM/WB → EX forwarding
     * - wb_mem_read_data: Used for MEM/WB → EX forwarding (load instructions)
     * - wb_rd_addr: Used by forwarding unit to detect hazards
     * - wb_RegWrite: Used by forwarding unit (only forward if RegWrite = 1)
     * 
     * Forwarding Path:
     * - MEM/WB register → Forwarding Unit → EX stage ALU operands
     * - Bypasses register file for data hazards
     * - Improves pipeline performance
     */
    
    /**
     * Writeback Stage Interface:
     * 
     * The MEM/WB register provides all data needed for writeback:
     * - Two data sources: ALU result and memory read data
     * - Control signal: MemToReg (selects which data source)
     * - Write enable: RegWrite (enables register file write)
     * - Destination: rd_addr (register to write)
     * 
     * WB Stage Operation:
     * - MemToReg mux selects: ALU result (0) or memory data (1)
     * - Selected data written to register file if RegWrite = 1
     * - Write occurs synchronously on clock edge
     */

endmodule

