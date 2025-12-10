/**
 * @file performance_monitor.sv
 * @brief RISC-V Pipeline Performance Monitoring Module
 * 
 * This module monitors and counts various performance metrics for the RISC-V 5-stage pipeline:
 * - Total clock cycles
 * - Instructions completed (WB stage commits)
 * - Pipeline stalls (from hazard detection)
 * - Pipeline flushes (from branch/jump)
 * - Load/store instruction counts
 * - Branch instruction counts
 * - CPI (Cycles Per Instruction) calculation
 * 
 * @details
 * Performance Metrics:
 * 
 * 1. Total Cycles:
 *    - Increments every clock cycle when monitoring is enabled
 *    - Represents total execution time
 * 
 * 2. Instructions Completed:
 *    - Counts instructions that complete writeback (wb_RegWrite asserted)
 *    - Excludes writes to x0 (hardwired zero register)
 *    - Represents useful work completed
 * 
 * 3. Pipeline Stalls:
 *    - Counts cycles when pipeline is stalled (hazard_stall or external stall)
 *    - Stalls occur due to load-use hazards that cannot be resolved by forwarding
 *    - Each stall cycle represents lost performance
 * 
 * 4. Pipeline Flushes:
 *    - Counts cycles when pipeline registers are flushed
 *    - Flushes occur when branch/jump is taken (incorrect instructions fetched)
 *    - Each flush represents wasted fetch/decode cycles
 * 
 * 5. Load/Store Instructions:
 *    - Counts load instructions (mem_MemRead asserted)
 *    - Counts store instructions (mem_MemWrite asserted)
 *    - Useful for analyzing memory access patterns
 * 
 * 6. Branch Instructions:
 *    - Counts branch instructions executed (mem_Branch asserted)
 *    - Includes all branch types (BEQ, BNE, BLT, BGE, BLTU, BGEU)
 *    - Useful for analyzing control flow behavior
 * 
 * 7. CPI (Cycles Per Instruction):
 *    - Calculated as: total_cycles / instructions_completed
 *    - Lower CPI indicates better performance
 *    - Ideal CPI = 1.0 (one instruction per cycle)
 *    - Actual CPI > 1.0 due to stalls and flushes
 * 
 * Usage:
 * - Connect to pipeline signals at appropriate stages
 * - Enable monitoring with enable signal
 * - Read counter values for performance analysis
 * - Reset counters for new measurement periods
 */

module performance_monitor #(
    parameter COUNTER_WIDTH = 32,      // Width of performance counters (32 bits)
    parameter CPI_WIDTH = 16            // Width of CPI fractional part (for fixed-point)
) (
    // Clock and Reset
    input  logic                        clk,                // Clock signal
    input  logic                        rst_n,              // Active-low reset
    
    // Enable Signal
    input  logic                        enable,              // Enable performance monitoring (active high)
    
    // Pipeline Signals for Monitoring
    // WB Stage Signals (for instruction completion)
    input  logic                        wb_RegWrite,        // Register write enable (indicates instruction completion)
    input  logic [4:0]                  wb_rd_addr,         // Destination register address (to exclude x0)
    
    // Pipeline Control Signals (for stall/flush counting)
    input  logic                        pipeline_stall,     // Pipeline stall signal (from hazard detection or external)
    input  logic                        pipeline_flush,     // Pipeline flush signal (from branch/jump or external)
    
    // MEM Stage Signals (for load/store counting)
    input  logic                        mem_MemRead,        // Memory read enable (load instruction)
    input  logic                        mem_MemWrite,       // Memory write enable (store instruction)
    
    // EX/MEM Register Signals (for branch counting)
    input  logic                        mem_Branch,         // Branch instruction indicator
    
    // Performance Counter Outputs
    output logic [COUNTER_WIDTH-1:0]    total_cycles,       // Total clock cycles counted
    output logic [COUNTER_WIDTH-1:0]    instructions_completed, // Instructions completed (WB commits)
    output logic [COUNTER_WIDTH-1:0]    pipeline_stalls,    // Pipeline stall cycles
    output logic [COUNTER_WIDTH-1:0]    pipeline_flushes,   // Pipeline flush cycles
    output logic [COUNTER_WIDTH-1:0]    load_instructions,  // Load instruction count
    output logic [COUNTER_WIDTH-1:0]    store_instructions, // Store instruction count
    output logic [COUNTER_WIDTH-1:0]    branch_instructions, // Branch instruction count
    
    // CPI Output (fixed-point representation: integer part + fractional part)
    // CPI = (total_cycles << CPI_WIDTH) / instructions_completed
    // To get actual CPI: cpi_value / (2^CPI_WIDTH)
    output logic [COUNTER_WIDTH+CPI_WIDTH-1:0] cpi_value    // CPI in fixed-point format
);

    // ============================================
    // Internal Counter Registers
    // ============================================
    
    /**
     * Total Cycles Counter
     * 
     * Increments every clock cycle when monitoring is enabled.
     * Represents total execution time in clock cycles.
     * 
     * Purpose: Measure overall execution time
     * Increment Condition: enable == 1'b1 (every cycle)
     */
    logic [COUNTER_WIDTH-1:0] total_cycles_reg;
    
    /**
     * Instructions Completed Counter
     * 
     * Increments when an instruction completes writeback (wb_RegWrite asserted).
     * Excludes writes to register x0 (hardwired to zero).
     * 
     * Purpose: Count useful work completed
     * Increment Condition: enable && wb_RegWrite && (wb_rd_addr != 5'b00000)
     */
    logic [COUNTER_WIDTH-1:0] instructions_completed_reg;
    
    /**
     * Pipeline Stalls Counter
     * 
     * Increments when pipeline is stalled (pipeline_stall asserted).
     * Stalls occur due to load-use hazards that cannot be resolved by forwarding.
     * 
     * Purpose: Measure performance loss due to data hazards
     * Increment Condition: enable && pipeline_stall
     */
    logic [COUNTER_WIDTH-1:0] pipeline_stalls_reg;
    
    /**
     * Pipeline Flushes Counter
     * 
     * Increments when pipeline registers are flushed (pipeline_flush asserted).
     * Flushes occur when branch/jump is taken (incorrect instructions fetched).
     * 
     * Purpose: Measure performance loss due to control hazards
     * Increment Condition: enable && pipeline_flush
     */
    logic [COUNTER_WIDTH-1:0] pipeline_flushes_reg;
    
    /**
     * Load Instructions Counter
     * 
     * Increments when a load instruction is in MEM stage (mem_MemRead asserted).
     * Counts LW, LH, LB, LHU, LBU instructions.
     * 
     * Purpose: Analyze memory read access patterns
     * Increment Condition: enable && mem_MemRead
     */
    logic [COUNTER_WIDTH-1:0] load_instructions_reg;
    
    /**
     * Store Instructions Counter
     * 
     * Increments when a store instruction is in MEM stage (mem_MemWrite asserted).
     * Counts SW, SH, SB instructions.
     * 
     * Purpose: Analyze memory write access patterns
     * Increment Condition: enable && mem_MemWrite
     */
    logic [COUNTER_WIDTH-1:0] store_instructions_reg;
    
    /**
     * Branch Instructions Counter
     * 
     * Increments when a branch instruction is in MEM stage (mem_Branch asserted).
     * Counts BEQ, BNE, BLT, BGE, BLTU, BGEU instructions.
     * 
     * Purpose: Analyze control flow behavior
     * Increment Condition: enable && mem_Branch
     */
    logic [COUNTER_WIDTH-1:0] branch_instructions_reg;
    
    // ============================================
    // CPI Calculation Signals
    // ============================================
    
    /**
     * CPI Calculation
     * 
     * CPI (Cycles Per Instruction) = total_cycles / instructions_completed
     * 
     * To avoid floating-point division, we use fixed-point arithmetic:
     * - Multiply total_cycles by 2^CPI_WIDTH
     * - Divide by instructions_completed
     * - Result is CPI in fixed-point format
     * 
     * Example: CPI_WIDTH = 16
     * - CPI = 1.5 → cpi_value = 1.5 * 65536 = 98304
     * - To get CPI: cpi_value / 65536 = 1.5
     * 
     * Special Cases:
     * - If instructions_completed == 0: cpi_value = 0 (avoid division by zero)
     * - CPI calculation is combinational (updated every cycle)
     */
    logic [COUNTER_WIDTH+CPI_WIDTH-1:0] cpi_value_reg;
    
    // ============================================
    // Counter Update Logic
    // ============================================
    
    /**
     * Counter Update Process
     * 
     * All counters update synchronously on clock edge.
     * Counters increment when:
     * - Monitoring is enabled (enable == 1'b1)
     * - Specific condition is met (e.g., wb_RegWrite for instruction completion)
     * 
     * Reset Behavior:
     * - All counters reset to 0 on reset (rst_n == 1'b0)
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all counters to zero
            total_cycles_reg <= {COUNTER_WIDTH{1'b0}};
            instructions_completed_reg <= {COUNTER_WIDTH{1'b0}};
            pipeline_stalls_reg <= {COUNTER_WIDTH{1'b0}};
            pipeline_flushes_reg <= {COUNTER_WIDTH{1'b0}};
            load_instructions_reg <= {COUNTER_WIDTH{1'b0}};
            store_instructions_reg <= {COUNTER_WIDTH{1'b0}};
            branch_instructions_reg <= {COUNTER_WIDTH{1'b0}};
        end else if (enable) begin
            // Total cycles: Increment every cycle when enabled
            total_cycles_reg <= total_cycles_reg + 1'b1;
            
            // Instructions completed: Increment when instruction completes writeback
            // Exclude writes to x0 (hardwired zero register)
            if (wb_RegWrite && (wb_rd_addr != 5'b00000)) begin
                instructions_completed_reg <= instructions_completed_reg + 1'b1;
            end
            
            // Pipeline stalls: Increment when pipeline is stalled
            if (pipeline_stall) begin
                pipeline_stalls_reg <= pipeline_stalls_reg + 1'b1;
            end
            
            // Pipeline flushes: Increment when pipeline is flushed
            if (pipeline_flush) begin
                pipeline_flushes_reg <= pipeline_flushes_reg + 1'b1;
            end
            
            // Load instructions: Increment when load instruction is in MEM stage
            if (mem_MemRead) begin
                load_instructions_reg <= load_instructions_reg + 1'b1;
            end
            
            // Store instructions: Increment when store instruction is in MEM stage
            if (mem_MemWrite) begin
                store_instructions_reg <= store_instructions_reg + 1'b1;
            end
            
            // Branch instructions: Increment when branch instruction is in MEM stage
            if (mem_Branch) begin
                branch_instructions_reg <= branch_instructions_reg + 1'b1;
            end
        end
        // If enable is low, counters hold their values (no update)
    end
    
    // ============================================
    // CPI Calculation Logic
    // ============================================
    
    /**
     * CPI Calculation (Combinational)
     * 
     * Calculates CPI using fixed-point arithmetic to avoid floating-point division.
     * 
     * Formula: CPI = (total_cycles << CPI_WIDTH) / instructions_completed
     * 
     * Fixed-Point Format:
     * - Integer part: Upper COUNTER_WIDTH bits
     * - Fractional part: Lower CPI_WIDTH bits
     * - Total width: COUNTER_WIDTH + CPI_WIDTH bits
     * 
     * Example (CPI_WIDTH = 16):
     * - CPI = 1.5 → cpi_value = 98304 (0x18000)
     * - CPI = 2.25 → cpi_value = 147456 (0x24000)
     * 
     * Division by Zero Handling:
     * - If instructions_completed == 0: cpi_value = 0
     * - Prevents division by zero error
     */
    always_comb begin
        if (instructions_completed_reg == {COUNTER_WIDTH{1'b0}}) begin
            // No instructions completed yet: CPI is undefined, set to 0
            cpi_value_reg = {(COUNTER_WIDTH+CPI_WIDTH){1'b0}};
        end else begin
            // Calculate CPI in fixed-point format
            // CPI = (total_cycles * 2^CPI_WIDTH) / instructions_completed
            // Use integer division (truncates fractional part)
            cpi_value_reg = (total_cycles_reg << CPI_WIDTH) / instructions_completed_reg;
        end
    end
    
    // ============================================
    // Output Assignments
    // ============================================
    
    /**
     * Output Signal Assignments
     * 
     * Connect internal registers to output ports.
     * All outputs are registered for proper timing.
     */
    assign total_cycles = total_cycles_reg;
    assign instructions_completed = instructions_completed_reg;
    assign pipeline_stalls = pipeline_stalls_reg;
    assign pipeline_flushes = pipeline_flushes_reg;
    assign load_instructions = load_instructions_reg;
    assign store_instructions = store_instructions_reg;
    assign branch_instructions = branch_instructions_reg;
    assign cpi_value = cpi_value_reg;
    
    // ============================================
    // Performance Metrics Summary
    // ============================================
    
    /**
     * Performance Metrics Interpretation:
     * 
     * 1. Total Cycles:
     *    - Total execution time in clock cycles
     *    - Includes all cycles: useful work, stalls, flushes
     * 
     * 2. Instructions Completed:
     *    - Number of instructions that completed writeback
     *    - Excludes NOPs and writes to x0
     * 
     * 3. Pipeline Stalls:
     *    - Cycles lost due to load-use hazards
     *    - Each stall represents one cycle of lost performance
     *    - Stalls occur when forwarding cannot resolve data hazard
     * 
     * 4. Pipeline Flushes:
     *    - Cycles lost due to branch/jump misprediction
     *    - Each flush clears 2 pipeline stages (IF/ID, ID/EX)
     *    - Represents wasted fetch/decode cycles
     * 
     * 5. Load/Store Instructions:
     *    - Memory access instruction counts
     *    - Useful for analyzing memory access patterns
     *    - High load/store ratio may indicate memory-bound workload
     * 
     * 6. Branch Instructions:
     *    - Control flow instruction count
     *    - High branch count may indicate complex control flow
     *    - Branch misprediction rate = flushes / branches
     * 
     * 7. CPI (Cycles Per Instruction):
     *    - Average cycles per instruction
     *    - Ideal CPI = 1.0 (one instruction per cycle)
     *    - Actual CPI > 1.0 due to stalls and flushes
     *    - Lower CPI indicates better performance
     * 
     * Performance Analysis:
     * - CPI = 1.0: Perfect pipeline utilization (no stalls/flushes)
     * - CPI = 1.5: Moderate pipeline efficiency
     * - CPI = 2.0+: Poor pipeline efficiency (many stalls/flushes)
     * 
     * Efficiency Metrics:
     * - Pipeline Efficiency = 1.0 / CPI
     * - Stall Rate = pipeline_stalls / total_cycles
     * - Flush Rate = pipeline_flushes / total_cycles
     * - Instruction Throughput = instructions_completed / total_cycles
     */
    
endmodule

