/**
 * @file if_id_reg.sv
 * @brief IF/ID Pipeline Register Module
 * 
 * This module implements the pipeline register between the Instruction Fetch (IF)
 * and Instruction Decode (ID) stages of the RISC-V 5-stage pipeline.
 * 
 * @details
 * Pipeline Register Purpose:
 * - Stores instruction and PC values from IF stage
 * - Passes data to ID stage on next clock cycle
 * - Enables pipeline operation by breaking combinational paths
 * - Provides pipeline control through enable and flush signals
 * 
 * Pipeline Control:
 * - Enable (stall): When low, holds current values (prevents new data)
 * - Flush: Clears register (inserts NOP/bubble in pipeline)
 * - Reset: Initializes register to zero
 * 
 * Pipeline Register Contents:
 * - instruction: 32-bit instruction word from IF stage
 * - PC: Current program counter value (for branch target calculation)
 * 
 * Control Signal Behavior:
 * - Normal operation: Register updates with new IF stage outputs
 * - Stall (enable=0): Register holds current values (bubble inserted)
 * - Flush: Register cleared (NOP instruction inserted)
 * - Reset: Register initialized to zero
 */

module if_id_reg #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32             // Address width (32 bits)
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // Pipeline Control Signals
    input  logic                        enable,           // Enable signal (0 = stall, 1 = normal)
    input  logic                        flush,            // Flush signal (1 = clear register)
    
    // Inputs from IF Stage
    input  logic [DATA_WIDTH-1:0]       if_instruction,  // Instruction from IF stage
    input  logic [ADDR_WIDTH-1:0]       if_PC,            // PC value from IF stage
    input  logic [ADDR_WIDTH-1:0]       if_PC_plus_4,     // PC+4 from IF stage
    
    // Outputs to ID Stage
    output logic [DATA_WIDTH-1:0]      id_instruction,   // Instruction to ID stage
    output logic [ADDR_WIDTH-1:0]       id_PC,            // PC value to ID stage
    output logic [ADDR_WIDTH-1:0]       id_PC_plus_4     // PC+4 to ID stage
);

    /**
     * Pipeline Register Storage
     * 
     * These registers store the pipeline data between IF and ID stages.
     * They are updated synchronously on the positive edge of the clock.
     */
    logic [DATA_WIDTH-1:0] instruction_reg;
    logic [ADDR_WIDTH-1:0] PC_reg;
    logic [ADDR_WIDTH-1:0] PC_plus_4_reg;
    
    /**
     * Pipeline Register Update Logic
     * 
     * The register updates based on control signals with the following priority:
     * 1. Reset (rst_n = 0): Initialize to zero
     * 2. Flush (flush = 1): Clear register (insert NOP)
     * 3. Stall (enable = 0): Hold current values
     * 4. Normal (enable = 1, flush = 0): Update with new data
     * 
     * NOP Instruction:
     * - When flushed, register contains NOP = ADDI x0, x0, 0 = 0x00000013
     * - NOP doesn't modify any processor state
     * - PC values are preserved (may be used for debugging)
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Initialize all registers to zero
            instruction_reg <= {DATA_WIDTH{1'b0}};
            PC_reg <= {ADDR_WIDTH{1'b0}};
            PC_plus_4_reg <= {ADDR_WIDTH{1'b0}};
        end else if (flush) begin
            // Flush: Clear instruction register (insert NOP)
            // NOP = ADDI x0, x0, 0 = 0x00000013
            instruction_reg <= 32'h00000013;
            // PC values are preserved for debugging/trace purposes
            // Alternatively, can be cleared: PC_reg <= {ADDR_WIDTH{1'b0}};
            PC_reg <= if_PC;
            PC_plus_4_reg <= if_PC_plus_4;
        end else if (enable) begin
            // Normal operation: Update register with new IF stage data
            instruction_reg <= if_instruction;
            PC_reg <= if_PC;
            PC_plus_4_reg <= if_PC_plus_4;
        end
        // If enable = 0 (stall), register holds current values (no change)
    end
    
    /**
     * Output Assignments
     * 
     * Connect internal registers to output ports.
     * Outputs are always available (combinational from registers).
     */
    assign id_instruction = instruction_reg;
    assign id_PC = PC_reg;
    assign id_PC_plus_4 = PC_plus_4_reg;
    
    /**
     * Pipeline Register Behavior Summary:
     * 
     * 1. Normal Operation (enable=1, flush=0):
     *    - Register updates with new IF stage outputs
     *    - Data flows through pipeline normally
     *    - One instruction per clock cycle
     * 
     * 2. Stall Operation (enable=0):
     *    - Register holds current values
     *    - Prevents new instruction from entering ID stage
     *    - Used for data hazards (e.g., load-use hazard)
     *    - Pipeline bubble inserted
     * 
     * 3. Flush Operation (flush=1):
     *    - Instruction register cleared (NOP inserted)
     *    - PC values may be preserved or cleared
     *    - Used on branch misprediction or exception
     *    - Discards incorrect instruction
     * 
     * 4. Reset Operation (rst_n=0):
     *    - All registers initialized to zero
     *    - Pipeline starts in known state
     *    - First instruction fetched after reset release
     * 
     * 5. Timing:
     *    - Register updates on positive clock edge
     *    - Outputs available immediately after clock edge
     *    - Setup/hold time requirements must be met
     */
    
    /**
     * Pipeline Control Signal Usage:
     * 
     * Enable Signal (stall control):
     * - Source: Hazard Detection Unit
     * - Purpose: Prevent new instruction from entering ID stage
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
     * Pipeline Register Contents:
     * 
     * 1. Instruction (id_instruction):
     *    - 32-bit RISC-V instruction word
    *    - Decoded by ID stage
     *    - Contains opcode, registers, immediate, etc.
     * 
     * 2. PC (id_PC):
     *    - Current program counter value
     *    - Used for branch target calculation
     *    - Used for exception handling
     *    - Used for debugging/tracing
     * 
     * 3. PC+4 (id_PC_plus_4):
     *    - Next sequential instruction address
     *    - Used for branch target calculation (PC + immediate)
     *    - Used for return address (JAL/JALR)
     *    - Used for exception return address
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Register Update:
     *    - Updates synchronously on clock edge
     *    - Requires setup time for inputs
     *    - Outputs stable after hold time
     * 
     * 2. Stall Behavior:
     *    - When stalled, register holds previous instruction
     *    - ID stage sees same instruction multiple cycles
     *    - Must be handled properly in ID stage
     * 
     * 3. Flush Behavior:
     *    - NOP instruction (0x00000013) inserted
     *    - NOP = ADDI x0, x0, 0 (no operation)
     *    - ID stage should recognize and handle NOP
     * 
     * 4. Reset Behavior:
     *    - All registers cleared to zero
     *    - Zero = NOP instruction (safe default)
     *    - Pipeline starts in safe state
     * 
     * 5. Timing Considerations:
     *    - Critical path: IF stage → IF/ID register → ID stage
     *    - Register adds one cycle latency
     *    - Enables pipelined operation
     * 
     * 6. Synthesis:
     *    - Registers synthesize to flip-flops
     *    - Enable signal gates clock or data
     *    - Flush can be implemented as data mux
     */

endmodule

