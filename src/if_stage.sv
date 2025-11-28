/**
 * @file if_stage.sv
 * @brief RISC-V Instruction Fetch (IF) Stage Module
 * 
 * This module implements the Instruction Fetch stage of the RISC-V 5-stage pipeline.
 * The IF stage is responsible for fetching instructions from instruction memory and
 * updating the program counter (PC) for the next instruction.
 * 
 * @details
 * Instruction Fetch Stage Responsibilities:
 * - Maintain the Program Counter (PC) register
 * - Fetch instructions from instruction memory
 * - Calculate PC+4 for next sequential instruction
 * - Handle branch/jump target updates
 * - Handle pipeline stalls and flushes
 * 
 * Pipeline Control:
 * - Stall: Prevents PC update, holds current instruction (inserts bubble)
 * - Flush: Clears pipeline register, discards current instruction
 * - Branch Taken: Updates PC to branch/jump target address
 * 
 * Memory Interface:
 * - Instruction memory is read-only during execution
 * - Address is 32-bit aligned (lower 2 bits are always 00)
 * - Instruction width is 32 bits (RV32I)
 */

module if_stage #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32,            // Address width (32 bits)
    parameter IMEM_DEPTH = 1024,          // Instruction memory depth (number of words)
    parameter IMEM_ADDR_WIDTH = 10        // Instruction memory address width (log2(IMEM_DEPTH))
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // Program Counter Control
    input  logic [ADDR_WIDTH-1:0]       branch_target,   // Branch/jump target address
    input  logic                        branch_taken,    // Branch/jump taken signal (from EX stage)
    
    // Pipeline Control Signals
    input  logic                        stall,            // Pipeline stall signal (from hazard unit)
    input  logic                        flush,           // Pipeline flush signal (on branch misprediction)
    
    // Instruction Memory Interface
    output logic [IMEM_ADDR_WIDTH-1:0] imem_addr,       // Instruction memory address
    input  logic [DATA_WIDTH-1:0]       imem_data,       // Instruction data from memory
    
    // Pipeline Register Outputs (to ID stage)
    output logic [DATA_WIDTH-1:0]      instruction,      // Fetched instruction
    output logic [ADDR_WIDTH-1:0]      PC,               // Current program counter value
    output logic [ADDR_WIDTH-1:0]      PC_plus_4        // PC + 4 (for next sequential instruction)
);

    /**
     * Program Counter (PC) Register
     * 
     * The PC register holds the address of the current instruction being fetched.
     * In RISC-V:
     * - PC points to the instruction being executed
     * - Instructions are 32 bits (4 bytes) wide
     * - PC is word-aligned (lower 2 bits are always 00)
     * - Reset value: Typically 0x00000000 or configurable reset vector
     */
    logic [ADDR_WIDTH-1:0] PC_reg;
    
    /**
     * PC Update Logic
     * 
     * The PC can be updated in three ways:
     * 1. Sequential: PC = PC + 4 (normal instruction flow)
     * 2. Branch/Jump: PC = branch_target (when branch_taken = 1)
     * 3. Stall: PC remains unchanged (when stall = 1)
     * 
     * Priority:
     * - Stall has highest priority (PC doesn't update)
     * - Branch taken has next priority (PC = branch_target)
     * - Otherwise, PC increments by 4
     */
    logic [ADDR_WIDTH-1:0] PC_next;
    
    /**
     * PC Increment Logic
     * 
     * Calculates PC + 4 for the next sequential instruction.
     * This value is passed to the ID stage and used for:
     * - Calculating branch targets (PC + immediate)
     * - Storing return address for JAL/JALR instructions
     * - Next instruction address when no branch is taken
     */
    logic [ADDR_WIDTH-1:0] PC_inc;
    assign PC_inc = PC_reg + 32'd4;
    
    /**
     * PC Next Value Selection
     * 
     * Multiplexer that selects the next PC value based on control signals:
     * - stall: PC_next = PC_reg (no change)
     * - branch_taken: PC_next = branch_target
     * - default: PC_next = PC_inc (PC + 4)
     */
    always_comb begin
        if (stall) begin
            // Stall: PC doesn't update, holds current value
            PC_next = PC_reg;
        end else if (branch_taken) begin
            // Branch/Jump taken: PC = branch target
            PC_next = branch_target;
        end else begin
            // Normal operation: PC increments by 4
            PC_next = PC_inc;
        end
    end
    
    /**
     * Program Counter Register Update (Synchronous)
     * 
     * PC register updates on the positive edge of the clock.
     * Reset initializes PC to zero (or reset vector).
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Initialize PC to zero (or reset vector)
            // In a real system, this might be configurable (e.g., 0x00000000 or 0x80000000)
            PC_reg <= {ADDR_WIDTH{1'b0}};
        end else begin
            // Update PC with next value
            PC_reg <= PC_next;
        end
    end
    
    /**
     * Instruction Memory Address
     * 
     * The instruction memory address is derived from the PC.
     * Since instructions are word-aligned, we use PC[IMEM_ADDR_WIDTH+1:2]
     * to index into the instruction memory.
     * 
     * Note: This assumes instruction memory is word-addressed internally.
     * If byte-addressed, use PC[ADDR_WIDTH-1:2] and handle accordingly.
     */
    assign imem_addr = PC_reg[IMEM_ADDR_WIDTH+1:2];
    
    /**
     * Pipeline Register: IF/ID
     * 
     * The IF/ID pipeline register stores:
     * - instruction: The fetched instruction
     * - PC: Current PC value (for branch target calculation)
     * - PC_plus_4: PC + 4 (for next instruction and return address)
     * 
     * Pipeline Control:
     * - Flush: Clears pipeline register (inserts NOP/bubble)
     * - Stall: Holds current values (prevents new instruction from entering ID stage)
     */
    logic [DATA_WIDTH-1:0] instruction_reg;
    logic [ADDR_WIDTH-1:0] PC_reg_out;
    logic [ADDR_WIDTH-1:0] PC_plus_4_reg;
    
    /**
     * IF/ID Pipeline Register Update
     * 
     * Updates pipeline register on clock edge.
     * Flush clears the register (inserts NOP instruction = 0x00000013, which is ADDI x0, x0, 0).
     * Stall holds current values.
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Clear pipeline register
            instruction_reg <= {DATA_WIDTH{1'b0}};
            PC_reg_out <= {ADDR_WIDTH{1'b0}};
            PC_plus_4_reg <= {ADDR_WIDTH{1'b0}};
        end else if (flush) begin
            // Flush: Insert NOP (clear pipeline register)
            // NOP = ADDI x0, x0, 0 = 0x00000013
            instruction_reg <= 32'h00000013;
            PC_reg_out <= PC_reg;
            PC_plus_4_reg <= PC_inc;
        end else if (!stall) begin
            // Normal operation: Update pipeline register with new instruction
            instruction_reg <= imem_data;
            PC_reg_out <= PC_reg;
            PC_plus_4_reg <= PC_inc;
        end
        // If stall = 1, pipeline register holds current values (no change)
    end
    
    /**
     * Output Assignments
     * 
     * Connect pipeline register outputs to module outputs.
     */
    assign instruction = instruction_reg;
    assign PC = PC_reg_out;
    assign PC_plus_4 = PC_plus_4_reg;
    
    /**
     * Stage Behavior Summary:
     * 
     * 1. PC Update (Synchronous):
     *    - On clock edge: PC = PC_next
     *    - PC_next = stall ? PC_reg : (branch_taken ? branch_target : PC + 4)
     * 
     * 2. Instruction Fetch (Combinational):
     *    - imem_addr = PC[IMEM_ADDR_WIDTH+1:2]
     *    - instruction = imem_data (from memory)
     * 
     * 3. Pipeline Register Update (Synchronous):
     *    - On clock edge: Update IF/ID register
     *    - Flush: Insert NOP (0x00000013)
     *    - Stall: Hold current values
     *    - Normal: Update with new instruction and PC values
     * 
     * 4. Timing:
     *    - PC update: 1 clock cycle
     *    - Instruction fetch: Combinational (memory access time)
     *    - Pipeline register: 1 clock cycle
     * 
     * 5. Pipeline Hazards:
     *    - Stall: Prevents new instruction fetch, holds PC
     *    - Flush: Discards current instruction, inserts NOP
     *    - Branch: Updates PC to target, flushes pipeline
     * 
     * 6. Branch Handling:
     *    - Branch decision made in EX stage
     *    - If taken: PC updated to branch_target, pipeline flushed
     *    - If not taken: PC increments normally
     *    - 1-cycle penalty for taken branches
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Instruction Memory:
     *    - Assumed to be synchronous read memory
     *    - Address is word-aligned
     *    - Read completes in 1 clock cycle
     * 
     * 2. PC Alignment:
     *    - PC is always word-aligned (lower 2 bits = 00)
     *    - Instruction memory address = PC >> 2
     * 
     * 3. Reset Vector:
     *    - Default reset value is 0x00000000
     *    - Can be parameterized for different reset vectors
     * 
     * 4. NOP Instruction:
     *    - Flush inserts 0x00000013 (ADDI x0, x0, 0)
     *    - This is a true NOP that doesn't modify any state
     * 
     * 5. Stall Behavior:
     *    - When stalled, PC doesn't update
     *    - Pipeline register holds previous instruction
     *    - Used for data hazards (load-use) and structural hazards
     */

endmodule

