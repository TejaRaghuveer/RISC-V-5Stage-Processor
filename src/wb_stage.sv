/**
 * @file wb_stage.sv
 * @brief RISC-V Writeback (WB) Stage Module
 * 
 * This module implements the Writeback stage of the RISC-V 5-stage pipeline.
 * The WB stage selects the correct data source (ALU result or memory data)
 * and writes it back to the register file.
 * 
 * @details
 * Writeback Stage Responsibilities:
 * - Select data source: ALU result or memory read data
 * - Write data to register file
 * - Complete the instruction execution cycle
 * 
 * Data Source Selection:
 * - MemToReg = 0: Write ALU result to register file (R-type, I-type, U-type, JAL, JALR)
 * - MemToReg = 1: Write memory data to register file (Load instructions: LW, LH, LB, LHU, LBU)
 * 
 * Pipeline Position:
 * - Final stage of the pipeline
 * - Receives data from MEM/WB pipeline register
 * - Writes back to register file (used by ID stage)
 */

module wb_stage #(
    parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
) (
    // Inputs from MEM/WB Pipeline Register
    input  logic [DATA_WIDTH-1:0]       mem_wb_alu_result,  // ALU result from EX stage
    input  logic [DATA_WIDTH-1:0]       mem_wb_mem_read_data, // Memory read data from MEM stage
    input  logic                        mem_wb_MemToReg,     // Memory to register select signal
    
    // Output to Register File
    output logic [DATA_WIDTH-1:0]       wb_write_data       // Data to write to register file
);

    /**
     * Writeback Data Selection Multiplexer
     * 
     * Selects between ALU result and memory read data based on MemToReg signal.
     * 
     * MemToReg Control:
     * - 0: Write ALU result (for arithmetic, logic, shift, compare operations)
     * - 1: Write memory data (for load instructions)
     * 
     * Instruction Types:
     * - MemToReg = 0: R-type, I-type immediate, U-type (LUI, AUIPC), JAL, JALR
     * - MemToReg = 1: Load instructions (LW, LH, LB, LHU, LBU)
     */
    assign wb_write_data = mem_wb_MemToReg ? mem_wb_mem_read_data : mem_wb_alu_result;
    
    /**
     * Writeback Stage Behavior Summary:
     * 
     * 1. Data Source Selection:
     *    - MemToReg = 0: ALU result (most instructions)
     *    - MemToReg = 1: Memory data (load instructions)
     * 
     * 2. Register File Write:
     *    - Write occurs in register file module
     *    - Controlled by RegWrite signal (from MEM/WB register)
     *    - Destination register address from MEM/WB register
     * 
     * 3. Pipeline Completion:
     *    - Final stage of pipeline
     *    - Instruction execution completes here
     *    - Result available for subsequent instructions
     * 
     * 4. Timing:
     *    - Combinational operation (no clock required)
     *    - Data available immediately after MEM/WB register update
     *    - Write to register file occurs on clock edge
     */
    
    /**
     * Data Flow Examples:
     * 
     * Example 1: ADD instruction (R-type)
     *   - ALU computes: rs1 + rs2
     *   - MemToReg = 0
     *   - Write-back: ALU result → register file
     * 
     * Example 2: LW instruction (Load)
     *   - ALU computes: rs1 + immediate (address)
     *   - Memory reads: data from computed address
     *   - MemToReg = 1
     *   - Write-back: Memory data → register file
     * 
     * Example 3: ADDI instruction (I-type)
     *   - ALU computes: rs1 + immediate
     *   - MemToReg = 0
     *   - Write-back: ALU result → register file
     * 
     * Example 4: LUI instruction (U-type)
     *   - ALU passes: immediate (upper 20 bits, lower 12 bits zero)
     *   - MemToReg = 0
     *   - Write-back: ALU result → register file
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Combinational Logic:
     *    - Writeback stage is combinational (no clock)
     *    - Data selection happens immediately
     *    - No pipeline register needed (final stage)
     * 
     * 2. Register File Interface:
     *    - Write data connects to register file write port
     *    - Write enable (RegWrite) and address (rd) from MEM/WB register
     *    - Write occurs synchronously on clock edge
     * 
     * 3. Pipeline Register:
     *    - MEM/WB register stores:
     *      * ALU result
     *      * Memory read data
     *      * MemToReg control signal
     *      * RegWrite control signal
     *      * Destination register address (rd)
     * 
     * 4. Forwarding Support:
     *    - Write-back data can be forwarded to earlier stages
     *    - Forwarding unit uses MEM/WB write data
     *    - Resolves data hazards without stalling
     * 
     * 5. Register x0 Handling:
     *    - Register file ignores writes to x0
     *    - No special handling needed in WB stage
     */

endmodule

