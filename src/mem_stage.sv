/**
 * @file mem_stage.sv
 * @brief RISC-V Memory (MEM) Stage Module
 * 
 * This module implements the Memory Access stage of the RISC-V 5-stage pipeline.
 * The MEM stage accesses data memory for load and store instructions.
 * 
 * @details
 * Memory Access Stage Responsibilities:
 * - Access data memory for load/store instructions
 * - Pass through ALU result for non-memory instructions
 * - Handle memory read and write operations
 * - Provide data to MEM/WB pipeline register
 * 
 * Operations:
 * - Load Instructions (LW, LH, LB, LHU, LBU):
 *   * Read data from memory at computed address
 *   * Data passed to WB stage for register write
 * 
 * - Store Instructions (SW, SH, SB):
 *   * Write data to memory at computed address
 *   * No register write (RegWrite = 0)
 * 
 * - Non-Memory Instructions:
 *   * ALU result passed through unchanged
 *   * No memory access performed
 */

module mem_stage #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32,            // Address width (32 bits)
    parameter MEM_DEPTH = 1024,          // Memory depth in words (default 1024)
    parameter MEM_ADDR_WIDTH = 10        // Memory address width (log2 of MEM_DEPTH)
) (
    // Clock
    input  logic                        clk,              // Clock signal
    
    // Inputs from EX/MEM Pipeline Register
    input  logic [DATA_WIDTH-1:0]       ex_mem_alu_result, // ALU result (memory address for load/store)
    input  logic [DATA_WIDTH-1:0]       ex_mem_rs2_data,   // Source register 2 data (for store instructions)
    input  logic                        ex_mem_MemRead,    // Memory read enable
    input  logic                        ex_mem_MemWrite,   // Memory write enable
    
    // Outputs to MEM/WB Pipeline Register
    output logic [DATA_WIDTH-1:0]       mem_read_data,     // Data read from memory
    output logic [DATA_WIDTH-1:0]       mem_alu_result     // ALU result (passthrough for non-memory instructions)
);

    /**
     * Data Memory Instantiation
     * 
     * Data memory module handles:
     * - Synchronous read operations (for load instructions)
     * - Synchronous write operations (for store instructions)
     * - Address validation and boundary checking
     * - Word-aligned access
     */
    dmem #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .MEM_DEPTH(MEM_DEPTH),
        .MEM_ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) data_memory (
        .clk(clk),
        .addr(ex_mem_alu_result),        // Memory address from ALU result
        .write_data(ex_mem_rs2_data),    // Data to write (from rs2, for store instructions)
        .MemRead(ex_mem_MemRead),        // Memory read enable
        .MemWrite(ex_mem_MemWrite),      // Memory write enable
        .read_data(mem_read_data),       // Data read from memory
        .addr_valid()                     // Address valid signal (not used in this stage)
    );
    
    /**
     * ALU Result Passthrough
     * 
     * ALU result is passed through to MEM/WB register.
     * This is used for:
     * - Non-memory instructions: ALU result is the final result
     * - Load instructions: ALU result is the address (not used in WB)
     * - Store instructions: ALU result is the address (not used in WB)
     * 
     * Note: For load instructions, mem_read_data is used in WB stage.
     * For other instructions, mem_alu_result is used in WB stage.
     */
    assign mem_alu_result = ex_mem_alu_result;
    
    /**
     * Load Operation Flow:
     * 
     * 1. EX Stage:
     *    - ALU computes: address = rs1 + immediate
     *    - Address stored in EX/MEM register
     * 
     * 2. MEM Stage:
     *    - Address from EX/MEM register → Data Memory
     *    - MemRead = 1 enables memory read
     *    - Memory reads data at computed address
     *    - Read data available on next clock cycle
     * 
     * 3. MEM/WB Register:
     *    - Stores: ALU result (address), memory read data, MemToReg = 1
     * 
     * 4. WB Stage:
     *    - MemToReg = 1 selects memory read data
     *    - Memory data written to register file
     * 
     * Example: LW x1, 0(x2)
     *   - Address = x2 + 0 = x2
     *   - Memory[x2] → x1
     */
    
    /**
     * Store Operation Flow:
     * 
     * 1. EX Stage:
     *    - ALU computes: address = rs1 + immediate
     *    - rs2_data forwarded to EX stage
     *    - Address and rs2_data stored in EX/MEM register
     * 
     * 2. MEM Stage:
     *    - Address from EX/MEM register → Data Memory
     *    - rs2_data from EX/MEM register → Data Memory write_data
     *    - MemWrite = 1 enables memory write
     *    - Memory writes data at computed address
     *    - Write completes on clock edge
     * 
     * 3. MEM/WB Register:
     *    - Stores: ALU result (address), memory read data (don't care)
     *    - RegWrite = 0 (no register write)
     * 
     * 4. WB Stage:
     *    - No register write (RegWrite = 0)
     *    - Instruction completes
     * 
     * Example: SW x3, 0(x1)
     *   - Address = x1 + 0 = x1
     *   - x3 → Memory[x1]
     */
    
    /**
     * Non-Memory Instruction Flow:
     * 
     * 1. EX Stage:
     *    - ALU computes result (arithmetic, logic, shift, compare)
     *    - Result stored in EX/MEM register
     * 
     * 2. MEM Stage:
     *    - ALU result passed through unchanged
     *    - No memory access (MemRead = 0, MemWrite = 0)
     *    - mem_read_data is don't care
     * 
     * 3. MEM/WB Register:
     *    - Stores: ALU result, memory read data (don't care), MemToReg = 0
     * 
     * 4. WB Stage:
     *    - MemToReg = 0 selects ALU result
     *    - ALU result written to register file
     * 
     * Example: ADD x3, x1, x2
     *   - ALU computes: x1 + x2
     *   - Result → x3
     */
    
    /**
     * Stage Behavior Summary:
     * 
     * 1. Load Instructions (MemRead = 1):
     *    - Read data from memory at ALU result address
     *    - Data available on next clock cycle
     *    - Passed to WB stage for register write
     * 
     * 2. Store Instructions (MemWrite = 1):
     *    - Write rs2_data to memory at ALU result address
     *    - Write completes on clock edge
     *    - No register write (RegWrite = 0)
     * 
     * 3. Non-Memory Instructions:
     *    - ALU result passed through unchanged
     *    - No memory access performed
     *    - Result passed to WB stage
     * 
     * 4. Timing:
     *    - Memory access: 1 clock cycle latency
     *    - Read data available on next clock edge
     *    - Write completes on clock edge
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Memory Interface:
     *    - Address: ALU result from EX stage
     *    - Write Data: rs2_data from EX stage (for stores)
     *    - Control: MemRead, MemWrite from control unit
     * 
     * 2. Address Calculation:
     *    - Address computed in EX stage: rs1 + immediate
     *    - Address passed through EX/MEM register
     *    - Used directly for memory access
     * 
     * 3. Data Forwarding:
     *    - rs2_data may be forwarded from earlier stages
     *    - Forwarding handled in EX stage
     *    - Correct data available in EX/MEM register
     * 
     * 4. Pipeline Register:
     *    - EX/MEM register stores:
     *      * ALU result (address)
     *      * rs2_data (for stores)
     *      * MemRead, MemWrite control signals
     *    - MEM/WB register stores:
     *      * ALU result (passthrough)
     *      * Memory read data
     *      * MemToReg control signal
     * 
     * 5. Memory Timing:
     *    - Synchronous read: Data available next cycle
     *    - Synchronous write: Write completes on clock edge
     *    - Both operations use registered address
     */

endmodule

