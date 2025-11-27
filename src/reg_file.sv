/**
 * @file reg_file.sv
 * @brief RISC-V 32-Register Register File Module
 * 
 * This module implements the RISC-V register file containing 32 general-purpose
 * registers (x0 through x31). The register file provides two asynchronous read
 * ports for reading source operands (rs1 and rs2) and one synchronous write port
 * for writing results back to the destination register (rd).
 * 
 * @details
 * RISC-V Register File Specifications:
 * - 32 registers, each 32 bits wide (for RV32I)
 * - Register x0 is hardwired to zero (reads return 0, writes are ignored)
 * - Two read ports: rs1 and rs2 (asynchronous, combinational reads)
 * - One write port: rd (synchronous, clocked writes)
 * - Write operations occur on the positive edge of the clock
 * - Reset initializes all registers to zero (except x0 which is always zero)
 * 
 * Register Usage Convention:
 * - x0: Zero register (hardwired to 0)
 * - x1: Return address (ra)
 * - x2: Stack pointer (sp)
 * - x3: Global pointer (gp)
 * - x4: Thread pointer (tp)
 * - x5-x7: Temporaries (t0-t2)
 * - x8-x9: Saved registers (s0-s1)
 * - x10-x17: Function arguments/return values (a0-a7)
 * - x18-x27: Saved registers (s2-s11)
 * - x28-x31: Temporaries (t3-t6)
 */

module reg_file #(
    parameter DATA_WIDTH = 32,        // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 5,          // Address width (5 bits for 32 registers)
    parameter NUM_REGS   = 32          // Number of registers
) (
    // Clock and Reset
    input  logic                        clk,           // Clock signal
    input  logic                        rst_n,         // Active-low reset
    
    // Read Port 1 (rs1 - Source Register 1)
    input  logic [ADDR_WIDTH-1:0]       rs1_addr,      // Read address for rs1
    output logic [DATA_WIDTH-1:0]       rs1_data,      // Read data from rs1
    
    // Read Port 2 (rs2 - Source Register 2)
    input  logic [ADDR_WIDTH-1:0]       rs2_addr,      // Read address for rs2
    output logic [DATA_WIDTH-1:0]       rs2_data,      // Read data from rs2
    
    // Write Port (rd - Destination Register)
    input  logic                        reg_write_en,  // Write enable signal
    input  logic [ADDR_WIDTH-1:0]       rd_addr,       // Write address (destination register)
    input  logic [DATA_WIDTH-1:0]       rd_data        // Write data (data to write)
);

    /**
     * Register File Storage
     * 
     * Array of 32 registers, each DATA_WIDTH bits wide.
     * Index 0 corresponds to x0, index 1 to x1, etc.
     * Note: x0 (registers[0]) is not actually used for storage since it's
     * hardwired to zero, but we include it for indexing consistency.
     */
    logic [DATA_WIDTH-1:0] registers [NUM_REGS-1:0];
    
    /**
     * Register File Write Operation (Synchronous)
     * 
     * Write operations occur on the positive edge of the clock when:
     * 1. reg_write_en is asserted (high)
     * 2. rd_addr is not zero (x0 cannot be written)
     * 
     * This implements the RISC-V convention where x0 is read-only (always zero).
     * Attempts to write to x0 are silently ignored.
     * 
     * Reset behavior: All registers are initialized to zero on active-low reset.
     * Note: x0 remains zero even after reset (it's hardwired).
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Initialize all registers to zero
            for (int i = 0; i < NUM_REGS; i++) begin
                registers[i] <= {DATA_WIDTH{1'b0}};
            end
        end else begin
            // Write operation: Only write if enabled and not writing to x0
            if (reg_write_en && (rd_addr != 5'b00000)) begin
                registers[rd_addr] <= rd_data;
            end
            // If writing to x0, the write is ignored (no action needed)
        end
    end
    
    /**
     * Register File Read Operation (Asynchronous/Combinational)
     * 
     * Read operations are asynchronous (combinational) and occur immediately
     * when the address changes. This allows the register file to be read in
     * the same clock cycle as the instruction decode stage.
     * 
     * Special handling for x0:
     * - Reading from x0 always returns zero, regardless of register contents
     * - This is implemented using a multiplexer that checks if the address is zero
     * 
     * Read Port 1 (rs1):
     * - Reads data from the register specified by rs1_addr
     * - Returns zero if rs1_addr is 0 (x0)
     */
    always_comb begin
        // Read Port 1 (rs1)
        if (rs1_addr == 5'b00000) begin
            // x0 is hardwired to zero
            rs1_data = {DATA_WIDTH{1'b0}};
        end else begin
            // Read from register array
            rs1_data = registers[rs1_addr];
        end
        
        // Read Port 2 (rs2)
        if (rs2_addr == 5'b00000) begin
            // x0 is hardwired to zero
            rs2_data = {DATA_WIDTH{1'b0}};
        end else begin
            // Read from register array
            rs2_data = registers[rs2_addr];
        end
    end
    
    /**
     * Alternative Implementation Note:
     * 
     * The above implementation uses always_comb blocks for reads. An alternative
     * approach would be to use continuous assignments:
     * 
     * assign rs1_data = (rs1_addr == 5'b00000) ? {DATA_WIDTH{1'b0}} : registers[rs1_addr];
     * assign rs2_data = (rs2_addr == 5'b00000) ? {DATA_WIDTH{1'b0}} : registers[rs2_addr];
     * 
     * Both approaches are functionally equivalent. The always_comb approach is
     * more explicit and easier to extend if additional read logic is needed.
     */
    
    /**
     * Register File Behavior Summary:
     * 
     * 1. Read Operations (Asynchronous):
     *    - rs1_data = registers[rs1_addr] if rs1_addr != 0, else 0
     *    - rs2_data = registers[rs2_addr] if rs2_addr != 0, else 0
     *    - Reads complete in zero clock cycles (combinational)
     * 
     * 2. Write Operations (Synchronous):
     *    - Writes occur on positive clock edge when:
     *      * reg_write_en == 1
     *      * rd_addr != 0 (x0 cannot be written)
     *    - Writes complete in one clock cycle
     * 
     * 3. Reset Behavior:
     *    - Active-low reset (rst_n = 0) initializes all registers to zero
     *    - x0 remains zero (hardwired, not affected by reset)
     * 
     * 4. Register x0 (Zero Register):
     *    - Always reads as zero
     *    - Writes to x0 are ignored
     *    - Used for: zeroing operands, discarding results, NOP operations
     * 
     * 5. Port Timing:
     *    - Read ports: Asynchronous (combinational delay only)
     *    - Write port: Synchronous (clocked, setup/hold time required)
     * 
     * 6. Pipeline Considerations:
     *    - Reads can occur in ID stage (same cycle as decode)
     *    - Writes occur in WB stage (one cycle after result available)
     *    - Forwarding may be needed if same register is read and written
     */

endmodule

