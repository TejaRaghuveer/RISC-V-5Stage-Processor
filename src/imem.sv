/**
 * @file imem.sv
 * @brief RISC-V Instruction Memory (IMEM) Module
 * 
 * This module implements the instruction memory for the RISC-V processor.
 * Instruction memory is read-only during execution and stores the program
 * instructions that the processor executes.
 * 
 * @details
 * Memory Organization:
 * - Word-addressable memory (each address contains one 32-bit instruction)
 * - Addresses are word-aligned (no byte addressing)
 * - Memory is initialized from a hex file using $readmemh
 * - Uninitialized locations default to zero (NOP instructions)
 * 
 * Memory Interface:
 * - Synchronous read: Address is registered, data available next cycle
 * - Address validation: Checks for out-of-bounds access
 * - Default behavior: Returns NOP (0x00000013) for invalid addresses
 * 
 * File Format:
 * - Hex file should contain one 32-bit instruction per line
 * - Format: 8 hex digits (e.g., "00100093" for ADDI x1, x0, 1)
 * - Comments starting with // or # are ignored
 * - Empty lines are ignored
 */

module imem #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 10,           // Address width (log2 of memory depth)
    parameter MEM_DEPTH = 1024,          // Memory depth in words (default 1024)
    parameter INIT_FILE = "mem/inst_mem.hex"  // Initialization file path
) (
    // Clock
    input  logic                        clk,              // Clock signal
    
    // Memory Interface
    input  logic [ADDR_WIDTH-1:0]      addr,             // Read address (word-aligned)
    output logic [DATA_WIDTH-1:0]       data,            // Read data (instruction)
    output logic                        addr_valid        // Address valid signal (1 if in range)
);

    /**
     * Instruction Memory Array
     * 
     * Memory is organized as an array of 32-bit words.
     * Each location stores one RISC-V instruction.
     * 
     * Memory Layout:
     * - Address 0: First instruction (typically reset vector)
     * - Address 1: Second instruction
     * - ...
     * - Address MEM_DEPTH-1: Last instruction
     * 
     * Address Space:
     * - Valid addresses: 0 to MEM_DEPTH-1
     * - Invalid addresses: Return NOP (0x00000013)
     */
    logic [DATA_WIDTH-1:0] memory [0:MEM_DEPTH-1];
    
    /**
     * Address Register (for synchronous read)
     * 
     * The address is registered to create a synchronous read interface.
     * This allows the memory to be used with registered address inputs
     * and provides predictable timing.
     */
    logic [ADDR_WIDTH-1:0] addr_reg;
    
    /**
     * Address Register Update (Synchronous)
     * 
     * Register the address on clock edge for synchronous read operation.
     */
    always_ff @(posedge clk) begin
        addr_reg <= addr;
    end
    
    /**
     * Address Boundary Checking
     * 
     * Validates that the registered address is within memory bounds.
     * The validation is computed from the registered address to ensure
     * timing alignment with the memory read operation.
     * 
     * Timing Alignment:
     * - addr_in_range_reg is computed from addr_reg in the same always_ff block
     * - Memory read uses addr_reg (same registered address)
     * - Both reference the same addr_reg value at the same time
     * - This ensures correct timing alignment
     * 
     * Address is valid if: 0 <= addr_reg < MEM_DEPTH
     */
    logic addr_in_range_reg;
    
    /**
     * Memory Initialization
     * 
     * Initialize memory from hex file using $readmemh.
     * This is executed once at time 0 during simulation.
     * 
     * File Format:
     * - Each line contains one 32-bit instruction in hex
     * - Format: 8 hex digits (e.g., "00100093")
     * - Comments (// or #) and empty lines are ignored
     * 
     * Example hex file content:
     *   00100093  // ADDI x1, x0, 1
     *   00200113  // ADDI x2, x0, 2
     *   002081B3  // ADD x3, x1, x2
     * 
     * If file doesn't exist or is empty:
     * - Memory locations default to zero
     * - Zero = 0x00000000 = NOP (ADDI x0, x0, 0)
     */
    initial begin
        // Initialize all memory locations to zero (NOP)
        for (int i = 0; i < MEM_DEPTH; i++) begin
            memory[i] = {DATA_WIDTH{1'b0}};
        end
        
        // Load instructions from hex file
        $readmemh(INIT_FILE, memory);
        
        // Display initialization status
        $display("========================================");
        $display("Instruction Memory Initialization");
        $display("========================================");
        $display("Memory Depth: %0d words", MEM_DEPTH);
        $display("Address Width: %0d bits", ADDR_WIDTH);
        $display("Data Width: %0d bits", DATA_WIDTH);
        $display("Init File: %s", INIT_FILE);
        $display("========================================");
    end
    
    /**
     * Memory Read Operation and Address Validation (Synchronous)
     * 
     * Read operation is synchronous:
     * - Address is registered on clock edge
     * - Data is available on next clock cycle
     * - Invalid addresses return NOP instruction
     * 
     * Read Behavior:
     * - Valid address: Returns memory[addr_reg]
     * - Invalid address: Returns NOP (0x00000013)
     * 
     * Timing Alignment:
     * - Address validation computed inline from addr_reg in the same always_ff block
     * - Memory read uses addr_reg (same registered address)
     * - Both reference the same addr_reg value (the OLD value before <= assignment)
     * - This ensures correct timing alignment
     * 
     * Note: In SystemVerilog always_ff blocks, all right-hand side expressions
     * use their values BEFORE the clock edge (old values). So addr_reg on the
     * right-hand side is the OLD value, and the validation is computed from
     * this same OLD value, ensuring perfect alignment.
     */
    always_ff @(posedge clk) begin
        // Compute address validation inline from registered address
        // This uses the OLD value of addr_reg (before the <= assignment)
        // Memory read uses the same OLD value of addr_reg
        if (addr_reg < MEM_DEPTH) begin
            // Valid address: Read from memory
            // Validation and memory access both use the same OLD addr_reg value
            data <= memory[addr_reg];
            addr_in_range_reg <= 1'b1;
        end else begin
            // Invalid address: Return NOP instruction
            // NOP = ADDI x0, x0, 0 = 0x00000013
            data <= 32'h00000013;
            addr_in_range_reg <= 1'b0;
        end
    end
    
    /**
     * Address Valid Signal
     * 
     * Indicates whether the registered address is within valid range.
     * This signal uses the registered validation to match the timing of the data output.
     * Both addr_valid and data are registered outputs that reference the
     * same addr_reg value, ensuring correct timing alignment.
     * 
     * This signal can be used by the processor to detect memory access errors.
     */
    assign addr_valid = addr_in_range_reg;
    
    /**
     * Memory Organization Summary:
     * 
     * 1. Memory Structure:
     *    - Word-addressable array of 32-bit instructions
     *    - Total size: MEM_DEPTH × 32 bits
     *    - Default: 1024 words × 32 bits = 32 KB
     * 
     * 2. Addressing:
     *    - Addresses are word-aligned (no byte addressing)
     *    - Address range: 0 to MEM_DEPTH-1
     *    - Address width: ADDR_WIDTH bits (log2(MEM_DEPTH))
     * 
     * 3. Read Operation:
     *    - Synchronous read (address registered)
     *    - Latency: 1 clock cycle
     *    - Valid address: Returns instruction
     *    - Invalid address: Returns NOP (0x00000013)
     * 
     * 4. Initialization:
     *    - Loaded from hex file at time 0
     *    - File format: One instruction per line (8 hex digits)
     *    - Uninitialized locations: Zero (NOP)
     * 
     * 5. Default Behavior:
     *    - Uninitialized memory: All zeros (NOP instructions)
     *    - Invalid address: Returns NOP
     *    - This ensures processor doesn't execute garbage instructions
     * 
     * 6. Integration with IF Stage:
     *    - IF stage provides word address (PC >> 2)
     *    - IMEM returns instruction at that address
     *    - Instruction is registered in IF/ID pipeline register
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Synchronous vs Combinational Read:
     *    - Current implementation: Synchronous (address registered)
     *    - For combinational read, change to:
     *      assign data = addr_in_range ? memory[addr] : 32'h00000013;
     * 
     * 2. Memory Initialization:
     *    - $readmemh reads hex file at simulation start
     *    - File path is parameterized for flexibility
     *    - Can be overridden during instantiation
     * 
     * 3. Address Validation:
     *    - Checks address bounds before access
     *    - Prevents array out-of-bounds errors
     *    - Returns safe default (NOP) for invalid addresses
     * 
     * 4. Memory Size:
     *    - Default: 1024 words (4 KB)
     *    - Can be increased for larger programs
     *    - Typical sizes: 1K, 2K, 4K, 8K words
     * 
     * 5. File Format Example:
     *    inst_mem.hex:
     *    00100093  // ADDI x1, x0, 1
     *    00200113  // ADDI x2, x0, 2
     *    002081B3  // ADD x3, x1, x2
     *    0030A023  // SW x3, 0(x1)
     * 
     * 6. Synthesis Considerations:
     *    - Memory may be synthesized as block RAM (BRAM)
     *    - For FPGA: Use vendor-specific memory primitives
     *    - For ASIC: Use memory compiler generated RAM
     */

endmodule

