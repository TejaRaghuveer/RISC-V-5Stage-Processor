/**
 * @file dmem.sv
 * @brief RISC-V Data Memory (DMEM) Module
 * 
 * This module implements the data memory for the RISC-V processor.
 * Data memory is read-write capable and stores data accessed by load/store instructions.
 * 
 * @details
 * Memory Organization:
 * - Word-addressable memory (each address contains one 32-bit word)
 * - Addresses are word-aligned (lower 2 bits of byte address must be 00)
 * - Memory is byte-addressable from processor perspective, but internally word-addressed
 * - Supports synchronous read and write operations
 * 
 * Memory Interface:
 * - Synchronous read: Address registered, data available next cycle
 * - Synchronous write: Address and data registered, write occurs on clock edge
 * - Address validation: Checks for out-of-bounds access
 * - Default behavior: Returns zero for invalid addresses
 * 
 * Initialization:
 * - Memory can be initialized from a hex file using $readmemh
 * - Uninitialized locations default to zero
 * - Useful for loading initial data values
 */

module dmem #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32,           // Address width (32 bits for byte addresses)
    parameter MEM_DEPTH = 1024,          // Memory depth in words (default 1024)
    parameter MEM_ADDR_WIDTH = 10,       // Memory address width (log2 of MEM_DEPTH)
    parameter INIT_FILE = ""              // Initialization file path (empty = no initialization)
) (
    // Clock
    input  logic                        clk,              // Clock signal
    
    // Memory Interface
    input  logic [ADDR_WIDTH-1:0]       addr,             // Byte address (word-aligned: addr[1:0] = 00)
    input  logic [DATA_WIDTH-1:0]       write_data,       // Data to write
    input  logic                        MemRead, // Memory read enable
    input  logic                        MemWrite,         // Memory write enable
    
    // Output
    output logic [DATA_WIDTH-1:0]       read_data,        // Read data (instruction)
    output logic                        addr_valid        // Address valid signal (1 if in range and aligned)
);

    /**
     * Data Memory Array
     * 
     * Memory is organized as an array of 32-bit words.
     * Each location stores one word of data.
     * 
     * Memory Layout:
     * - Word address 0: First word (byte addresses 0x00000000-0x00000003)
     * - Word address 1: Second word (byte addresses 0x00000004-0x00000007)
     * - ...
     * - Word address MEM_DEPTH-1: Last word
     * 
     * Address Space:
     * - Valid word addresses: 0 to MEM_DEPTH-1
     * - Byte addresses must be word-aligned (addr[1:0] = 2'b00)
     * - Invalid addresses: Return zero
     */
    logic [DATA_WIDTH-1:0] memory [0:MEM_DEPTH-1];
    
    /**
     * Address Register (for synchronous read/write)
     * 
     * The address is registered to create a synchronous interface.
     * This allows the memory to be used with registered address inputs
     * and provides predictable timing.
     */
    logic [ADDR_WIDTH-1:0] addr_reg;
    logic [DATA_WIDTH-1:0] write_data_reg;
    logic MemRead_reg;
    logic MemWrite_reg;
    
    /**
     * Address Boundary and Alignment Checking
     * 
     * Validates that:
     * 1. Address is within memory bounds
     * 2. Address is word-aligned (lower 2 bits = 00)
     * 
     * Word address calculation: addr[MEM_ADDR_WIDTH+1:2]
     * This extracts the word address from the byte address.
     */
    logic [MEM_ADDR_WIDTH-1:0] word_addr;
    logic [MEM_ADDR_WIDTH-1:0] word_addr_reg;
    logic addr_in_range;
    logic addr_aligned;
    logic addr_in_range_reg;
    logic addr_aligned_reg;
    
    // Extract word address from byte address (combinational for current address)
    assign word_addr = addr[MEM_ADDR_WIDTH+1:2];
    
    // Check if address is within valid range (combinational)
    assign addr_in_range = (word_addr < MEM_DEPTH);
    
    // Check if address is word-aligned (lower 2 bits must be 00)
    assign addr_aligned = (addr[1:0] == 2'b00);
    
    // Address is valid if both in range and aligned (combinational)
    assign addr_valid = addr_in_range && addr_aligned;
    
    /**
     * Registered Address Validation
     * 
     * Register address validation for synchronous operations.
     * This ensures timing alignment between address check and data access.
     * 
     * Note: word_addr_reg is computed from addr_reg to ensure timing alignment
     * with the registered address used for memory access.
     */
    always_ff @(posedge clk) begin
        // Compute word address from registered byte address
        word_addr_reg <= addr_reg[MEM_ADDR_WIDTH+1:2];
        // Register validation signals
        addr_in_range_reg <= (addr_reg[MEM_ADDR_WIDTH+1:2] < MEM_DEPTH);
        addr_aligned_reg <= (addr_reg[1:0] == 2'b00);
    end
    
    /**
     * Address and Control Register Update (Synchronous)
     * 
     * Register address, write data, and control signals on clock edge
     * for synchronous read/write operation.
     */
    always_ff @(posedge clk) begin
        addr_reg <= addr;
        write_data_reg <= write_data;
        MemRead_reg <= MemRead;
        MemWrite_reg <= MemWrite;
    end
    
    /**
     * Memory Initialization
     * 
     * Initialize memory from hex file using $readmemh.
     * This is executed once at time 0 during simulation.
     * 
     * File Format:
     * - Each line contains one 32-bit word in hex
     * - Format: 8 hex digits (e.g., "12345678")
     * - Comments (// or #) and empty lines are ignored
     * 
     * Example hex file content:
     *   00000000  // Initial value at address 0
     *   00000001  // Initial value at address 1
     *   DEADBEEF  // Initial value at address 2
     * 
     * If INIT_FILE is empty or file doesn't exist:
     * - Memory locations default to zero
     */
    initial begin
        // Initialize all memory locations to zero
        for (int i = 0; i < MEM_DEPTH; i++) begin
            memory[i] = {DATA_WIDTH{1'b0}};
        end
        
        // Load data from hex file if specified
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, memory);
            $display("========================================");
            $display("Data Memory Initialization");
            $display("========================================");
            $display("Memory Depth: %0d words", MEM_DEPTH);
            $display("Address Width: %0d bits", MEM_ADDR_WIDTH);
            $display("Data Width: %0d bits", DATA_WIDTH);
            $display("Init File: %s", INIT_FILE);
            $display("========================================");
        end
    end
    
    /**
     * Memory Write Operation (Synchronous)
     * 
     * Write operation is synchronous:
     * - Address and data are registered on clock edge
     * - Write occurs on next clock cycle
     * - Only writes when MemWrite is enabled and address is valid
     * 
     * Write Behavior:
     * - Valid address: Writes write_data_reg to memory[word_addr_reg]
     * - Invalid address: Write is ignored
     * - Word-aligned: Only writes when address is aligned
     * 
     * Timing Alignment:
     * - Uses registered address validation (addr_in_range_reg, addr_aligned_reg)
     * - Uses registered word address (word_addr_reg)
     * - Ensures address check and memory access reference same address
     */
    always_ff @(posedge clk) begin
        if (MemWrite_reg && addr_in_range_reg && addr_aligned_reg) begin
            // Valid write: Write to memory
            // word_addr_reg contains the registered word address from addr_reg
            memory[word_addr_reg] <= write_data_reg;
        end
        // Invalid address or not word-aligned: Write is ignored
    end
    
    /**
     * Memory Read Operation (Synchronous)
     * 
     * Read operation is synchronous:
     * - Address is registered on clock edge
     * - Data is available on next clock cycle
     * - Invalid addresses return zero
     * 
     * Read Behavior:
     * - Valid address: Returns memory[word_addr_reg]
     * - Invalid address: Returns zero
     * - Word-aligned: Only reads when address is aligned
     * 
     * Timing Alignment:
     * - Uses registered address validation (addr_in_range_reg, addr_aligned_reg)
     * - Uses registered word address (word_addr_reg)
     * - Ensures address check and memory access reference same address
     */
    always_ff @(posedge clk) begin
        if (MemRead_reg && addr_in_range_reg && addr_aligned_reg) begin
            // Valid read: Read from memory
            // word_addr_reg contains the registered word address from addr_reg
            read_data <= memory[word_addr_reg];
        end else begin
            // Invalid address, not aligned, or read not enabled: Return zero
            read_data <= {DATA_WIDTH{1'b0}};
        end
    end
    
    /**
     * Memory Organization Summary:
     * 
     * 1. Memory Structure:
     *    - Word-addressable array of 32-bit words
     *    - Total size: MEM_DEPTH × 32 bits
     *    - Default: 1024 words × 32 bits = 32 KB
     * 
     * 2. Addressing:
     *    - Byte addresses from processor (32-bit)
     *    - Word addresses internally (MEM_ADDR_WIDTH bits)
     *    - Word address = byte_address >> 2
     *    - Address range: 0 to MEM_DEPTH-1 words
     *    - Word alignment required: addr[1:0] = 00
     * 
     * 3. Read Operation:
     *    - Synchronous read (address registered)
     *    - Latency: 1 clock cycle
     *    - Valid address: Returns word data
     *    - Invalid address: Returns zero
     * 
     * 4. Write Operation:
     *    - Synchronous write (address and data registered)
     *    - Latency: 1 clock cycle
     *    - Valid address: Writes word data
     *    - Invalid address: Write ignored
     * 
     * 5. Initialization:
     *    - Loaded from hex file at time 0 (if INIT_FILE specified)
     *    - File format: One word per line (8 hex digits)
     *    - Uninitialized locations: Zero
     * 
     * 6. Default Behavior:
     *    - Uninitialized memory: All zeros
     *    - Invalid address: Returns zero (read), ignored (write)
     *    - Misaligned address: Returns zero (read), ignored (write)
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Synchronous vs Combinational Read:
     *    - Current implementation: Synchronous (address registered)
     *    - For combinational read, change to:
     *      assign read_data = (addr_valid && MemRead) ? memory[word_addr] : 32'b0;
     * 
     * 2. Memory Initialization:
     *    - $readmemh reads hex file at simulation start
     *    - File path is parameterized for flexibility
     *    - Empty string means no initialization
     * 
     * 3. Address Validation:
     *    - Checks address bounds before access
     *    - Checks word alignment (lower 2 bits = 00)
     *    - Prevents array out-of-bounds errors
     *    - Returns safe default (zero) for invalid addresses
     * 
     * 4. Word Alignment:
     *    - RISC-V requires word-aligned addresses for LW/SW
     *    - Lower 2 bits of byte address must be 00
     *    - Misaligned accesses return zero (read) or are ignored (write)
     *    - Can be extended to support byte/halfword operations
     * 
     * 5. Memory Size:
     *    - Default: 1024 words (4 KB)
     *    - Can be increased for larger data requirements
     *    - Typical sizes: 1K, 2K, 4K, 8K words
     * 
     * 6. Synthesis Considerations:
     *    - Memory may be synthesized as block RAM (BRAM)
     *    - For FPGA: Use vendor-specific memory primitives
     *    - For ASIC: Use memory compiler generated RAM
     *    - Write enable signal gates clock or data
     * 
     * 7. Byte/Halfword Support (Future Enhancement):
     *    - Current implementation supports word operations only
     *    - Can be extended to support LB, LH, LBU, LHU, SB, SH
     *    - Requires byte/halfword selection logic
     *    - Requires sign/zero extension for loads
     */

endmodule

