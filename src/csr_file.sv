/**
 * @file csr_file.sv
 * @brief Control and Status Register (CSR) File Module
 * 
 * This module implements the CSR file for RISC-V RV32I privilege level.
 * It provides basic CSRs needed for compliance: mstatus, mtvec, mcause, mepc.
 * 
 * @details
 * CSR File Responsibilities:
 * - Store and manage control and status registers
 * - Support CSR read operations (read CSR value)
 * - Support CSR write operations (CSRRW, CSRRS, CSRRC)
 * - Provide CSR addresses for instruction decoding
 * 
 * Supported CSRs (Machine Mode):
 * - mstatus (0x300): Machine status register
 * - mtvec (0x305): Machine trap-handler base address
 * - mcause (0x342): Machine cause register
 * - mepc (0x341): Machine exception program counter
 * 
 * CSR Instructions Supported:
 * - CSRRW: Atomic read/write CSR
 * - CSRRS: Atomic read and set bits in CSR
 * - CSRRC: Atomic read and clear bits in CSR
 * 
 * CSR Address Encoding:
 * - 12-bit CSR address in instruction [31:20]
 * - Read-only CSRs: Upper bit indicates read-only
 * - Write-only CSRs: Upper bit indicates write-only
 */

module csr_file #(
    parameter DATA_WIDTH = 32           // Data width (32 bits for RV32I)
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // CSR Read Interface
    input  logic [11:0]                 csr_addr,         // CSR address (12 bits)
    output logic [DATA_WIDTH-1:0]       csr_read_data,    // CSR read data
    
    // CSR Write Interface
    input  logic                        csr_write_en,     // CSR write enable
    input  logic [11:0]                 csr_write_addr,   // CSR write address
    input  logic [DATA_WIDTH-1:0]      csr_write_data,   // CSR write data
    input  logic [1:0]                  csr_write_type,   // CSR write type: 00=CSRRW, 01=CSRRS, 10=CSRRC
    
    // CSR-specific outputs (for exception handling)
    output logic [DATA_WIDTH-1:0]      mstatus,          // Machine status register
    output logic [DATA_WIDTH-1:0]      mtvec,            // Machine trap vector base address
    output logic [DATA_WIDTH-1:0]      mcause,            // Machine cause register
    output logic [DATA_WIDTH-1:0]      mepc               // Machine exception PC
);

    // ============================================
    // CSR Address Definitions
    // ============================================
    
    // Machine Mode CSRs
    localparam CSR_MSTATUS = 12'h300;  // Machine status register
    localparam CSR_MTVEC   = 12'h305;  // Machine trap vector base address
    localparam CSR_MEPC    = 12'h341;  // Machine exception program counter
    localparam CSR_MCAUSE  = 12'h342;  // Machine cause register
    
    // ============================================
    // CSR Register Declarations
    // ============================================
    
    /**
     * mstatus (Machine Status Register)
     * 
     * Bits [31:0]:
     * - [31:8]: Reserved
     * - [7]: MPIE (Machine Previous Interrupt Enable)
     * - [6:4]: Reserved
     * - [3]: MIE (Machine Interrupt Enable)
     * - [2:0]: Reserved
     * 
     * For basic compliance, we implement minimal fields.
     */
    logic [DATA_WIDTH-1:0] mstatus_reg;
    
    /**
     * mtvec (Machine Trap Vector Base Address Register)
     * 
     * Bits [31:0]:
     * - [31:2]: Base address (word-aligned)
     * - [1:0]: Mode (00 = Direct, 01 = Vectored)
     * 
     * For basic compliance, we support direct mode only.
     */
    logic [DATA_WIDTH-1:0] mtvec_reg;
    
    /**
     * mcause (Machine Cause Register)
     * 
     * Bits [31:0]:
     * - [31]: Interrupt bit (1 = interrupt, 0 = exception)
     * - [30:0]: Exception code
     * 
     * Stores the cause of the most recent trap.
     */
    logic [DATA_WIDTH-1:0] mcause_reg;
    
    /**
     * mepc (Machine Exception Program Counter)
     * 
     * Bits [31:0]:
     * - [31:0]: PC value when exception occurred
     * 
     * Stores the PC of the instruction that caused the exception.
     */
    logic [DATA_WIDTH-1:0] mepc_reg;
    
    // ============================================
    // CSR Read Logic
    // ============================================
    
    /**
     * CSR Read Operation
     * 
     * Reads CSR value based on CSR address.
     * Returns 0 for unimplemented CSRs.
     */
    always_comb begin
        case (csr_addr)
            CSR_MSTATUS: csr_read_data = mstatus_reg;
            CSR_MTVEC:   csr_read_data = mtvec_reg;
            CSR_MEPC:    csr_read_data = mepc_reg;
            CSR_MCAUSE:  csr_read_data = mcause_reg;
            default:     csr_read_data = {DATA_WIDTH{1'b0}};  // Unimplemented CSR returns 0
        endcase
    end
    
    // ============================================
    // CSR Write Logic
    // ============================================
    
    /**
     * CSR Write Operation
     * 
     * Writes to CSR based on write type:
     * - CSRRW (00): Write rs1 value to CSR
     * - CSRRS (01): Set bits in CSR (CSR = CSR | rs1)
     * - CSRRC (10): Clear bits in CSR (CSR = CSR & ~rs1)
     * 
     * Only updates if csr_write_en is asserted and CSR is writable.
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: Initialize CSRs to default values
            mstatus_reg <= {DATA_WIDTH{1'b0}};  // All zeros
            mtvec_reg   <= {DATA_WIDTH{1'b0}};  // Trap vector base = 0
            mcause_reg  <= {DATA_WIDTH{1'b0}};  // No cause
            mepc_reg    <= {DATA_WIDTH{1'b0}};  // Exception PC = 0
        end else if (csr_write_en) begin
            case (csr_write_addr)
                CSR_MSTATUS: begin
                    // mstatus: Writable
                    case (csr_write_type)
                        2'b00: mstatus_reg <= csr_write_data;                    // CSRRW
                        2'b01: mstatus_reg <= mstatus_reg | csr_write_data;      // CSRRS
                        2'b10: mstatus_reg <= mstatus_reg & ~csr_write_data;      // CSRRC
                        default: mstatus_reg <= mstatus_reg;                     // No change
                    endcase
                end
                
                CSR_MTVEC: begin
                    // mtvec: Writable (lower 2 bits should be 0 for direct mode)
                    case (csr_write_type)
                        2'b00: mtvec_reg <= csr_write_data & 32'hFFFFFFFC;        // CSRRW (clear mode bits)
                        2'b01: mtvec_reg <= (mtvec_reg | csr_write_data) & 32'hFFFFFFFC;  // CSRRS
                        2'b10: mtvec_reg <= (mtvec_reg & ~csr_write_data) & 32'hFFFFFFFC; // CSRRC
                        default: mtvec_reg <= mtvec_reg;
                    endcase
                end
                
                CSR_MEPC: begin
                    // mepc: Writable (lower bit should be 0, word-aligned)
                    case (csr_write_type)
                        2'b00: mepc_reg <= csr_write_data & 32'hFFFFFFFE;         // CSRRW (clear LSB)
                        2'b01: mepc_reg <= (mepc_reg | csr_write_data) & 32'hFFFFFFFE;  // CSRRS
                        2'b10: mepc_reg <= (mepc_reg & ~csr_write_data) & 32'hFFFFFFFE; // CSRRC
                        default: mepc_reg <= mepc_reg;
                    endcase
                end
                
                CSR_MCAUSE: begin
                    // mcause: Writable
                    case (csr_write_type)
                        2'b00: mcause_reg <= csr_write_data;                     // CSRRW
                        2'b01: mcause_reg <= mcause_reg | csr_write_data;        // CSRRS
                        2'b10: mcause_reg <= mcause_reg & ~csr_write_data;       // CSRRC
                        default: mcause_reg <= mcause_reg;
                    endcase
                end
                
                default: begin
                    // Unimplemented CSR: No write
                    // (Silently ignore writes to unimplemented CSRs)
                end
            endcase
        end
    end
    
    // ============================================
    // CSR Output Assignments
    // ============================================
    
    /**
     * CSR Outputs
     * 
     * Provide direct access to CSR values for exception handling.
     */
    assign mstatus = mstatus_reg;
    assign mtvec = mtvec_reg;
    assign mcause = mcause_reg;
    assign mepc = mepc_reg;
    
    // ============================================
    // Implementation Notes
    // ============================================
    
    /**
     * CSR Access Behavior:
     * 
     * 1. Read Operations:
     *    - Always returns current CSR value
     *    - Unimplemented CSRs return 0
     *    - Read-only CSRs can be read normally
     * 
     * 2. Write Operations:
     *    - CSRRW: Atomic read/write (read old, write new)
     *    - CSRRS: Atomic read and set bits
     *    - CSRRC: Atomic read and clear bits
     *    - Write-only CSRs: Can only be written
     * 
     * 3. CSR Field Handling:
     *    - mstatus: Full 32-bit writable
     *    - mtvec: Lower 2 bits cleared (direct mode)
     *    - mepc: Lower bit cleared (word-aligned)
     *    - mcause: Full 32-bit writable
     * 
     * 4. Reset Behavior:
     *    - All CSRs initialized to 0
     *    - Can be configured with different reset values if needed
     * 
     * 5. Compliance:
     *    - Implements minimum CSRs for RV32I compliance
     *    - Additional CSRs can be added as needed
     *    - Supports all three CSR write types
     */
    
endmodule

