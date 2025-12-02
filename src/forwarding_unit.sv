/**
 * @file forwarding_unit.sv
 * @brief RISC-V Pipeline Forwarding Unit for Data Hazard Mitigation
 * 
 * This module implements the forwarding (bypassing) unit for the RISC-V 5-stage pipeline.
 * It detects Read-After-Write (RAW) data hazards and selects the correct data source
 * for ALU operands to avoid pipeline stalls.
 * 
 * @details
 * Forwarding Unit Purpose:
 * - Detects RAW data hazards between pipeline stages
 * - Selects correct data source for ALU operands (rs1, rs2)
 * - Eliminates need for pipeline stalls in most cases
 * - Improves pipeline performance by allowing instructions to proceed
 * 
 * Data Hazard Detection:
 * - RAW Hazard: Instruction reads a register that a previous instruction writes
 * - Forwarding allows data to be used before it's written back to register file
 * - Data can be forwarded from EX/MEM or MEM/WB pipeline registers
 * 
 * Forwarding Paths:
 * - EX/MEM → EX: Forward ALU result from previous instruction
 * - MEM/WB → EX: Forward data from memory or ALU result
 * - Register File → EX: Normal case (no hazard)
 * 
 * Forwarding Priority:
 * - EX/MEM forwarding has priority over MEM/WB forwarding
 * - If both stages write to same register, use EX/MEM data
 */

module forwarding_unit (
    // Register Addresses from Pipeline Registers
    input  logic [4:0]  id_ex_rs1_addr,   // Source register 1 address (from ID/EX)
    input  logic [4:0]  id_ex_rs2_addr,   // Source register 2 address (from ID/EX)
    input  logic [4:0]  ex_mem_rd_addr,   // Destination register address (from EX/MEM)
    input  logic [4:0]  mem_wb_rd_addr,   // Destination register address (from MEM/WB)
    
    // Register Write Enable Signals
    input  logic        ex_mem_reg_write, // Register write enable (from EX/MEM)
    input  logic        mem_wb_reg_write, // Register write enable (from MEM/WB)
    
    // Forwarding Control Signals
    output logic [1:0]  ForwardA,         // Forwarding control for rs1 (ALU operand A)
    output logic [1:0]  ForwardB          // Forwarding control for rs2 (ALU operand B)
);

    /**
     * Forwarding Control Signal Encoding:
     * 
     * ForwardA/ForwardB Encoding:
     * - 2'b00: No forwarding (use register file data from ID/EX)
     * - 2'b01: Forward from MEM/WB stage (use data from MEM/WB pipeline register)
     * - 2'b10: Forward from EX/MEM stage (use data from EX/MEM pipeline register)
     * - 2'b11: Reserved (could be used for additional forwarding paths)
     * 
     * Forwarding Mux Selection:
     * - ForwardA = 00: ALU operand A = id_ex_rs1_data (from register file)
     * - ForwardA = 01: ALU operand A = mem_wb_write_data (from MEM/WB)
     * - ForwardA = 10: ALU operand A = ex_mem_alu_result (from EX/MEM)
     * 
     * - ForwardB = 00: ALU operand B = id_ex_rs2_data (from register file)
     * - ForwardB = 01: ALU operand B = mem_wb_write_data (from MEM/WB)
     * - ForwardB = 10: ALU operand B = ex_mem_alu_result (from EX/MEM)
     */
    
    /**
     * Hazard Detection Logic for rs1 (ForwardA)
     * 
     * Detects RAW hazard for source register 1:
     * 1. EX/MEM Hazard: 
     *    - EX/MEM stage writes to register (ex_mem_reg_write = 1)
     *    - EX/MEM destination matches rs1 (ex_mem_rd_addr == id_ex_rs1_addr)
     *    - Destination is not x0 (ex_mem_rd_addr != 0)
     *    - Priority: Forward from EX/MEM (ForwardA = 10)
     * 
     * 2. MEM/WB Hazard:
     *    - MEM/WB stage writes to register (mem_wb_reg_write = 1)
     *    - MEM/WB destination matches rs1 (mem_wb_rd_addr == id_ex_rs1_addr)
     *    - Destination is not x0 (mem_wb_rd_addr != 0)
     *    - EX/MEM hazard not present (to avoid double forwarding)
     *    - Priority: Forward from MEM/WB (ForwardA = 01)
     * 
     * 3. No Hazard:
     *    - No matching destination register
     *    - Use register file data (ForwardA = 00)
     */
    always_comb begin
        // Default: No forwarding
        ForwardA = 2'b00;
        
        // EX/MEM Hazard Detection (Priority 1)
        if (ex_mem_reg_write && 
            (ex_mem_rd_addr != 5'b00000) && 
            (ex_mem_rd_addr == id_ex_rs1_addr)) begin
            // Forward from EX/MEM stage
            ForwardA = 2'b10;
        end
        // MEM/WB Hazard Detection (Priority 2, only if EX/MEM hazard not present)
        else if (mem_wb_reg_write && 
                 (mem_wb_rd_addr != 5'b00000) && 
                 (mem_wb_rd_addr == id_ex_rs1_addr)) begin
            // Forward from MEM/WB stage
            ForwardA = 2'b01;
        end
    end
    
    /**
     * Hazard Detection Logic for rs2 (ForwardB)
     * 
     * Detects RAW hazard for source register 2:
     * 1. EX/MEM Hazard:
     *    - EX/MEM stage writes to register (ex_mem_reg_write = 1)
     *    - EX/MEM destination matches rs2 (ex_mem_rd_addr == id_ex_rs2_addr)
     *    - Destination is not x0 (ex_mem_rd_addr != 0)
     *    - Priority: Forward from EX/MEM (ForwardB = 10)
     * 
     * 2. MEM/WB Hazard:
     *    - MEM/WB stage writes to register (mem_wb_reg_write = 1)
     *    - MEM/WB destination matches rs2 (mem_wb_rd_addr == id_ex_rs2_addr)
     *    - Destination is not x0 (mem_wb_rd_addr != 0)
     *    - EX/MEM hazard not present (to avoid double forwarding)
     *    - Priority: Forward from MEM/WB (ForwardB = 01)
     * 
     * 3. No Hazard:
     *    - No matching destination register
     *    - Use register file data (ForwardB = 00)
     */
    always_comb begin
        // Default: No forwarding
        ForwardB = 2'b00;
        
        // EX/MEM Hazard Detection (Priority 1)
        if (ex_mem_reg_write && 
            (ex_mem_rd_addr != 5'b00000) && 
            (ex_mem_rd_addr == id_ex_rs2_addr)) begin
            // Forward from EX/MEM stage
            ForwardB = 2'b10;
        end
        // MEM/WB Hazard Detection (Priority 2, only if EX/MEM hazard not present)
        else if (mem_wb_reg_write && 
                 (mem_wb_rd_addr != 5'b00000) && 
                 (mem_wb_rd_addr == id_ex_rs2_addr)) begin
            // Forward from MEM/WB stage
            ForwardB = 2'b01;
        end
    end
    
    /**
     * RAW Hazard Detection Logic Explanation:
     * 
     * Read-After-Write (RAW) Hazard:
     * - Occurs when instruction I2 reads a register that instruction I1 writes
     * - I1 is earlier in program order but later in pipeline
     * - Data dependency: I2 depends on result from I1
     * 
     * Example Hazard Scenarios:
     * 
     * Scenario 1: EX/MEM Forwarding
     *   I1: ADD x1, x2, x3    (writes x1 in MEM stage)
     *   I2: ADD x4, x1, x5    (reads x1 in EX stage)
     *   - I2's EX stage executes while I1 is in MEM stage
     *   - Forward x1 from EX/MEM pipeline register to I2's ALU
     *   - ForwardA = 10 (forward from EX/MEM)
     * 
     * Scenario 2: MEM/WB Forwarding
     *   I1: ADD x1, x2, x3    (writes x1 in WB stage)
     *   I2: ADD x4, x1, x5    (reads x1 in EX stage)
     *   - I2's EX stage executes while I1 is in WB stage
     *   - Forward x1 from MEM/WB pipeline register to I2's ALU
     *   - ForwardA = 01 (forward from MEM/WB)
     * 
     * Scenario 3: No Hazard
     *   I1: ADD x1, x2, x3    (writes x1)
     *   I2: ADD x4, x6, x5    (reads x6, not x1)
     *   - No data dependency
     *   - Use register file data
     *   - ForwardA = 00 (no forwarding)
     * 
     * Scenario 4: Multiple Hazards (Priority)
     *   I1: ADD x1, x2, x3    (writes x1 in MEM stage)
     *   I2: ADD x1, x4, x5    (writes x1 in EX stage)
     *   I3: ADD x6, x1, x7    (reads x1 in EX stage)
     *   - I3 needs x1 from I2 (EX/MEM), not I1 (MEM/WB)
     *   - EX/MEM forwarding takes priority
     *   - ForwardA = 10 (forward from EX/MEM)
     */
    
    /**
     * Register x0 Handling:
     * 
     * Register x0 is hardwired to zero and cannot be written.
     * Forwarding unit checks that rd_addr != 0 before forwarding:
     * - Prevents forwarding from x0 writes (which are ignored)
     * - Ensures correct behavior when x0 is destination
     * - x0 reads always return zero (handled by register file)
     */
    
    /**
     * Forwarding Priority Logic:
     * 
     * Priority Order:
     * 1. EX/MEM forwarding (highest priority)
     *    - Data is one cycle newer
     *    - Represents most recent write to register
     * 
     * 2. MEM/WB forwarding (lower priority)
     *    - Data is two cycles older
     *    - Used when EX/MEM doesn't write to same register
     * 
     * 3. Register file (lowest priority)
     *    - No forwarding needed
     *    - Use data from register file read
     * 
     * Why EX/MEM Priority?
     * - If both EX/MEM and MEM/WB write to same register,
     *   EX/MEM has the more recent value
     * - EX/MEM instruction is one cycle ahead of MEM/WB instruction
     * - Forwarding from EX/MEM ensures correct data
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Combinational Logic:
     *    - Forwarding detection is combinational (no clock)
     *    - Control signals available in same cycle as address comparison
     * 
     * 2. Timing:
     *    - Forwarding signals generated in EX stage
     *    - Used to select ALU operands in same cycle
     *    - No additional pipeline delay
     * 
     * 3. Hazard Coverage:
     *    - Handles most RAW hazards without stalling
     *    - Cannot handle load-use hazards (requires stall)
     *    - Cannot handle structural hazards (handled separately)
     * 
     * 4. Load-Use Hazard Exception:
     *    - When load instruction is in EX/MEM stage
     *    - Data not available until MEM stage completes
     *    - Requires pipeline stall (handled by hazard detection unit)
     *    - Forwarding unit alone cannot resolve this
     * 
     * 5. Forwarding Paths:
     *    - EX/MEM → EX: ALU result forwarded
     *    - MEM/WB → EX: Memory data or ALU result forwarded
     *    - Both paths are combinational multiplexers
     * 
     * 6. Performance Impact:
     *    - Eliminates most pipeline stalls due to data hazards
     *    - Improves instruction throughput
     *    - Critical for pipeline performance
     */

endmodule

