/**
 * @file hazard_detection_unit.sv
 * @brief RISC-V Pipeline Hazard Detection Unit Module
 * 
 * This module implements hazard detection for the RISC-V 5-stage pipeline,
 * specifically detecting load-use data hazards that cannot be resolved by forwarding.
 * 
 * @details
 * Load-Use Data Hazard Detection:
 * - Detects when a load instruction in ID/EX stage will produce data that is
 *   needed by the instruction currently in ID stage
 * - Load instructions require data from memory, which is only available after
 *   the MEM stage completes
 * - Forwarding cannot resolve this hazard because the data is not yet available
 * - Solution: Stall the pipeline for one cycle to allow load to complete
 * 
 * Hazard Condition:
 * - Instruction in ID/EX stage is a load (MemRead = 1)
 * - AND the destination register (rd) of the load matches either:
 *   * rs1 of the instruction in ID stage, OR
 *   * rs2 of the instruction in ID stage
 * 
 * Pipeline Control Actions:
 * - Stall Signal: Prevents PC update and holds IF/ID register
 * - NOP Insertion: Flushes ID/EX register to insert NOP/bubble
 * 
 * Timing:
 * - Detection is combinational (based on current pipeline state)
 * - Stall/flush signals take effect on next clock cycle
 * - One-cycle stall allows load to complete before dependent instruction uses data
 * 
 * Example Load-Use Hazard:
 *   Cycle 1: lw x1, 0(x2)    (ID/EX stage)
 *   Cycle 2: add x3, x1, x4   (ID stage, needs x1)
 *   Solution: Stall pipeline for 1 cycle, insert NOP in ID/EX
 */

module hazard_detection_unit (
    // ============================================
    // Inputs from Pipeline Stages
    // ============================================
    
    /**
     * ID/EX Pipeline Register Signals
     * These signals come from the instruction currently in the EX stage
     * (which was in ID/EX register in previous cycle)
     */
    input  logic                        id_ex_MemRead,  // Memory read enable from ID/EX register
    input  logic [4:0]                  id_ex_rd_addr,  // Destination register address from ID/EX register
    
    /**
     * IF/ID Pipeline Register Signals
     * These signals come from the instruction currently in the ID stage
     * (which is in IF/ID register)
     */
    input  logic [4:0]                  if_id_rs1_addr,  // Source register 1 address from IF/ID register
    input  logic [4:0]                  if_id_rs2_addr, // Source register 2 address from IF/ID register
    
    // ============================================
    // Outputs for Pipeline Control
    // ============================================
    
    /**
     * Pipeline Stall Signal
     * 
     * When asserted (high):
     * - Prevents PC from updating (holds current PC value)
     * - Prevents IF/ID register from updating (holds current instruction)
     * - Causes pipeline bubble in IF and ID stages
     * 
     * When deasserted (low):
     * - Pipeline operates normally
     * - PC increments, IF/ID register updates
     */
    output logic                        stall,          // Pipeline stall signal
    
    /**
     * ID/EX Register Flush Signal
     * 
     * When asserted (high):
     * - Clears ID/EX register (inserts NOP/bubble)
     * - Prevents load instruction from propagating to EX stage
     * - Control signals set to safe defaults (no operation)
     * 
     * When deasserted (low):
     * - ID/EX register operates normally
     * - Data flows from ID stage to EX stage
     */
    output logic                        id_ex_flush     // ID/EX register flush signal
);

    /**
     * Load-Use Hazard Detection Logic
     * 
     * Hazard occurs when:
     * 1. Instruction in ID/EX stage is a load (id_ex_MemRead = 1)
     * 2. AND the load's destination register (id_ex_rd_addr) matches:
     *    - rs1 of instruction in ID stage (if_id_rs1_addr), OR
     *    - rs2 of instruction in ID stage (if_id_rs2_addr)
     * 
     * Special Case: x0 Register
     * - x0 is hardwired to zero and cannot be written
     * - If id_ex_rd_addr = 0, no hazard exists (load doesn't write to x0)
     * - Check: id_ex_rd_addr != 0 before comparing addresses
     * 
     * Detection is combinational:
     * - Compares register addresses from ID/EX and IF/ID registers
     * - Checks if load instruction will write to a register needed by next instruction
     * - Generates stall and flush signals immediately
     */
    
    /**
     * Register Address Comparison
     * 
     * Check if load's destination register matches source registers of next instruction.
     * Only compare if:
     * - Load instruction is present (id_ex_MemRead = 1)
     * - Destination register is not x0 (id_ex_rd_addr != 0)
     */
    logic rs1_hazard;  // Hazard detected on rs1
    logic rs2_hazard;  // Hazard detected on rs2
    logic load_use_hazard;  // Overall load-use hazard detected
    
    /**
     * RS1 Hazard Detection
     * 
     * Check if load's rd matches rs1 of instruction in ID stage.
     * Hazard exists if:
     * - Load instruction in ID/EX (id_ex_MemRead = 1)
     * - Load writes to non-zero register (id_ex_rd_addr != 0)
     * - Next instruction reads from same register (id_ex_rd_addr == if_id_rs1_addr)
     */
    assign rs1_hazard = id_ex_MemRead && 
                        (id_ex_rd_addr != 5'b00000) && 
                        (id_ex_rd_addr == if_id_rs1_addr);
    
    /**
     * RS2 Hazard Detection
     * 
     * Check if load's rd matches rs2 of instruction in ID stage.
     * Hazard exists if:
     * - Load instruction in ID/EX (id_ex_MemRead = 1)
     * - Load writes to non-zero register (id_ex_rd_addr != 0)
     * - Next instruction reads from same register (id_ex_rd_addr == if_id_rs2_addr)
     */
    assign rs2_hazard = id_ex_MemRead && 
                        (id_ex_rd_addr != 5'b00000) && 
                        (id_ex_rd_addr == if_id_rs2_addr);
    
    /**
     * Overall Load-Use Hazard Detection
     * 
     * Hazard exists if either rs1 or rs2 hazard is detected.
     * This means the instruction in ID stage depends on data from
     * the load instruction in ID/EX stage.
     */
    assign load_use_hazard = rs1_hazard || rs2_hazard;
    
    /**
     * Pipeline Stall Signal Generation
     * 
     * When load-use hazard is detected:
     * - Assert stall signal (stall = 1)
     * - This prevents PC from updating
     * - This prevents IF/ID register from updating
     * - Pipeline holds current instruction in ID stage
     * 
     * Stall signal is combinational and takes effect immediately.
     * On next clock cycle:
     * - PC remains unchanged
     * - IF/ID register holds current instruction
     * - ID stage continues decoding same instruction
     */
    assign stall = load_use_hazard;
    
    /**
     * ID/EX Register Flush Signal Generation
     * 
     * When load-use hazard is detected:
     * - Assert flush signal (id_ex_flush = 1)
     * - This clears ID/EX register (inserts NOP)
     * - Prevents load instruction from entering EX stage
     * - Control signals set to safe defaults (no operation)
     * 
     * Flush signal is combinational and takes effect on next clock cycle.
     * On next clock cycle:
     * - ID/EX register cleared (NOP inserted)
     * - EX stage receives NOP (no operation performed)
     * - Load instruction moves to EX/MEM stage (where it can access memory)
     * 
     * Note: The flush signal is the same as the stall signal because:
     * - When stalling, we want to insert a NOP in ID/EX register
     * - This prevents the load instruction from entering EX stage
     * - The load instruction will complete in MEM stage
     * - Next cycle, the dependent instruction can proceed with correct data
     */
    assign id_ex_flush = load_use_hazard;
    
    /**
     * Hazard Detection Unit Behavior Summary:
     * 
     * 1. Normal Operation (no hazard):
     *    - stall = 0, id_ex_flush = 0
     *    - Pipeline operates normally
     *    - Instructions flow through pipeline
     * 
     * 2. Load-Use Hazard Detected:
     *    - stall = 1, id_ex_flush = 1
     *    - PC holds current value (doesn't update)
     *    - IF/ID register holds current instruction
     *    - ID/EX register cleared (NOP inserted)
     *    - Load instruction completes in MEM stage
     *    - Next cycle: Hazard resolved, pipeline resumes
     * 
     * 3. Timing:
     *    - Detection: Combinational (immediate)
     *    - Stall/Flush: Takes effect on next clock cycle
     *    - Resolution: One cycle stall allows load to complete
     * 
     * 4. Example Scenario:
     *    Cycle N:   lw x1, 0(x2)    (ID/EX stage)
     *              add x3, x1, x4   (ID stage, needs x1)
     *              Hazard detected!
     *    
     *    Cycle N+1: Stall asserted
     *              PC holds, IF/ID holds "add" instruction
     *              ID/EX flushed (NOP inserted)
     *              Load moves to EX/MEM stage
     *    
     *    Cycle N+2: Stall deasserted
     *              Load completes in MEM stage (data available)
     *              "add" instruction moves to ID/EX stage
     *              Can now read x1 from register file (data written in WB stage)
     *    
     *    Cycle N+3: "add" instruction executes in EX stage
     *              Uses correct data from x1
     */
    
    /**
     * Implementation Notes:
     * 
     * 1. Register Address Comparison:
     *    - Uses equality comparison (==) for 5-bit addresses
     *    - x0 check prevents false hazards (x0 cannot be written)
     *    - Comparison is combinational (no clock needed)
     * 
     * 2. Stall Signal:
     *    - Connected to IF stage (prevents PC update)
     *    - Connected to IF/ID register (prevents update)
     *    - Active high (1 = stall, 0 = normal)
     * 
     * 3. Flush Signal:
     *    - Connected to ID/EX register (clears register)
     *    - Active high (1 = flush, 0 = normal)
     *    - Same as stall signal for load-use hazards
     * 
     * 4. Forwarding vs. Stalling:
     *    - Forwarding resolves most RAW hazards (ALU results)
     *    - Stalling required for load-use hazards (memory data not available)
     *    - Forwarding unit handles EX/MEM and MEM/WB forwarding
     *    - Hazard unit handles load-use stalling
     * 
     * 5. Pipeline Performance:
     *    - Load-use hazard causes 1-cycle stall
     *    - CPI increases by 1 for each load-use hazard
     *    - Compiler can schedule instructions to minimize hazards
     *    - Some processors use branch prediction to hide stalls
     * 
     * 6. Synthesis:
     *    - Combinational logic synthesizes to gates
     *    - No flip-flops required (purely combinational)
     *    - Critical path: Address comparison → Stall/Flush generation
     *    - Should be fast enough to meet timing requirements
     */

endmodule

