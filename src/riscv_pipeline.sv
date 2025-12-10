/**
 * @file riscv_pipeline.sv
 * @brief RISC-V 5-Stage Pipeline Processor Top-Level Module
 * 
 * This module integrates all 5 stages of the RISC-V pipeline processor:
 * - Instruction Fetch (IF)
 * - Instruction Decode (ID)
 * - Execute (EX)
 * - Memory Access (MEM)
 * - Writeback (WB)
 * 
 * @details
 * Pipeline Architecture:
 * - 5-stage pipeline: IF → ID → EX → MEM → WB
 * - Pipeline registers: IF/ID, ID/EX, EX/MEM, MEM/WB
 * - Forwarding unit for data hazard resolution (RAW hazards)
 * - Hazard detection unit for load-use hazard stalling
 * - Instruction memory (IMEM) and data memory (DMEM)
 * 
 * Data Flow:
 * - Forward Path: IF → ID → EX → MEM → WB
 * - Write-back Path: WB → Register File (in ID stage)
 * - Branch/Jump Feedback: EX → IF (PC update)
 * - Forwarding Paths: EX/MEM → EX, MEM/WB → EX
 * 
 * Pipeline Control:
 * - Stall: Prevents new instructions from entering pipeline
 * - Flush: Clears pipeline registers (on branch misprediction)
 * - Reset: Initializes all pipeline stages
 */

module riscv_pipeline #(
    parameter DATA_WIDTH = 32,           // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32,            // Address width (32 bits)
    parameter IMEM_DEPTH = 1024,          // Instruction memory depth (words)
    parameter IMEM_ADDR_WIDTH = 10,       // Instruction memory address width
    parameter DMEM_DEPTH = 1024,          // Data memory depth (words)
    parameter DMEM_ADDR_WIDTH = 10,       // Data memory address width
    parameter IMEM_INIT_FILE = "mem/inst_mem.hex",  // Instruction memory init file
    parameter DMEM_INIT_FILE = ""          // Data memory init file (empty = no init)
) (
    // Clock and Reset
    input  logic                        clk,              // Clock signal
    input  logic                        rst_n,            // Active-low reset
    
    // Pipeline Control (External - optional, can be tied to 0 if not used)
    input  logic                        pipeline_stall,   // External pipeline stall signal (optional)
    input  logic                        pipeline_flush    // External pipeline flush signal (optional)
);

    // ============================================
    // Internal Signal Declarations
    // ============================================
    
    /**
     * IF Stage Signals
     */
    logic [IMEM_ADDR_WIDTH-1:0]        if_imem_addr;     // Instruction memory address
    logic [DATA_WIDTH-1:0]             if_imem_data;     // Instruction from memory
    logic [DATA_WIDTH-1:0]             if_instruction;   // Fetched instruction
    logic [ADDR_WIDTH-1:0]             if_PC;            // Program counter
    logic [ADDR_WIDTH-1:0]             if_PC_plus_4;     // PC + 4
    
    /**
     * IF/ID Pipeline Register Signals
     */
    logic [DATA_WIDTH-1:0]             id_instruction;   // Instruction to ID stage
    logic [ADDR_WIDTH-1:0]             id_PC;            // PC to ID stage
    logic [ADDR_WIDTH-1:0]             id_PC_plus_4;      // PC+4 to ID stage
    
    /**
     * ID Stage Signals
     */
    logic [DATA_WIDTH-1:0]             id_rs1_data;      // Source register 1 data
    logic [DATA_WIDTH-1:0]             id_rs2_data;      // Source register 2 data
    logic [DATA_WIDTH-1:0]             id_immediate;     // Sign-extended immediate
    logic [ADDR_WIDTH-1:0]             id_PC_out;        // PC (passed through)
    logic [ADDR_WIDTH-1:0]             id_PC_plus_4_out; // PC+4 (passed through)
    logic [4:0]                        id_rs1_addr;     // Source register 1 address
    logic [4:0]                        id_rs2_addr;     // Source register 2 address
    logic [4:0]                        id_rd_addr;       // Destination register address
    logic [2:0]                        id_funct3;        // Function field [14:12]
    logic [6:0]                        id_funct7;        // Function field [31:25]
    logic [6:0]                        id_opcode;        // Instruction opcode [6:0]
    logic                              id_RegWrite;      // Register write enable
    logic                              id_MemRead;        // Memory read enable
    logic                              id_MemWrite;       // Memory write enable
    logic                              id_MemToReg;       // Memory to register select
    logic                              id_ALUSrc;         // ALU source select
    logic [1:0]                        id_ALUOp;         // ALU operation type
    logic                              id_Branch;         // Branch instruction
    logic                              id_Jump;           // Jump instruction
    
    /**
     * ID/EX Pipeline Register Signals
     */
    logic [DATA_WIDTH-1:0]             ex_rs1_data;      // Source register 1 data
    logic [DATA_WIDTH-1:0]             ex_rs2_data;      // Source register 2 data
    logic [DATA_WIDTH-1:0]             ex_immediate;     // Sign-extended immediate
    logic [ADDR_WIDTH-1:0]             ex_PC;            // PC value
    logic [ADDR_WIDTH-1:0]             ex_PC_plus_4;     // PC+4 value
    logic [4:0]                        ex_rs1_addr;      // Source register 1 address
    logic [4:0]                        ex_rs2_addr;      // Source register 2 address
    logic [4:0]                        ex_rd_addr;       // Destination register address
    logic [2:0]                        ex_funct3;        // Function field
    logic [6:0]                        ex_funct7;        // Function field
    logic [6:0]                        ex_opcode;        // Instruction opcode
    logic                              ex_RegWrite;      // Register write enable
    logic                              ex_MemRead;       // Memory read enable
    logic                              ex_MemWrite;      // Memory write enable
    logic                              ex_MemToReg;      // Memory to register select
    logic                              ex_ALUSrc;        // ALU source select
    logic [1:0]                        ex_ALUOp;         // ALU operation type
    logic                              ex_Branch;        // Branch instruction
    logic                              ex_Jump;          // Jump instruction
    
    /**
     * EX Stage Signals
     */
    logic [DATA_WIDTH-1:0]             ex_alu_result;   // ALU computation result
    logic                               ex_zero_flag;     // ALU zero flag
    logic                               ex_branch_taken;  // Branch condition evaluation
    logic [ADDR_WIDTH-1:0]             ex_branch_target; // Branch target address
    logic [ADDR_WIDTH-1:0]             ex_jump_target;   // Jump target address
    logic [DATA_WIDTH-1:0]             ex_rs2_data_out;  // rs2 data (for stores)
    
    /**
     * Forwarding Unit Signals
     */
    logic [1:0]                        ForwardA;         // Forwarding control for rs1
    logic [1:0]                        ForwardB;         // Forwarding control for rs2
    
    /**
     * EX/MEM Pipeline Register Signals
     */
    logic [DATA_WIDTH-1:0]             mem_alu_result;   // ALU result (memory address)
    logic [DATA_WIDTH-1:0]             mem_rs2_data;    // Source register 2 data
    logic [4:0]                        mem_rd_addr;      // Destination register address
    logic                              mem_MemRead;      // Memory read enable
    logic                              mem_MemWrite;     // Memory write enable
    logic                              mem_MemToReg;     // Memory to register select
    logic                              mem_RegWrite;     // Register write enable
    logic                              mem_Branch;       // Branch instruction
    logic                              mem_Jump;         // Jump instruction
    logic                              mem_branch_taken; // Branch condition evaluation
    logic [ADDR_WIDTH-1:0]             mem_branch_target; // Branch target address
    logic [ADDR_WIDTH-1:0]             mem_jump_target;  // Jump target address
    
    /**
     * MEM Stage Signals
     */
    logic [DATA_WIDTH-1:0]             mem_read_data;    // Data read from memory
    logic [DATA_WIDTH-1:0]             mem_alu_result_out; // ALU result (passthrough)
    
    /**
     * MEM/WB Pipeline Register Signals
     */
    logic [DATA_WIDTH-1:0]             wb_mem_read_data; // Memory read data
    logic [DATA_WIDTH-1:0]             wb_alu_result;    // ALU result
    logic [4:0]                        wb_rd_addr;       // Destination register address
    logic                              wb_MemToReg;      // Memory to register select
    logic                              wb_RegWrite;      // Register write enable
    
    /**
     * WB Stage Signals
     */
    logic [DATA_WIDTH-1:0]             wb_write_data;   // Data to write to register file
    
    /**
     * Branch/Jump Feedback Signals
     */
    logic [ADDR_WIDTH-1:0]             branch_target;    // Branch/jump target address
    logic                              branch_taken;      // Branch/jump taken signal
    
    /**
     * Branch/Jump Control Signals
     */
    logic                              ex_PCSrc;          // PC source select from EX stage (combinational)
    logic                              ex_branch_flush;   // Branch/jump flush from EX stage (combinational)
    logic                              mem_PCSrc;         // PC source select from EX/MEM register (registered)
    logic                              mem_branch_flush;  // Branch/jump flush from EX/MEM register (registered)
    
    /**
     * Signals for Branch/Jump Control Unit
     * These are needed for branch condition evaluation
     */
    logic [DATA_WIDTH-1:0]             ex_rs1_data_forwarded; // Forwarded rs1 data for branch control
    logic [DATA_WIDTH-1:0]             ex_rs2_data_forwarded; // Forwarded rs2 data for branch control
    
    /**
     * Pipeline Control Signals (Internal)
     */
    logic                              hazard_stall;            // Stall signal from hazard detection unit
    logic                              hazard_id_ex_flush;      // ID/EX flush signal from hazard detection unit
    logic                              pipeline_stall_internal;  // Combined stall signal (hazard + external)
    logic                              pipeline_flush_internal;  // Combined flush signal (branch/jump + external + hazard)
    
    // ============================================
    // Instruction Memory (IMEM) Instantiation
    // ============================================
    
    /**
     * Instruction Memory Module
     * 
     * Stores program instructions. Read-only during execution.
     * Initialized from hex file at simulation start.
     */
    imem #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .MEM_DEPTH(IMEM_DEPTH),
        .MEM_ADDR_WIDTH(IMEM_ADDR_WIDTH),
        .INIT_FILE(IMEM_INIT_FILE)
    ) instruction_memory (
        .clk(clk),
        .addr(if_imem_addr),
        .data(if_imem_data),
        .addr_valid()
    );
    
    // ============================================
    // Instruction Fetch (IF) Stage
    // ============================================
    
    /**
     * IF Stage Module
     * 
     * Fetches instructions from instruction memory.
     * Updates program counter (PC) based on branch/jump signals.
     */
    if_stage #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .IMEM_DEPTH(IMEM_DEPTH),
        .IMEM_ADDR_WIDTH(IMEM_ADDR_WIDTH)
    ) if_stage_inst (
        .clk(clk),
        .rst_n(rst_n),
        .branch_target(branch_target),      // Branch/jump target address (selected)
        .branch_taken(mem_PCSrc),           // PC source select (1 = branch/jump taken, 0 = sequential)
        .stall(pipeline_stall_internal),     // Pipeline stall signal (hazard + external)
        .flush(pipeline_flush_internal),     // Pipeline flush signal (internal or external)
        .imem_addr(if_imem_addr),            // Instruction memory address
        .imem_data(if_imem_data),            // Instruction from memory
        .instruction(if_instruction),        // Fetched instruction
        .PC(if_PC),                          // Current PC value
        .PC_plus_4(if_PC_plus_4)             // PC + 4
    );
    
    // ============================================
    // IF/ID Pipeline Register
    // ============================================
    
    /**
     * IF/ID Pipeline Register
     * 
     * Stores instruction and PC values from IF stage.
     * Passes data to ID stage on next clock cycle.
     */
    if_id_reg #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) if_id_reg_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(~pipeline_stall_internal),  // Enable when not stalled (active low: ~stall)
        .flush(pipeline_flush_internal),    // Flush on branch/jump taken or external flush
        .if_instruction(if_instruction),    // Instruction from IF stage
        .if_PC(if_PC),                       // PC from IF stage
        .if_PC_plus_4(if_PC_plus_4),         // PC+4 from IF stage
        .id_instruction(id_instruction),     // Instruction to ID stage
        .id_PC(id_PC),                       // PC to ID stage
        .id_PC_plus_4(id_PC_plus_4)          // PC+4 to ID stage
    );
    
    // ============================================
    // Instruction Decode (ID) Stage
    // ============================================
    
    /**
     * ID Stage Module
     * 
     * Decodes instructions, reads register file, generates control signals.
     * Receives write-back data from WB stage for register file writes.
     */
    id_stage #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) id_stage_inst (
        .clk(clk),
        .rst_n(rst_n),
        .instruction(id_instruction),        // Instruction from IF/ID register
        .PC(id_PC),                          // PC from IF/ID register
        .PC_plus_4(id_PC_plus_4),            // PC+4 from IF/ID register
        .wb_reg_write_en(wb_RegWrite),       // Write enable from WB stage
        .wb_rd_addr(wb_rd_addr),             // Write address from WB stage
        .wb_rd_data(wb_write_data),          // Write data from WB stage
        .stall(pipeline_stall_internal),     // Pipeline stall signal (hazard + external)
        .flush(pipeline_flush_internal),     // Pipeline flush signal (internal or external)
        .rs1_data(id_rs1_data),              // Source register 1 data
        .rs2_data(id_rs2_data),              // Source register 2 data
        .immediate(id_immediate),            // Sign-extended immediate
        .PC_out(id_PC_out),                  // PC (passed through)
        .PC_plus_4_out(id_PC_plus_4_out),    // PC+4 (passed through)
        .RegWrite(id_RegWrite),              // Register write enable
        .MemRead(id_MemRead),                // Memory read enable
        .MemWrite(id_MemWrite),              // Memory write enable
        .MemToReg(id_MemToReg),              // Memory to register select
        .ALUSrc(id_ALUSrc),                  // ALU source select
        .ALUOp(id_ALUOp),                    // ALU operation type
        .Branch(id_Branch),                  // Branch instruction
        .Jump(id_Jump),                      // Jump instruction
        .rs1_addr(id_rs1_addr),               // Source register 1 address
        .rs2_addr(id_rs2_addr),              // Source register 2 address
        .rd_addr(id_rd_addr),                // Destination register address
        .funct3(id_funct3),                  // Function field [14:12]
        .funct7(id_funct7),                  // Function field [31:25]
        .opcode(id_opcode)                   // Instruction opcode [6:0]
    );
    
    // ============================================
    // ID/EX Pipeline Register
    // ============================================
    
    /**
     * ID/EX Pipeline Register
     * 
     * Stores register data, immediate, PC, and control signals from ID stage.
     * Passes data to EX stage on next clock cycle.
     */
    id_ex_reg #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) id_ex_reg_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(~pipeline_stall_internal),    // Enable when not stalled (active low: ~stall)
        .flush(pipeline_flush_internal || hazard_id_ex_flush), // Flush: branch/jump + external + hazard NOP insertion
        .id_rs1_data(id_rs1_data),           // Source register 1 data
        .id_rs2_data(id_rs2_data),           // Source register 2 data
        .id_rs1_addr(id_rs1_addr),            // Source register 1 address
        .id_rs2_addr(id_rs2_addr),            // Source register 2 address
        .id_rd_addr(id_rd_addr),              // Destination register address
        .id_immediate(id_immediate),          // Sign-extended immediate
        .id_PC(id_PC_out),                    // PC value
        .id_PC_plus_4(id_PC_plus_4_out),      // PC+4 value
        .id_RegWrite(id_RegWrite),           // Register write enable
        .id_MemRead(id_MemRead),             // Memory read enable
        .id_MemWrite(id_MemWrite),           // Memory write enable
        .id_MemToReg(id_MemToReg),           // Memory to register select
        .id_ALUSrc(id_ALUSrc),               // ALU source select
        .id_ALUOp(id_ALUOp),                 // ALU operation type
        .id_Branch(id_Branch),               // Branch instruction
        .id_Jump(id_Jump),                   // Jump instruction
        .id_funct3(id_funct3),               // Function field [14:12]
        .id_funct7(id_funct7),               // Function field [31:25]
        .id_opcode(id_opcode),               // Instruction opcode [6:0]
        .ex_rs1_data(ex_rs1_data),           // Source register 1 data to EX
        .ex_rs2_data(ex_rs2_data),          // Source register 2 data to EX
        .ex_rs1_addr(ex_rs1_addr),            // Source register 1 address to EX
        .ex_rs2_addr(ex_rs2_addr),           // Source register 2 address to EX
        .ex_rd_addr(ex_rd_addr),              // Destination register address to EX
        .ex_immediate(ex_immediate),         // Sign-extended immediate to EX
        .ex_PC(ex_PC),                       // PC value to EX
        .ex_PC_plus_4(ex_PC_plus_4),         // PC+4 value to EX
        .ex_RegWrite(ex_RegWrite),           // Register write enable to EX
        .ex_MemRead(ex_MemRead),             // Memory read enable to EX
        .ex_MemWrite(ex_MemWrite),           // Memory write enable to EX
        .ex_MemToReg(ex_MemToReg),           // Memory to register select to EX
        .ex_ALUSrc(ex_ALUSrc),               // ALU source select to EX
        .ex_ALUOp(ex_ALUOp),                 // ALU operation type to EX
        .ex_Branch(ex_Branch),               // Branch instruction to EX
        .ex_Jump(ex_Jump),                   // Jump instruction to EX
        .ex_funct3(ex_funct3),               // Function field to EX
        .ex_funct7(ex_funct7),               // Function field to EX
        .ex_opcode(ex_opcode)                // Instruction opcode to EX
    );
    
    // ============================================
    // Hazard Detection Unit
    // ============================================
    
    /**
     * Hazard Detection Unit Module
     * 
     * Detects load-use data hazards that cannot be resolved by forwarding.
     * When a load instruction in ID/EX stage produces data needed by the
     * instruction in ID stage, the pipeline must be stalled for one cycle.
     * 
     * Hazard Condition:
     * - Load instruction in ID/EX stage (ex_MemRead = 1)
     * - AND load's destination register (ex_rd_addr) matches rs1 or rs2
     *   of instruction in ID stage (id_rs1_addr or id_rs2_addr)
     * 
     * Pipeline Control Actions:
     * - Stall: Prevents PC update and holds IF/ID register
     * - Flush ID/EX: Inserts NOP/bubble in ID/EX register
     * 
     * Timing:
     * - Detection is combinational (based on current pipeline state)
     * - Stall/flush signals take effect on next clock cycle
     * - One-cycle stall allows load to complete before dependent instruction uses data
     */
    hazard_detection_unit hazard_detection_unit_inst (
        // Inputs from Pipeline Registers
        .id_ex_MemRead(ex_MemRead),          // Memory read enable from ID/EX register
        .id_ex_rd_addr(ex_rd_addr),           // Destination register address from ID/EX register
        .if_id_rs1_addr(id_rs1_addr),        // Source register 1 address from IF/ID register (via ID stage)
        .if_id_rs2_addr(id_rs2_addr),        // Source register 2 address from IF/ID register (via ID stage)
        
        // Outputs for Pipeline Control
        .stall(hazard_stall),                 // Pipeline stall signal (active high)
        .id_ex_flush(hazard_id_ex_flush)     // ID/EX register flush signal (active high)
    );
    
    /**
     * Combined Pipeline Stall Signal Generation
     * 
     * Combines hazard stall with external stall signal.
     * Pipeline stalls when:
     * - Load-use hazard detected (hazard_stall = 1), OR
     * - External stall requested (pipeline_stall = 1)
     * 
     * Stall Behavior:
     * - Prevents PC from updating (PC holds current value)
     * - Prevents IF/ID register from updating (holds current instruction)
     * - Causes pipeline bubble in IF and ID stages
     * - Used for data hazards that cannot be resolved by forwarding
     */
    assign pipeline_stall_internal = hazard_stall || pipeline_stall;
    
    // ============================================
    // Forwarding Unit
    // ============================================
    
    /**
     * Forwarding Unit Module
     * 
     * Detects RAW data hazards and generates forwarding control signals.
     * Allows data from EX/MEM and MEM/WB stages to bypass register file.
     */
    forwarding_unit forwarding_unit_inst (
        .id_ex_rs1_addr(ex_rs1_addr),        // Source register 1 address from ID/EX
        .id_ex_rs2_addr(ex_rs2_addr),        // Source register 2 address from ID/EX
        .ex_mem_rd_addr(mem_rd_addr),        // Destination register address from EX/MEM
        .mem_wb_rd_addr(wb_rd_addr),         // Destination register address from MEM/WB
        .ex_mem_reg_write(mem_RegWrite),     // Register write enable from EX/MEM
        .mem_wb_reg_write(wb_RegWrite),     // Register write enable from MEM/WB
        .ForwardA(ForwardA),                 // Forwarding control for rs1
        .ForwardB(ForwardB)                   // Forwarding control for rs2
    );
    
    // ============================================
    // Execute (EX) Stage
    // ============================================
    
    /**
     * EX Stage Module
     * 
     * Performs ALU operations, calculates addresses, evaluates branch conditions.
     * Receives forwarded data from EX/MEM and MEM/WB stages.
     */
    ex_stage #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) ex_stage_inst (
        .id_ex_rs1_data(ex_rs1_data),        // Source register 1 data from ID/EX
        .id_ex_rs2_data(ex_rs2_data),        // Source register 2 data from ID/EX
        .id_ex_immediate(ex_immediate),      // Sign-extended immediate from ID/EX
        .id_ex_PC(ex_PC),                    // PC value from ID/EX
        .id_ex_PC_plus_4(ex_PC_plus_4),     // PC+4 value from ID/EX
        .ex_mem_alu_result(mem_alu_result), // ALU result from EX/MEM (for forwarding)
        .mem_wb_write_data(wb_write_data),   // Write data from MEM/WB (for forwarding)
        .ForwardA(ForwardA),                 // Forwarding control for rs1
        .ForwardB(ForwardB),                  // Forwarding control for rs2
        .ALUSrc(ex_ALUSrc),                  // ALU source select
        .ALUOp(ex_ALUOp),                    // ALU operation type
        .funct3(ex_funct3),                  // Function field
        .funct7(ex_funct7),                  // Function field
        .opcode(ex_opcode),                  // Instruction opcode
        .Branch(ex_Branch),                  // Branch instruction
        .Jump(ex_Jump),                      // Jump instruction
        .alu_result(ex_alu_result),          // ALU computation result
        .zero_flag(ex_zero_flag),            // ALU zero flag
        .branch_taken(ex_branch_taken),      // Branch condition evaluation
        .branch_target(ex_branch_target),    // Branch target address
        .jump_target(ex_jump_target),        // Jump target address
        .rs2_data_out(ex_rs2_data_out),      // rs2 data (for store instructions)
        .rs1_data_forwarded_out(ex_rs1_data_forwarded), // Forwarded rs1 data (for branch control)
        .rs2_data_forwarded_out(ex_rs2_data_forwarded)  // Forwarded rs2 data (for branch control)
    );
    
    // ============================================
    // Branch/Jump Control Unit
    // ============================================
    
    /**
     * Branch/Jump Control Unit Module
     * 
     * Evaluates branch conditions and generates PC source selection and flush signals.
     * Uses combinational signals from EX stage for immediate evaluation.
     * 
     * Branch Condition Evaluation:
     * - BEQ (000): rs1 == rs2 → uses zero_flag
     * - BNE (001): rs1 != rs2 → uses !zero_flag
     * - BLT (100): rs1 < rs2 (signed) → uses signed comparison
     * - BGE (101): rs1 >= rs2 (signed) → uses signed comparison
     * - BLTU (110): rs1 < rs2 (unsigned) → uses unsigned comparison
     * - BGEU (111): rs1 >= rs2 (unsigned) → uses unsigned comparison
     * 
     * Jump Control:
     * - JAL: Always taken (PC-relative)
     * - JALR: Always taken (register + immediate)
     * 
     * Outputs:
     * - PCSrc: PC source select (0 = PC+4, 1 = branch/jump target)
     * - flush: Pipeline flush signal (clears IF/ID and ID/EX registers)
     * - branch_taken: Branch condition evaluation result
     */
    branch_jump_control #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) branch_jump_control_inst (
        // Control Signals from ID/EX Register
        .Branch(ex_Branch),                  // Branch instruction indicator
        .Jump(ex_Jump),                      // Jump instruction indicator
        .funct3(ex_funct3),                  // Function field [14:12] (branch type)
        
        // ALU Results for Branch Condition Evaluation
        .zero_flag(ex_zero_flag),            // ALU zero flag (from SUB operation)
        .rs1_data(ex_rs1_data_forwarded),   // Forwarded rs1 data (for comparison)
        .rs2_data(ex_rs2_data_forwarded),   // Forwarded rs2 data (for comparison)
        
        // Target Addresses from EX Stage
        .branch_target(ex_branch_target),    // Branch target address (PC + immediate)
        .jump_target(ex_jump_target),        // Jump target address (PC + immediate or rs1 + immediate)
        
        // Outputs for Pipeline Control
        .PCSrc(ex_PCSrc),                    // PC source select (0 = PC+4, 1 = target)
        .flush(ex_branch_flush),             // Pipeline flush signal (active high)
        .branch_taken()                      // Branch condition evaluation (not used, ex_branch_taken used instead)
    );
    
    // ============================================
    // Branch/Jump Target Selection and PC Control
    // ============================================
    
    /**
     * Branch/Jump Target Multiplexer and PC Source Selection
     * 
     * Selects target address for PC update using REGISTERED signals from EX/MEM.
     * This ensures proper timing alignment - branch/jump decision is made in EX stage
     * and registered in EX/MEM register before being used for PC update.
     * 
     * Timing:
     * - EX stage: Branch/jump control evaluates condition (combinational)
     * - EX/MEM register: Stores PCSrc, flush, and target addresses (registered on clock edge)
     * - IF stage: Uses registered PCSrc for PC update (proper timing)
     * 
     * PC Source Selection:
     * - PCSrc = 0: Sequential execution (PC + 4, handled in IF stage)
     * - PCSrc = 1: Branch/Jump taken (use target address)
     * 
     * Target Address Selection:
     * - Jump instructions: Use jump_target
     * - Branch instructions: Use branch_target
     * - Otherwise: Don't care (PCSrc = 0)
     */
    always_comb begin
        // Use registered signals from EX/MEM register for proper timing
        if (mem_PCSrc) begin
            // Branch/Jump taken: Select target address
            if (mem_Jump) begin
                // Jump instruction: Use jump target (always taken)
                branch_target = mem_jump_target;
            end else begin
                // Branch instruction: Use branch target (if taken)
                branch_target = mem_branch_target;
            end
            branch_taken = 1'b1;
        end else begin
            // No branch/jump or not taken: Sequential execution (IF stage handles PC+4)
            branch_target = {ADDR_WIDTH{1'b0}};  // Don't care
            branch_taken = 1'b0;
        end
    end
    
    /**
     * Pipeline Flush Signal Generation
     * 
     * Generate flush signal when branch/jump is taken to flush incorrect
     * instructions that were fetched speculatively.
     * 
     * Flush clears IF/ID and ID/EX pipeline registers, inserting NOPs.
     * External flush signal (pipeline_flush) can also be used for exceptions.
     * 
     * Flush Sources:
     * - Branch/Jump taken: mem_branch_flush (from branch/jump control, registered)
     * - External flush: pipeline_flush (for exceptions, interrupts)
     * 
     * Note: Hazard detection unit generates its own ID/EX flush signal
     * (hazard_id_ex_flush) which is ORed directly into ID/EX register flush.
     * This allows hazard NOP insertion without flushing other pipeline registers.
     */
    assign pipeline_flush_internal = mem_branch_flush || pipeline_flush;
    
    // ============================================
    // EX/MEM Pipeline Register
    // ============================================
    
    /**
     * EX/MEM Pipeline Register
     * 
     * Stores ALU result, rs2 data, and control signals from EX stage.
     * Passes data to MEM stage on next clock cycle.
     */
    ex_mem_reg #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) ex_mem_reg_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(~pipeline_stall_internal),   // Enable when not stalled (active low: ~stall)
        .flush(pipeline_flush_internal),     // Flush on branch/jump taken or external flush
        .ex_alu_result(ex_alu_result),       // ALU result from EX stage
        .ex_rs2_data(ex_rs2_data_out),       // Source register 2 data from EX stage
        .ex_rd_addr(ex_rd_addr),             // Destination register address from EX stage
        .ex_MemRead(ex_MemRead),             // Memory read enable from EX stage
        .ex_MemWrite(ex_MemWrite),           // Memory write enable from EX stage
        .ex_MemToReg(ex_MemToReg),           // Memory to register select from EX stage
        .ex_RegWrite(ex_RegWrite),           // Register write enable from EX stage
        .ex_Branch(ex_Branch),               // Branch instruction from EX stage
        .ex_Jump(ex_Jump),                   // Jump instruction from EX stage
        .ex_branch_taken(ex_branch_taken),   // Branch condition evaluation from EX stage
        .ex_branch_target(ex_branch_target), // Branch target address from EX stage
        .ex_jump_target(ex_jump_target),     // Jump target address from EX stage
        .ex_PCSrc(ex_PCSrc),                 // PC source select from branch/jump control
        .ex_branch_flush(ex_branch_flush),   // Branch/jump flush signal from branch/jump control
        .mem_alu_result(mem_alu_result),     // ALU result to MEM stage
        .mem_rs2_data(mem_rs2_data),         // Source register 2 data to MEM stage
        .mem_rd_addr(mem_rd_addr),           // Destination register address to MEM stage
        .mem_MemRead(mem_MemRead),           // Memory read enable to MEM stage
        .mem_MemWrite(mem_MemWrite),         // Memory write enable to MEM stage
        .mem_MemToReg(mem_MemToReg),         // Memory to register select to MEM stage
        .mem_RegWrite(mem_RegWrite),         // Register write enable to MEM stage
        .mem_Branch(mem_Branch),             // Branch instruction to MEM stage
        .mem_Jump(mem_Jump),                 // Jump instruction to MEM stage
        .mem_branch_taken(mem_branch_taken), // Branch condition evaluation to MEM stage
        .mem_branch_target(mem_branch_target), // Branch target address to MEM stage
        .mem_jump_target(mem_jump_target),   // Jump target address to MEM stage
        .mem_PCSrc(mem_PCSrc),               // PC source select to MEM stage (registered)
        .mem_branch_flush(mem_branch_flush)  // Branch/jump flush to MEM stage (registered)
    );
    
    // ============================================
    // Memory Access (MEM) Stage
    // ============================================
    
    /**
     * Note: Data Memory (DMEM) is instantiated inside MEM stage module.
     * MEM stage handles all data memory access operations.
     */
    
    /**
     * MEM Stage Module
     * 
     * Accesses data memory for load/store instructions.
     * Passes through ALU result for non-memory instructions.
     */
    mem_stage #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .MEM_DEPTH(DMEM_DEPTH),
        .MEM_ADDR_WIDTH(DMEM_ADDR_WIDTH)
    ) mem_stage_inst (
        .clk(clk),
        .ex_mem_alu_result(mem_alu_result),  // ALU result (memory address)
        .ex_mem_rs2_data(mem_rs2_data),      // Source register 2 data (for stores)
        .ex_mem_MemRead(mem_MemRead),        // Memory read enable
        .ex_mem_MemWrite(mem_MemWrite),      // Memory write enable
        .mem_read_data(mem_read_data),       // Data read from memory
        .mem_alu_result(mem_alu_result_out)  // ALU result (passthrough)
    );
    
    // ============================================
    // MEM/WB Pipeline Register
    // ============================================
    
    /**
     * MEM/WB Pipeline Register
     * 
     * Stores memory read data, ALU result, and control signals from MEM stage.
     * Passes data to WB stage on next clock cycle.
     */
    mem_wb_reg #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) mem_wb_reg_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(~pipeline_stall_internal),   // Enable when not stalled (active low: ~stall)
        .flush(pipeline_flush_internal),     // Flush on branch/jump taken or external flush
        .mem_read_data(mem_read_data),       // Memory read data from MEM stage
        .mem_alu_result(mem_alu_result_out), // ALU result from MEM stage
        .mem_rd_addr(mem_rd_addr),           // Destination register address from MEM stage
        .mem_MemToReg(mem_MemToReg),         // Memory to register select from MEM stage
        .mem_RegWrite(mem_RegWrite),         // Register write enable from MEM stage
        .wb_mem_read_data(wb_mem_read_data), // Memory read data to WB stage
        .wb_alu_result(wb_alu_result),        // ALU result to WB stage
        .wb_rd_addr(wb_rd_addr),             // Destination register address to WB stage
        .wb_MemToReg(wb_MemToReg),           // Memory to register select to WB stage
        .wb_RegWrite(wb_RegWrite)            // Register write enable to WB stage
    );
    
    // ============================================
    // Writeback (WB) Stage
    // ============================================
    
    /**
     * WB Stage Module
     * 
     * Selects data source (ALU result or memory data) for register write.
     * Outputs write data to register file in ID stage.
     */
    wb_stage #(
        .DATA_WIDTH(DATA_WIDTH)
    ) wb_stage_inst (
        .mem_wb_alu_result(wb_alu_result),      // ALU result from MEM/WB register
        .mem_wb_mem_read_data(wb_mem_read_data), // Memory read data from MEM/WB register
        .mem_wb_MemToReg(wb_MemToReg),           // Memory to register select
        .wb_write_data(wb_write_data)            // Write data to register file
    );
    
    /**
     * Pipeline Data Flow Summary:
     * 
     * Forward Path (Instruction Flow):
     * 1. IF Stage: Fetches instruction from IMEM
     * 2. IF/ID Register: Stores instruction and PC
     * 3. ID Stage: Decodes instruction, reads register file
     * 4. ID/EX Register: Stores register data and control signals
     * 5. EX Stage: Performs ALU operations, evaluates branches
     * 6. EX/MEM Register: Stores ALU result and control signals
     * 7. MEM Stage: Accesses data memory (load/store)
     * 8. MEM/WB Register: Stores memory data and ALU result
     * 9. WB Stage: Selects write-back data source
     * 
     * Write-back Path (Data Flow):
     * - WB Stage → Register File (in ID stage)
     * - Write occurs synchronously on clock edge
     * - Data available for next instruction in ID stage
     * 
     * Branch/Jump Feedback Path:
     * - EX Stage → Branch/Jump Target Selection → IF Stage
     * - Updates PC when branch/jump is taken
     * - Causes pipeline flush if branch was mispredicted
     * 
     * Forwarding Paths (Data Hazard Resolution):
     * - EX/MEM → EX Stage: Forward ALU result
     * - MEM/WB → EX Stage: Forward write-back data
     * - Bypasses register file for RAW hazards
     * - Eliminates need for pipeline stalls in most cases
     */
    
    /**
     * Pipeline Control Summary:
     * 
     * Stall Signal:
     * - When asserted: Prevents new instructions from entering pipeline
     * - Pipeline registers hold current values
     * - Used for data hazards that cannot be resolved by forwarding
     * 
     * Flush Signal:
     * - When asserted: Clears pipeline registers (inserts NOP)
     * - Used on branch misprediction or exception
     * - Discards incorrect instructions in pipeline
     * 
     * Reset Signal:
     * - Initializes all pipeline stages and registers
     * - PC starts at reset vector (typically 0x00000000)
     * - All registers cleared to zero
     */

endmodule

