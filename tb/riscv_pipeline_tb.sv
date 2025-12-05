/**
 * @file riscv_pipeline_tb.sv
 * @brief Comprehensive Testbench Template for RISC-V 5-Stage Pipeline Processor
 * 
 * This testbench provides a complete testing framework for the RISC-V 5-stage
 * pipelined processor. It includes clock generation, reset sequences, memory
 * initialization, register monitoring, and result verification capabilities.
 * 
 * Features:
 * - 50MHz clock generation (20ns period)
 * - Reset sequence with proper timing
 * - Instruction memory initialization from hex file
 * - Data memory monitoring and dump capabilities
 * - Register file state display and verification
 * - Simulation control with configurable cycle count
 * - VCD waveform dump for debugging
 * - Self-checking test framework with pass/fail reporting
 * 
 * Usage:
 * 1. Create a test program hex file in mem/ directory
 * 2. Set IMEM_INIT_FILE parameter to point to your hex file
 * 3. Configure simulation cycles and expected results
 * 4. Run simulation and check results
 * 
 * Note: For register file and memory monitoring, debug ports may need to be
 * added to the pipeline module. See comments in monitoring tasks.
 */

`timescale 1ns / 1ps

module riscv_pipeline_tb;

    // ============================================
    // Parameters
    // ============================================
    
    parameter DATA_WIDTH = 32;              // Data width (32 bits for RV32I)
    parameter ADDR_WIDTH = 32;               // Address width (32 bits)
    parameter IMEM_DEPTH = 1024;             // Instruction memory depth (words)
    parameter IMEM_ADDR_WIDTH = 10;          // Instruction memory address width
    parameter DMEM_DEPTH = 1024;             // Data memory depth (words)
    parameter DMEM_ADDR_WIDTH = 10;          // Data memory address width
    
    // Clock parameters
    parameter CLK_PERIOD = 20;               // Clock period in ns (50MHz = 20ns)
    parameter CLK_HALF_PERIOD = CLK_PERIOD / 2;
    
    // Simulation control parameters
    parameter SIM_CYCLES = 1000;             // Number of clock cycles to simulate
    parameter RESET_CYCLES = 5;              // Number of cycles to hold reset
    
    // Memory initialization files
    parameter IMEM_INIT_FILE = "mem/inst_mem.hex";  // Instruction memory init file
    parameter DMEM_INIT_FILE = "";                   // Data memory init file (empty = no init)
    
    // Test result file paths
    parameter REG_DUMP_FILE = "reg_dump.txt";        // Register dump output file
    parameter DMEM_DUMP_FILE = "dmem_dump.txt";     // Data memory dump output file
    
    // ============================================
    // Testbench Signals
    // ============================================
    
    // Clock and Reset
    logic                        clk;              // Clock signal
    logic                        rst_n;            // Active-low reset
    
    // Pipeline Control Signals
    logic                        pipeline_stall;   // External pipeline stall signal
    logic                        pipeline_flush;   // External pipeline flush signal
    
    // Simulation Control
    int                          cycle_count = 0;  // Current simulation cycle count
    int                          test_count = 0;   // Total number of test cases
    int                          pass_count = 0;   // Number of passed tests
    int                          fail_count = 0;   // Number of failed tests
    
    // Expected Results (for verification)
    // These arrays can be used to store expected register values
    logic [DATA_WIDTH-1:0]      expected_regs [31:0];  // Expected register values
    logic [DATA_WIDTH-1:0]      expected_dmem [DMEM_DEPTH-1:0];  // Expected data memory values
    
    // ============================================
    // Clock Generation
    // ============================================
    
    /**
     * Clock Generator
     * 
     * Generates a 50MHz clock signal (20ns period).
     * Clock starts at time 0 and runs continuously.
     */
    initial begin
        clk = 1'b0;
        forever begin
            #CLK_HALF_PERIOD clk = ~clk;
        end
    end
    
    // ============================================
    // Cycle Counter
    // ============================================
    
    /**
     * Cycle Counter
     * 
     * Counts clock cycles for simulation control.
     * Increments on every positive clock edge.
     */
    always @(posedge clk) begin
        if (rst_n) begin
            cycle_count <= cycle_count + 1;
        end
    end
    
    // ============================================
    // DUT Instantiation
    // ============================================
    
    /**
     * RISC-V Pipeline Processor Instance
     * 
     * Instantiate the top-level pipeline module with all parameters.
     * Connect clock, reset, and control signals.
     */
    riscv_pipeline #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .IMEM_DEPTH(IMEM_DEPTH),
        .IMEM_ADDR_WIDTH(IMEM_ADDR_WIDTH),
        .DMEM_DEPTH(DMEM_DEPTH),
        .DMEM_ADDR_WIDTH(DMEM_ADDR_WIDTH),
        .IMEM_INIT_FILE(IMEM_INIT_FILE),
        .DMEM_INIT_FILE(DMEM_INIT_FILE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .pipeline_stall(pipeline_stall),
        .pipeline_flush(pipeline_flush)
    );
    
    // ============================================
    // Test Tasks and Functions
    // ============================================
    
    /**
     * Task: Reset Sequence
     * 
     * Performs a proper reset sequence:
     * 1. Assert reset (active low)
     * 2. Hold reset for RESET_CYCLES clock cycles
     * 3. Deassert reset
     * 4. Wait for pipeline to stabilize
     * 
     * @param cycles Optional number of reset cycles (default: RESET_CYCLES)
     */
    task reset_sequence(int cycles = RESET_CYCLES);
        $display("========================================");
        $display("Reset Sequence");
        $display("========================================");
        $display("Time: %0t ns", $time);
        
        // Initialize control signals
        pipeline_stall = 1'b0;
        pipeline_flush = 1'b0;
        
        // Assert reset (active low)
        rst_n = 1'b0;
        $display("Asserting reset (rst_n = 0)...");
        
        // Hold reset for specified number of cycles
        repeat (cycles) @(posedge clk);
        
        // Deassert reset
        rst_n = 1'b1;
        $display("Deasserting reset (rst_n = 1)...");
        
        // Wait for pipeline to stabilize (a few cycles)
        repeat (3) @(posedge clk);
        
        $display("Reset sequence complete at time %0t ns", $time);
        $display("========================================\n");
    endtask
    
    /**
     * Task: Load Test Program
     * 
     * Loads a test program into instruction memory.
     * The program is loaded via the IMEM_INIT_FILE parameter.
     * 
     * Note: This task primarily displays information about the program.
     * Actual loading is done by the IMEM module during initialization.
     * 
     * @param hex_file Path to hex file containing instructions
     */
    task load_test_program(string hex_file = IMEM_INIT_FILE);
        $display("========================================");
        $display("Loading Test Program");
        $display("========================================");
        $display("Time: %0t ns", $time);
        $display("Instruction Memory Init File: %s", hex_file);
        $display("Instruction Memory Depth: %0d words", IMEM_DEPTH);
        $display("Instruction Memory Address Width: %0d bits", IMEM_ADDR_WIDTH);
        $display("========================================\n");
        
        // Note: Actual loading happens in IMEM module's initial block
        // This task is for documentation and verification purposes
    endtask
    
    /**
     * Task: Dump Register File Contents
     * 
     * Displays the contents of all 32 registers in the register file.
     * 
     * Note: This task requires debug ports to be added to the pipeline module
     * to access register file contents. Alternatively, register values can be
     * monitored through write-back signals.
     * 
     * To add debug ports:
     * 1. Add a debug port to the register file module
     * 2. Connect it through the ID stage to the top-level module
     * 3. Use hierarchical access: dut.id_stage_inst.reg_file_inst.registers
     * 
     * @param file_name Optional output file name (default: REG_DUMP_FILE)
     */
    task dump_registers(string file_name = REG_DUMP_FILE);
        int file_handle;
        $display("========================================");
        $display("Dumping Register File Contents");
        $display("========================================");
        $display("Time: %0t ns", $time);
        $display("Cycle Count: %0d", cycle_count);
        $display("----------------------------------------");
        
        // Open file for writing
        file_handle = $fopen(file_name, "w");
        if (file_handle == 0) begin
            $display("ERROR: Could not open file %s for writing", file_name);
            return;
        end
        
        // Write header
        $fdisplay(file_handle, "========================================");
        $fdisplay(file_handle, "Register File Dump");
        $fdisplay(file_handle, "========================================");
        $fdisplay(file_handle, "Time: %0t ns", $time);
        $fdisplay(file_handle, "Cycle Count: %0d", cycle_count);
        $fdisplay(file_handle, "----------------------------------------");
        $fdisplay(file_handle, "Register | Hex Value      | Decimal Value");
        $fdisplay(file_handle, "----------------------------------------");
        
        // Display and write register contents
        // Note: This uses hierarchical access - may need adjustment based on actual hierarchy
        for (int i = 0; i < 32; i++) begin
            logic [DATA_WIDTH-1:0] reg_value;
            
            // Try to access register value hierarchically
            // Adjust path based on actual module hierarchy
            // reg_value = dut.id_stage_inst.reg_file_inst.registers[i];
            
            // For now, display placeholder (replace with actual access when debug ports are added)
            reg_value = 32'h00000000;  // Placeholder
            
            $display("x%2d (x%2d) | 0x%08h | %0d", 
                     i, i, reg_value, $signed(reg_value));
            $fdisplay(file_handle, "x%2d (x%2d) | 0x%08h | %0d", 
                      i, i, reg_value, $signed(reg_value));
        end
        
        // Write footer
        $fdisplay(file_handle, "----------------------------------------");
        $fdisplay(file_handle, "End of Register Dump");
        $fdisplay(file_handle, "========================================");
        
        // Close file
        $fclose(file_handle);
        
        $display("----------------------------------------");
        $display("Register dump saved to: %s", file_name);
        $display("========================================\n");
    endtask
    
    /**
     * Task: Dump Data Memory Contents
     * 
     * Displays the contents of data memory.
     * 
     * Note: This task requires debug ports or hierarchical access to memory.
     * 
     * @param file_name Optional output file name (default: DMEM_DUMP_FILE)
     * @param start_addr Starting address for dump (default: 0)
     * @param end_addr Ending address for dump (default: DMEM_DEPTH-1)
     */
    task dump_data_memory(
        string file_name = DMEM_DUMP_FILE,
        int start_addr = 0,
        int end_addr = DMEM_DEPTH - 1
    );
        int file_handle;
        $display("========================================");
        $display("Dumping Data Memory Contents");
        $display("========================================");
        $display("Time: %0t ns", $time);
        $display("Cycle Count: %0d", cycle_count);
        $display("Address Range: 0x%08h to 0x%08h", start_addr, end_addr);
        $display("----------------------------------------");
        
        // Open file for writing
        file_handle = $fopen(file_name, "w");
        if (file_handle == 0) begin
            $display("ERROR: Could not open file %s for writing", file_name);
            return;
        end
        
        // Write header
        $fdisplay(file_handle, "========================================");
        $fdisplay(file_handle, "Data Memory Dump");
        $fdisplay(file_handle, "========================================");
        $fdisplay(file_handle, "Time: %0t ns", $time);
        $fdisplay(file_handle, "Cycle Count: %0d", cycle_count);
        $fdisplay(file_handle, "Address Range: 0x%08h to 0x%08h", start_addr, end_addr);
        $fdisplay(file_handle, "----------------------------------------");
        $fdisplay(file_handle, "Address    | Hex Value      | Decimal Value");
        $fdisplay(file_handle, "----------------------------------------");
        
        // Display and write memory contents
        // Note: This uses hierarchical access - may need adjustment
        for (int i = start_addr; i <= end_addr; i++) begin
            logic [DATA_WIDTH-1:0] mem_value;
            
            // Try to access memory value hierarchically
            // Adjust path based on actual module hierarchy
            // mem_value = dut.mem_stage_inst.dmem_inst.memory[i];
            
            // For now, display placeholder (replace with actual access when debug ports are added)
            mem_value = 32'h00000000;  // Placeholder
            
            // Only display non-zero values or all values if requested
            if (mem_value != 0 || i < 32) begin  // Display first 32 words always
                $display("0x%08h | 0x%08h | %0d", 
                         i * 4, mem_value, $signed(mem_value));
                $fdisplay(file_handle, "0x%08h | 0x%08h | %0d", 
                          i * 4, mem_value, $signed(mem_value));
            end
        end
        
        // Write footer
        $fdisplay(file_handle, "----------------------------------------");
        $fdisplay(file_handle, "End of Data Memory Dump");
        $fdisplay(file_handle, "========================================");
        
        // Close file
        $fclose(file_handle);
        
        $display("----------------------------------------");
        $display("Data memory dump saved to: %s", file_name);
        $display("========================================\n");
    endtask
    
    /**
     * Task: Display Register File State
     * 
     * Displays a formatted view of the register file state.
     * Shows register names, values in hex and decimal, and highlights
     * non-zero registers.
     * 
     * RISC-V Register Names:
     * - x0: zero (always zero)
     * - x1: ra (return address)
     * - x2: sp (stack pointer)
     * - x3: gp (global pointer)
     * - x4: tp (thread pointer)
     * - x5-x7: t0-t2 (temporaries)
     * - x8-x9: s0-s1 (saved registers)
     * - x10-x17: a0-a7 (arguments/return values)
     * - x18-x27: s2-s11 (saved registers)
     * - x28-x31: t3-t6 (temporaries)
     */
    task display_register_state();
        string reg_names [31:0];
        
        // Initialize register names
        reg_names[0]  = "zero";
        reg_names[1]  = "ra";
        reg_names[2]  = "sp";
        reg_names[3]  = "gp";
        reg_names[4]  = "tp";
        reg_names[5]  = "t0";
        reg_names[6]  = "t1";
        reg_names[7]  = "t2";
        reg_names[8]  = "s0";
        reg_names[9]  = "s1";
        reg_names[10] = "a0";
        reg_names[11] = "a1";
        reg_names[12] = "a2";
        reg_names[13] = "a3";
        reg_names[14] = "a4";
        reg_names[15] = "a5";
        reg_names[16] = "a6";
        reg_names[17] = "a7";
        reg_names[18] = "s2";
        reg_names[19] = "s3";
        reg_names[20] = "s4";
        reg_names[21] = "s5";
        reg_names[22] = "s6";
        reg_names[23] = "s7";
        reg_names[24] = "s8";
        reg_names[25] = "s9";
        reg_names[26] = "s10";
        reg_names[27] = "s11";
        reg_names[28] = "t3";
        reg_names[29] = "t4";
        reg_names[30] = "t5";
        reg_names[31] = "t6";
        
        $display("========================================");
        $display("Register File State");
        $display("========================================");
        $display("Time: %0t ns | Cycle: %0d", $time, cycle_count);
        $display("----------------------------------------");
        $display("Reg | Name | Hex Value      | Decimal Value");
        $display("----------------------------------------");
        
        // Display registers in groups of 8 for readability
        for (int i = 0; i < 32; i += 8) begin
            for (int j = 0; j < 8 && (i + j) < 32; j++) begin
                int reg_idx = i + j;
                logic [DATA_WIDTH-1:0] reg_value;
                
                // Try to access register value (placeholder for now)
                reg_value = 32'h00000000;  // Placeholder
                
                // Display register with highlighting for non-zero values
                if (reg_value == 0 && reg_idx != 0) begin
                    $display("x%2d | %-4s | 0x%08h | %0d", 
                             reg_idx, reg_names[reg_idx], reg_value, $signed(reg_value));
                end else begin
                    $display("x%2d | %-4s | 0x%08h | %0d <--", 
                             reg_idx, reg_names[reg_idx], reg_value, $signed(reg_value));
                end
            end
            $display("----------------------------------------");
        end
        
        $display("========================================\n");
    endtask
    
    /**
     * Task: Check Expected Results
     * 
     * Compares actual register values with expected values.
     * Reports pass/fail for each register and overall test result.
     * 
     * @param expected_regs Array of expected register values
     * @param reg_mask Optional bitmask to specify which registers to check
     */
    task check_expected_results(
        logic [DATA_WIDTH-1:0] expected_regs [31:0],
        logic [31:0] reg_mask = 32'hFFFFFFFF
    );
        int mismatch_count = 0;
        
        $display("========================================");
        $display("Checking Expected Results");
        $display("========================================");
        $display("Time: %0t ns | Cycle: %0d", $time, cycle_count);
        $display("----------------------------------------");
        
        for (int i = 0; i < 32; i++) begin
            if (reg_mask[i]) begin
                logic [DATA_WIDTH-1:0] actual_value;
                logic [DATA_WIDTH-1:0] expected_value;
                
                // Get actual register value (placeholder for now)
                actual_value = 32'h00000000;  // Placeholder
                expected_value = expected_regs[i];
                
                if (actual_value == expected_value) begin
                    $display("PASS: x%2d = 0x%08h (expected: 0x%08h)", 
                             i, actual_value, expected_value);
                    pass_count++;
                end else begin
                    $display("FAIL: x%2d = 0x%08h (expected: 0x%08h)", 
                             i, actual_value, expected_value);
                    fail_count++;
                    mismatch_count++;
                end
                test_count++;
            end
        end
        
        $display("----------------------------------------");
        if (mismatch_count == 0) begin
            $display("✓ All register checks passed!");
        end else begin
            $display("✗ %0d register mismatch(es) detected!", mismatch_count);
        end
        $display("========================================\n");
    endtask
    
    /**
     * Task: Run Simulation
     * 
     * Runs the simulation for a specified number of clock cycles.
     * 
     * @param cycles Number of clock cycles to simulate
     */
    task run_simulation(int cycles = SIM_CYCLES);
        $display("========================================");
        $display("Running Simulation");
        $display("========================================");
        $display("Simulating for %0d clock cycles...", cycles);
        $display("Starting at time %0t ns", $time);
        $display("Starting cycle count: %0d", cycle_count);
        $display("----------------------------------------");
        
        repeat (cycles) @(posedge clk);
        
        $display("----------------------------------------");
        $display("Simulation complete at time %0t ns", $time);
        $display("Final cycle count: %0d", cycle_count);
        $display("========================================\n");
    endtask
    
    /**
     * Task: Monitor Pipeline Activity
     * 
     * Monitors pipeline activity and displays information about
     * instructions flowing through the pipeline.
     * 
     * Note: This requires access to pipeline register contents.
     * Can be extended with hierarchical access or debug ports.
     */
    task monitor_pipeline_activity();
        $display("========================================");
        $display("Pipeline Activity Monitor");
        $display("========================================");
        $display("Time: %0t ns | Cycle: %0d", $time, cycle_count);
        $display("----------------------------------------");
        $display("Stage | PC        | Instruction");
        $display("----------------------------------------");
        
        // Display pipeline stage information
        // Note: Requires hierarchical access or debug ports
        // Example: $display("IF   | 0x%08h | 0x%08h", dut.if_PC, dut.if_instruction);
        // Example: $display("ID   | 0x%08h | 0x%08h", dut.id_PC, dut.id_instruction);
        // etc.
        
        $display("----------------------------------------");
        $display("(Pipeline monitoring requires debug ports)");
        $display("========================================\n");
    endtask
    
    /**
     * Task: Print Test Summary
     * 
     * Prints a summary of test results including pass/fail counts
     * and overall test status.
     */
    task print_test_summary();
        $display("========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Total Tests:  %0d", test_count);
        $display("Passed:       %0d", pass_count);
        $display("Failed:       %0d", fail_count);
        if (test_count > 0) begin
            $display("Pass Rate:    %0.2f%%", (pass_count * 100.0) / test_count);
        end
        $display("Simulation Cycles: %0d", cycle_count);
        $display("Simulation Time:   %0t ns", $time);
        $display("========================================");
        
        if (fail_count == 0 && test_count > 0) begin
            $display("✓ ALL TESTS PASSED!");
        end else if (fail_count > 0) begin
            $display("✗ SOME TESTS FAILED!");
        end else begin
            $display("No tests executed.");
        end
        $display("========================================\n");
    endtask
    
    // ============================================
    // Main Test Procedure
    // ============================================
    
    /**
     * Main Test Procedure
     * 
     * This is the main test sequence. Modify this section to:
     * 1. Load your test program
     * 2. Set expected results
     * 3. Run simulation
     * 4. Check results
     * 5. Dump memory/registers if needed
     */
    initial begin
        // ========================================
        // VCD Waveform Dump Setup
        // ========================================
        $dumpfile("riscv_pipeline_tb.vcd");
        $dumpvars(0, riscv_pipeline_tb);
        $display("VCD waveform dump enabled: riscv_pipeline_tb.vcd");
        
        // ========================================
        // Test Initialization
        // ========================================
        $display("========================================");
        $display("RISC-V 5-Stage Pipeline Testbench");
        $display("========================================");
        $display("Starting testbench initialization...");
        $display("Clock Period: %0d ns (50MHz)", CLK_PERIOD);
        $display("Simulation Cycles: %0d", SIM_CYCLES);
        $display("========================================\n");
        
        // Initialize control signals
        pipeline_stall = 1'b0;
        pipeline_flush = 1'b0;
        
        // Initialize expected results (set these based on your test program)
        for (int i = 0; i < 32; i++) begin
            expected_regs[i] = 32'h00000000;
        end
        
        // ========================================
        // Reset Sequence
        // ========================================
        reset_sequence(RESET_CYCLES);
        
        // ========================================
        // Load Test Program
        // ========================================
        load_test_program(IMEM_INIT_FILE);
        
        // Wait a few cycles after reset
        repeat (5) @(posedge clk);
        
        // ========================================
        // Run Simulation
        // ========================================
        // Uncomment and modify as needed:
        // run_simulation(SIM_CYCLES);
        
        // Or run simulation with periodic monitoring:
        $display("Running simulation with periodic monitoring...");
        for (int i = 0; i < SIM_CYCLES; i++) begin
            @(posedge clk);
            
            // Monitor every N cycles (adjust as needed)
            if (i % 100 == 0 && i > 0) begin
                $display("Cycle %0d: Pipeline running...", cycle_count);
                // Uncomment to display register state periodically:
                // display_register_state();
            end
        end
        
        // ========================================
        // Post-Simulation Analysis
        // ========================================
        $display("\nPost-simulation analysis...");
        
        // Display final register state
        display_register_state();
        
        // Dump registers to file
        dump_registers(REG_DUMP_FILE);
        
        // Dump data memory to file
        dump_data_memory(DMEM_DUMP_FILE);
        
        // Check expected results (uncomment and set expected values):
        // check_expected_results(expected_regs);
        
        // ========================================
        // Test Summary
        // ========================================
        print_test_summary();
        
        // ========================================
        // End Simulation
        // ========================================
        $display("Simulation complete. Ending...");
        #(CLK_PERIOD * 10);  // Wait a bit before finishing
        $finish;
    end
    
    // ============================================
    // Timeout Protection
    // ============================================
    
    /**
     * Timeout Protection
     * 
     * Automatically ends simulation if it runs too long.
     * Prevents infinite loops or hanging simulations.
     */
    initial begin
        #(SIM_CYCLES * CLK_PERIOD * 2);  // Allow 2x simulation time
        $display("========================================");
        $display("WARNING: Simulation timeout reached!");
        $display("========================================");
        $display("Maximum simulation time exceeded.");
        $display("Current cycle: %0d", cycle_count);
        $display("Ending simulation...");
        $display("========================================\n");
        $finish;
    end
    
    // ============================================
    // Assertions (Optional)
    // ============================================
    
    /**
     * Assertions for runtime checking
     * 
     * Add assertions here to check for illegal states or conditions
     * during simulation. These will help catch bugs early.
     */
    
    // Example assertion: Check that reset is properly handled
    // assert property (@(posedge clk) rst_n |-> ##1 $stable(dut.if_PC) || (dut.if_PC == 0))
    //     else $error("PC not stable after reset");
    
endmodule

