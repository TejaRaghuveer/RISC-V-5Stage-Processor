/**
 * @file golden_test_tb_example.sv
 * @brief Example testbench using golden reference for verification
 * 
 * This testbench demonstrates how to use the golden reference generator
 * output to verify processor execution results.
 */

module golden_test_tb_example;

    // Clock and reset
    logic clk;
    logic rst_n;
    
    // Test parameters
    parameter int CLOCK_PERIOD = 20;  // 50 MHz
    parameter int MAX_CYCLES = 10000;
    
    // DUT instantiation
    riscv_pipeline uut (
        .clk(clk),
        .rst_n(rst_n)
        // ... other signals
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLOCK_PERIOD/2) clk = ~clk;
    end
    
    // Golden reference data structures
    // These will be populated from the reference file
    logic [31:0] expected_registers [0:31];
    logic [31:0] expected_memory [0:1023];
    
    // Load golden reference
    // Include the generated reference file
    `ifdef GOLDEN_REF_FILE
        `include `GOLDEN_REF_FILE
    `else
        // Default reference file
        `include "mem/golden_test_ref.sv"
    `endif
    
    // Load instruction memory
    initial begin
        `ifdef TEST_HEX_FILE
            $readmemh(`TEST_HEX_FILE, uut.if_stage_inst.imem_inst.memory);
        `else
            $readmemh("mem/golden_test.hex", uut.if_stage_inst.imem_inst.memory);
        `endif
    end
    
    // Initialize memory
    initial begin
        // Initialize data memory to zero
        for (int i = 0; i < 1024; i++) begin
            uut.dmem_inst.memory[i] = 32'h00000000;
        end
    end
    
    /**
     * Task: Check register file against golden reference
     */
    task check_registers();
        int errors = 0;
        int total_checked = 0;
        
        $display("=== Checking Register File ===");
        
        for (int i = 0; i < 32; i++) begin
            logic [31:0] actual_value;
            logic [31:0] expected_value;
            
            // Get actual register value from DUT
            actual_value = uut.register_file_inst.registers[i];
            expected_value = expected_registers[i];
            
            // Check if register should be non-zero or is x0
            if (expected_value != 0 || i == 0) begin
                total_checked++;
                
                if (actual_value != expected_value) begin
                    $error("[REG] x%d: Expected 0x%08X (%0d), Got 0x%08X (%0d)",
                           i, expected_value, expected_value,
                           actual_value, actual_value);
                    errors++;
                end else begin
                    $display("[REG] x%d: OK (0x%08X)", i, actual_value);
                end
            end
        end
        
        $display("Register Check: %0d/%0d passed", total_checked - errors, total_checked);
        return errors;
    endtask
    
    /**
     * Task: Check memory against golden reference
     */
    task check_memory();
        int errors = 0;
        int total_checked = 0;
        
        $display("=== Checking Memory ===");
        
        // Check all memory locations that should have been written
        for (int i = 0; i < 1024; i++) begin
            logic [31:0] actual_value;
            logic [31:0] expected_value;
            
            actual_value = uut.dmem_inst.memory[i];
            expected_value = expected_memory[i];
            
            // Only check locations that should have been written
            if (expected_value != 0) begin
                total_checked++;
                
                if (actual_value != expected_value) begin
                    $error("[MEM] memory[%0d]: Expected 0x%08X (%0d), Got 0x%08X (%0d)",
                           i, expected_value, expected_value,
                           actual_value, actual_value);
                    errors++;
                end else begin
                    $display("[MEM] memory[%0d]: OK (0x%08X)", i, actual_value);
                end
            end
        end
        
        if (total_checked == 0) begin
            $display("Memory Check: No memory writes expected");
        end else begin
            $display("Memory Check: %0d/%0d passed", total_checked - errors, total_checked);
        end
        
        return errors;
    endtask
    
    /**
     * Task: Comprehensive result checking
     */
    task check_results();
        int reg_errors, mem_errors;
        int total_errors;
        
        $display("\n========================================");
        $display("Golden Reference Comparison");
        $display("========================================\n");
        
        // Wait a few cycles for pipeline to flush
        #(CLOCK_PERIOD * 5);
        
        // Check registers
        reg_errors = check_registers();
        
        // Check memory
        mem_errors = check_memory();
        
        total_errors = reg_errors + mem_errors;
        
        $display("\n========================================");
        if (total_errors == 0) begin
            $display("PASS: All comparisons match golden reference");
        end else begin
            $error("FAIL: %0d total mismatches found (%0d registers, %0d memory)",
                   total_errors, reg_errors, mem_errors);
        end
        $display("========================================\n");
    endtask
    
    /**
     * Task: Display execution summary
     */
    task display_summary();
        $display("\n=== Execution Summary ===");
        $display("Simulation cycles: %0d", $time / CLOCK_PERIOD);
        $display("Final PC: 0x%08X", uut.pc_reg);
        $display("=======================\n");
    endtask
    
    /**
     * Main test sequence
     */
    initial begin
        $display("========================================");
        $display("Golden Reference Test");
        $display("========================================\n");
        
        // Initialize
        rst_n = 0;
        #(CLOCK_PERIOD * 2);
        
        // Release reset
        rst_n = 1;
        $display("Reset released at time %0t", $time);
        
        // Run simulation
        #(MAX_CYCLES * CLOCK_PERIOD);
        
        // Check results
        check_results();
        
        // Display summary
        display_summary();
        
        // Finish
        $finish;
    end
    
    /**
     * Timeout watchdog
     */
    initial begin
        #(MAX_CYCLES * CLOCK_PERIOD);
        $error("Timeout: Simulation exceeded maximum cycles");
        $finish;
    end
    
    /**
     * Waveform dump (optional)
     */
    initial begin
        `ifdef DUMP_WAVEFORM
            $dumpfile("golden_test_tb.vcd");
            $dumpvars(0, golden_test_tb_example);
        `endif
    end

endmodule

