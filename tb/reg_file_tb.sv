/**
 * @file reg_file_tb.sv
 * @brief Comprehensive Testbench for RISC-V Register File
 * 
 * This testbench thoroughly tests the register file module with various scenarios:
 * - Simultaneous reads from two ports (rs1 and rs2)
 * - Write-then-read operations to verify data persistence
 * - x0 register always reads zero, even with write attempts
 * - Read-during-write behavior (forwarding vs. old data)
 * - Reset functionality and initialization
 * - Edge cases and corner conditions
 * 
 * The testbench includes self-checking assertions and detailed output messages
 * for each test case.
 */

`timescale 1ns / 1ps

module reg_file_tb;

    // Parameters
    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 5;
    parameter NUM_REGS = 32;
    parameter CLK_PERIOD = 10;  // 10ns clock period (100 MHz)
    
    // Testbench signals
    logic                        clk;
    logic                        rst_n;
    logic [ADDR_WIDTH-1:0]       rs1_addr;
    logic [ADDR_WIDTH-1:0]       rs2_addr;
    logic                        reg_write_en;
    logic [ADDR_WIDTH-1:0]       rd_addr;
    logic [DATA_WIDTH-1:0]       rd_data;
    logic [DATA_WIDTH-1:0]       rs1_data;
    logic [DATA_WIDTH-1:0]       rs2_data;
    
    // Expected values for verification
    logic [DATA_WIDTH-1:0]       expected_rs1_data;
    logic [DATA_WIDTH-1:0]       expected_rs2_data;
    
    // Test statistics
    int test_count = 0;
    int pass_count = 0;
    int fail_count = 0;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // Instantiate the register file module
    reg_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .NUM_REGS(NUM_REGS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .rs1_addr(rs1_addr),
        .rs1_data(rs1_data),
        .rs2_addr(rs2_addr),
        .rs2_data(rs2_data),
        .reg_write_en(reg_write_en),
        .rd_addr(rd_addr),
        .rd_data(rd_data)
    );
    
    /**
     * Task to run a single test case
     * @param test_name: Name of the test case
     * @param delay: Delay before checking results (in time units)
     */
    task check_result(string test_name, int delay = 0);
        begin
            test_count++;
            if (delay > 0) #delay;
            
            // Check rs1_data
            if (rs1_data == expected_rs1_data) begin
                // rs1_data matches
            end else begin
                $display("[FAIL] Test %0d: %s - rs1_data mismatch", test_count, test_name);
                $display("       rs1_addr: 0x%02h (%0d)", rs1_addr, rs1_addr);
                $display("       Expected rs1_data: 0x%08h (%0d)", expected_rs1_data, $signed(expected_rs1_data));
                $display("       Actual rs1_data:   0x%08h (%0d)", rs1_data, $signed(rs1_data));
                fail_count++;
                return;
            end
            
            // Check rs2_data
            if (rs2_data == expected_rs2_data) begin
                $display("[PASS] Test %0d: %s", test_count, test_name);
                pass_count++;
            end else begin
                $display("[FAIL] Test %0d: %s - rs2_data mismatch", test_count, test_name);
                $display("       rs2_addr: 0x%02h (%0d)", rs2_addr, rs2_addr);
                $display("       Expected rs2_data: 0x%08h (%0d)", expected_rs2_data, $signed(expected_rs2_data));
                $display("       Actual rs2_data:   0x%08h (%0d)", rs2_data, $signed(rs2_data));
                fail_count++;
            end
        end
    endtask
    
    /**
     * Task to write to a register
     * @param addr: Register address to write
     * @param data: Data to write
     * @param enable: Write enable (default 1)
     */
    task write_register(
        input logic [ADDR_WIDTH-1:0] addr,
        input logic [DATA_WIDTH-1:0] data,
        input logic enable = 1'b1
    );
        begin
            @(posedge clk);
            #1; // Small delay after clock edge
            reg_write_en = enable;
            rd_addr = addr;
            rd_data = data;
            @(posedge clk);
            #1;
            reg_write_en = 1'b0;
        end
    endtask
    
    /**
     * Task to read from registers
     * @param addr1: Address for rs1
     * @param addr2: Address for rs2
     */
    task read_registers(
        input logic [ADDR_WIDTH-1:0] addr1,
        input logic [ADDR_WIDTH-1:0] addr2
    );
        begin
            rs1_addr = addr1;
            rs2_addr = addr2;
            #1; // Small delay for combinational logic
        end
    endtask
    
    /**
     * Test Suite 1: Reset Functionality
     */
    task test_reset;
        $display("========================================");
        $display("Testing Reset Functionality");
        $display("========================================");
        
        // Initialize with some values
        write_register(5'd1, 32'h12345678);
        write_register(5'd2, 32'hABCDEF00);
        write_register(5'd31, 32'hFFFFFFFF);
        
        // Read to verify values were written
        read_registers(5'd1, 5'd2);
        expected_rs1_data = 32'h12345678;
        expected_rs2_data = 32'hABCDEF00;
        check_result("Reset: Verify initial writes");
        
        // Apply reset
        $display("\nApplying reset...");
        rst_n = 1'b0;
        #(CLK_PERIOD * 2);
        
        // Check that all registers read as zero (except x0 which is always zero)
        read_registers(5'd1, 5'd2);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("Reset: Registers cleared after reset");
        
        // Check x0 during reset
        read_registers(5'd0, 5'd0);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("Reset: x0 reads zero during reset");
        
        // Release reset
        rst_n = 1'b1;
        #(CLK_PERIOD);
        
        // Verify registers remain zero after reset release
        read_registers(5'd1, 5'd31);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("Reset: Registers remain zero after reset release");
        
        $display("");
    endtask
    
    /**
     * Test Suite 2: Write-Then-Read Operations
     */
    task test_write_then_read;
        $display("========================================");
        $display("Testing Write-Then-Read Operations");
        $display("========================================");
        
        // Write to various registers
        write_register(5'd1, 32'h11111111);
        write_register(5'd2, 32'h22222222);
        write_register(5'd10, 32'hAAAAAAAA);
        write_register(5'd20, 32'h55555555);
        write_register(5'd31, 32'hFFFFFFFF);
        
        // Read back and verify
        read_registers(5'd1, 5'd2);
        expected_rs1_data = 32'h11111111;
        expected_rs2_data = 32'h22222222;
        check_result("Write-Read: Verify x1 and x2");
        
        read_registers(5'd10, 5'd20);
        expected_rs1_data = 32'hAAAAAAAA;
        expected_rs2_data = 32'h55555555;
        check_result("Write-Read: Verify x10 and x20");
        
        read_registers(5'd31, 5'd1);
        expected_rs1_data = 32'hFFFFFFFF;
        expected_rs2_data = 32'h11111111;
        check_result("Write-Read: Verify x31 and x1");
        
        // Overwrite and verify
        write_register(5'd1, 32'hDEADBEEF);
        read_registers(5'd1, 5'd1);
        expected_rs1_data = 32'hDEADBEEF;
        expected_rs2_data = 32'hDEADBEEF;
        check_result("Write-Read: Overwrite x1 and verify");
        
        // Write zero and verify
        write_register(5'd5, 32'd0);
        read_registers(5'd5, 5'd5);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("Write-Read: Write zero to x5");
        
        $display("");
    endtask
    
    /**
     * Test Suite 3: Simultaneous Reads from Two Ports
     */
    task test_simultaneous_reads;
        $display("========================================");
        $display("Testing Simultaneous Reads from Two Ports");
        $display("========================================");
        
        // Write different values to different registers
        write_register(5'd1, 32'h11111111);
        write_register(5'd2, 32'h22222222);
        write_register(5'd3, 32'h33333333);
        write_register(5'd4, 32'h44444444);
        
        // Read from both ports simultaneously
        read_registers(5'd1, 5'd2);
        expected_rs1_data = 32'h11111111;
        expected_rs2_data = 32'h22222222;
        check_result("Simultaneous Read: x1 and x2");
        
        read_registers(5'd3, 5'd4);
        expected_rs1_data = 32'h33333333;
        expected_rs2_data = 32'h44444444;
        check_result("Simultaneous Read: x3 and x4");
        
        read_registers(5'd1, 5'd4);
        expected_rs1_data = 32'h11111111;
        expected_rs2_data = 32'h44444444;
        check_result("Simultaneous Read: x1 and x4");
        
        // Read same register from both ports
        read_registers(5'd2, 5'd2);
        expected_rs1_data = 32'h22222222;
        expected_rs2_data = 32'h22222222;
        check_result("Simultaneous Read: Same register (x2) from both ports");
        
        // Change addresses and verify immediate response (asynchronous reads)
        rs1_addr = 5'd1;
        rs2_addr = 5'd3;
        #1;
        expected_rs1_data = 32'h11111111;
        expected_rs2_data = 32'h33333333;
        check_result("Simultaneous Read: Immediate response to address change");
        
        $display("");
    endtask
    
    /**
     * Test Suite 4: Register x0 Always Reads Zero
     */
    task test_x0_zero_register;
        $display("========================================");
        $display("Testing Register x0 (Zero Register)");
        $display("========================================");
        
        // Read x0 - should be zero
        read_registers(5'd0, 5'd0);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("x0 Test: Read x0 (should be zero)");
        
        // Try to write to x0 (should be ignored)
        $display("\nAttempting to write to x0 (should be ignored)...");
        write_register(5'd0, 32'hDEADBEEF);
        
        // Read x0 again - should still be zero
        read_registers(5'd0, 5'd0);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("x0 Test: Read x0 after write attempt (should still be zero)");
        
        // Write to another register, then read x0
        write_register(5'd1, 32'h12345678);
        read_registers(5'd0, 5'd1);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'h12345678;
        check_result("x0 Test: x0 reads zero while x1 has data");
        
        // Try multiple writes to x0
        write_register(5'd0, 32'h11111111);
        write_register(5'd0, 32'h22222222);
        write_register(5'd0, 32'hFFFFFFFF);
        read_registers(5'd0, 5'd0);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("x0 Test: Multiple write attempts to x0 (all ignored)");
        
        // Read x0 from both ports simultaneously
        read_registers(5'd0, 5'd0);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("x0 Test: x0 from both ports simultaneously");
        
        $display("");
    endtask
    
    /**
     * Test Suite 5: Read-During-Write Behavior
     * 
     * This tests the behavior when reading and writing to the same register
     * in the same cycle. In a typical register file:
     * - If reading and writing same register: reads old data (no forwarding)
     * - If reading different register while writing: reads normally
     */
    task test_read_during_write;
        $display("========================================");
        $display("Testing Read-During-Write Behavior");
        $display("========================================");
        
        // Initialize register with known value
        write_register(5'd10, 32'hAAAAAAAA);
        read_registers(5'd10, 5'd10);
        expected_rs1_data = 32'hAAAAAAAA;
        expected_rs2_data = 32'hAAAAAAAA;
        check_result("Read-During-Write: Initialize x10");
        
        // Read from x10 while writing to x10 (should read old value)
        $display("\nReading x10 while writing new value to x10...");
        rs1_addr = 5'd10;
        rs2_addr = 5'd10;
        #1;
        
        // Set up write (but don't clock yet)
        reg_write_en = 1'b1;
        rd_addr = 5'd10;
        rd_data = 32'hBBBBBBBB;
        
        // Read should still show old value (before clock edge)
        expected_rs1_data = 32'hAAAAAAAA;
        expected_rs2_data = 32'hAAAAAAAA;
        check_result("Read-During-Write: Read old value before clock edge");
        
        // Clock edge - write occurs
        @(posedge clk);
        #1;
        reg_write_en = 1'b0;
        
        // Now read should show new value
        read_registers(5'd10, 5'd10);
        expected_rs1_data = 32'hBBBBBBBB;
        expected_rs2_data = 32'hBBBBBBBB;
        check_result("Read-During-Write: Read new value after clock edge");
        
        // Read from different register while writing to another
        write_register(5'd5, 32'h55555555);
        rs1_addr = 5'd5;
        rs2_addr = 5'd10;
        #1;
        expected_rs1_data = 32'h55555555;
        expected_rs2_data = 32'hBBBBBBBB;
        check_result("Read-During-Write: Read different registers simultaneously");
        
        // Write to x20, read from x10 and x20 simultaneously
        reg_write_en = 1'b1;
        rd_addr = 5'd20;
        rd_data = 32'h20202020;
        rs1_addr = 5'd10;
        rs2_addr = 5'd20;
        #1;
        // Before clock: x20 should read old value (0 after reset)
        expected_rs1_data = 32'hBBBBBBBB;
        expected_rs2_data = 32'd0;
        check_result("Read-During-Write: Read x20 before write completes");
        
        @(posedge clk);
        #1;
        reg_write_en = 1'b0;
        
        // After clock: x20 should have new value
        read_registers(5'd10, 5'd20);
        expected_rs1_data = 32'hBBBBBBBB;
        expected_rs2_data = 32'h20202020;
        check_result("Read-During-Write: Read x20 after write completes");
        
        $display("");
    endtask
    
    /**
     * Test Suite 6: Write Enable Control
     */
    task test_write_enable;
        $display("========================================");
        $display("Testing Write Enable Control");
        $display("========================================");
        
        // Write with enable = 1
        write_register(5'd15, 32'hF0F0F0F0, 1'b1);
        read_registers(5'd15, 5'd15);
        expected_rs1_data = 32'hF0F0F0F0;
        expected_rs2_data = 32'hF0F0F0F0;
        check_result("Write Enable: Write with enable=1");
        
        // Attempt write with enable = 0 (should not write)
        @(posedge clk);
        #1;
        reg_write_en = 1'b0;
        rd_addr = 5'd15;
        rd_data = 32'hAAAAAAAA;
        @(posedge clk);
        #1;
        
        // Register should still have old value
        read_registers(5'd15, 5'd15);
        expected_rs1_data = 32'hF0F0F0F0;
        expected_rs2_data = 32'hF0F0F0F0;
        check_result("Write Enable: Write with enable=0 (should not write)");
        
        // Write with enable = 1 again
        write_register(5'd15, 32'hAAAAAAAA, 1'b1);
        read_registers(5'd15, 5'd15);
        expected_rs1_data = 32'hAAAAAAAA;
        expected_rs2_data = 32'hAAAAAAAA;
        check_result("Write Enable: Write with enable=1 again");
        
        $display("");
    endtask
    
    /**
     * Test Suite 7: Edge Cases
     */
    task test_edge_cases;
        $display("========================================");
        $display("Testing Edge Cases");
        $display("========================================");
        
        // Write to all registers
        for (int i = 1; i < NUM_REGS; i++) begin
            write_register(i[4:0], 32'h10000000 + i);
        end
        
        // Read from various registers
        read_registers(5'd1, 5'd31);
        expected_rs1_data = 32'h10000001;
        expected_rs2_data = 32'h1000001F;
        check_result("Edge Cases: Write to all registers, read x1 and x31");
        
        // Test maximum and minimum values
        write_register(5'd7, 32'h7FFFFFFF);
        write_register(5'd8, 32'h80000000);
        write_register(5'd9, 32'hFFFFFFFF);
        read_registers(5'd7, 5'd8);
        expected_rs1_data = 32'h7FFFFFFF;
        expected_rs2_data = 32'h80000000;
        check_result("Edge Cases: Maximum positive value");
        
        read_registers(5'd8, 5'd9);
        expected_rs1_data = 32'h80000000;
        expected_rs2_data = 32'hFFFFFFFF;
        check_result("Edge Cases: Minimum and -1 values");
        
        // Test all zeros
        write_register(5'd12, 32'd0);
        read_registers(5'd12, 5'd12);
        expected_rs1_data = 32'd0;
        expected_rs2_data = 32'd0;
        check_result("Edge Cases: Write zero to non-x0 register");
        
        $display("");
    endtask
    
    /**
     * Main test procedure
     */
    initial begin
        // Dump VCD file for waveform viewing
        $dumpfile("reg_file_tb.vcd");
        $dumpvars(0, reg_file_tb);
        
        $display("========================================");
        $display("RISC-V Register File Comprehensive Testbench");
        $display("========================================");
        $display("Starting tests...\n");
        
        // Initialize signals
        rst_n = 1'b1;
        rs1_addr = 5'd0;
        rs2_addr = 5'd0;
        reg_write_en = 1'b0;
        rd_addr = 5'd0;
        rd_data = 32'd0;
        
        // Wait a few clock cycles
        #(CLK_PERIOD * 2);
        
        // Run all test suites
        test_reset();
        test_write_then_read();
        test_simultaneous_reads();
        test_x0_zero_register();
        test_read_during_write();
        test_write_enable();
        test_edge_cases();
        
        // Print test summary
        $display("========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Total Tests:  %0d", test_count);
        $display("Passed:       %0d", pass_count);
        $display("Failed:       %0d", fail_count);
        $display("Pass Rate:    %0.2f%%", (pass_count * 100.0) / test_count);
        $display("========================================");
        
        if (fail_count == 0) begin
            $display("✓ ALL TESTS PASSED!");
        end else begin
            $display("✗ SOME TESTS FAILED!");
        end
        $display("========================================");
        
        #100;
        $finish;
    end
    
    /**
     * Assertions for runtime checking
     */
    // Assertion: x0 should always read as zero
    always @(*) begin
        if (rs1_addr == 5'd0) begin
            assert (rs1_data == 32'd0) else begin
                $error("Assertion failed: x0 (rs1) should always read zero! Got: 0x%08h", rs1_data);
            end
        end
        
        if (rs2_addr == 5'd0) begin
            assert (rs2_data == 32'd0) else begin
                $error("Assertion failed: x0 (rs2) should always read zero! Got: 0x%08h", rs2_data);
            end
        end
    end
    
    // Assertion: Write to x0 should be ignored
    always @(posedge clk) begin
        if (reg_write_en && rd_addr == 5'd0) begin
            // This is expected behavior - write to x0 is ignored
            // We verify this in the test cases
        end
    end

endmodule

