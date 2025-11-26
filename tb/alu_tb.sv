/**
 * @file alu_tb.sv
 * @brief Comprehensive Testbench for 32-bit RISC-V ALU
 * 
 * This testbench thoroughly tests all ALU operations with various edge cases
 * including zero operands, maximum values, negative numbers, overflow conditions,
 * and different shift amounts. The testbench includes self-checking assertions
 * and displays pass/fail results for each test case.
 * 
 * Test Coverage:
 * - All ALU operations (ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU)
 * - Edge cases: zero, max values, negative numbers
 * - Overflow conditions
 * - Shift operations with various amounts (0-31)
 * - Signed and unsigned comparisons
 */

`timescale 1ns / 1ps

module alu_tb;

    // Parameters
    parameter DATA_WIDTH = 32;
    parameter CLK_PERIOD = 10;  // 10ns clock period (100 MHz)
    
    // Testbench signals
    logic [DATA_WIDTH-1:0] operand_a;
    logic [DATA_WIDTH-1:0] operand_b;
    logic [3:0]             alu_control;
    logic [DATA_WIDTH-1:0] result;
    logic                   zero_flag;
    
    // Expected values for verification
    logic [DATA_WIDTH-1:0] expected_result;
    logic                   expected_zero_flag;
    
    // Test statistics
    int test_count = 0;
    int pass_count = 0;
    int fail_count = 0;
    
    // Instantiate the ALU module
    alu #(
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .operand_a(operand_a),
        .operand_b(operand_b),
        .alu_control(alu_control),
        .result(result),
        .zero_flag(zero_flag)
    );
    
    /**
     * Task to run a single test case
     * @param op_a: First operand
     * @param op_b: Second operand
     * @param control: ALU control signal
     * @param exp_result: Expected result
     * @param exp_zero: Expected zero flag
     * @param test_name: Name of the test case
     */
    task test_case(
        input logic [DATA_WIDTH-1:0] op_a,
        input logic [DATA_WIDTH-1:0] op_b,
        input logic [3:0]             control,
        input logic [DATA_WIDTH-1:0] exp_result,
        input logic                   exp_zero,
        input string                  test_name
    );
        begin
            test_count++;
            operand_a = op_a;
            operand_b = op_b;
            alu_control = control;
            expected_result = exp_result;
            expected_zero_flag = exp_zero;
            
            #5; // Wait for propagation delay
            
            // Check result
            if (result == expected_result && zero_flag == expected_zero_flag) begin
                $display("[PASS] Test %0d: %s", test_count, test_name);
                $display("       Operand A: 0x%08h (%0d)", operand_a, $signed(operand_a));
                $display("       Operand B: 0x%08h (%0d)", operand_b, $signed(operand_b));
                $display("       Control: 0x%01h", alu_control);
                $display("       Result: 0x%08h (%0d)", result, $signed(result));
                $display("       Zero Flag: %0b", zero_flag);
                pass_count++;
            end else begin
                $display("[FAIL] Test %0d: %s", test_count, test_name);
                $display("       Operand A: 0x%08h (%0d)", operand_a, $signed(operand_a));
                $display("       Operand B: 0x%08h (%0d)", operand_b, $signed(operand_b));
                $display("       Control: 0x%01h", alu_control);
                $display("       Expected Result: 0x%08h (%0d)", expected_result, $signed(expected_result));
                $display("       Actual Result:   0x%08h (%0d)", result, $signed(result));
                $display("       Expected Zero: %0b", expected_zero_flag);
                $display("       Actual Zero:   %0b", zero_flag);
                fail_count++;
            end
            $display("");
        end
    endtask
    
    /**
     * Test Suite: ADD Operations
     */
    task test_add;
        $display("========================================");
        $display("Testing ADD Operation (alu_control = 4'b0010)");
        $display("========================================");
        
        // Basic addition
        test_case(32'd10, 32'd20, 4'b0010, 32'd30, 1'b0, "ADD: 10 + 20 = 30");
        
        // Zero operands
        test_case(32'd0, 32'd0, 4'b0010, 32'd0, 1'b1, "ADD: 0 + 0 = 0");
        test_case(32'd100, 32'd0, 4'b0010, 32'd100, 1'b0, "ADD: 100 + 0 = 100");
        test_case(32'd0, 32'd100, 4'b0010, 32'd100, 1'b0, "ADD: 0 + 100 = 100");
        
        // Maximum positive values
        test_case(32'h7FFFFFFF, 32'd1, 4'b0010, 32'h80000000, 1'b0, "ADD: MAX_INT + 1 (overflow)");
        test_case(32'h7FFFFFFF, 32'h7FFFFFFF, 4'b0010, 32'hFFFFFFFE, 1'b0, "ADD: MAX_INT + MAX_INT");
        
        // Negative numbers
        test_case(32'hFFFFFFFF, 32'd1, 4'b0010, 32'd0, 1'b1, "ADD: -1 + 1 = 0");
        test_case(32'hFFFFFFFE, 32'd1, 4'b0010, 32'hFFFFFFFF, 1'b0, "ADD: -2 + 1 = -1");
        test_case(32'h80000000, 32'h80000000, 4'b0010, 32'd0, 1'b1, "ADD: MIN_INT + MIN_INT (overflow)");
        
        // Mixed positive and negative
        test_case(32'd50, 32'hFFFFFFF6, 4'b0010, 32'd40, 1'b0, "ADD: 50 + (-10) = 40");
        test_case(32'hFFFFFFF6, 32'd50, 4'b0010, 32'd40, 1'b0, "ADD: (-10) + 50 = 40");
        
        $display("");
    endtask
    
    /**
     * Test Suite: SUB Operations
     */
    task test_sub;
        $display("========================================");
        $display("Testing SUB Operation (alu_control = 4'b0110)");
        $display("========================================");
        
        // Basic subtraction
        test_case(32'd30, 32'd10, 4'b0110, 32'd20, 1'b0, "SUB: 30 - 10 = 20");
        test_case(32'd10, 32'd30, 4'b0110, 32'hFFFFFFE6, 1'b0, "SUB: 10 - 30 = -20");
        
        // Zero operands
        test_case(32'd0, 32'd0, 4'b0110, 32'd0, 1'b1, "SUB: 0 - 0 = 0");
        test_case(32'd100, 32'd0, 4'b0110, 32'd100, 1'b0, "SUB: 100 - 0 = 100");
        test_case(32'd0, 32'd100, 4'b0110, 32'hFFFFFF9C, 1'b0, "SUB: 0 - 100 = -100");
        
        // Maximum values
        test_case(32'h7FFFFFFF, 32'h7FFFFFFF, 4'b0110, 32'd0, 1'b1, "SUB: MAX_INT - MAX_INT = 0");
        test_case(32'h80000000, 32'd1, 4'b0110, 32'h7FFFFFFF, 1'b0, "SUB: MIN_INT - 1");
        
        // Negative numbers
        test_case(32'hFFFFFFFF, 32'hFFFFFFFF, 4'b0110, 32'd0, 1'b1, "SUB: -1 - (-1) = 0");
        test_case(32'hFFFFFFFE, 32'hFFFFFFFF, 4'b0110, 32'd1, 1'b0, "SUB: -2 - (-1) = -1");
        
        // Equality check (for branch instructions)
        test_case(32'd100, 32'd100, 4'b0110, 32'd0, 1'b1, "SUB: 100 - 100 = 0 (equal)");
        
        $display("");
    endtask
    
    /**
     * Test Suite: AND Operations
     */
    task test_and;
        $display("========================================");
        $display("Testing AND Operation (alu_control = 4'b0000)");
        $display("========================================");
        
        // Basic AND
        test_case(32'hAAAAAAAA, 32'h55555555, 4'b0000, 32'd0, 1'b1, "AND: 0xAAAA & 0x5555 = 0");
        test_case(32'hFFFFFFFF, 32'h55555555, 4'b0000, 32'h55555555, 1'b0, "AND: 0xFFFF & 0x5555 = 0x5555");
        test_case(32'h12345678, 32'h87654321, 4'b0000, 32'h02244220, 1'b0, "AND: 0x12345678 & 0x87654321");
        
        // Zero operands
        test_case(32'd0, 32'hFFFFFFFF, 4'b0000, 32'd0, 1'b1, "AND: 0 & 0xFFFF = 0");
        test_case(32'hFFFFFFFF, 32'd0, 4'b0000, 32'd0, 1'b1, "AND: 0xFFFF & 0 = 0");
        
        // All ones
        test_case(32'hFFFFFFFF, 32'hFFFFFFFF, 4'b0000, 32'hFFFFFFFF, 1'b0, "AND: 0xFFFF & 0xFFFF = 0xFFFF");
        
        $display("");
    endtask
    
    /**
     * Test Suite: OR Operations
     */
    task test_or;
        $display("========================================");
        $display("Testing OR Operation (alu_control = 4'b0001)");
        $display("========================================");
        
        // Basic OR
        test_case(32'hAAAAAAAA, 32'h55555555, 4'b0001, 32'hFFFFFFFF, 1'b0, "OR: 0xAAAA | 0x5555 = 0xFFFF");
        test_case(32'h12345678, 32'h87654321, 4'b0001, 32'h97755779, 1'b0, "OR: 0x12345678 | 0x87654321");
        
        // Zero operands
        test_case(32'd0, 32'hFFFFFFFF, 4'b0001, 32'hFFFFFFFF, 1'b0, "OR: 0 | 0xFFFF = 0xFFFF");
        test_case(32'hFFFFFFFF, 32'd0, 4'b0001, 32'hFFFFFFFF, 1'b0, "OR: 0xFFFF | 0 = 0xFFFF");
        test_case(32'd0, 32'd0, 4'b0001, 32'd0, 1'b1, "OR: 0 | 0 = 0");
        
        // All ones
        test_case(32'hFFFFFFFF, 32'hFFFFFFFF, 4'b0001, 32'hFFFFFFFF, 1'b0, "OR: 0xFFFF | 0xFFFF = 0xFFFF");
        
        $display("");
    endtask
    
    /**
     * Test Suite: XOR Operations
     */
    task test_xor;
        $display("========================================");
        $display("Testing XOR Operation (alu_control = 4'b1100)");
        $display("========================================");
        
        // Basic XOR
        test_case(32'hAAAAAAAA, 32'h55555555, 4'b1100, 32'hFFFFFFFF, 1'b0, "XOR: 0xAAAA ^ 0x5555 = 0xFFFF");
        test_case(32'hFFFFFFFF, 32'hFFFFFFFF, 4'b1100, 32'd0, 1'b1, "XOR: 0xFFFF ^ 0xFFFF = 0");
        test_case(32'h12345678, 32'h12345678, 4'b1100, 32'd0, 1'b1, "XOR: Same operands = 0");
        
        // Zero operands
        test_case(32'd0, 32'hFFFFFFFF, 4'b1100, 32'hFFFFFFFF, 1'b0, "XOR: 0 ^ 0xFFFF = 0xFFFF");
        test_case(32'hFFFFFFFF, 32'd0, 4'b1100, 32'hFFFFFFFF, 1'b0, "XOR: 0xFFFF ^ 0 = 0xFFFF");
        test_case(32'd0, 32'd0, 4'b1100, 32'd0, 1'b1, "XOR: 0 ^ 0 = 0");
        
        $display("");
    endtask
    
    /**
     * Test Suite: SLL (Shift Left Logical) Operations
     */
    task test_sll;
        $display("========================================");
        $display("Testing SLL Operation (alu_control = 4'b1000)");
        $display("========================================");
        
        // Basic shifts
        test_case(32'd1, 32'd1, 4'b1000, 32'd2, 1'b0, "SLL: 1 << 1 = 2");
        test_case(32'd1, 32'd5, 4'b1000, 32'd32, 1'b0, "SLL: 1 << 5 = 32");
        test_case(32'd1, 32'd31, 4'b1000, 32'h80000000, 1'b0, "SLL: 1 << 31 = 0x80000000");
        
        // Zero shift
        test_case(32'hFFFFFFFF, 32'd0, 4'b1000, 32'hFFFFFFFF, 1'b0, "SLL: 0xFFFF << 0 = 0xFFFF");
        test_case(32'd100, 32'd0, 4'b1000, 32'd100, 1'b0, "SLL: 100 << 0 = 100");
        
        // Maximum shift
        test_case(32'd1, 32'd31, 4'b1000, 32'h80000000, 1'b0, "SLL: 1 << 31 = MIN_INT");
        test_case(32'h00000001, 32'd31, 4'b1000, 32'h80000000, 1'b0, "SLL: Max shift left");
        
        // Shift beyond 31 (should use lower 5 bits)
        test_case(32'd1, 32'd63, 4'b1000, 32'h80000000, 1'b0, "SLL: 1 << 63 (uses 31)");
        test_case(32'd1, 32'h0000001F, 4'b1000, 32'h80000000, 1'b0, "SLL: 1 << 0x1F = 1 << 31");
        
        // Pattern shifts
        test_case(32'h0000000F, 32'd4, 4'b1000, 32'h000000F0, 1'b0, "SLL: 0x0F << 4 = 0xF0");
        test_case(32'h12345678, 32'd8, 4'b1000, 32'h34567800, 1'b0, "SLL: Pattern shift");
        
        $display("");
    endtask
    
    /**
     * Test Suite: SRL (Shift Right Logical) Operations
     */
    task test_srl;
        $display("========================================");
        $display("Testing SRL Operation (alu_control = 4'b1001)");
        $display("========================================");
        
        // Basic shifts
        test_case(32'd32, 32'd5, 4'b1001, 32'd1, 1'b0, "SRL: 32 >> 5 = 1");
        test_case(32'h80000000, 32'd1, 4'b1001, 32'h40000000, 1'b0, "SRL: 0x80000000 >> 1 = 0x40000000");
        test_case(32'h80000000, 32'd31, 4'b1001, 32'd1, 1'b0, "SRL: 0x80000000 >> 31 = 1");
        
        // Zero shift
        test_case(32'hFFFFFFFF, 32'd0, 4'b1001, 32'hFFFFFFFF, 1'b0, "SRL: 0xFFFF >> 0 = 0xFFFF");
        
        // Maximum shift
        test_case(32'hFFFFFFFF, 32'd31, 4'b1001, 32'd1, 1'b0, "SRL: 0xFFFF >> 31 = 1");
        test_case(32'd1, 32'd31, 4'b1001, 32'd0, 1'b1, "SRL: 1 >> 31 = 0");
        
        // Pattern shifts
        test_case(32'hF0F0F0F0, 32'd4, 4'b1001, 32'h0F0F0F0F, 1'b0, "SRL: Pattern shift");
        test_case(32'h12345678, 32'd8, 4'b1001, 32'h00123456, 1'b0, "SRL: Pattern shift");
        
        $display("");
    endtask
    
    /**
     * Test Suite: SRA (Shift Right Arithmetic) Operations
     */
    task test_sra;
        $display("========================================");
        $display("Testing SRA Operation (alu_control = 4'b1010)");
        $display("========================================");
        
        // Positive number shifts (should behave like SRL)
        test_case(32'd32, 32'd5, 4'b1010, 32'd1, 1'b0, "SRA: 32 >>> 5 = 1 (positive)");
        test_case(32'h7FFFFFFF, 32'd1, 4'b1010, 32'h3FFFFFFF, 1'b0, "SRA: MAX_INT >>> 1");
        
        // Negative number shifts (sign extension)
        test_case(32'h80000000, 32'd1, 4'b1010, 32'hC0000000, 1'b0, "SRA: MIN_INT >>> 1 (sign extend)");
        test_case(32'hFFFFFFFF, 32'd1, 4'b1010, 32'hFFFFFFFF, 1'b0, "SRA: -1 >>> 1 = -1 (sign extend)");
        test_case(32'hFFFFFF00, 32'd4, 4'b1010, 32'hFFFFFFF0, 1'b0, "SRA: -256 >>> 4 = -16");
        
        // Zero shift
        test_case(32'hFFFFFFFF, 32'd0, 4'b1010, 32'hFFFFFFFF, 1'b0, "SRA: -1 >>> 0 = -1");
        
        // Maximum shift
        test_case(32'hFFFFFFFF, 32'd31, 4'b1010, 32'hFFFFFFFF, 1'b0, "SRA: -1 >>> 31 = -1");
        test_case(32'h80000000, 32'd31, 4'b1010, 32'hFFFFFFFF, 1'b0, "SRA: MIN_INT >>> 31 = -1");
        
        // Mixed positive and negative
        test_case(32'h00000080, 32'd4, 4'b1010, 32'h00000008, 1'b0, "SRA: 128 >>> 4 = 8 (positive)");
        test_case(32'hFFFFFF80, 32'd4, 4'b1010, 32'hFFFFFFF8, 1'b0, "SRA: -128 >>> 4 = -8 (negative)");
        
        $display("");
    endtask
    
    /**
     * Test Suite: SLT (Set Less Than - Signed) Operations
     */
    task test_slt;
        $display("========================================");
        $display("Testing SLT Operation (alu_control = 4'b0111)");
        $display("========================================");
        
        // Basic comparisons
        test_case(32'd10, 32'd20, 4'b0111, 32'd1, 1'b0, "SLT: 10 < 20 = 1");
        test_case(32'd20, 32'd10, 4'b0111, 32'd0, 1'b1, "SLT: 20 < 10 = 0");
        test_case(32'd10, 32'd10, 4'b0111, 32'd0, 1'b1, "SLT: 10 < 10 = 0");
        
        // Negative numbers
        test_case(32'hFFFFFFFF, 32'd0, 4'b0111, 32'd1, 1'b0, "SLT: -1 < 0 = 1");
        test_case(32'd0, 32'hFFFFFFFF, 4'b0111, 32'd0, 1'b1, "SLT: 0 < -1 = 0");
        test_case(32'hFFFFFFFE, 32'hFFFFFFFF, 4'b0111, 32'd1, 1'b0, "SLT: -2 < -1 = 1");
        
        // Edge cases
        test_case(32'h80000000, 32'h7FFFFFFF, 4'b0111, 32'd1, 1'b0, "SLT: MIN_INT < MAX_INT = 1");
        test_case(32'h7FFFFFFF, 32'h80000000, 4'b0111, 32'd0, 1'b1, "SLT: MAX_INT < MIN_INT = 0");
        
        // Zero comparisons
        test_case(32'd0, 32'd0, 4'b0111, 32'd0, 1'b1, "SLT: 0 < 0 = 0");
        test_case(32'd1, 32'd0, 4'b0111, 32'd0, 1'b1, "SLT: 1 < 0 = 0");
        test_case(32'd0, 32'd1, 4'b0111, 32'd1, 1'b0, "SLT: 0 < 1 = 1");
        
        $display("");
    endtask
    
    /**
     * Test Suite: SLTU (Set Less Than Unsigned) Operations
     */
    task test_sltu;
        $display("========================================");
        $display("Testing SLTU Operation (alu_control = 4'b1101)");
        $display("========================================");
        
        // Basic comparisons
        test_case(32'd10, 32'd20, 4'b1101, 32'd1, 1'b0, "SLTU: 10 < 20 = 1");
        test_case(32'd20, 32'd10, 4'b1101, 32'd0, 1'b1, "SLTU: 20 < 10 = 0");
        test_case(32'd10, 32'd10, 4'b1101, 32'd0, 1'b1, "SLTU: 10 < 10 = 0");
        
        // Unsigned comparison (treats as unsigned, so 0xFFFFFFFF > 0)
        test_case(32'hFFFFFFFF, 32'd0, 4'b1101, 32'd0, 1'b1, "SLTU: 0xFFFF < 0 = 0 (unsigned)");
        test_case(32'd0, 32'hFFFFFFFF, 4'b1101, 32'd1, 1'b0, "SLTU: 0 < 0xFFFF = 1 (unsigned)");
        test_case(32'hFFFFFFFE, 32'hFFFFFFFF, 4'b1101, 32'd1, 1'b0, "SLTU: 0xFFFE < 0xFFFF = 1");
        
        // Edge cases
        test_case(32'h80000000, 32'h7FFFFFFF, 4'b1101, 32'd0, 1'b1, "SLTU: 0x80000000 < 0x7FFFFFFF = 0 (unsigned)");
        test_case(32'h7FFFFFFF, 32'h80000000, 4'b1101, 32'd1, 1'b0, "SLTU: 0x7FFFFFFF < 0x80000000 = 1 (unsigned)");
        
        // Zero comparisons
        test_case(32'd0, 32'd0, 4'b1101, 32'd0, 1'b1, "SLTU: 0 < 0 = 0");
        test_case(32'd1, 32'd0, 4'b1101, 32'd0, 1'b1, "SLTU: 1 < 0 = 0");
        test_case(32'd0, 32'd1, 4'b1101, 32'd1, 1'b0, "SLTU: 0 < 1 = 1");
        
        $display("");
    endtask
    
    /**
     * Test Suite: Invalid Control Signals
     */
    task test_invalid_control;
        $display("========================================");
        $display("Testing Invalid Control Signals");
        $display("========================================");
        
        // Test default case (should default to ADD)
        test_case(32'd10, 32'd20, 4'b1111, 32'd30, 1'b0, "Invalid control (defaults to ADD)");
        test_case(32'd5, 32'd3, 4'b1011, 32'd8, 1'b0, "Invalid control (defaults to ADD)");
        
        $display("");
    endtask
    
    /**
     * Main test procedure
     */
    initial begin
        // Dump VCD file for waveform viewing (Icarus Verilog)
        $dumpfile("alu_tb.vcd");
        $dumpvars(0, alu_tb);
        
        $display("========================================");
        $display("RISC-V ALU Comprehensive Testbench");
        $display("========================================");
        $display("Starting tests...\n");
        $display("VCD waveform file: alu_tb.vcd\n");
        
        // Initialize signals
        operand_a = 32'd0;
        operand_b = 32'd0;
        alu_control = 4'b0000;
        
        // Run all test suites
        test_add();
        test_sub();
        test_and();
        test_or();
        test_xor();
        test_sll();
        test_srl();
        test_sra();
        test_slt();
        test_sltu();
        test_invalid_control();
        
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
     * Note: These are checked during simulation
     */
    always @(*) begin
        // Assertion: Result should be 32 bits (always true for 32-bit signal)
        // This is implicit in the signal declaration
        
        // Assertion: Zero flag should be correct
        if (zero_flag !== (result == 0)) begin
            $error("Zero flag mismatch! Result: 0x%08h, Zero flag: %0b, Expected: %0b", 
                   result, zero_flag, (result == 0));
        end
    end

endmodule

