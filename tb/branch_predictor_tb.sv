/**
 * @file branch_predictor_tb.sv
 * @brief Testbench for 1-Bit Branch Predictor
 * 
 * This testbench verifies the branch predictor functionality:
 * - Prediction accuracy
 * - BHT update behavior
 * - Misprediction detection
 * - Integration scenarios
 */

`timescale 1ns / 1ps

module branch_predictor_tb;

    // Parameters
    parameter ADDR_WIDTH = 32;
    parameter BHT_SIZE = 256;
    parameter BHT_ADDR_WIDTH = 8;
    
    // Clock and Reset
    logic clk;
    logic rst_n;
    
    // Prediction Interface (IF Stage)
    logic [ADDR_WIDTH-1:0] if_PC;
    logic predict_taken;
    logic [ADDR_WIDTH-1:0] predict_target;
    
    // Update Interface (EX Stage)
    logic update_enable;
    logic [ADDR_WIDTH-1:0] ex_PC;
    logic actual_taken;
    logic [ADDR_WIDTH-1:0] actual_target;
    
    // Branch Instruction Information
    logic is_branch;
    logic [ADDR_WIDTH-1:0] branch_immediate;
    
    // Test Statistics
    int total_branches = 0;
    int correct_predictions = 0;
    int mispredictions = 0;
    
    // Instantiate DUT
    branch_predictor #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .BHT_SIZE(BHT_SIZE),
        .BHT_ADDR_WIDTH(BHT_ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .if_PC(if_PC),
        .predict_taken(predict_taken),
        .predict_target(predict_target),
        .update_enable(update_enable),
        .ex_PC(ex_PC),
        .actual_taken(actual_taken),
        .actual_target(actual_target),
        .is_branch(is_branch),
        .branch_immediate(branch_immediate)
    );
    
    // Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz clock
    end
    
    // Test Task: Simulate Branch Prediction
    task test_branch(
        input logic [ADDR_WIDTH-1:0] pc,
        input logic [ADDR_WIDTH-1:0] imm,
        input logic actual_outcome,
        input string test_name
    );
        begin
            // IF Stage: Make prediction
            if_PC = pc;
            is_branch = 1'b1;
            branch_immediate = imm;
            #10;  // Wait for prediction
            
            // Store prediction
            logic predicted_outcome = predict_taken;
            logic [ADDR_WIDTH-1:0] predicted_tgt = predict_target;
            
            $display("[%0t] Test: %s", $time, test_name);
            $display("  PC: 0x%08h", pc);
            $display("  Prediction: %s (target: 0x%08h)", 
                     predicted_outcome ? "TAKEN" : "NOT TAKEN", predicted_tgt);
            
            // EX Stage: Branch resolves
            #10;
            ex_PC = pc;
            actual_taken = actual_outcome;
            actual_target = pc + imm;
            update_enable = 1'b1;
            #10;
            update_enable = 1'b0;
            
            // Check result
            total_branches++;
            if (predicted_outcome == actual_outcome) begin
                correct_predictions++;
                $display("  Result: CORRECT");
            end else begin
                mispredictions++;
                $display("  Result: MISPREDICTION (predicted %s, actual %s)",
                         predicted_outcome ? "TAKEN" : "NOT TAKEN",
                         actual_outcome ? "TAKEN" : "NOT TAKEN");
            end
            $display("");
        end
    endtask
    
    // Main Test Sequence
    initial begin
        $display("========================================");
        $display("Branch Predictor Testbench");
        $display("========================================\n");
        
        // Reset
        rst_n = 0;
        if_PC = 0;
        is_branch = 0;
        branch_immediate = 0;
        update_enable = 0;
        ex_PC = 0;
        actual_taken = 0;
        actual_target = 0;
        #20;
        rst_n = 1;
        #10;
        
        $display("Test 1: Initial Prediction (Should predict NOT TAKEN)\n");
        // First prediction: BHT initialized to 0 (predict not taken)
        test_branch(32'h00001000, 32'h00000020, 1'b1, "First branch (taken)");
        
        $display("Test 2: Second Prediction (Should predict TAKEN after update)\n");
        // Second prediction: BHT updated to 1 (predict taken)
        test_branch(32'h00001000, 32'h00000020, 1'b1, "Same branch (taken)");
        
        $display("Test 3: Consistent Branch Pattern\n");
        // Test consistent pattern (always taken)
        for (int i = 0; i < 5; i++) begin
            test_branch(32'h00002000 + i*4, 32'h00000010, 1'b1, 
                       $sformatf("Consistent branch %0d", i));
        end
        
        $display("Test 4: Alternating Pattern (Will mispredict)\n");
        // Test alternating pattern (T-NT-T-NT)
        test_branch(32'h00003000, 32'h00000010, 1'b1, "Alternating 1 (taken)");
        test_branch(32'h00003000, 32'h00000010, 1'b0, "Alternating 2 (not taken)");
        test_branch(32'h00003000, 32'h00000010, 1'b1, "Alternating 3 (taken)");
        test_branch(32'h00003000, 32'h00000010, 1'b0, "Alternating 4 (not taken)");
        
        $display("Test 5: Different PC Addresses\n");
        // Test different PC addresses (different BHT entries)
        test_branch(32'h00004000, 32'h00000020, 1'b1, "Branch at 0x4000");
        test_branch(32'h00005000, 32'h00000020, 1'b0, "Branch at 0x5000");
        test_branch(32'h00006000, 32'h00000020, 1'b1, "Branch at 0x6000");
        
        $display("Test 6: BHT Aliasing\n");
        // Test aliasing: Different PCs mapping to same BHT entry
        // PC[9:2] determines index, so PCs with same lower bits alias
        logic [ADDR_WIDTH-1:0] pc1 = 32'h00007000;
        logic [ADDR_WIDTH-1:0] pc2 = 32'h00008000;
        // If BHT_ADDR_WIDTH = 8, then pc1[9:2] = pc2[9:2] if lower bits match
        // For this test, use PCs that alias
        test_branch(32'h00007000, 32'h00000010, 1'b1, "Aliased branch 1");
        test_branch(32'h00008000, 32'h00000010, 1'b0, "Aliased branch 2");
        test_branch(32'h00007000, 32'h00000010, 1'b1, "Aliased branch 1 again");
        
        // Print Statistics
        $display("========================================");
        $display("Test Statistics");
        $display("========================================");
        $display("Total Branches: %0d", total_branches);
        $display("Correct Predictions: %0d", correct_predictions);
        $display("Mispredictions: %0d", mispredictions);
        $display("Accuracy: %.2f%%", (correct_predictions * 100.0) / total_branches);
        $display("========================================\n");
        
        #100;
        $finish;
    end
    
endmodule

