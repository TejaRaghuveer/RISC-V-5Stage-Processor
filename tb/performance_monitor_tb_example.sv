/**
 * @file performance_monitor_tb_example.sv
 * @brief Example testbench showing performance monitor integration
 * 
 * This testbench demonstrates how to integrate and use the performance monitor
 * module with the RISC-V pipeline processor.
 */

module performance_monitor_tb_example;

    // ============================================
    // Testbench Parameters
    // ============================================
    
    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 32;
    parameter COUNTER_WIDTH = 32;
    parameter CPI_WIDTH = 16;
    
    // ============================================
    // Clock and Reset
    // ============================================
    
    logic clk;
    logic rst_n;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100 MHz clock (10 ns period)
    end
    
    // Reset generation
    initial begin
        rst_n = 0;
        #20;
        rst_n = 1;
    end
    
    // ============================================
    // Performance Monitor Signals
    // ============================================
    
    logic                        perf_enable;
    logic                        wb_RegWrite;
    logic [4:0]                  wb_rd_addr;
    logic                        pipeline_stall;
    logic                        pipeline_flush;
    logic                        mem_MemRead;
    logic                        mem_MemWrite;
    logic                        mem_Branch;
    
    logic [COUNTER_WIDTH-1:0]    perf_total_cycles;
    logic [COUNTER_WIDTH-1:0]    perf_instructions_completed;
    logic [COUNTER_WIDTH-1:0]    perf_pipeline_stalls;
    logic [COUNTER_WIDTH-1:0]    perf_pipeline_flushes;
    logic [COUNTER_WIDTH-1:0]    perf_load_instructions;
    logic [COUNTER_WIDTH-1:0]    perf_store_instructions;
    logic [COUNTER_WIDTH-1:0]    perf_branch_instructions;
    logic [COUNTER_WIDTH+CPI_WIDTH-1:0] perf_cpi_value;
    
    // ============================================
    // Performance Monitor Instantiation
    // ============================================
    
    performance_monitor #(
        .COUNTER_WIDTH(COUNTER_WIDTH),
        .CPI_WIDTH(CPI_WIDTH)
    ) perf_monitor_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(perf_enable),
        .wb_RegWrite(wb_RegWrite),
        .wb_rd_addr(wb_rd_addr),
        .pipeline_stall(pipeline_stall),
        .pipeline_flush(pipeline_flush),
        .mem_MemRead(mem_MemRead),
        .mem_MemWrite(mem_MemWrite),
        .mem_Branch(mem_Branch),
        .total_cycles(perf_total_cycles),
        .instructions_completed(perf_instructions_completed),
        .pipeline_stalls(perf_pipeline_stalls),
        .pipeline_flushes(perf_pipeline_flushes),
        .load_instructions(perf_load_instructions),
        .store_instructions(perf_store_instructions),
        .branch_instructions(perf_branch_instructions),
        .cpi_value(perf_cpi_value)
    );
    
    // ============================================
    // Test Stimulus
    // ============================================
    
    initial begin
        // Initialize signals
        perf_enable = 0;
        wb_RegWrite = 0;
        wb_rd_addr = 5'b00000;
        pipeline_stall = 0;
        pipeline_flush = 0;
        mem_MemRead = 0;
        mem_MemWrite = 0;
        mem_Branch = 0;
        
        // Wait for reset to complete
        @(posedge rst_n);
        #10;
        
        // Enable performance monitoring
        perf_enable = 1;
        
        $display("=== Performance Monitor Test ===");
        $display("Time: %0t - Starting performance monitoring", $time);
        
        // Simulate normal instruction execution (CPI = 1.0)
        repeat(10) begin
            @(posedge clk);
            wb_RegWrite = 1;
            wb_rd_addr = 5'b00001;  // Write to x1
            @(posedge clk);
            wb_RegWrite = 0;
        end
        
        // Simulate load instruction
        @(posedge clk);
        mem_MemRead = 1;
        @(posedge clk);
        mem_MemRead = 0;
        @(posedge clk);
        wb_RegWrite = 1;
        wb_rd_addr = 5'b00010;  // Write to x2
        @(posedge clk);
        wb_RegWrite = 0;
        
        // Simulate store instruction
        @(posedge clk);
        mem_MemWrite = 1;
        @(posedge clk);
        mem_MemWrite = 0;
        
        // Simulate branch instruction
        @(posedge clk);
        mem_Branch = 1;
        @(posedge clk);
        mem_Branch = 0;
        @(posedge clk);
        pipeline_flush = 1;  // Branch taken, flush pipeline
        @(posedge clk);
        pipeline_flush = 0;
        
        // Simulate pipeline stall (load-use hazard)
        @(posedge clk);
        pipeline_stall = 1;
        @(posedge clk);
        pipeline_stall = 0;
        
        // Continue normal execution
        repeat(5) begin
            @(posedge clk);
            wb_RegWrite = 1;
            wb_rd_addr = 5'b00011;  // Write to x3
            @(posedge clk);
            wb_RegWrite = 0;
        end
        
        // Wait a few cycles for counters to update
        repeat(5) @(posedge clk);
        
        // Display performance metrics
        $display("\n=== Performance Metrics ===");
        $display("Total Cycles: %d", perf_total_cycles);
        $display("Instructions Completed: %d", perf_instructions_completed);
        $display("Pipeline Stalls: %d", perf_pipeline_stalls);
        $display("Pipeline Flushes: %d", perf_pipeline_flushes);
        $display("Load Instructions: %d", perf_load_instructions);
        $display("Store Instructions: %d", perf_store_instructions);
        $display("Branch Instructions: %d", perf_branch_instructions);
        
        // Calculate and display CPI
        real cpi;
        if (perf_instructions_completed > 0) begin
            cpi = $itor(perf_cpi_value) / $itor(1 << CPI_WIDTH);
            $display("CPI: %.4f", cpi);
            
            // Calculate efficiency metrics
            real stall_rate, flush_rate, efficiency;
            if (perf_total_cycles > 0) begin
                stall_rate = $itor(perf_pipeline_stalls) / $itor(perf_total_cycles);
                flush_rate = $itor(perf_pipeline_flushes) / $itor(perf_total_cycles);
            end else begin
                stall_rate = 0.0;
                flush_rate = 0.0;
            end
            efficiency = 1.0 / cpi;
            
            $display("\n=== Efficiency Metrics ===");
            $display("Stall Rate: %.2f%%", stall_rate * 100.0);
            $display("Flush Rate: %.2f%%", flush_rate * 100.0);
            $display("Pipeline Efficiency: %.2f%%", efficiency * 100.0);
        end else begin
            $display("CPI: N/A (no instructions completed)");
        end
        
        $display("\n=== Test Complete ===");
        
        // Disable monitoring
        perf_enable = 0;
        
        #100;
        $finish;
    end
    
    // ============================================
    // Monitoring Task (Optional)
    // ============================================
    
    // Task to display performance metrics periodically
    task display_performance_metrics;
        real cpi, stall_rate, flush_rate, efficiency;
        
        if (perf_instructions_completed > 0) begin
            cpi = $itor(perf_cpi_value) / $itor(1 << CPI_WIDTH);
            if (perf_total_cycles > 0) begin
                stall_rate = $itor(perf_pipeline_stalls) / $itor(perf_total_cycles);
                flush_rate = $itor(perf_pipeline_flushes) / $itor(perf_total_cycles);
            end else begin
                stall_rate = 0.0;
                flush_rate = 0.0;
            end
            efficiency = 1.0 / cpi;
            
            $display("[%0t] Cycles: %d | Instructions: %d | CPI: %.2f | Efficiency: %.1f%% | Stalls: %d | Flushes: %d",
                     $time, perf_total_cycles, perf_instructions_completed, cpi, 
                     efficiency * 100.0, perf_pipeline_stalls, perf_pipeline_flushes);
        end else begin
            $display("[%0t] Cycles: %d | Instructions: 0 | CPI: N/A",
                     $time, perf_total_cycles);
        end
    endtask
    
    // Uncomment to enable periodic monitoring
    // initial begin
    //     wait(rst_n);
    //     forever begin
    //         #1000;  // Every 1000 time units
    //         display_performance_metrics();
    //     end
    // end

endmodule

