/**
 * @file debounce.sv
 * @brief Button Debounce Module for FPGA
 * 
 * Debounces mechanical button inputs to prevent multiple triggers
 * from a single button press. Uses a counter-based debounce algorithm.
 * 
 * @details
 * Mechanical buttons bounce when pressed/released, causing multiple
 * transitions. This module filters out bounces by requiring the signal
 * to be stable for DEBOUNCE_CYCLES clock cycles before the output changes.
 * 
 * Typical debounce time: 10-20 ms
 * For 100 MHz clock: 1,000,000 - 2,000,000 cycles
 */

module debounce #(
    parameter DEBOUNCE_CYCLES = 1000000,  // Number of cycles for debounce (default: 10 ms @ 100 MHz)
    parameter COUNTER_WIDTH = 20           // Counter width (should be >= log2(DEBOUNCE_CYCLES))
) (
    input  logic clk,        // Clock signal
    input  logic rst_n,      // Active-low reset
    input  logic button_in,  // Raw button input (may bounce)
    output logic button_out  // Debounced button output (stable)
);

    // Internal signals
    logic [COUNTER_WIDTH-1:0] counter;
    logic button_sync;
    logic button_prev;
    
    /**
     * Synchronizer: Double-flop for metastability prevention
     * 
     * First flop samples async button input
     * Second flop provides stable synchronized signal
     */
    logic button_sync1;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_sync1 <= 1'b0;
            button_sync <= 1'b0;
        end else begin
            button_sync1 <= button_in;
            button_sync <= button_sync1;
        end
    end
    
    /**
     * Debounce Counter Logic
     * 
     * When button state changes, start counting.
     * If button stays stable for DEBOUNCE_CYCLES, update output.
     * Otherwise, reset counter and keep output unchanged.
     */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= {COUNTER_WIDTH{1'b0}};
            button_out <= 1'b0;
            button_prev <= 1'b0;
        end else begin
            button_prev <= button_sync;
            
            if (button_sync != button_prev) begin
                // Button state changed, reset counter
                counter <= {COUNTER_WIDTH{1'b0}};
            end else begin
                // Button state stable, increment counter
                if (counter < DEBOUNCE_CYCLES) begin
                    counter <= counter + 1;
                end else begin
                    // Counter reached threshold, update output
                    button_out <= button_sync;
                end
            end
        end
    end

endmodule

