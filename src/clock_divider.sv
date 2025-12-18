/**
 * @file clock_divider.sv
 * @brief Clock Divider Module for FPGA Demonstration
 * 
 * Divides the input clock frequency by a configurable divisor.
 * Useful for slowing down the processor clock for observation on LEDs.
 * 
 * @details
 * The divider creates a slower clock by counting input clock cycles
 * and toggling the output clock every DIVISOR/2 cycles.
 * 
 * Example:
 * - Input: 100 MHz (10 ns period)
 * - Divisor: 1000000
 * - Output: 100 Hz (10 ms period) - visible on LEDs
 */

module clock_divider #(
    parameter DIVISOR = 1000000  // Clock division factor (default: 1 MHz -> 1 Hz)
) (
    input  logic clk_in,    // Input clock (fast, e.g., 100 MHz)
    input  logic rst_n,     // Active-low reset
    output logic clk_out    // Output clock (slow, divided)
);

    // Counter for clock division
    localparam COUNTER_WIDTH = $clog2(DIVISOR);
    logic [COUNTER_WIDTH-1:0] counter;
    
    /**
     * Clock Division Logic
     * 
     * Counts up to DIVISOR/2, then toggles output clock.
     * This creates a 50% duty cycle output clock.
     */
    always_ff @(posedge clk_in or negedge rst_n) begin
        if (!rst_n) begin
            counter <= {COUNTER_WIDTH{1'b0}};
            clk_out <= 1'b0;
        end else begin
            if (counter == (DIVISOR/2 - 1)) begin
                counter <= {COUNTER_WIDTH{1'b0}};
                clk_out <= ~clk_out;
            end else begin
                counter <= counter + 1;
            end
        end
    end

endmodule

