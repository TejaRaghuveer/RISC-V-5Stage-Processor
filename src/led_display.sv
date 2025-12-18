/**
 * @file led_display.sv
 * @brief LED Display Module for FPGA Demonstration
 * 
 * Displays processor state information on LEDs.
 * Can show PC value, register values, or processor status.
 * 
 * @details
 * This module multiplexes different display modes:
 * - Mode 0: Show PC[15:0] (lower 16 bits of program counter)
 * - Mode 1: Show PC[31:16] (upper 16 bits of program counter)
 * - Mode 2: Show register x1 value[15:0]
 * - Mode 3: Show register x2 value[15:0]
 * - Mode 4: Show processor status (running/stalled/flushed)
 * 
 * Display mode can be changed via switches or buttons.
 */

module led_display #(
    parameter DATA_WIDTH = 32,
    parameter LED_WIDTH = 16  // Number of LEDs available (typically 16)
) (
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Display mode selection (from switches)
    input  logic [2:0]              display_mode,
    
    // Processor state inputs
    input  logic [DATA_WIDTH-1:0]  pc_value,        // Program counter value
    input  logic [DATA_WIDTH-1:0]   reg_x1_value,    // Register x1 value
    input  logic [DATA_WIDTH-1:0]   reg_x2_value,   // Register x2 value
    input  logic                    processor_running, // Processor is running
    input  logic                    processor_stalled, // Processor is stalled
    input  logic                    processor_flushed, // Processor is flushed
    
    // LED outputs
    output logic [LED_WIDTH-1:0]    leds             // LED outputs
);

    /**
     * Display Mode Multiplexer
     * 
     * Selects which data to display based on display_mode input.
     */
    always_comb begin
        case (display_mode)
            3'b000: leds = pc_value[15:0];              // PC[15:0]
            3'b001: leds = pc_value[31:16];              // PC[31:16]
            3'b010: leds = reg_x1_value[15:0];          // x1[15:0]
            3'b011: leds = reg_x2_value[15:0];           // x2[15:0]
            3'b100: begin
                // Status display: bit pattern showing processor state
                leds[15:13] = 3'b000;
                leds[12] = processor_running;
                leds[11] = processor_stalled;
                leds[10] = processor_flushed;
                leds[9:0] = 10'b0;
            end
            default: leds = pc_value[15:0];              // Default: PC[15:0]
        endcase
    end

endmodule

