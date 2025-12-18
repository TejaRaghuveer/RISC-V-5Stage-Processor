/**
 * @file fpga_top.sv
 * @brief FPGA Top-Level Module for RISC-V Processor Demonstration
 * 
 * This module wraps the RISC-V processor for FPGA demonstration.
 * Features:
 * - Clock divider for slower operation (visible on LEDs)
 * - Reset button debouncing
 * - LED display showing PC or register values
 * - Optional UART interface for program loading
 * 
 * @details
 * This top-level module connects the RISC-V processor to FPGA I/O:
 * - Clock: Uses board clock (typically 100 MHz)
 * - Reset: Debounced button input
 * - LEDs: Display processor state (PC, registers, status)
 * - Switches: Select display mode
 * - UART: Optional interface for program loading
 * 
 * Board Support:
 * - Basys3 (Xilinx Artix-7)
 * - DE10-Lite (Intel Cyclone V)
 * - Other boards with similar I/O
 */

module fpga_top #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter IMEM_DEPTH = 1024,
    parameter IMEM_ADDR_WIDTH = 10,
    parameter DMEM_DEPTH = 1024,
    parameter DMEM_ADDR_WIDTH = 10,
    parameter IMEM_INIT_FILE = "mem/inst_mem.hex",
    parameter DMEM_INIT_FILE = "",
    
    // Clock divider settings
    parameter CLOCK_DIVISOR = 1000000,  // Divide 100 MHz to 100 Hz (visible on LEDs)
    
    // Debounce settings
    parameter DEBOUNCE_CYCLES = 1000000  // 10 ms debounce @ 100 MHz
) (
    // Clock and Reset (from FPGA board)
    input  logic                    clk_board,      // Board clock (e.g., 100 MHz)
    input  logic                    btn_reset,      // Reset button (active high, needs debouncing)
    
    // Switches (for display mode selection)
    input  logic [2:0]              sw_display_mode, // Display mode selection
    
    // LEDs (for displaying processor state)
    output logic [15:0]             leds,           // 16 LEDs for display
    
    // Optional: UART Interface (for program loading)
    input  logic                    uart_rx,        // UART receive (from PC)
    output logic                    uart_tx,         // UART transmit (to PC)
    
    // Optional: Additional status LEDs
    output logic                    led_running,     // Processor running indicator
    output logic                    led_stalled,     // Processor stalled indicator
    output logic                    led_flushed      // Processor flushed indicator
);

    // ============================================
    // Internal Signals
    // ============================================
    
    // Clock signals
    logic clk_processor;        // Processor clock (divided from board clock)
    logic clk_processor_slow;   // Slow processor clock (for LED observation)
    
    // Reset signals
    logic rst_n_debounced;      // Debounced reset (active low)
    logic rst_n;                // Final reset signal
    
    // Processor control signals
    logic pipeline_stall;
    logic pipeline_flush;
    
    // Processor state (for LED display)
    // Note: These would need to be exposed from the processor module
    // For now, we'll use placeholder signals that can be connected
    // if the processor is modified to expose these signals
    logic [DATA_WIDTH-1:0] pc_value;
    logic [DATA_WIDTH-1:0] reg_x1_value;
    logic [DATA_WIDTH-1:0] reg_x2_value;
    logic processor_running;
    logic processor_stalled;
    logic processor_flushed;
    
    // ============================================
    // Clock Divider
    // ============================================
    
    /**
     * Clock Divider for Processor
     * 
     * Divides board clock to create slower processor clock.
     * This allows observation of processor operation on LEDs.
     * 
     * For demonstration: Use slow clock (100 Hz)
     * For performance: Use fast clock (100 MHz) - bypass divider
     */
    clock_divider #(
        .DIVISOR(CLOCK_DIVISOR)
    ) clock_divider_inst (
        .clk_in(clk_board),
        .rst_n(1'b1),  // Clock divider doesn't need reset
        .clk_out(clk_processor_slow)
    );
    
    /**
     * Clock Selection
     * 
     * Choose between fast clock (performance) and slow clock (demonstration).
     * For now, use slow clock for LED observation.
     * Can be changed via switch or parameter.
     */
    assign clk_processor = clk_processor_slow;  // Use slow clock for demo
    // assign clk_processor = clk_board;  // Use fast clock for performance
    
    // ============================================
    // Reset Debouncing
    // ============================================
    
    /**
     * Reset Button Debouncer
     * 
     * Debounces the reset button to prevent multiple resets
     * from a single button press.
     */
    debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) reset_debounce_inst (
        .clk(clk_board),
        .rst_n(1'b1),  // Debouncer doesn't need reset
        .button_in(btn_reset),
        .button_out(rst_n_debounced)
    );
    
    /**
     * Reset Signal Generation
     * 
     * Convert debounced button (active high) to processor reset (active low).
     */
    assign rst_n = ~rst_n_debounced;
    
    // ============================================
    // RISC-V Processor Instance
    // ============================================
    
    /**
     * RISC-V 5-Stage Pipeline Processor
     * 
     * Main processor instance.
     * Pipeline control signals are tied to 0 (not used externally).
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
    ) processor_inst (
        .clk(clk_processor),
        .rst_n(rst_n),
        .pipeline_stall(pipeline_stall),  // Tied to 0 (not used)
        .pipeline_flush(pipeline_flush)   // Tied to 0 (not used)
    );
    
    // Pipeline control signals (not used in this demo)
    assign pipeline_stall = 1'b0;
    assign pipeline_flush = 1'b0;
    
    // ============================================
    // Processor State Monitoring
    // ============================================
    
    /**
     * NOTE: The current processor module doesn't expose PC or register values.
     * 
     * To display these values, you have two options:
     * 
     * Option 1: Modify processor to expose debug signals
     *   - Add output ports for PC and register values
     *   - Connect internal signals to outputs
     * 
     * Option 2: Use a debug interface module
     *   - Create a module that reads from instruction/data memory
     *   - Monitor memory addresses to infer processor state
     * 
     * For now, we'll use placeholder signals that show the processor is running.
     * These can be connected to actual processor signals if modified.
     */
    
    // Placeholder: Simple counter to show processor is running
    logic [DATA_WIDTH-1:0] demo_counter;
    always_ff @(posedge clk_processor or negedge rst_n) begin
        if (!rst_n) begin
            demo_counter <= {DATA_WIDTH{1'b0}};
        end else begin
            demo_counter <= demo_counter + 1;
        end
    end
    
    // Use counter as placeholder for PC value
    assign pc_value = demo_counter;
    assign reg_x1_value = demo_counter + 1;
    assign reg_x2_value = demo_counter + 2;
    
    // Processor status signals (always running when not reset)
    assign processor_running = rst_n;
    assign processor_stalled = 1'b0;  // Not used in this demo
    assign processor_flushed = 1'b0;   // Not used in this demo
    
    // ============================================
    // LED Display
    // ============================================
    
    /**
     * LED Display Module
     * 
     * Displays processor state on LEDs based on switch selection.
     */
    led_display #(
        .DATA_WIDTH(DATA_WIDTH),
        .LED_WIDTH(16)
    ) led_display_inst (
        .clk(clk_board),
        .rst_n(rst_n),
        .display_mode(sw_display_mode),
        .pc_value(pc_value),
        .reg_x1_value(reg_x1_value),
        .reg_x2_value(reg_x2_value),
        .processor_running(processor_running),
        .processor_stalled(processor_stalled),
        .processor_flushed(processor_flushed),
        .leds(leds)
    );
    
    // Status LEDs
    assign led_running = processor_running;
    assign led_stalled = processor_stalled;
    assign led_flushed = processor_flushed;
    
    // ============================================
    // Optional: UART Interface
    // ============================================
    
    /**
     * UART Interface (Optional)
     * 
     * Can be used for:
     * - Loading programs into instruction memory
     * - Reading processor state
     * - Debugging
     * 
     * For now, UART signals are left unconnected.
     * A UART module can be added here if needed.
     */
    
    // UART placeholder (tie to safe values)
    // assign uart_tx = 1'b1;  // Idle state (high)
    
    // TODO: Add UART module for program loading
    // uart_loader uart_loader_inst (
    //     .clk(clk_board),
    //     .rst_n(rst_n),
    //     .uart_rx(uart_rx),
    //     .uart_tx(uart_tx),
    //     .imem_write_en(imem_write_en),
    //     .imem_addr(imem_addr),
    //     .imem_data(imem_data)
    // );

endmodule

