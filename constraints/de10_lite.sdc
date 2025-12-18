################################################################################
# DE10-Lite (Intel Cyclone V) Constraint File for RISC-V Processor Demo
# 
# Board: DE10-Lite FPGA Board
# FPGA: 5CEBA4F23C7
# 
# Pin assignments based on DE10-Lite board reference manual
################################################################################

################################################################################
# Clock Constraint
################################################################################

# System Clock: 50 MHz from on-board oscillator
# Pin: PIN_R8 (CLOCK_50)
set_location_assignment PIN_R8 -to clk_board
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to clk_board
create_clock -name clk_board -period 20.000 [get_ports {clk_board}]

################################################################################
# Reset Button
################################################################################

# Reset Button: KEY[0] (Leftmost button)
# Pin: PIN_N9 (KEY[0])
set_location_assignment PIN_N9 -to btn_reset
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to btn_reset

################################################################################
# Switches (Display Mode Selection)
################################################################################

# Switch 0: SW[0]
set_location_assignment PIN_U11 -to sw_display_mode[0]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to sw_display_mode[0]

# Switch 1: SW[1]
set_location_assignment PIN_V11 -to sw_display_mode[1]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to sw_display_mode[1]

# Switch 2: SW[2]
set_location_assignment PIN_M9 -to sw_display_mode[2]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to sw_display_mode[2]

################################################################################
# LEDs (10 LEDs available on DE10-Lite)
################################################################################

# LED 0-9: LEDR[0] through LEDR[9]
# Note: DE10-Lite has 10 LEDs, we'll use LEDR[0:9] for leds[9:0]
# and can use LEDR[9:0] reversed or use HEX displays for upper bits

set_location_assignment PIN_AA2 -to leds[0]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[0]

set_location_assignment PIN_AA1 -to leds[1]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[1]

set_location_assignment PIN_W2 -to leds[2]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[2]

set_location_assignment PIN_Y3 -to leds[3]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[3]

set_location_assignment PIN_N2 -to leds[4]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[4]

set_location_assignment PIN_N1 -to leds[5]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[5]

set_location_assignment PIN_U2 -to leds[6]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[6]

set_location_assignment PIN_U1 -to leds[7]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[7]

set_location_assignment PIN_L2 -to leds[8]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[8]

set_location_assignment PIN_L1 -to leds[9]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to leds[9]

# For leds[10:15], we can use HEX displays or leave unconnected
# Option 1: Leave unconnected (LEDs 10-15 won't work)
# Option 2: Use HEX displays (requires additional module)

# HEX Display Option (if using 7-segment displays):
# HEX0 can show leds[3:0]
# HEX1 can show leds[7:4]
# HEX2 can show leds[11:8]
# HEX3 can show leds[15:12]

################################################################################
# Status LEDs (Optional - can use remaining LEDs)
################################################################################

# Running LED: LEDR[9] (if not used for leds[9])
# set_location_assignment PIN_L1 -to led_running
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led_running

# Stalled LED: LEDR[8]
# set_location_assignment PIN_L2 -to led_stalled
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led_stalled

# Flushed LED: LEDR[7]
# set_location_assignment PIN_U1 -to led_flushed
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led_flushed

################################################################################
# UART Interface (Optional - via GPIO header)
################################################################################

# UART RX: GPIO_0[0] (if using GPIO header)
# set_location_assignment PIN_A7 -to uart_rx
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to uart_rx

# UART TX: GPIO_0[1] (if using GPIO header)
# set_location_assignment PIN_B8 -to uart_tx
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to uart_tx

################################################################################
# Timing Constraints
################################################################################

# Clock uncertainty
set_clock_uncertainty -setup 0.5 [get_clocks {clk_board}]
set_clock_uncertainty -hold 0.2 [get_clocks {clk_board}]

# Input delays for buttons/switches
set_input_delay -clock clk_board -max 2.0 [get_ports {btn_reset}]
set_input_delay -clock clk_board -min 0.5 [get_ports {btn_reset}]

set_input_delay -clock clk_board -max 2.0 [get_ports {sw_display_mode[*]}]
set_input_delay -clock clk_board -min 0.5 [get_ports {sw_display_mode[*]}]

# Output delays for LEDs
set_output_delay -clock clk_board -max 2.0 [get_ports {leds[*]}]
set_output_delay -clock clk_board -min 0.5 [get_ports {leds[*]}]

# False paths
set_false_path -from [get_ports {btn_reset}] -to [all_registers]

################################################################################
# End of DE10-Lite Constraints
################################################################################

