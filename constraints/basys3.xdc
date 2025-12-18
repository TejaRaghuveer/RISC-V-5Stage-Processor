################################################################################
# Basys3 (Xilinx Artix-7) Constraint File for RISC-V Processor Demo
# 
# Board: Basys3 Artix-7 FPGA Board
# FPGA: XC7A35TCPG236-1
# 
# Pin assignments based on Basys3 board reference manual
################################################################################

################################################################################
# Clock Constraint
################################################################################

# System Clock: 100 MHz from on-board oscillator
# Pin: W5 (CLK100MHZ)
set_property PACKAGE_PIN W5 [get_ports {clk_board}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk_board}]
create_clock -period 10.000 -name clk_board [get_ports {clk_board}]

################################################################################
# Reset Button
################################################################################

# Reset Button: BTNC (Center button)
# Pin: U18 (BTNC)
set_property PACKAGE_PIN U18 [get_ports {btn_reset}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_reset}]

################################################################################
# Switches (Display Mode Selection)
################################################################################

# Switch 0: SW0
set_property PACKAGE_PIN V17 [get_ports {sw_display_mode[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw_display_mode[0]}]

# Switch 1: SW1
set_property PACKAGE_PIN V16 [get_ports {sw_display_mode[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw_display_mode[1]}]

# Switch 2: SW2
set_property PACKAGE_PIN W16 [get_ports {sw_display_mode[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw_display_mode[2]}]

################################################################################
# LEDs (16 LEDs for Display)
################################################################################

# LED 0-7: LD0-LD7
set_property PACKAGE_PIN U16 [get_ports {leds[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[0]}]

set_property PACKAGE_PIN E19 [get_ports {leds[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[1]}]

set_property PACKAGE_PIN U19 [get_ports {leds[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[2]}]

set_property PACKAGE_PIN V19 [get_ports {leds[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[3]}]

set_property PACKAGE_PIN W18 [get_ports {leds[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[4]}]

set_property PACKAGE_PIN U15 [get_ports {leds[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[5]}]

set_property PACKAGE_PIN U14 [get_ports {leds[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[6]}]

set_property PACKAGE_PIN V14 [get_ports {leds[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[7]}]

# LED 8-15: LD8-LD15
set_property PACKAGE_PIN V13 [get_ports {leds[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[8]}]

set_property PACKAGE_PIN V3 [get_ports {leds[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[9]}]

set_property PACKAGE_PIN W3 [get_ports {leds[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[10]}]

set_property PACKAGE_PIN U3 [get_ports {leds[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[11]}]

set_property PACKAGE_PIN P3 [get_ports {leds[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[12]}]

set_property PACKAGE_PIN N3 [get_ports {leds[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[13]}]

set_property PACKAGE_PIN P1 [get_ports {leds[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[14]}]

set_property PACKAGE_PIN L1 [get_ports {leds[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[15]}]

################################################################################
# Status LEDs (Optional)
################################################################################

# Running LED: LD15 (can reuse or use different LED)
# set_property PACKAGE_PIN L1 [get_ports {led_running}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led_running}]

# Stalled LED: LD14
# set_property PACKAGE_PIN P1 [get_ports {led_stalled}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led_stalled}]

# Flushed LED: LD13
# set_property PACKAGE_PIN N3 [get_ports {led_flushed}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led_flushed}]

################################################################################
# UART Interface (Optional)
################################################################################

# UART RX: Connected to USB-UART bridge (FTDI)
# Pin: B18 (USB-UART RX)
# set_property PACKAGE_PIN B18 [get_ports {uart_rx}]
# set_property IOSTANDARD LVCMOS33 [get_ports {uart_rx}]

# UART TX: Connected to USB-UART bridge (FTDI)
# Pin: A18 (USB-UART TX)
# set_property PACKAGE_PIN A18 [get_ports {uart_tx}]
# set_property IOSTANDARD LVCMOS33 [get_ports {uart_tx}]

################################################################################
# Timing Constraints
################################################################################

# Clock uncertainty
set_clock_uncertainty -setup 0.5 [get_clocks clk_board]
set_clock_uncertainty -hold 0.2 [get_clocks clk_board]

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
# End of Basys3 Constraints
################################################################################

