################################################################################
# Intel Quartus Prime SDC Constraint File for RISC-V 5-Stage Pipeline Processor
# 
# This file contains timing constraints for FPGA synthesis and timing analysis.
#
# Usage:
#   1. Add this file to your Quartus project (Assignments → Settings → Timing
#      Analysis Settings → TimeQuest Timing Analyzer → Additional SDC Files)
#   2. Modify clock frequency and I/O pin assignments for your FPGA board
#   3. Run synthesis and timing analysis
#
# Author: RISC-V Processor Project
# Date: 2024
################################################################################

################################################################################
# Clock Constraints
################################################################################

# Primary Clock: System Clock
# Modify period based on your target frequency:
#   - 50 MHz: period = 20.000 ns
#   - 100 MHz: period = 10.000 ns
#   - 200 MHz: period = 5.000 ns
create_clock -name clk -period 10.000 [get_ports {clk}]

# Clock Uncertainty (jitter and skew)
set_clock_uncertainty -setup 0.5 [get_clocks {clk}]
set_clock_uncertainty -hold 0.2 [get_clocks {clk}]

# Clock Groups (if multiple clocks exist)
# set_clock_groups -asynchronous -group [get_clocks {clk}]

################################################################################
# Reset Constraints
################################################################################

# Reset is asynchronous and should be treated as false path for timing
# However, we still need to ensure proper reset behavior

# Async reset recovery and removal time
# Quartus handles async reset automatically, but we can specify constraints

# False path from reset (reset doesn't need to meet timing)
set_false_path -from [get_ports {rst_n}] -to [all_registers]

# Reset delay (ensure reset is stable)
set_input_delay -clock clk -max 1.0 [get_ports {rst_n}]
set_input_delay -clock clk -min 0.5 [get_ports {rst_n}]

################################################################################
# Input/Output Constraints
################################################################################

# Pipeline Control Signals (Optional External Control)
# These are optional and can be tied to 0 if not used externally

# Pipeline Stall Input
# If not used externally, tie to 0 in top-level wrapper
set_input_delay -clock clk -max 2.0 [get_ports {pipeline_stall}]
set_input_delay -clock clk -min 0.5 [get_ports {pipeline_stall}]

# Pipeline Flush Input
# If not used externally, tie to 0 in top-level wrapper
set_input_delay -clock clk -max 2.0 [get_ports {pipeline_flush}]
set_input_delay -clock clk -min 0.5 [get_ports {pipeline_flush}]

# Note: If these signals are tied to 0 internally, you can comment out
# the above constraints or add them to false paths:
# set_false_path -from [get_ports {pipeline_stall}]
# set_false_path -from [get_ports {pipeline_flush}]

# Output delays (if there are any outputs - currently none in this design)
# set_output_delay -clock clk -max 2.0 [get_ports {some_output}]
# set_output_delay -clock clk -min 0.5 [get_ports {some_output}]

################################################################################
# Timing Exceptions
################################################################################

# Multi-Cycle Paths (if any paths can take multiple cycles)
# Example: Memory access might take multiple cycles
# set_multicycle_path -setup 2 -from [get_registers -hierarchical -filter {NAME =~ "*mem*"}]
# set_multicycle_path -hold 1 -from [get_registers -hierarchical -filter {NAME =~ "*mem*"}]

# False Paths (paths that don't need timing analysis)
# Reset paths already handled above

# Maximum Delay Constraints (if needed)
# set_max_delay -from [get_registers -hierarchical -filter {NAME =~ "*alu*"}] \
#               -to [get_registers -hierarchical -filter {NAME =~ "*ex_mem*"}] 5.0

# Minimum Delay Constraints (for hold time)
# Usually handled automatically by tools, but can be specified:
# set_min_delay -from [get_registers -hierarchical -filter {NAME =~ "*reg_file*"}] \
#               -to [get_registers -hierarchical -filter {NAME =~ "*id_ex*"}] 0.5

################################################################################
# Physical Constraints (Pin Assignments)
################################################################################
# IMPORTANT: Pin assignments in Quartus are typically done through:
#   - Assignment Editor (Assignments → Pin Planner)
#   - QSF file (Quartus Settings File)
#   - Or using set_location_assignment commands
#
# Example pin assignments (MODIFY FOR YOUR BOARD):
#   - DE10-Lite (Cyclone V): Clock on PIN_R8, Reset on PIN_N9
#   - DE2-115 (Cyclone IV): Clock on PIN_Y2, Reset on PIN_M23
#   - Check your FPGA board schematic for correct pin numbers

# Clock Pin Assignment (Example - MODIFY FOR YOUR BOARD)
# set_location_assignment PIN_R8 -to clk
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to clk

# Reset Pin Assignment (Example - MODIFY FOR YOUR BOARD)
# set_location_assignment PIN_N9 -to rst_n
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to rst_n

# Pipeline Control Pins (if used externally - Example)
# set_location_assignment PIN_M23 -to pipeline_stall
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pipeline_stall
# set_location_assignment PIN_M24 -to pipeline_flush
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pipeline_flush

# Note: Pin assignments are usually done in the QSF file or Pin Planner GUI,
# not in SDC file. SDC is primarily for timing constraints.

################################################################################
# I/O Standard Constraints
################################################################################

# I/O standards are typically set in QSF file or Pin Planner, but can be
# specified here using instance assignments:
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to clk
# set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to rst_n

# Common I/O standards:
#   - "3.3-V LVTTL": 3.3V (most common)
#   - "2.5-V": 2.5V
#   - "1.8-V": 1.8V
#   - "LVDS": Low-voltage differential signaling

################################################################################
# Memory Constraints
################################################################################

# Memory initialization files are specified in the SystemVerilog source
# using parameters. Ensure the paths are correct relative to your project.

# Force memories to use M9K/M10K blocks (recommended for Cyclone V/10)
# This is typically done through synthesis attributes in the HDL code:
#   (* ramstyle = "M9K" *) reg [31:0] memory_array [0:1023];
# Or through assignment:
# set_instance_assignment -name RAM_STYLE "M9K" -to instruction_memory
# set_instance_assignment -name RAM_STYLE "M9K" -to data_memory

################################################################################
# Synthesis Optimization Directives
################################################################################

# These can be set in Quartus Settings or through assignments:

# Keep hierarchy (useful for debugging)
# set_instance_assignment -name PRESERVE_FANOUT_FREE_NODE ON -to riscv_pipeline

# Don't touch certain cells (preserve structure)
# set_instance_assignment -name DONT_TOUCH ON -to riscv_pipeline

# Maximum fanout (for better timing)
# set_global_assignment -name MAX_FANOUT 50

################################################################################
# Timing Analysis Settings
################################################################################

# Enable timing-driven compilation
# set_global_assignment -name TIMING_ANALYZER_MULTICORNER_ANALYSIS ON

# Timing analysis mode
# set_global_assignment -name TIMEQUEST_MULTICORNER_ANALYSIS ON

################################################################################
# Power Optimization Constraints
################################################################################

# Clock gating (if enabled)
# set_global_assignment -name POWER_PRESET_COOLING_SOLUTION "23 MM HEAT SINK WITH 200 LFPM AIRFLOW"
# set_global_assignment -name POWER_BOARD_THERMAL_MODEL "NONE (CONSERVATIVE)"

################################################################################
# Notes and Best Practices
################################################################################

# 1. Always verify clock frequency matches your design requirements
# 2. Run TimeQuest Timing Analyzer after compilation to check timing
# 3. Adjust constraints based on your FPGA board's pin assignments
# 4. Use false paths sparingly - only for truly asynchronous signals
# 5. Review Fitter reports to ensure design fits in target FPGA
# 6. Test design at multiple clock frequencies to find maximum operating frequency
# 7. Pin assignments are typically done in Pin Planner or QSF file, not SDC

################################################################################
# End of Constraint File
################################################################################

