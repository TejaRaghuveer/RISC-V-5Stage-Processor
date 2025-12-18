################################################################################
# Xilinx Vivado Constraint File for RISC-V 5-Stage Pipeline Processor
# 
# This file contains timing and physical constraints for FPGA synthesis
# and implementation.
#
# Usage:
#   1. Add this file to your Vivado project constraints set
#   2. Modify clock frequency and I/O pin assignments for your FPGA board
#   3. Run synthesis and implementation
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
create_clock -period 10.000 -name clk [get_ports clk]

# Clock Uncertainty (jitter and skew)
set_clock_uncertainty -setup 0.5 [get_clocks clk]
set_clock_uncertainty -hold 0.2 [get_clocks clk]

# Clock Groups (if multiple clocks exist)
# set_clock_groups -asynchronous -group [get_clocks clk]

################################################################################
# Reset Constraints
################################################################################

# Reset is asynchronous and should be treated as false path for timing
# However, we still need to ensure proper reset behavior

# Async reset recovery and removal time
set_property ASYNC_REG TRUE [get_cells -hierarchical -filter {NAME =~ "*rst*"}]

# False path from reset (reset doesn't need to meet timing)
set_false_path -from [get_ports rst_n] -to [all_registers]

# Reset delay (ensure reset is stable)
set_input_delay -clock clk -max 1.0 [get_ports rst_n]
set_input_delay -clock clk -min 0.5 [get_ports rst_n]

################################################################################
# Input/Output Constraints
################################################################################

# Pipeline Control Signals (Optional External Control)
# These are optional and can be tied to 0 if not used externally

# Pipeline Stall Input
# If not used externally, tie to 0 in top-level wrapper
set_input_delay -clock clk -max 2.0 [get_ports pipeline_stall]
set_input_delay -clock clk -min 0.5 [get_ports pipeline_stall]

# Pipeline Flush Input
# If not used externally, tie to 0 in top-level wrapper
set_input_delay -clock clk -max 2.0 [get_ports pipeline_flush]
set_input_delay -clock clk -min 0.5 [get_ports pipeline_flush]

# Note: If these signals are tied to 0 internally, you can comment out
# the above constraints or add them to false paths:
# set_false_path -from [get_ports pipeline_stall]
# set_false_path -from [get_ports pipeline_flush]

################################################################################
# Timing Exceptions
################################################################################

# Multi-Cycle Paths (if any paths can take multiple cycles)
# Example: Memory access might take multiple cycles
# set_multicycle_path -setup 2 -from [get_cells -hierarchical -filter {NAME =~ "*mem*"}]
# set_multicycle_path -hold 1 -from [get_cells -hierarchical -filter {NAME =~ "*mem*"}]

# False Paths (paths that don't need timing analysis)
# Reset paths already handled above

# Maximum Delay Constraints (if needed)
# set_max_delay -from [get_cells -hierarchical -filter {NAME =~ "*alu*"}] \
#               -to [get_cells -hierarchical -filter {NAME =~ "*ex_mem*"}] 5.0

# Minimum Delay Constraints (for hold time)
# Usually handled automatically by tools, but can be specified:
# set_min_delay -from [get_cells -hierarchical -filter {NAME =~ "*reg_file*"}] \
#               -to [get_cells -hierarchical -filter {NAME =~ "*id_ex*"}] 0.5

################################################################################
# Physical Constraints (Pin Assignments)
################################################################################
# IMPORTANT: Modify these pin assignments based on your FPGA board!
# 
# Example for common FPGA boards:
# - Artix-7 (Basys 3): Clock on W5, Reset on T18
# - Zynq-7000 (Zybo): Clock on Y9, Reset on K17
# - Kintex-7: Check your board schematic
#
# To find pin locations:
#   1. Check your FPGA board schematic
#   2. Use Vivado's Pin Planner tool
#   3. Or use board-specific constraint files if available

# Clock Pin Assignment (Example - MODIFY FOR YOUR BOARD)
# set_property PACKAGE_PIN W5 [get_ports clk]
# set_property IOSTANDARD LVCMOS33 [get_ports clk]

# Reset Pin Assignment (Example - MODIFY FOR YOUR BOARD)
# set_property PACKAGE_PIN T18 [get_ports rst_n]
# set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

# Pipeline Control Pins (if used externally - Example)
# set_property PACKAGE_PIN U16 [get_ports pipeline_stall]
# set_property IOSTANDARD LVCMOS33 [get_ports pipeline_stall]
# set_property PACKAGE_PIN V16 [get_ports pipeline_flush]
# set_property IOSTANDARD LVCMOS33 [get_ports pipeline_flush]

################################################################################
# I/O Standard Constraints
################################################################################

# Set I/O standards for all ports (modify based on your board voltage)
# Common standards:
#   - LVCMOS33: 3.3V (most common)
#   - LVCMOS25: 2.5V
#   - LVCMOS18: 1.8V
#   - LVTTL: 3.3V (older standard)

# set_property IOSTANDARD LVCMOS33 [get_ports clk]
# set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
# set_property IOSTANDARD LVCMOS33 [get_ports pipeline_stall]
# set_property IOSTANDARD LVCMOS33 [get_ports pipeline_flush]

################################################################################
# Drive Strength and Slew Rate
################################################################################

# Clock drive strength (usually default is fine)
# set_property DRIVE 12 [get_ports clk]

# Slew rate (SLOW = lower power, FAST = better timing)
# set_property SLEW SLOW [get_ports clk]

################################################################################
# Memory Constraints
################################################################################

# Memory initialization files are specified in the SystemVerilog source
# using parameters. Ensure the paths are correct relative to your project.

# If using external memory controllers, add constraints here:
# set_property RAM_STYLE BLOCK [get_cells -hierarchical -filter {NAME =~ "*imem*"}]
# set_property RAM_STYLE BLOCK [get_cells -hierarchical -filter {NAME =~ "*dmem*"}]

# Force memories to use BRAM (recommended)
set_property RAM_STYLE BLOCK [get_cells -hierarchical -filter {NAME =~ "*instruction_memory*"}]
set_property RAM_STYLE BLOCK [get_cells -hierarchical -filter {NAME =~ "*data_memory*"}]

################################################################################
# Synthesis Optimization Directives
################################################################################

# These can also be set in synthesis settings, but can be specified here:

# Keep hierarchy (useful for debugging)
# set_property KEEP_HIERARCHY TRUE [get_cells riscv_pipeline]

# Don't touch certain cells (preserve structure)
# set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter {NAME =~ "*pipeline*"}]

# Maximum fanout (for better timing)
# set_max_fanout 50 [get_nets -hierarchical]

################################################################################
# Implementation Constraints
################################################################################

# These constraints are applied during place-and-route:

# Clock region constraints (if needed for clock routing)
# set_property CLOCK_REGION X0Y0:X1Y1 [get_cells -hierarchical -filter {NAME =~ "*if_stage*"}]

# Placement constraints (usually auto-placed, but can be constrained)
# set_property LOC SLICE_X0Y0 [get_cells -hierarchical -filter {NAME =~ "*alu*"}]

################################################################################
# Power Optimization Constraints
################################################################################

# Clock gating (if enabled)
# set_property CLOCK_GATING_CHECK TRUE [get_clocks clk]

# Power optimization
# set_property POWER_OPT true [get_runs impl_1]

################################################################################
# Debug Constraints (for ChipScope/ILA)
################################################################################

# If using Integrated Logic Analyzer (ILA) for debugging:
# set_property MARK_DEBUG true [get_nets -hierarchical -filter {NAME =~ "*debug_signal*"}]

################################################################################
# Notes and Best Practices
################################################################################

# 1. Always verify clock frequency matches your design requirements
# 2. Check timing reports after synthesis and implementation
# 3. Adjust constraints based on your FPGA board's pin assignments
# 4. Use false paths sparingly - only for truly asynchronous signals
# 5. Review resource utilization to ensure design fits in target FPGA
# 6. Test design at multiple clock frequencies to find maximum operating frequency

################################################################################
# End of Constraint File
################################################################################

