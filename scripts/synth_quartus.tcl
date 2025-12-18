################################################################################
# Intel Quartus Prime Synthesis Script for RISC-V 5-Stage Pipeline Processor
#
# This script automates the complete synthesis flow for Quartus Prime.
#
# Usage:
#   quartus_sh -t scripts/synth_quartus.tcl
#   OR
#   quartus --64bit -t scripts/synth_quartus.tcl
#
# Author: RISC-V Processor Project
# Date: 2024
################################################################################

################################################################################
# Configuration Variables
################################################################################

# Project settings
set project_name "riscv_pipeline"
set project_dir "./quartus_project"

# FPGA device (modify for your target FPGA)
# Examples:
#   Cyclone V: 5CEBA4F23C7
#   Cyclone 10 LP: 10CL025YU256C8G
#   Arria 10: 10AX115H3F34E2SG
set fpga_family "Cyclone V"
set fpga_device "5CEBA4F23C7"

# Clock frequency (MHz) - modify based on your requirements
set clock_frequency 100.0
set clock_period [expr 1000.0 / $clock_frequency]

# Source files directory
set src_dir "../src"
set constraints_dir "../constraints"

################################################################################
# Create Project
################################################################################

puts "=========================================="
puts "Creating Quartus Project: $project_name"
puts "=========================================="

# Create project directory if it doesn't exist
file mkdir $project_dir

# Create project
project_new $project_name -overwrite

# Set device
set_global_assignment -name FAMILY $fpga_family
set_global_assignment -name DEVICE $fpga_device

################################################################################
# Add Source Files
################################################################################

puts "=========================================="
puts "Adding Source Files"
puts "=========================================="

# Add all SystemVerilog source files
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/riscv_pipeline.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/if_stage.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/id_stage.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/ex_stage.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/mem_stage.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/wb_stage.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/alu.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/control_unit.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/reg_file.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/forwarding_unit.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/hazard_detection_unit.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/branch_jump_control.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/imm_gen.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/imem.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/dmem.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/if_id_reg.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/id_ex_reg.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/ex_mem_reg.sv"
set_global_assignment -name SYSTEMVERILOG_FILE "$src_dir/mem_wb_reg.sv"

# Set SystemVerilog language version
set_global_assignment -name VERILOG_INPUT_VERSION SYSTEMVERILOG_2005

# Set top-level entity
set_global_assignment -name TOP_LEVEL_ENTITY riscv_pipeline

################################################################################
# Add Constraint Files
################################################################################

puts "=========================================="
puts "Adding Constraint Files"
puts "=========================================="

# Add SDC constraint file
set_global_assignment -name SDC_FILE "$constraints_dir/riscv_pipeline.sdc"

################################################################################
# Configure Synthesis Settings
################################################################################

puts "=========================================="
puts "Configuring Synthesis Settings"
puts "=========================================="

# Set synthesis optimization mode
set_global_assignment -name OPTIMIZATION_MODE "HIGH PERFORMANCE EFFORT"

# Enable timing-driven compilation
set_global_assignment -name TIMING_ANALYZER_MULTICORNER_ANALYSIS ON

# Set maximum fanout (for better timing)
set_global_assignment -name MAX_FANOUT 50

# Enable register retiming (can help with timing closure)
set_global_assignment -name OPTIMIZATION_TECHNIQUE SPEED

# Set memory style (force M9K/M10K blocks)
# set_instance_assignment -name RAM_STYLE "M9K" -to instruction_memory
# set_instance_assignment -name RAM_STYLE "M9K" -to data_memory

################################################################################
# Commit Assignments and Close Project
################################################################################

# Commit assignments
project_close

################################################################################
# Run Synthesis (Analysis & Synthesis)
################################################################################

puts "=========================================="
puts "Running Synthesis (Analysis & Synthesis)"
puts "=========================================="

# Load flow package
load_package flow

# Execute synthesis
if {[catch {execute_module -tool map} result]} {
    puts "ERROR: Synthesis failed!"
    puts $result
    exit 1
}

puts "Synthesis completed successfully!"

################################################################################
# Run Fitter (Place & Route)
################################################################################

puts "=========================================="
puts "Running Fitter (Place & Route)"
puts "=========================================="

# Execute fitter
if {[catch {execute_module -tool fit} result]} {
    puts "ERROR: Fitter failed!"
    puts $result
    exit 1
}

puts "Fitter completed successfully!"

################################################################################
# Run Timing Analysis
################################################################################

puts "=========================================="
puts "Running Timing Analysis"
puts "=========================================="

# Load STA package
load_package sta

# Execute timing analysis
if {[catch {execute_module -tool sta} result]} {
    puts "WARNING: Timing analysis failed or warnings occurred!"
    puts $result
}

puts "Timing analysis completed!"

################################################################################
# Generate Reports
################################################################################

puts "=========================================="
puts "Generating Reports"
puts "=========================================="

# Create reports directory
file mkdir "$project_dir/reports"

# Load report package
load_package report

# Resource utilization report
execute_module -tool rpt -args "--partition"
file copy -force "$project_dir/output_files/$project_name.fit.summary" "$project_dir/reports/utilization.rpt"
puts "Resource utilization report: $project_dir/reports/utilization.rpt"

# Timing report
file copy -force "$project_dir/output_files/$project_name.sta.rpt" "$project_dir/reports/timing.rpt"
puts "Timing report: $project_dir/reports/timing.rpt"

# Check for timing violations
set timing_file [open "$project_dir/reports/timing.rpt" r]
set timing_content [read $timing_file]
close $timing_file

if {[string match "*VIOLATED*" $timing_content] || [string match "*violated*" $timing_content]} {
    puts "WARNING: Timing violations detected! Check timing reports."
} else {
    puts "Timing check passed!"
}

################################################################################
# Optional: Generate Programming File
################################################################################

# Uncomment the following section if you want to generate programming file

# puts "=========================================="
# puts "Generating Programming File"
# puts "=========================================="
# 
# # Load assembler package
# load_package flow
# 
# # Execute assembler
# if {[catch {execute_module -tool asm} result]} {
#     puts "ERROR: Assembler failed!"
#     puts $result
#     exit 1
# }
# 
# puts "Programming file (.sof) generated successfully!"

################################################################################
# Summary
################################################################################

puts "=========================================="
puts "Synthesis Complete!"
puts "=========================================="
puts "Project: $project_name"
puts "FPGA Family: $fpga_family"
puts "FPGA Device: $fpga_device"
puts "Target Clock: ${clock_frequency} MHz (${clock_period} ns period)"
puts ""
puts "Reports generated in: $project_dir/reports/"
puts "  - Utilization: utilization.rpt"
puts "  - Timing: timing.rpt"
puts ""
puts "To view results in GUI:"
puts "  quartus $project_dir/$project_name.qpf"
puts "=========================================="

################################################################################
# End of Script
################################################################################

