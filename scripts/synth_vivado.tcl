################################################################################
# Xilinx Vivado Synthesis Script for RISC-V 5-Stage Pipeline Processor
#
# This script automates the complete synthesis flow for Vivado.
#
# Usage:
#   vivado -mode batch -source scripts/synth_vivado.tcl
#   OR
#   vivado -mode gui -source scripts/synth_vivado.tcl
#
# Author: RISC-V Processor Project
# Date: 2024
################################################################################

################################################################################
# Configuration Variables
################################################################################

# Project settings
set project_name "riscv_pipeline"
set project_dir "./vivado_project"

# FPGA part (modify for your target FPGA)
# Examples:
#   Artix-7: xc7a35tcpg236-1
#   Kintex-7: xc7k70tfbg676-1
#   Zynq-7000: xc7z020clg400-1
set fpga_part "xc7a35tcpg236-1"

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
puts "Creating Vivado Project: $project_name"
puts "=========================================="

# Create project directory if it doesn't exist
file mkdir $project_dir

# Create project
create_project $project_name $project_dir -part $fpga_part -force

# Set project properties
set_property target_language SystemVerilog [current_project]
set_property default_lib work [current_project]

################################################################################
# Add Source Files
################################################################################

puts "=========================================="
puts "Adding Source Files"
puts "=========================================="

# Add all SystemVerilog source files
add_files -fileset sources_1 "$src_dir/riscv_pipeline.sv"
add_files -fileset sources_1 "$src_dir/if_stage.sv"
add_files -fileset sources_1 "$src_dir/id_stage.sv"
add_files -fileset sources_1 "$src_dir/ex_stage.sv"
add_files -fileset sources_1 "$src_dir/mem_stage.sv"
add_files -fileset sources_1 "$src_dir/wb_stage.sv"
add_files -fileset sources_1 "$src_dir/alu.sv"
add_files -fileset sources_1 "$src_dir/control_unit.sv"
add_files -fileset sources_1 "$src_dir/reg_file.sv"
add_files -fileset sources_1 "$src_dir/forwarding_unit.sv"
add_files -fileset sources_1 "$src_dir/hazard_detection_unit.sv"
add_files -fileset sources_1 "$src_dir/branch_jump_control.sv"
add_files -fileset sources_1 "$src_dir/imm_gen.sv"
add_files -fileset sources_1 "$src_dir/imem.sv"
add_files -fileset sources_1 "$src_dir/dmem.sv"
add_files -fileset sources_1 "$src_dir/if_id_reg.sv"
add_files -fileset sources_1 "$src_dir/id_ex_reg.sv"
add_files -fileset sources_1 "$src_dir/ex_mem_reg.sv"
add_files -fileset sources_1 "$src_dir/mem_wb_reg.sv"

# Set file type to SystemVerilog
set_property file_type SystemVerilog [get_files *.sv]

# Set top module
set_property top riscv_pipeline [current_fileset]

################################################################################
# Add Constraint Files
################################################################################

puts "=========================================="
puts "Adding Constraint Files"
puts "=========================================="

# Add constraint file
add_files -fileset constrs_1 "$constraints_dir/riscv_pipeline.xdc"

# Update constraint file to set correct clock period
# Note: This modifies the constraint file - you may want to do this manually
# set_property CLOCK_PERIOD $clock_period [get_files riscv_pipeline.xdc]

################################################################################
# Configure Synthesis Settings
################################################################################

puts "=========================================="
puts "Configuring Synthesis Settings"
puts "=========================================="

# Set synthesis strategy
set_property strategy Flow_PerfOptimized_High [get_runs synth_1]

# Enable retiming (can help with timing closure)
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]

# Set memory initialization file paths (if needed)
# set_property MEMORY_INIT_FILE "../mem/inst_mem.hex" [get_files imem.sv]

# Update compile order
update_compile_order -fileset sources_1

################################################################################
# Run Synthesis
################################################################################

puts "=========================================="
puts "Running Synthesis"
puts "=========================================="

# Launch synthesis
launch_runs synth_1

# Wait for synthesis to complete
wait_on_run synth_1

# Check if synthesis succeeded
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

puts "Synthesis completed successfully!"

################################################################################
# Generate Reports
################################################################################

puts "=========================================="
puts "Generating Reports"
puts "=========================================="

# Create reports directory
file mkdir "$project_dir/reports"

# Open synthesized design
open_run synth_1

# Resource utilization report
report_utilization -file "$project_dir/reports/utilization_synth.rpt" -hierarchical
puts "Resource utilization report: $project_dir/reports/utilization_synth.rpt"

# Timing summary report
report_timing_summary -file "$project_dir/reports/timing_synth.rpt" -delay_type min_max -report_unconstrained -check_timing_verbose -max_paths 10
puts "Timing summary report: $project_dir/reports/timing_synth.rpt"

# Detailed timing report for critical paths
report_timing -file "$project_dir/reports/timing_detailed_synth.rpt" -delay_type min_max -max_paths 20 -nworst 5
puts "Detailed timing report: $project_dir/reports/timing_detailed_synth.rpt"

# Power estimation (if available)
if {[file exists "$project_dir/reports/power_synth.rpt"]} {
    report_power -file "$project_dir/reports/power_synth.rpt"
    puts "Power report: $project_dir/reports/power_synth.rpt"
}

# Check timing
set timing_check [report_timing_summary -return_string]
if {[string match "*VIOLATED*" $timing_check]} {
    puts "WARNING: Timing violations detected! Check timing reports."
} else {
    puts "Timing check passed!"
}

################################################################################
# Optional: Run Implementation
################################################################################

# Uncomment the following section if you want to run implementation automatically

# puts "=========================================="
# puts "Running Implementation"
# puts "=========================================="
# 
# # Launch implementation
# launch_runs impl_1
# wait_on_run impl_1
# 
# # Check if implementation succeeded
# if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
#     puts "ERROR: Implementation failed!"
#     exit 1
# }
# 
# puts "Implementation completed successfully!"
# 
# # Open implemented design
# open_run impl_1
# 
# # Generate implementation reports
# report_utilization -file "$project_dir/reports/utilization_impl.rpt" -hierarchical
# report_timing_summary -file "$project_dir/reports/timing_impl.rpt" -delay_type min_max -report_unconstrained -check_timing_verbose -max_paths 10
# report_timing -file "$project_dir/reports/timing_detailed_impl.rpt" -delay_type min_max -max_paths 20 -nworst 5
# 
# # Generate bitstream (optional)
# # launch_runs impl_1 -to_step write_bitstream
# # wait_on_run impl_1

################################################################################
# Summary
################################################################################

puts "=========================================="
puts "Synthesis Complete!"
puts "=========================================="
puts "Project: $project_name"
puts "FPGA Part: $fpga_part"
puts "Target Clock: ${clock_frequency} MHz (${clock_period} ns period)"
puts ""
puts "Reports generated in: $project_dir/reports/"
puts "  - Utilization: utilization_synth.rpt"
puts "  - Timing Summary: timing_synth.rpt"
puts "  - Detailed Timing: timing_detailed_synth.rpt"
puts ""
puts "To view results in GUI:"
puts "  vivado $project_dir/$project_name.xpr"
puts "=========================================="

################################################################################
# End of Script
################################################################################

