# FPGA Synthesis Guide for RISC-V 5-Stage Pipeline Processor

This guide provides comprehensive instructions for synthesizing the RISC-V processor for FPGA implementation using Xilinx Vivado or Intel Quartus Prime.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Design Considerations](#design-considerations)
4. [Xilinx Vivado Synthesis](#xilinx-vivado-synthesis)
5. [Intel Quartus Prime Synthesis](#intel-quartus-prime-synthesis)
6. [Timing Analysis](#timing-analysis)
7. [Resource Utilization](#resource-utilization)
8. [Optimization Strategies](#optimization-strategies)
9. [Troubleshooting](#troubleshooting)

---

## Overview

### Target Architecture

The RISC-V processor is designed as a 5-stage pipeline:
- **Instruction Fetch (IF)**: Fetches instructions from instruction memory
- **Instruction Decode (ID)**: Decodes instructions, reads register file
- **Execute (EX)**: Performs ALU operations, evaluates branches
- **Memory Access (MEM)**: Accesses data memory for load/store
- **Writeback (WB)**: Writes results back to register file

### Key Design Features

- **32-bit RISC-V RV32I instruction set**
- **Pipeline registers** between each stage
- **Data forwarding** for RAW hazard resolution
- **Hazard detection** for load-use stalls
- **Branch/jump control** with pipeline flushing

### Expected Performance Targets

- **Clock Frequency**: 50-100 MHz (depending on FPGA)
- **LUT Utilization**: ~2000-4000 LUTs
- **BRAM Utilization**: 2-4 BRAM blocks (for memories)
- **FF Utilization**: ~1500-3000 flip-flops
- **Critical Path**: Typically in ALU or forwarding logic

---

## Prerequisites

### Software Requirements

#### Xilinx Vivado
- **Version**: 2020.1 or later (recommended: 2021.1+)
- **License**: Free WebPACK license sufficient
- **Download**: [Xilinx Downloads](https://www.xilinx.com/support/download.html)

#### Intel Quartus Prime
- **Version**: 20.1 or later (recommended: 21.1+)
- **License**: Free Lite Edition sufficient
- **Download**: [Intel FPGA Downloads](https://www.intel.com/content/www/us/en/software/programmable/quartus-prime/download.html)

### Hardware Requirements

- **FPGA Board**: Any Xilinx 7-series or newer (e.g., Artix-7, Kintex-7, Zynq-7000)
- **FPGA Board**: Any Intel Cyclone V or newer (e.g., Cyclone V, Arria 10, Cyclone 10)
- **Memory**: Minimum 4GB RAM
- **Storage**: 10GB free space for tools

### Design Files

Ensure you have the following files:
```
src/
├── riscv_pipeline.sv          # Top-level module
├── if_stage.sv
├── id_stage.sv
├── ex_stage.sv
├── mem_stage.sv
├── wb_stage.sv
├── alu.sv
├── control_unit.sv
├── reg_file.sv
├── forwarding_unit.sv
├── hazard_detection_unit.sv
├── branch_jump_control.sv
├── imm_gen.sv
├── imem.sv
├── dmem.sv
├── if_id_reg.sv
├── id_ex_reg.sv
├── ex_mem_reg.sv
└── mem_wb_reg.sv
```

---

## Design Considerations

### Memory Implementation

The design uses two memories:
- **Instruction Memory (IMEM)**: 1024 words × 32 bits
- **Data Memory (DMEM)**: 1024 words × 32 bits

**FPGA Implementation Options**:
1. **Block RAM (BRAM)**: Recommended for memories
   - Single-port or dual-port BRAM
   - Synchronous read/write
   - Initialized from `.hex` files

2. **Distributed RAM**: Not recommended (too large)
   - Use BRAM instead

3. **External Memory**: For larger programs
   - Requires memory controller
   - Not covered in this guide

### Clock Domain

- **Single clock domain**: All stages use the same clock (`clk`)
- **Positive-edge triggered**: All registers use `posedge clk`
- **Reset**: Active-low asynchronous reset (`rst_n`)

### Pipeline Registers

All pipeline registers should be inferred as:
- **Synchronous registers** with enable
- **Reset to zero** (NOP insertion)
- **Flush capability** (clear on branch/jump)

### Critical Paths

Expected critical paths:
1. **ALU operations**: Especially shifts (SLL, SRL, SRA)
2. **Forwarding multiplexers**: EX stage forwarding logic
3. **Branch condition evaluation**: EX stage branch control
4. **Register file**: Dual-port read (usually not critical)

---

## Xilinx Vivado Synthesis

### Step 1: Create Project

#### Using Vivado GUI

1. **Launch Vivado**
   ```bash
   vivado
   ```

2. **Create Project**
   - File → New Project
   - Project Name: `riscv_pipeline`
   - Project Location: Choose your directory
   - Project Type: RTL Project
   - Add Sources: Add all `.sv` files from `src/`
   - Add Constraints: Add `constraints/riscv_pipeline.xdc`
   - Default Part: Select your FPGA (e.g., `xc7a35tcpg236-1` for Artix-7)

#### Using TCL Script

```tcl
# Create project
create_project riscv_pipeline ./vivado_project -part xc7a35tcpg236-1

# Add source files
add_files {src/riscv_pipeline.sv}
add_files {src/if_stage.sv}
add_files {src/id_stage.sv}
add_files {src/ex_stage.sv}
add_files {src/mem_stage.sv}
add_files {src/wb_stage.sv}
add_files {src/alu.sv}
add_files {src/control_unit.sv}
add_files {src/reg_file.sv}
add_files {src/forwarding_unit.sv}
add_files {src/hazard_detection_unit.sv}
add_files {src/branch_jump_control.sv}
add_files {src/imm_gen.sv}
add_files {src/imem.sv}
add_files {src/dmem.sv}
add_files {src/if_id_reg.sv}
add_files {src/id_ex_reg.sv}
add_files {src/ex_mem_reg.sv}
add_files {src/mem_wb_reg.sv}

# Set top module
set_property top riscv_pipeline [current_fileset]

# Add constraints
add_files -fileset constrs_1 constraints/riscv_pipeline.xdc

# Update compile order
update_compile_order -fileset sources_1
```

### Step 2: Configure Synthesis Settings

#### Set SystemVerilog Language Standard

```tcl
set_property file_type SystemVerilog [get_files *.sv]
```

#### Set Memory Initialization Files

```tcl
# Set memory initialization file paths
set_property MEMORY_INIT_FILE "mem/inst_mem.hex" [get_files imem.sv]
```

#### Synthesis Strategy

```tcl
# Set synthesis strategy
set_property strategy Flow_PerfOptimized_High [get_runs synth_1]
```

### Step 3: Run Synthesis

#### Using GUI
- Click **Run Synthesis** in Flow Navigator

#### Using TCL
```tcl
launch_runs synth_1
wait_on_run synth_1
```

### Step 4: Review Synthesis Results

#### Open Synthesized Design
```tcl
open_run synth_1
```

#### Check Resource Utilization
```tcl
report_utilization -file reports/utilization_synth.rpt
```

#### Check Timing
```tcl
report_timing_summary -file reports/timing_synth.rpt
```

#### View Schematic
```tcl
start_gui
# In GUI: Tools → Schematic → Open Synthesized Design
```

### Step 5: Run Implementation (Optional)

If you want to see post-implementation results:

```tcl
launch_runs impl_1
wait_on_run impl_1
open_run impl_1
report_timing_summary -file reports/timing_impl.rpt
report_utilization -file reports/utilization_impl.rpt
```

---

## Intel Quartus Prime Synthesis

### Step 1: Create Project

#### Using Quartus GUI

1. **Launch Quartus Prime**
   ```bash
   quartus
   ```

2. **Create Project**
   - File → New Project Wizard
   - Project Name: `riscv_pipeline`
   - Project Directory: Choose your directory
   - Top-level Entity: `riscv_pipeline`
   - Add Files: Add all `.sv` files from `src/`
   - Family: Select your FPGA family (e.g., Cyclone V)
   - Device: Select specific device (e.g., `5CEBA4F23C7`)

#### Using TCL Script

```tcl
# Create project
project_new riscv_pipeline -overwrite

# Set device
set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CEBA4F23C7

# Add source files
set_global_assignment -name SYSTEMVERILOG_FILE src/riscv_pipeline.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/if_stage.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/id_stage.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/ex_stage.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/mem_stage.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/wb_stage.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/alu.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/control_unit.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/reg_file.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/forwarding_unit.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/hazard_detection_unit.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/branch_jump_control.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/imm_gen.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/imem.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/dmem.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/if_id_reg.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/id_ex_reg.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/ex_mem_reg.sv
set_global_assignment -name SYSTEMVERILOG_FILE src/mem_wb_reg.sv

# Add constraints
set_global_assignment -name SDC_FILE constraints/riscv_pipeline.sdc

# Set top-level entity
set_global_assignment -name TOP_LEVEL_ENTITY riscv_pipeline

# Commit assignments
project_close
```

### Step 2: Configure Synthesis Settings

#### Set SystemVerilog Language Standard

```tcl
set_global_assignment -name VERILOG_INPUT_VERSION SYSTEMVERILOG_2005
```

#### Synthesis Optimization

```tcl
# Set synthesis optimization
set_global_assignment -name OPTIMIZATION_MODE "HIGH PERFORMANCE EFFORT"
```

### Step 3: Run Synthesis

#### Using GUI
- Click **Start Compilation** (or press Ctrl+L)

#### Using TCL
```tcl
load_package flow
execute_module -tool map
```

### Step 4: Review Synthesis Results

#### Check Resource Utilization
```tcl
load_package report
execute_module -tool rpt -args "--partition"
```

#### Check Timing
```tcl
load_package sta
execute_module -tool sta
```

#### View RTL Viewer
```tcl
load_package rtl
execute_module -tool rtl
```

---

## Timing Analysis

### Understanding Timing Reports

#### Setup Time Violations

**What it means**: Signal arrives too late at destination register

**Example from Vivado**:
```
Slack (VIOLATED) : -0.234 ns
  Source: ex_stage_inst/alu_inst/alu_result_reg[31]
  Destination: ex_mem_reg_inst/mem_alu_result_reg[31]
  Delay: 5.234 ns (logic: 4.500 ns, route: 0.734 ns)
  Required: 5.000 ns (at 200 MHz)
```

**Interpretation**:
- Signal needs 5.234 ns but only 5.000 ns available
- Violation: -0.234 ns (negative slack = violation)
- Critical path: ALU result → EX/MEM register

#### Hold Time Violations

**What it means**: Signal changes too early at destination register

**Example from Vivado**:
```
Slack (VIOLATED) : -0.123 ns
  Source: id_ex_reg_inst/ex_rs1_data_reg[15]
  Destination: ex_stage_inst/forwarding_mux_inst/mux_out[15]
  Delay: 0.123 ns (logic: 0.050 ns, route: 0.073 ns)
  Required: 0.200 ns (minimum hold time)
```

**Interpretation**:
- Signal changes 0.123 ns after clock edge
- Required: 0.200 ns minimum hold time
- Violation: -0.077 ns

#### Clock Skew

**What it means**: Clock arrives at different times at different registers

**Example**:
```
Clock Skew: 0.456 ns
  Source Clock: clk (rising edge)
  Destination Clock: clk (rising edge)
  Skew: 0.456 ns
```

**Interpretation**:
- Clock arrives 0.456 ns later at destination
- Reduces available time for data path
- Can be positive (helps hold time) or negative (hurts setup time)

### Critical Path Analysis

#### Identify Critical Paths

**Vivado**:
```tcl
report_timing -from [get_cells -hierarchical -filter {NAME =~ "*alu*"}] \
              -to [get_cells -hierarchical -filter {NAME =~ "*ex_mem*"}] \
              -file reports/critical_path_alu.rpt
```

**Quartus**:
```tcl
report_timing -from "*alu*" -to "*ex_mem*" -file critical_path_alu.rpt
```

#### Common Critical Paths

1. **ALU → EX/MEM Register**
   - Path: ALU computation → Pipeline register
   - Typical delay: 4-6 ns (at 200 MHz)
   - Optimization: Pipeline ALU operations

2. **Forwarding Multiplexers**
   - Path: EX/MEM or MEM/WB → EX stage ALU inputs
   - Typical delay: 2-3 ns
   - Optimization: Reduce multiplexer levels

3. **Branch Condition Evaluation**
   - Path: Register data → Branch control → PC update
   - Typical delay: 3-5 ns
   - Optimization: Early branch resolution

4. **Register File Read**
   - Path: Address → Register file → Data out
   - Typical delay: 1-2 ns (usually not critical)
   - Optimization: Use BRAM for register file (if large)

### Timing Constraints

#### Clock Period Constraint

**Vivado (.xdc)**:
```tcl
create_clock -period 10.000 -name clk [get_ports clk]
```

**Quartus (.sdc)**:
```tcl
create_clock -name clk -period 10.000 [get_ports {clk}]
```

#### Input/Output Delays

**Vivado**:
```tcl
set_input_delay -clock clk -max 2.0 [get_ports pipeline_stall]
set_output_delay -clock clk -max 2.0 [get_ports {some_output}]
```

**Quartus**:
```tcl
set_input_delay -clock clk -max 2.0 [get_ports {pipeline_stall}]
set_output_delay -clock clk -max 2.0 [get_ports {some_output}]
```

#### False Paths

**Vivado**:
```tcl
set_false_path -from [get_ports rst_n] -to [all_registers]
```

**Quartus**:
```tcl
set_false_path -from [get_ports {rst_n}] -to [all_registers]
```

---

## Resource Utilization

### Understanding Resource Reports

#### LUT (Look-Up Table) Utilization

**What it means**: Logic elements used for combinational logic

**Example**:
```
LUTs: 2,456 / 20,800 (11.8%)
  LUT Logic: 2,234
  LUT Memory: 222
```

**Interpretation**:
- 2,456 LUTs used out of 20,800 available
- 11.8% utilization (plenty of room)
- Most used for combinational logic (ALU, forwarding, control)

#### Flip-Flop (FF) Utilization

**What it means**: Storage elements (registers)

**Example**:
```
FFs: 1,892 / 41,600 (4.5%)
  FF Logic: 1,892
```

**Interpretation**:
- 1,892 flip-flops used
- 4.5% utilization
- Used for pipeline registers, state machines

#### BRAM (Block RAM) Utilization

**What it means**: Memory blocks for instruction/data memory

**Example**:
```
BRAM: 4 / 50 (8.0%)
  BRAM36K: 4
```

**Interpretation**:
- 4 BRAM blocks used (2 for IMEM, 2 for DMEM)
- 8% utilization
- Each BRAM36K can store 36K bits

#### DSP Utilization

**What it means**: Dedicated multiplier blocks (not used in this design)

**Example**:
```
DSPs: 0 / 90 (0%)
```

**Interpretation**:
- No DSP blocks used (ALU uses LUTs for multiplication)
- Could optimize by using DSP blocks for MUL instruction

### Resource Breakdown by Module

#### Typical Utilization (Artix-7)

| Module | LUTs | FFs | BRAM |
|--------|------|-----|------|
| ALU | 400-600 | 0 | 0 |
| Register File | 200-300 | 0 | 0 |
| Control Unit | 100-150 | 0 | 0 |
| Forwarding Unit | 50-100 | 0 | 0 |
| Hazard Detection | 30-50 | 0 | 0 |
| Pipeline Registers | 0 | 800-1200 | 0 |
| IMEM | 0 | 0 | 1-2 |
| DMEM | 0 | 0 | 1-2 |
| **Total** | **800-1200** | **800-1200** | **2-4** |

### Memory Resource Analysis

#### Instruction Memory (IMEM)

- **Size**: 1024 words × 32 bits = 32,768 bits
- **BRAM36K**: 1 block (36K bits per block)
- **Implementation**: Single-port BRAM, read-only

#### Data Memory (DMEM)

- **Size**: 1024 words × 32 bits = 32,768 bits
- **BRAM36K**: 1 block (36K bits per block)
- **Implementation**: Single-port BRAM, read/write

---

## Optimization Strategies

### Meeting Timing Constraints

#### 1. Pipeline ALU Operations

**Problem**: ALU operations take too long (critical path)

**Solution**: Split ALU into two pipeline stages

**Before**:
```systemverilog
// Single-cycle ALU
always_comb begin
    alu_result = alu_operation(rs1_data, rs2_data, alu_control);
end
```

**After**:
```systemverilog
// Two-stage ALU pipeline
// Stage 1: Simple operations (ADD, SUB, AND, OR, XOR)
// Stage 2: Complex operations (SLL, SRL, SRA, SLT)
```

**Trade-off**: Increases latency but improves frequency

#### 2. Reduce Forwarding Multiplexer Levels

**Problem**: Too many multiplexer levels in forwarding path

**Solution**: Use priority encoders or reduce forwarding cases

**Before**:
```systemverilog
// Three-level multiplexer
assign forwarded_rs1 = (ForwardA == 2'b10) ? mem_alu_result :
                      (ForwardA == 2'b01) ? wb_write_data :
                      rs1_data;
```

**After**:
```systemverilog
// Use case statement (may synthesize better)
always_comb begin
    case (ForwardA)
        2'b10: forwarded_rs1 = mem_alu_result;
        2'b01: forwarded_rs1 = wb_write_data;
        default: forwarded_rs1 = rs1_data;
    endcase
end
```

#### 3. Register Retiming

**Problem**: Long combinational paths between pipeline stages

**Solution**: Move registers to balance pipeline stages

**Vivado**:
```tcl
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
```

**Quartus**:
```tcl
set_global_assignment -name OPTIMIZATION_TECHNIQUE SPEED
```

#### 4. Use DSP Blocks for Multiplication

**Problem**: MUL instruction uses many LUTs and is slow

**Solution**: Use FPGA DSP blocks (if MUL instruction added)

**Vivado**:
```tcl
# Force MUL to use DSP
set_property USE_DSP48 YES [get_cells -hierarchical -filter {NAME =~ "*mul*"}]
```

**Quartus**:
```tcl
set_global_assignment -name DSP_BLOCK_BALANCING "AUTO"
```

### Reducing Resource Utilization

#### 1. Optimize Register File

**Problem**: Register file uses many LUTs

**Solution**: Use BRAM for register file (if supported)

**Note**: Not recommended for 32 registers (too small for BRAM)

#### 2. Share ALU Resources

**Problem**: Multiple ALU instances

**Solution**: Single ALU shared across pipeline (already done)

#### 3. Optimize Control Logic

**Problem**: Control unit uses many LUTs

**Solution**: Use case statements instead of nested if-else

**Before**:
```systemverilog
if (opcode == OP_LOAD) begin
    if (funct3 == 3'b000) begin
        // LB
    end else if (funct3 == 3'b001) begin
        // LH
    end
end
```

**After**:
```systemverilog
case ({opcode, funct3})
    {OP_LOAD, 3'b000}: // LB
    {OP_LOAD, 3'b001}: // LH
    default: // Default case
endcase
```

### Power Optimization

#### 1. Clock Gating

**Problem**: Unnecessary clock toggling

**Solution**: Gate clock when pipeline is stalled

**Note**: Use tool-specific clock gating (Vivado/Quartus can infer)

#### 2. Reduce Switching Activity

**Problem**: High power consumption

**Solution**: Use enable signals to prevent unnecessary updates

**Already implemented**: Pipeline registers use enable signals

---

## Troubleshooting

### Common Synthesis Errors

#### 1. "Cannot find module"

**Error**:
```
ERROR: [Synth 8-27] cannot find module 'riscv_pipeline'
```

**Solution**:
- Check file paths in project
- Ensure all source files are added
- Verify top-level module name matches

#### 2. "Unsupported SystemVerilog construct"

**Error**:
```
ERROR: [Synth 8-27] unsupported SystemVerilog construct
```

**Solution**:
- Check SystemVerilog version support
- Use `-sv` flag in synthesis settings
- Replace unsupported constructs (e.g., `always_comb` → `always @*`)

#### 3. "Memory initialization file not found"

**Error**:
```
WARNING: [Synth 8-3936] memory initialization file not found
```

**Solution**:
- Check file paths relative to project directory
- Use absolute paths if needed
- Ensure `.hex` files exist

### Common Timing Issues

#### 1. Setup Time Violations

**Symptoms**: Negative slack in timing report

**Solutions**:
- Reduce clock frequency
- Add pipeline stages
- Optimize critical paths (see Optimization Strategies)

#### 2. Hold Time Violations

**Symptoms**: Hold time violations in timing report

**Solutions**:
- Add delay buffers (usually auto-fixed by tools)
- Adjust clock skew
- Use `set_false_path` for asynchronous paths

#### 3. Clock Domain Crossing

**Symptoms**: Timing violations between different clock domains

**Solutions**:
- Use synchronizers for CDC signals
- Add `set_clock_groups` constraints
- This design uses single clock domain (not applicable)

### Common Resource Issues

#### 1. BRAM Exhaustion

**Symptoms**: "Insufficient BRAM resources"

**Solutions**:
- Reduce memory sizes
- Use external memory
- Share BRAM blocks between IMEM and DMEM (not recommended)

#### 2. LUT Exhaustion

**Symptoms**: "Insufficient LUT resources"

**Solutions**:
- Optimize combinational logic
- Use BRAM for large memories
- Reduce pipeline width (not recommended)

---

## Example Synthesis Scripts

### Vivado Complete Synthesis Flow

See `scripts/synth_vivado.tcl` for complete script.

### Quartus Complete Synthesis Flow

See `scripts/synth_quartus.tcl` for complete script.

---

## Next Steps

After successful synthesis:

1. **Implementation**: Run place-and-route (Vivado) or fitter (Quartus)
2. **Bitstream Generation**: Generate `.bit` (Vivado) or `.sof` (Quartus) file
3. **Hardware Testing**: Program FPGA and test processor
4. **Performance Analysis**: Measure actual clock frequency and power consumption

---

## References

- [Xilinx Vivado User Guide](https://www.xilinx.com/support/documentation/sw_manuals/xilinx2021_1/ug910-vivado-getting-started.pdf)
- [Intel Quartus Prime Handbook](https://www.intel.com/content/www/us/en/programmable/documentation/)
- [RISC-V Instruction Set Manual](https://riscv.org/technical/specifications/)
- [FPGA Timing Closure Guide](https://www.xilinx.com/support/documentation/white_papers/wp272.pdf)

---

## Appendix: Constraint File Examples

### Vivado Constraints (.xdc)

See `constraints/riscv_pipeline.xdc` for complete constraint file.

### Quartus Constraints (.sdc)

See `constraints/riscv_pipeline.sdc` for complete constraint file.

