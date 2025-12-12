# Simulation Guide - RISC-V 5-Stage Pipeline Processor

## Table of Contents

1. [Quick Start](#quick-start)
2. [Running Simulations](#running-simulations)
3. [Selecting Test Programs](#selecting-test-programs)
4. [Viewing Waveforms](#viewing-waveforms)
5. [Interpreting Results](#interpreting-results)
6. [Advanced Simulation](#advanced-simulation)
7. [Troubleshooting](#troubleshooting)

---

## Quick Start

### Prerequisites

Before running simulations, ensure you have:
1. ✅ Compiled the project (see `BUILD.md`)
2. ✅ Installed a simulator (Icarus Verilog or ModelSim)
3. ✅ Selected a test program from `mem/` directory

### 5-Minute Quick Test

**Step 1: Compile ALU testbench**
```bash
cd simulation
iverilog -g2012 -o alu_tb -I../src ../src/alu.sv ../tb/alu_tb.sv
```

**Step 2: Run simulation**
```bash
vvp alu_tb
```

**Step 3: View waveforms (if VCD generated)**
```bash
gtkwave alu_tb.vcd
```

**Expected Output:**
```
Simulation output showing ALU test results
VCD file: alu_tb.vcd
```

---

## Running Simulations

### Method 1: Using Provided Scripts (Easiest)

#### ALU Simulation Script

**Basic Usage:**
```bash
./scripts/simulate_alu.sh
```

**With Options:**
```bash
# Force specific simulator
./scripts/simulate_alu.sh iverilog
./scripts/simulate_alu.sh modelsim

# Skip waveform viewer
./scripts/simulate_alu.sh --no-waveform

# Clean before simulation
./scripts/simulate_alu.sh --clean
```

**Expected Output:**
```
========================================
RISC-V ALU Simulation Script
========================================

[INFO] Auto-detected: Icarus Verilog
[INFO] Source directory: /path/to/src
[INFO] Testbench directory: /path/to/tb
[INFO] Starting Icarus Verilog simulation...
[INFO] Compiling ALU module and testbench...
[SUCCESS] Compilation successful
[INFO] Running simulation and generating VCD waveform...
========================================
[SUCCESS] Simulation completed successfully
[INFO] Simulation output:
----------------------------------------
[Testbench output here]
----------------------------------------
[SUCCESS] VCD waveform file generated: simulation/alu_tb.vcd
```

---

### Method 2: Manual Compilation and Simulation

#### Using Icarus Verilog

**Step 1: Compile Pipeline Testbench**

```bash
cd simulation

iverilog -g2012 -o pipeline_tb \
  -I../src \
  ../src/alu.sv \
  ../src/reg_file.sv \
  ../src/control_unit.sv \
  ../src/imm_gen.sv \
  ../src/imem.sv \
  ../src/dmem.sv \
  ../src/forwarding_unit.sv \
  ../src/hazard_detection_unit.sv \
  ../src/branch_jump_control.sv \
  ../src/if_id_reg.sv \
  ../src/id_ex_reg.sv \
  ../src/ex_mem_reg.sv \
  ../src/mem_wb_reg.sv \
  ../src/if_stage.sv \
  ../src/id_stage.sv \
  ../src/ex_stage.sv \
  ../src/mem_stage.sv \
  ../src/wb_stage.sv \
  ../src/riscv_pipeline.sv \
  ../tb/riscv_pipeline_tb.sv
```

**Step 2: Run Simulation**

```bash
vvp pipeline_tb
```

**Step 3: Check Output**

Simulation will:
- Load instruction memory from `mem/inst_mem.hex` (default)
- Run for specified number of cycles
- Generate VCD waveform file (if testbench includes `$dumpfile`)
- Print register/memory contents

**Expected Output:**
```
Simulation started...
Cycle: 0
Cycle: 1
...
Cycle: 1000
Simulation completed.
```

---

#### Using ModelSim/QuestaSim

**Step 1: Create Work Library**

```bash
mkdir -p work
cd work
vlib work
vmap work work
```

**Step 2: Compile All Files**

```bash
# Compile source files (in dependency order)
vlog -work work ../src/alu.sv
vlog -work work ../src/reg_file.sv
vlog -work work ../src/control_unit.sv
vlog -work work ../src/imm_gen.sv
vlog -work work ../src/imem.sv
vlog -work work ../src/dmem.sv
vlog -work work ../src/forwarding_unit.sv
vlog -work work ../src/hazard_detection_unit.sv
vlog -work work ../src/branch_jump_control.sv
vlog -work work ../src/if_id_reg.sv
vlog -work work ../src/id_ex_reg.sv
vlog -work work ../src/ex_mem_reg.sv
vlog -work work ../src/mem_wb_reg.sv
vlog -work work ../src/if_stage.sv
vlog -work work ../src/id_stage.sv
vlog -work work ../src/ex_stage.sv
vlog -work work ../src/mem_stage.sv
vlog -work work ../src/wb_stage.sv
vlog -work work ../src/riscv_pipeline.sv

# Compile testbench
vlog -work work ../tb/riscv_pipeline_tb.sv
```

**Step 3: Run Simulation**

**Interactive Mode:**
```bash
vsim -voptargs=+acc work.riscv_pipeline_tb
```

**Batch Mode (Non-Interactive):**
```bash
vsim -c -voptargs=+acc -do "run -all; quit -f" work.riscv_pipeline_tb
```

**With Waveforms:**
```bash
vsim -voptargs=+acc -do "add wave -radix hex /riscv_pipeline_tb/*; run -all" work.riscv_pipeline_tb
```

---

### Method 3: Using Tcl Scripts (ModelSim)

Create `simulate.tcl`:
```tcl
# Load design
vsim -voptargs=+acc work.riscv_pipeline_tb

# Add waves
add wave -radix hex /riscv_pipeline_tb/clk
add wave -radix hex /riscv_pipeline_tb/rst_n
add wave -radix hex /riscv_pipeline_tb/uut/PC
add wave -radix hex /riscv_pipeline_tb/uut/if_id_instruction
add wave -radix hex /riscv_pipeline_tb/uut/id_ex_rs1_data
add wave -radix hex /riscv_pipeline_tb/uut/ex_mem_alu_result

# Run simulation
run 1000ns

# Save waveform
dataset save vsim.wlf simulation/pipeline_tb.wlf

# Quit
quit -f
```

Run:
```bash
vsim -c -do simulate.tcl
```

---

## Selecting Test Programs

### Available Test Programs

The `mem/` directory contains several test programs:

| Test Program | File | Description | Expected Cycles |
|-------------|------|-------------|----------------|
| **Basic Test** | `test_program.hex` | Basic functionality | ~50 |
| **Arithmetic** | `add_sub_test.hex` | ADD/SUB operations | ~60 |
| **Immediate** | `addi_subi_test.hex` | ADDI operations | ~50 |
| **Logical** | `logical_ops_test.hex` | AND/OR/XOR operations | ~80 |
| **Memory** | `memory_ops_test.hex` | LW/SW operations | ~150 |
| **Branch** | `branch_test.hex` | Branch instructions | ~120 |
| **Jump** | `jump_test.hex` | JAL/JALR instructions | ~100 |
| **Hazards** | `raw_hazard_test.hex` | Data hazard tests | ~80 |
| **Comparison** | `slt_sltu_test.hex` | SLT/SLTU operations | ~70 |

### Changing Test Program

**Method 1: Edit Testbench Parameter**

Edit `tb/riscv_pipeline_tb.sv`:
```systemverilog
parameter IMEM_INIT_FILE = "mem/branch_test.hex";  // Change this line
```

**Method 2: Use Command-Line (if supported)**

Some simulators allow parameter override:
```bash
# ModelSim example
vsim -voptargs=+acc -GIMEM_INIT_FILE=\"mem/branch_test.hex\" work.riscv_pipeline_tb
```

**Method 3: Copy File**

```bash
cp mem/branch_test.hex mem/inst_mem.hex
```

---

## Viewing Waveforms

### Using GTKWave (Icarus Verilog)

**Step 1: Generate VCD File**

Ensure testbench includes:
```systemverilog
initial begin
    $dumpfile("pipeline_tb.vcd");
    $dumpvars(0, riscv_pipeline_tb);
end
```

**Step 2: Run Simulation**

```bash
vvp pipeline_tb
```

**Step 3: Open Waveform**

```bash
gtkwave pipeline_tb.vcd
```

**GTKWave Usage:**

1. **Navigate Hierarchy**: Expand signals in left panel
2. **Add Signals**: Drag signals to waveform view
3. **Zoom**: Use mouse wheel or zoom buttons
4. **Measure Time**: Click and drag to measure time intervals
5. **Save Session**: File → Save Waveform → Save as `.gtkw`

**Key Signals to View:**

- `clk`: Clock signal
- `rst_n`: Reset signal
- `PC`: Program counter
- `if_id_instruction`: Instruction in IF/ID register
- `id_ex_rs1_data`, `id_ex_rs2_data`: Register data
- `ex_mem_alu_result`: ALU result
- `mem_wb_reg_write`: Register write enable
- `ForwardA`, `ForwardB`: Forwarding control signals
- `stall`, `flush`: Pipeline control signals

---

### Using ModelSim Waveform Viewer

**Step 1: Generate Waveform**

Run simulation with waveform:
```bash
vsim -voptargs=+acc -do "add wave -radix hex /riscv_pipeline_tb/*; run -all" work.riscv_pipeline_tb
```

**Step 2: Save Waveform**

In ModelSim GUI:
```
File → Save Format → Save as simulation/pipeline_tb.wlf
```

**Step 3: Reload Waveform**

```bash
vsim -view simulation/pipeline_tb.wlf -do "add wave -radix hex /riscv_pipeline_tb/*"
```

**ModelSim Waveform Features:**

- **Zoom**: Mouse wheel or zoom buttons
- **Cursor**: Click to place cursor, drag to measure
- **Radix**: Right-click signal → Radix → Select format (hex, binary, decimal)
- **Groups**: Create signal groups for organization
- **Bookmarks**: Mark important time points

---

### Waveform Analysis Tips

#### 1. Pipeline Stage Progression

**What to Look For:**
- Instructions moving through pipeline stages
- Each instruction takes 5 cycles to complete
- Multiple instructions in pipeline simultaneously

**Example Timeline:**
```
Cycle 0: Instruction 1 in IF
Cycle 1: Instruction 1 in ID, Instruction 2 in IF
Cycle 2: Instruction 1 in EX, Instruction 2 in ID, Instruction 3 in IF
Cycle 3: Instruction 1 in MEM, Instruction 2 in EX, Instruction 3 in ID, Instruction 4 in IF
Cycle 4: Instruction 1 in WB, Instruction 2 in MEM, Instruction 3 in EX, Instruction 4 in ID, Instruction 5 in IF
```

#### 2. Data Forwarding

**What to Look For:**
- `ForwardA` or `ForwardB` signals asserting (value `2'b10` or `2'b01`)
- ALU operands (`rs1_data_forwarded`, `rs2_data_forwarded`) showing forwarded values
- Correct ALU results despite data dependencies

**Example:**
```
Cycle N:   ADD x5, x0, 5      (in MEM stage, x5 = 5)
Cycle N+1: ADD x6, x5, x2     (in EX stage, ForwardA = 10, uses x5 from EX/MEM)
```

#### 3. Pipeline Stalls

**What to Look For:**
- `stall` signal asserting high
- `PC` not updating during stall
- `if_id_enable` low (IF/ID register held)
- `id_ex_flush` high (bubble inserted)

**Example:**
```
Cycle N:   LW x1, 0(x0)       (in EX stage)
Cycle N+1: ADD x2, x1, x0     (in ID stage, stall detected)
           stall = 1, PC held, bubble in ID/EX
Cycle N+2: ADD x2, x1, x0     (moves to EX stage, uses forwarded x1)
```

#### 4. Branch/Jump Flushes

**What to Look For:**
- `flush` signal asserting high
- `PCSrc` selecting branch/jump target
- `PC` jumping to new address
- IF/ID and ID/EX registers cleared

**Example:**
```
Cycle N:   BEQ x1, x1, 8      (in EX stage, condition true)
Cycle N+1: flush = 1, PC jumps to target, IF/ID and ID/EX cleared
```

---

## Interpreting Results

### Console Output

#### Register File Dump

**Example Output:**
```
=== Register File Contents ===
x0  = 0x00000000 (0)
x1  = 0x0000000A (10)
x2  = 0x00000014 (20)
x3  = 0x0000001E (30)
...
```

**What to Check:**
- ✅ Register values match expected results
- ✅ x0 is always zero
- ✅ Results are correct for test program

#### Memory Dump

**Example Output:**
```
=== Data Memory Contents ===
memory[0]  = 0x0000000A (10)
memory[1]  = 0x00000014 (20)
memory[2]  = 0x0000001E (30)
...
```

**What to Check:**
- ✅ Memory values match expected results
- ✅ Store operations wrote correct values
- ✅ Load operations read correct values

#### Performance Metrics

**Example Output:**
```
=== Performance Metrics ===
Total Cycles: 1250
Instructions Completed: 1000
Pipeline Stalls: 150
Pipeline Flushes: 100
CPI: 1.25
```

**What to Check:**
- ✅ CPI is reasonable (1.0 - 1.5 for typical workloads)
- ✅ Stall rate is acceptable (< 20% for memory-intensive code)
- ✅ Flush rate is acceptable (< 15% for branch-heavy code)

---

### Verification Checklist

#### For Arithmetic Tests

- [ ] Register values match expected results
- [ ] Overflow cases handled correctly (two's complement wrap)
- [ ] Zero operands work correctly
- [ ] Negative numbers handled correctly

#### For Memory Tests

- [ ] Load operations read correct values
- [ ] Store operations write correct values
- [ ] Load-use hazards cause pipeline stalls
- [ ] Memory addressing with offsets works correctly

#### For Branch Tests

- [ ] Branch conditions evaluated correctly
- [ ] Taken branches jump to correct address
- [ ] Not-taken branches continue sequentially
- [ ] Pipeline flushes occur on taken branches
- [ ] Signed vs unsigned comparisons work correctly

#### For Jump Tests

- [ ] JAL saves return address correctly
- [ ] JALR calculates target address correctly
- [ ] Procedure calls and returns work correctly
- [ ] Nested calls preserve return addresses

#### For Hazard Tests

- [ ] EX/MEM forwarding works correctly
- [ ] MEM/WB forwarding works correctly
- [ ] Load-use hazards cause stalls
- [ ] Forwarding prevents incorrect results

---

## Advanced Simulation

### Custom Test Programs

**Step 1: Write Assembly**

Create `mem/my_test.s`:
```assembly
# My custom test program
ADDI x1, x0, 10     # x1 = 10
ADDI x2, x0, 20     # x2 = 20
ADD x3, x1, x2      # x3 = x1 + x2 = 30
SW x3, 0(x0)        # Store x3 to memory[0]
```

**Step 2: Assemble to Hex**

Use RISC-V assembler (e.g., `riscv64-unknown-elf-as`):
```bash
riscv64-unknown-elf-as -march=rv32i my_test.s -o my_test.o
riscv64-unknown-elf-objcopy -O binary my_test.o my_test.bin
xxd -p -c 4 my_test.bin > my_test.hex
```

**Step 3: Update Testbench**

Edit `tb/riscv_pipeline_tb.sv`:
```systemverilog
parameter IMEM_INIT_FILE = "mem/my_test.hex";
```

**Step 4: Run Simulation**

```bash
# Recompile and simulate
iverilog -g2012 -o pipeline_tb ... (compile command)
vvp pipeline_tb
```

---

### Performance Analysis

#### Using Performance Monitor

The `performance_monitor` module tracks:
- Total cycles
- Instructions completed
- Pipeline stalls
- Pipeline flushes
- CPI (Cycles Per Instruction)

**Enable Performance Monitoring:**

In testbench, instantiate performance monitor:
```systemverilog
performance_monitor #(
    .COUNTER_WIDTH(32),
    .CPI_WIDTH(16)
) perf_monitor (
    .clk(clk),
    .rst_n(rst_n),
    .enable(1'b1),
    .wb_RegWrite(wb_RegWrite),
    .wb_rd_addr(wb_rd_addr),
    .pipeline_stall(pipeline_stall),
    .pipeline_flush(pipeline_flush),
    .mem_MemRead(mem_MemRead),
    .mem_MemWrite(mem_MemWrite),
    .mem_Branch(mem_Branch),
    .total_cycles(total_cycles),
    .instructions_completed(instructions_completed),
    .cpi_value(cpi_value)
);
```

**Display Results:**

```systemverilog
initial begin
    #10000;  // Wait for simulation
    $display("CPI: %f", $itor(cpi_value) / 65536.0);
    $display("Stall Rate: %f%%", $itor(pipeline_stalls) * 100.0 / $itor(total_cycles));
end
```

---

### Debugging Techniques

#### 1. Add Debug Prints

In testbench or modules:
```systemverilog
always_ff @(posedge clk) begin
    if (some_condition) begin
        $display("Cycle %d: PC = 0x%08X, Instruction = 0x%08X", 
                 cycle_count, PC, instruction);
    end
end
```

#### 2. Monitor Specific Signals

Add to waveform:
- Pipeline control signals (`stall`, `flush`)
- Forwarding signals (`ForwardA`, `ForwardB`)
- Hazard detection signals
- Branch/jump control signals

#### 3. Check Pipeline Registers

Verify instructions progress correctly:
- IF/ID register contents
- ID/EX register contents
- EX/MEM register contents
- MEM/WB register contents

#### 4. Verify Memory Contents

Check instruction and data memory:
- Instruction memory loaded correctly
- Data memory reads/writes correct
- Memory addresses within bounds

---

## Troubleshooting

### Issue 1: Simulation Hangs or Runs Forever

**Symptoms:**
- Simulation doesn't complete
- No output after many cycles

**Solutions:**

1. **Check for Infinite Loop:**
   ```systemverilog
   // Add timeout in testbench
   initial begin
       #1000000;  // 1 million cycles max
       $display("ERROR: Simulation timeout");
       $finish;
   end
   ```

2. **Check Reset Signal:**
   - Ensure `rst_n` asserts correctly
   - Check reset timing

3. **Check Branch/Jump Logic:**
   - Verify branch targets are correct
   - Check for infinite branch loops

4. **Reduce Simulation Cycles:**
   ```systemverilog
   parameter SIM_CYCLES = 100;  // Reduce from 1000
   ```

---

### Issue 2: Wrong Results

**Symptoms:**
- Register values don't match expected
- Memory contents incorrect

**Solutions:**

1. **Check Instruction Memory:**
   ```bash
   # Verify hex file format
   cat mem/test_program.hex
   # Should be one 32-bit hex value per line
   ```

2. **Check Testbench Parameters:**
   ```systemverilog
   parameter IMEM_INIT_FILE = "mem/test_program.hex";  // Correct path?
   ```

3. **Verify Instruction Encoding:**
   - Check instruction opcodes
   - Verify register addresses
   - Verify immediate values

4. **Check Pipeline Hazards:**
   - Verify forwarding works correctly
   - Check for load-use hazards
   - Verify stalls occur when needed

---

### Issue 3: No Waveform File Generated

**Symptoms:**
- Simulation runs but no `.vcd` or `.wlf` file

**Solutions:**

1. **Check Testbench for VCD Dump:**
   ```systemverilog
   initial begin
       $dumpfile("pipeline_tb.vcd");
       $dumpvars(0, riscv_pipeline_tb);
   end
   ```

2. **Check File Permissions:**
   ```bash
   ls -la simulation/
   chmod 755 simulation/
   ```

3. **Check Output Directory:**
   ```bash
   # Ensure simulation directory exists
   mkdir -p simulation
   ```

---

### Issue 4: Compilation Errors

**Symptoms:**
- `iverilog` or `vlog` reports errors

**Common Errors:**

**"Cannot find module":**
```bash
# Solution: Add include directory
iverilog -I../src ...
```

**"Syntax error":**
```bash
# Solution: Use SystemVerilog flag
iverilog -g2012 ...
```

**"Multiple definitions":**
```bash
# Solution: Don't compile same file twice
# Check compilation order
```

---

### Issue 5: ModelSim License Issues

**Symptoms:**
- "License checkout failed"
- "No valid license"

**Solutions:**

1. **Check License File:**
   ```bash
   echo $LM_LICENSE_FILE
   ```

2. **Set License File:**
   ```bash
   export LM_LICENSE_FILE=/path/to/license.dat
   ```

3. **Use Free Student Version:**
   - Download from Intel website
   - No license required for student version

---

### Issue 6: Waveform Viewer Won't Open

**Symptoms:**
- GTKWave or ModelSim viewer doesn't launch

**Solutions:**

1. **Install GTKWave:**
   ```bash
   sudo apt-get install gtkwave  # Linux
   brew install gtkwave          # macOS
   ```

2. **Check File Format:**
   ```bash
   file pipeline_tb.vcd
   # Should show: ASCII text
   ```

3. **Manual Open:**
   ```bash
   gtkwave pipeline_tb.vcd &
   ```

---

## Quick Reference

### Common Commands

**Icarus Verilog:**
```bash
# Compile
iverilog -g2012 -o output -Iinclude source.sv testbench.sv

# Run
vvp output

# View waveforms
gtkwave output.vcd
```

**ModelSim:**
```bash
# Compile
vlog -work work source.sv

# Simulate
vsim -voptargs=+acc work.module_name

# Batch mode
vsim -c -do "run -all; quit -f" work.module_name
```

### Test Program Quick Reference

| Test | File | Key Features |
|------|------|-------------|
| Basic | `test_program.hex` | All instruction types |
| Arithmetic | `add_sub_test.hex` | ADD, SUB, overflow |
| Memory | `memory_ops_test.hex` | LW, SW, hazards |
| Branch | `branch_test.hex` | All 6 branch types |
| Jump | `jump_test.hex` | JAL, JALR, calls |

---

## Next Steps

After successful simulation:

1. **Analyze Waveforms**: Study pipeline behavior, forwarding, stalls
2. **Try Different Tests**: Run various test programs
3. **Modify Tests**: Create custom test programs
4. **Performance Analysis**: Use performance monitor
5. **Read Documentation**: See `docs/ARCHITECTURE.md` for design details

---

**Last Updated**: Based on current project structure  
**Compatible With**: Icarus Verilog 12.0+, ModelSim 2020.1+

