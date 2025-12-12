# Build Guide - RISC-V 5-Stage Pipeline Processor

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Directory Structure](#directory-structure)
3. [Installation](#installation)
4. [Compilation Steps](#compilation-steps)
5. [Verifying Installation](#verifying-installation)
6. [Common Issues](#common-issues)

---

## Prerequisites

### Required Tools

You need at least **one** of the following simulators to compile and simulate the RISC-V processor:

#### Option 1: Icarus Verilog (Recommended for Beginners)

**Why Choose Icarus Verilog?**
- ✅ Free and open-source
- ✅ Easy to install
- ✅ Cross-platform (Linux, macOS, Windows)
- ✅ Good for learning and development
- ✅ Lightweight and fast

**Installation:**

**Linux (Ubuntu/Debian):**
```bash
sudo apt-get update
sudo apt-get install iverilog gtkwave
```

**Linux (Fedora/RHEL):**
```bash
sudo dnf install iverilog gtkwave
```

**macOS (using Homebrew):**
```bash
brew install icarus-verilog gtkwave
```

**Windows:**
- Option A: Use WSL (Windows Subsystem for Linux) and follow Linux instructions
- Option B: Use Git Bash and install through WSL
- Option C: Download pre-built binaries from [Icarus Verilog website](http://iverilog.icarus.com/)

**Verify Installation:**
```bash
iverilog -v
vvp -v
gtkwave --version
```

**Expected Output:**
```
Icarus Verilog version 12.0 (or similar)
```

---

#### Option 2: ModelSim/QuestaSim (Professional Tools)

**Why Choose ModelSim/QuestaSim?**
- ✅ Industry-standard tool
- ✅ Advanced debugging features
- ✅ Better waveform viewer
- ⚠️ Requires license (free student version available)
- ⚠️ Larger installation size

**Installation:**

1. **Download**: Get ModelSim-Intel or QuestaSim from Intel/AMD website
2. **Install**: Follow installation wizard
3. **License**: Set up license server (for paid versions) or use free student version
4. **Add to PATH**: Ensure `vlog` and `vsim` commands are available

**Verify Installation:**
```bash
vlog -version
vsim -version
```

**Expected Output:**
```
ModelSim/QuestaSim version information
```

---

#### Option 3: Xilinx Vivado (FPGA Development)

**Why Choose Vivado?**
- ✅ Includes simulator
- ✅ Can synthesize to FPGA
- ✅ Good for hardware implementation
- ⚠️ Large installation (~20GB+)
- ⚠️ Requires registration

**Installation:**

1. Download Vivado from Xilinx website
2. Install with simulator option enabled
3. Launch Vivado and use Tcl console for simulation

---

### Optional Tools

**GTKWave** (Waveform Viewer):
- Already included with Icarus Verilog installation
- Standalone installation: `sudo apt-get install gtkwave` (Linux)

**Python 3** (for test generation scripts):
```bash
python3 --version  # Should be 3.6 or higher
```

**Git** (for cloning repository):
```bash
git --version
```

**Make** (optional, for build automation):
```bash
make --version
```

---

## Directory Structure

Understanding the project structure will help you navigate and build the project:

```
RISC-V-5Stage-Processor/
│
├── src/                          # Source code (SystemVerilog modules)
│   ├── riscv_pipeline.sv        # Top-level pipeline module
│   ├── if_stage.sv              # Instruction Fetch stage
│   ├── id_stage.sv              # Instruction Decode stage
│   ├── ex_stage.sv              # Execute stage
│   ├── mem_stage.sv             # Memory Access stage
│   ├── wb_stage.sv              # Writeback stage
│   ├── alu.sv                   # Arithmetic Logic Unit
│   ├── reg_file.sv              # Register file
│   ├── control_unit.sv          # Control unit
│   ├── forwarding_unit.sv       # Data forwarding unit
│   ├── hazard_detection_unit.sv  # Hazard detection unit
│   ├── branch_jump_control.sv   # Branch/jump control
│   ├── imm_gen.sv               # Immediate generator
│   ├── imem.sv                  # Instruction memory
│   ├── dmem.sv                  # Data memory
│   ├── if_id_reg.sv             # IF/ID pipeline register
│   ├── id_ex_reg.sv             # ID/EX pipeline register
│   ├── ex_mem_reg.sv            # EX/MEM pipeline register
│   ├── mem_wb_reg.sv            # MEM/WB pipeline register
│   └── performance_monitor.sv   # Performance monitoring module
│
├── tb/                           # Testbenches
│   ├── riscv_pipeline_tb.sv    # Main pipeline testbench
│   ├── alu_tb.sv                # ALU unit testbench
│   ├── reg_file_tb.sv           # Register file testbench
│   ├── performance_monitor_tb_example.sv
│   └── golden_test_tb_example.sv
│
├── mem/                         # Memory initialization files
│   ├── inst_mem.hex             # Default instruction memory
│   ├── test_program.hex         # Basic test program
│   ├── add_sub_test.hex         # Arithmetic test
│   ├── logical_ops_test.hex     # Logical operations test
│   ├── memory_ops_test.hex      # Memory operations test
│   ├── branch_test.hex          # Branch instructions test
│   ├── jump_test.hex            # Jump instructions test
│   ├── raw_hazard_test.hex      # Data hazard test
│   └── *.s                      # Assembly source files
│
├── scripts/                     # Build and simulation scripts
│   ├── simulate_alu.sh          # ALU simulation script
│   ├── generate_random_test.py  # Random test generator
│   ├── generate_golden_test.py # Golden reference generator
│   └── parse_performance_logs.py # Performance log parser
│
├── docs/                        # Documentation
│   ├── BUILD.md                 # This file
│   ├── SIMULATION.md            # Simulation guide
│   ├── ARCHITECTURE.md          # Architecture documentation
│   ├── MODULES.md               # Module documentation
│   └── ...                     # Other documentation files
│
├── work/                        # Compilation work directory (created during build)
├── simulation/                  # Simulation output directory (created during simulation)
│   ├── *.vcd                    # Waveform files (Icarus Verilog)
│   ├── *.wlf                    # Waveform files (ModelSim)
│   └── simulation.log           # Simulation logs
│
└── README.md                    # Project README
```

### Key Directories Explained

- **`src/`**: Contains all SystemVerilog source files (`.sv` extension)
- **`tb/`**: Contains testbench files that instantiate and test the modules
- **`mem/`**: Contains memory initialization files (`.hex` format) with test programs
- **`scripts/`**: Helper scripts for automation
- **`docs/`**: Comprehensive documentation
- **`work/`**: Created during compilation (can be deleted, will be regenerated)
- **`simulation/`**: Created during simulation (contains waveforms and logs)

---

## Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/TejaRaghuveer/RISC-V-5Stage-Processor.git
cd RISC-V-5Stage-Processor
```

### Step 2: Verify Prerequisites

Check that your simulator is installed:

**For Icarus Verilog:**
```bash
iverilog -v
vvp -v
```

**For ModelSim:**
```bash
vlog -version
vsim -version
```

### Step 3: Make Scripts Executable (Linux/macOS)

```bash
chmod +x scripts/*.sh
```

**Note**: On Windows with Git Bash, scripts should work without this step.

---

## Compilation Steps

### Method 1: Using Icarus Verilog (Recommended)

#### Basic Compilation

**Compile the ALU testbench:**
```bash
cd simulation
iverilog -g2012 -o alu_tb -I../src ../src/alu.sv ../tb/alu_tb.sv
```

**Explanation:**
- `-g2012`: Use SystemVerilog-2012 standard
- `-o alu_tb`: Output executable name
- `-I../src`: Include directory for source files
- `../src/alu.sv`: ALU source file
- `../tb/alu_tb.sv`: ALU testbench file

**Expected Output:**
```
(No errors if compilation successful)
```

#### Compile Full Pipeline Testbench

**Compile the complete pipeline:**
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

**Note**: This is a long command. Consider using a script or Makefile (see below).

#### Using Provided Scripts

**Compile and simulate ALU:**
```bash
./scripts/simulate_alu.sh iverilog
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
```

---

### Method 2: Using ModelSim/QuestaSim

#### Step 1: Create Work Library

```bash
mkdir -p work
cd work
vlib work
vmap work work
```

**Expected Output:**
```
# vlib work
# ** Warning: (vlib-34) Library already exists at "work".
# vmap work work
```

#### Step 2: Compile Source Files

**Compile ALU module:**
```bash
vlog -work work ../src/alu.sv
```

**Compile ALU testbench:**
```bash
vlog -work work ../tb/alu_tb.sv
```

**Expected Output:**
```
Model Technology ModelSim - Intel FPGA Edition vlog 2020.1 Compiler 2020.01 Jan 24 2020
-- Compiling module alu
-- Compiling module alu_tb

Top level modules:
	alu_tb
```

#### Step 3: Compile Full Pipeline

**Compile all source files:**
```bash
# Compile all source modules
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

**Note**: Order matters! Compile dependencies before modules that use them.

#### Using Tcl Script (Alternative)

Create `compile.tcl`:
```tcl
vlib work
vmap work work

# Compile source files
vlog -work work ../src/alu.sv
vlog -work work ../src/reg_file.sv
# ... (add all source files)

# Compile testbench
vlog -work work ../tb/riscv_pipeline_tb.sv
```

Run:
```bash
vsim -c -do compile.tcl
```

---

### Method 3: Using Makefile (Optional)

Create `Makefile`:
```makefile
# Compiler settings
IVERILOG = iverilog
IVERILOG_FLAGS = -g2012 -I src

# Directories
SRC_DIR = src
TB_DIR = tb
SIM_DIR = simulation

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.sv)
TB_SOURCES = $(wildcard $(TB_DIR)/*_tb.sv)

# Targets
.PHONY: all clean alu pipeline

all: alu pipeline

alu: $(SIM_DIR)/alu_tb

pipeline: $(SIM_DIR)/pipeline_tb

$(SIM_DIR)/alu_tb: $(SRC_DIR)/alu.sv $(TB_DIR)/alu_tb.sv
	@mkdir -p $(SIM_DIR)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $@ $^

$(SIM_DIR)/pipeline_tb: $(SOURCES) $(TB_DIR)/riscv_pipeline_tb.sv
	@mkdir -p $(SIM_DIR)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $@ $^

clean:
	rm -rf $(SIM_DIR)/*.vcd $(SIM_DIR)/*.log work/ transcript
```

**Usage:**
```bash
make alu          # Compile ALU testbench
make pipeline     # Compile pipeline testbench
make clean        # Clean build artifacts
```

---

## Verifying Installation

### Test 1: Compile ALU Module

```bash
cd simulation
iverilog -g2012 -o test_alu -I../src ../src/alu.sv ../tb/alu_tb.sv
```

**Success Criteria:**
- No error messages
- Executable file `test_alu` created (or `test_alu.exe` on Windows)

### Test 2: Run Simple Simulation

```bash
vvp test_alu
```

**Expected Output:**
```
Simulation output from testbench
(May include register dumps, memory contents, etc.)
```

### Test 3: Check Waveform Generation

After running simulation, check for VCD file:
```bash
ls -la *.vcd
```

**Expected Output:**
```
-rw-r--r-- 1 user user 12345 Jan 01 12:00 alu_tb.vcd
```

---

## Common Issues

### Issue 1: "Command not found: iverilog"

**Problem**: Icarus Verilog is not installed or not in PATH.

**Solution:**
```bash
# Linux
sudo apt-get install iverilog

# macOS
brew install icarus-verilog

# Verify installation
which iverilog
```

---

### Issue 2: "Cannot find module 'alu'"

**Problem**: Include path not set correctly.

**Solution:**
```bash
# Use -I flag to specify include directory
iverilog -g2012 -I../src -I. ../src/alu.sv ../tb/alu_tb.sv
```

---

### Issue 3: "Syntax error" or "Parse error"

**Problem**: SystemVerilog syntax not recognized.

**Solution:**
```bash
# Use -g2012 flag for SystemVerilog-2012
iverilog -g2012 -o test ../src/alu.sv ../tb/alu_tb.sv
```

---

### Issue 4: "Multiple definitions" or "Redefinition"

**Problem**: Module compiled multiple times or circular dependencies.

**Solution:**
- Compile dependencies first
- Don't compile the same file twice
- Check for duplicate module definitions

---

### Issue 5: ModelSim License Error

**Problem**: ModelSim license not found.

**Solution:**
```bash
# Check license file location
echo $LM_LICENSE_FILE

# Set license file (if needed)
export LM_LICENSE_FILE=/path/to/license.dat

# Or use free student version
# Download from Intel website
```

---

### Issue 6: "Permission denied" (Linux/macOS)

**Problem**: Scripts not executable.

**Solution:**
```bash
chmod +x scripts/*.sh
```

---

### Issue 7: Windows Path Issues

**Problem**: Backslashes vs forward slashes, path separators.

**Solution:**
- Use Git Bash or WSL for better compatibility
- Use forward slashes `/` instead of backslashes `\`
- Use relative paths: `../src/alu.sv` instead of `C:\...\alu.sv`

---

### Issue 8: "File not found" for Memory Initialization

**Problem**: Hex file path incorrect.

**Solution:**
```bash
# Check file exists
ls mem/*.hex

# Use relative path from simulation directory
# In testbench: parameter IMEM_INIT_FILE = "../mem/test_program.hex"
```

---

## Next Steps

After successful compilation:

1. **Read SIMULATION.md**: Learn how to run simulations
2. **Try ALU Test**: Run `./scripts/simulate_alu.sh` to test compilation
3. **Explore Test Programs**: Check `mem/` directory for test programs
4. **Read Documentation**: See `docs/ARCHITECTURE.md` for design details

---

## Quick Reference

### Icarus Verilog Commands

```bash
# Compile
iverilog -g2012 -o output -Iinclude_dir source.sv testbench.sv

# Run simulation
vvp output

# View waveforms
gtkwave waveform.vcd
```

### ModelSim Commands

```bash
# Create library
vlib work
vmap work work

# Compile
vlog -work work source.sv

# Simulate
vsim -voptargs=+acc work.module_name
```

---

## Getting Help

If you encounter issues:

1. **Check Error Messages**: Read compiler/simulator error messages carefully
2. **Verify File Paths**: Ensure all file paths are correct
3. **Check Documentation**: See `docs/` directory for detailed guides
4. **Review Examples**: Look at `scripts/simulate_alu.sh` for working examples
5. **GitHub Issues**: Check existing issues or create new one

---

**Last Updated**: Based on current project structure  
**Compatible With**: Icarus Verilog 12.0+, ModelSim 2020.1+, SystemVerilog-2012

