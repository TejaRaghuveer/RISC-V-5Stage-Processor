# RISC-V RV32I 5-Stage Pipelined Processor

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![RISC-V](https://img.shields.io/badge/RISC--V-RV32I-green.svg)](https://riscv.org/)

A complete implementation of a 5-stage pipelined RISC-V RV32I processor in Verilog/SystemVerilog. This project implements the base integer instruction set (RV32I) with full pipeline support, hazard detection, and forwarding mechanisms.

## 📋 Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Architecture](#architecture)
- [Instruction Support](#instruction-support)
- [Folder Structure](#folder-structure)
- [Build Instructions](#build-instructions)
- [Simulation Guide](#simulation-guide)
- [Testing](#testing)
- [Future Enhancements](#future-enhancements)
- [References](#references)

## 🎯 Project Overview

This project implements a 5-stage pipelined RISC-V processor compliant with the RV32I base instruction set. The processor is designed to execute RISC-V instructions efficiently through a classic 5-stage pipeline architecture, supporting data forwarding and hazard detection to maximize performance.

### Key Highlights

- **Full RV32I Compliance**: Implements all base integer instructions
- **5-Stage Pipeline**: Classic IF-ID-EX-MEM-WB architecture
- **Hazard Handling**: Data forwarding and pipeline stall mechanisms
- **Modular Design**: Clean, well-organized RTL code structure
- **Comprehensive Testing**: Extensive testbenches and test cases

## ✨ Features

- ✅ **Complete RV32I Instruction Set Support**
  - Arithmetic and Logic Operations (ADD, SUB, AND, OR, XOR, SLT, etc.)
  - Shift Operations (SLL, SRL, SRA)
  - Load/Store Operations (LB, LH, LW, SB, SH, SW)
  - Control Flow (BEQ, BNE, BLT, BGE, BLTU, BGEU, JAL, JALR)
  - System Instructions (ECALL, EBREAK)

- ✅ **Pipeline Architecture**
  - 5-stage pipeline: Instruction Fetch, Decode, Execute, Memory, Write Back
  - Data forwarding unit for hazard resolution
  - Hazard detection unit for pipeline control
  - Branch prediction support (optional)

- ✅ **Memory Interface**
  - Separate instruction and data memory interfaces
  - Configurable memory size
  - Memory-mapped I/O support ready

- ✅ **Design Quality**
  - Synthesizable Verilog/SystemVerilog code
  - Well-documented modules
  - Modular and scalable architecture

## 🏗️ Architecture

### 5-Stage Pipeline Overview

The processor implements a classic 5-stage pipeline architecture:

```
┌──────┐    ┌──────┐    ┌──────┐    ┌──────┐    ┌──────┐
│  IF  │───▶│  ID  │───▶│  EX  │───▶│ MEM  │───▶│  WB  │
└──────┘    └──────┘    └──────┘    └──────┘    └──────┘
```

#### Stage 1: Instruction Fetch (IF)
- Fetches instructions from instruction memory
- Updates Program Counter (PC)
- Handles branch and jump target calculation
- **Pipeline Register**: IF/ID

#### Stage 2: Instruction Decode (ID)
- Decodes instruction opcode and fields
- Reads register file (rs1, rs2)
- Sign-extends immediate values
- Generates control signals
- **Pipeline Register**: ID/EX

#### Stage 3: Execute (EX)
- Performs arithmetic/logic operations (ALU)
- Calculates memory addresses
- Evaluates branch conditions
- Handles data forwarding from MEM and WB stages
- **Pipeline Register**: EX/MEM

#### Stage 4: Memory Access (MEM)
- Accesses data memory (load/store)
- Handles memory read/write operations
- Passes ALU results for non-memory instructions
- **Pipeline Register**: MEM/WB

#### Stage 5: Write Back (WB)
- Writes results back to register file
- Selects data from ALU result or memory data
- Updates destination register (rd)

### Pipeline Hazards

The processor handles three types of hazards:

1. **Data Hazards**
   - Resolved using forwarding (data forwarding unit)
   - Forwarding paths: EX→EX, MEM→EX, WB→EX
   - Pipeline stalls when forwarding cannot resolve hazard

2. **Control Hazards**
   - Branch instructions cause pipeline flush
   - Branch target calculated in EX stage
   - 1-cycle penalty for taken branches

3. **Structural Hazards**
   - Separate instruction and data memory (Harvard architecture)
   - No structural hazards in this design

### Block Diagram

```
                    ┌─────────────┐
                    │ Instruction │
                    │   Memory    │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  IF Stage   │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  ID Stage   │
                    │             │
                    │ ┌─────────┐ │
                    │ │Register │ │
                    │ │  File   │ │
                    │ └─────────┘ │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐      ┌──────────────┐
                    │  EX Stage   │◀─────│   Forwarding │
                    │             │      │     Unit     │
                    │ ┌─────────┐ │      └──────────────┘
                    │ │   ALU   │ │
                    │ └─────────┘ │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │ MEM Stage   │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  Data      │
                    │  Memory    │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  WB Stage   │
                    └─────────────┘
```

## 📚 Instruction Support

### RV32I Base Instruction Set

The processor supports all instructions from the RV32I base instruction set:

#### R-Type Instructions (Register-Register)
| Instruction | Description | Opcode |
|------------|-------------|--------|
| ADD | Add | 0x33 |
| SUB | Subtract | 0x33 |
| SLL | Shift Left Logical | 0x33 |
| SLT | Set Less Than | 0x33 |
| SLTU | Set Less Than Unsigned | 0x33 |
| XOR | XOR | 0x33 |
| SRL | Shift Right Logical | 0x33 |
| SRA | Shift Right Arithmetic | 0x33 |
| OR | OR | 0x33 |
| AND | AND | 0x33 |

#### I-Type Instructions (Immediate)
| Instruction | Description | Opcode |
|------------|-------------|--------|
| ADDI | Add Immediate | 0x13 |
| SLTI | Set Less Than Immediate | 0x13 |
| SLTIU | Set Less Than Immediate Unsigned | 0x13 |
| XORI | XOR Immediate | 0x13 |
| ORI | OR Immediate | 0x13 |
| ANDI | AND Immediate | 0x13 |
| SLLI | Shift Left Logical Immediate | 0x13 |
| SRLI | Shift Right Logical Immediate | 0x13 |
| SRAI | Shift Right Arithmetic Immediate | 0x13 |
| JALR | Jump and Link Register | 0x67 |
| LB | Load Byte | 0x03 |
| LH | Load Halfword | 0x03 |
| LW | Load Word | 0x03 |
| LBU | Load Byte Unsigned | 0x03 |
| LHU | Load Halfword Unsigned | 0x03 |

#### S-Type Instructions (Store)
| Instruction | Description | Opcode |
|------------|-------------|--------|
| SB | Store Byte | 0x23 |
| SH | Store Halfword | 0x23 |
| SW | Store Word | 0x23 |

#### B-Type Instructions (Branch)
| Instruction | Description | Opcode |
|------------|-------------|--------|
| BEQ | Branch if Equal | 0x63 |
| BNE | Branch if Not Equal | 0x63 |
| BLT | Branch if Less Than | 0x63 |
| BGE | Branch if Greater or Equal | 0x63 |
| BLTU | Branch if Less Than Unsigned | 0x63 |
| BGEU | Branch if Greater or Equal Unsigned | 0x63 |

#### U-Type Instructions (Upper Immediate)
| Instruction | Description | Opcode |
|------------|-------------|--------|
| LUI | Load Upper Immediate | 0x37 |
| AUIPC | Add Upper Immediate to PC | 0x17 |

#### J-Type Instructions (Jump)
| Instruction | Description | Opcode |
|------------|-------------|--------|
| JAL | Jump and Link | 0x6F |

#### System Instructions
| Instruction | Description | Opcode |
|------------|-------------|--------|
| ECALL | Environment Call | 0x73 |
| EBREAK | Environment Break | 0x73 |

## 📁 Folder Structure

```
RISC-V-5-Stage-Pipeline/
│
├── src/                    # RTL source code
│   ├── riscv_core.v       # Top-level processor module
│   ├── if_stage.v         # Instruction Fetch stage
│   ├── id_stage.v         # Instruction Decode stage
│   ├── ex_stage.v         # Execute stage
│   ├── mem_stage.v        # Memory Access stage
│   ├── wb_stage.v         # Write Back stage
│   ├── reg_file.v         # Register file
│   ├── alu.v              # Arithmetic Logic Unit
│   ├── control_unit.v     # Main control unit
│   ├── hazard_unit.v      # Hazard detection unit
│   ├── forwarding_unit.v  # Data forwarding unit
│   ├── imm_gen.v          # Immediate generator
│   └── branch_unit.v      # Branch control unit
│
├── tb/                     # Testbenches
│   ├── riscv_core_tb.v    # Top-level testbench
│   ├── alu_tb.v           # ALU testbench
│   ├── reg_file_tb.v      # Register file testbench
│   └── pipeline_tb.v      # Pipeline testbench
│
├── mem/                    # Memory files
│   ├── inst_mem.mem       # Instruction memory initialization
│   ├── data_mem.mem       # Data memory initialization
│   └── test_programs/     # Test program memory files
│
├── scripts/                # Simulation and build scripts
│   ├── compile.sh         # Compilation script
│   ├── simulate.sh        # Simulation script
│   ├── run_tests.sh       # Test runner script
│   └── clean.sh           # Cleanup script
│
├── docs/                   # Documentation
│   ├── architecture.md    # Detailed architecture documentation
│   ├── instruction_set.md # Instruction set reference
│   └── timing_diagrams/   # Timing diagrams and waveforms
│
├── .gitignore             # Git ignore file
└── README.md              # This file
```

## 🔨 Build Instructions

### Prerequisites

- **Verilog/SystemVerilog Simulator**: Choose one of the following:
  - ModelSim/QuestaSim (Intel/AMD)
  - Xilinx Vivado Simulator
  - Icarus Verilog (iverilog)
  - Verilator
  - Synopsys VCS
  - Cadence Xcelium

- **Optional Tools**:
  - GTKWave (for waveform viewing)
  - Make (for build automation)

### Compilation

#### Using Icarus Verilog

```bash
# Compile all source files
iverilog -o riscv_core -I src src/*.v tb/riscv_core_tb.v

# Run simulation
vvp riscv_core

# View waveforms (if VCD file generated)
gtkwave riscv_core.vcd
```

#### Using ModelSim/QuestaSim

```tcl
# Create work library
vlib work
vmap work work

# Compile source files
vlog -work work src/*.v
vlog -work work tb/riscv_core_tb.v

# Run simulation
vsim -voptargs=+acc work.riscv_core_tb

# Add signals to waveform
add wave -radix hex /riscv_core_tb/uut/*
run -all
```

#### Using Xilinx Vivado

```tcl
# In Vivado Tcl console or script
read_verilog src/*.v
read_verilog tb/riscv_core_tb.v

# Set top module
set_property top riscv_core_tb [current_fileset]

# Run simulation
launch_simulation
run -all
```

#### Using Verilator

```bash
# Compile
verilator --cc --exe --build src/*.v tb/riscv_core_tb.cpp

# Run
./obj_dir/Vriscv_core
```

### Using Provided Scripts

```bash
# Make scripts executable (Linux/Mac)
chmod +x scripts/*.sh

# Compile
./scripts/compile.sh

# Run simulation
./scripts/simulate.sh

# Run all tests
./scripts/run_tests.sh

# Clean build artifacts
./scripts/clean.sh
```

## 🧪 Simulation Guide

### Basic Simulation Flow

1. **Prepare Test Program**
   - Write or compile your RISC-V assembly program
   - Convert to memory initialization file format
   - Place in `mem/inst_mem.mem`

2. **Run Simulation**
   ```bash
   # Using provided script
   ./scripts/simulate.sh
   
   # Or manually
   vvp riscv_core
   ```

3. **Analyze Results**
   - Check console output for register values
   - View waveforms in GTKWave or simulator's waveform viewer
   - Verify instruction execution order
   - Check pipeline behavior and hazard handling

### Waveform Analysis

Key signals to monitor:

- **Pipeline Stages**: `pc`, `instruction`, `reg_write`, `mem_write`, `alu_result`
- **Pipeline Registers**: `if_id_pc`, `id_ex_rs1`, `ex_mem_alu_result`, `mem_wb_data`
- **Hazard Detection**: `stall`, `flush`, `forward_a`, `forward_b`
- **Control Signals**: `reg_write_en`, `mem_read`, `mem_write`, `branch_taken`

### Example Test Program

```assembly
# Simple addition program
addi x1, x0, 5      # x1 = 5
addi x2, x0, 10     # x2 = 10
add  x3, x1, x2     # x3 = x1 + x2 = 15
sw   x3, 0(x0)      # Store result to memory[0]
```

## 🧪 Testing

### Test Strategy

The project includes comprehensive testbenches for:

1. **Unit Tests**
   - ALU functionality
   - Register file read/write
   - Control unit signal generation
   - Immediate generation

2. **Integration Tests**
   - Pipeline execution
   - Data forwarding
   - Hazard detection and resolution
   - Branch handling

3. **Instruction Tests**
   - Individual instruction verification
   - Instruction sequences
   - Edge cases and corner conditions

### Running Tests

```bash
# Run all testbenches
./scripts/run_tests.sh

# Run specific testbench
iverilog -o alu_test -I src src/alu.v tb/alu_tb.v
vvp alu_test
```

### Test Coverage Goals

- ✅ All RV32I instructions tested
- ✅ Pipeline hazards covered
- ✅ Forwarding paths verified
- ✅ Branch prediction tested
- ✅ Memory operations validated

## 🚀 Future Enhancements

### Planned Features

- [ ] **Cache System**
  - Instruction cache
  - Data cache
  - Cache coherence protocols

- [ ] **Advanced Pipeline Features**
  - Branch prediction (2-bit predictor, BTB)
  - Out-of-order execution
  - Superscalar architecture

- [ ] **Extended Instruction Sets**
  - RV32M (Multiply/Divide)
  - RV32A (Atomic operations)
  - RV32F (Single-precision floating-point)
  - RV32D (Double-precision floating-point)
  - RV32C (Compressed instructions)

- [ ] **Performance Optimizations**
  - Pipeline depth optimization
  - Critical path optimization
  - Power optimization

- [ ] **Debug and Trace Support**
  - JTAG interface
  - Instruction trace
  - Performance counters

- [ ] **Synthesis and FPGA Implementation**
  - FPGA synthesis scripts
  - Timing constraints
  - Resource utilization reports

- [ ] **Formal Verification**
  - Property-based verification
  - Instruction set compliance verification

## 📖 References

### Official Specifications

- [The RISC-V Instruction Set Manual, Volume I: User-Level ISA](https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf)
- [The RISC-V Instruction Set Manual, Volume II: Privileged Architecture](https://riscv.org/wp-content/uploads/2019/08/riscv-privileged-20190608-1.pdf)

### Books and Resources

- Patterson, D. A., & Hennessy, J. L. (2017). *Computer Organization and Design RISC-V Edition: The Hardware Software Interface*. Morgan Kaufmann.
- Harris, D., & Harris, S. (2012). *Digital Design and Computer Architecture*. Morgan Kaufmann.

### Online Resources

- [RISC-V Foundation](https://riscv.org/)
- [RISC-V Instruction Set Cheat Sheet](https://github.com/jameslzhu/riscv-card)
- [RISC-V Assembly Language](https://github.com/riscv/riscv-asm-manual)

### Related Projects

- [Berkeley RISC-V](https://github.com/ucb-bar)
- [Rocket Chip](https://github.com/chipsalliance/rocket-chip)
- [PicoRV32](https://github.com/YosysHQ/picorv32)

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👥 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 🙏 Acknowledgments

- RISC-V Foundation for the open instruction set architecture
- The open-source hardware community
- Contributors and testers

---

**Note**: This is an educational project. For production use, please ensure compliance with RISC-V specifications and perform thorough verification.

