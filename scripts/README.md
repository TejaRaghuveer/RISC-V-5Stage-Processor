# Simulation Scripts

This directory contains simulation scripts for the RISC-V processor project.

## ALU Simulation Script

### `simulate_alu.sh`

A comprehensive bash script to compile and simulate the ALU module and testbench.

### Prerequisites

**For ModelSim/QuestaSim:**
- ModelSim or QuestaSim installed and in PATH
- Commands: `vlog`, `vsim`

**For Icarus Verilog:**
- Icarus Verilog installed
- Commands: `iverilog`, `vvp`
- Optional: GTKWave for waveform viewing

### Installation

**Linux:**
```bash
sudo apt-get install iverilog gtkwave
```

**macOS:**
```bash
brew install icarus-verilog gtkwave
```

**Windows:**
- Use Git Bash or WSL (Windows Subsystem for Linux)
- Install Icarus Verilog through WSL or use ModelSim

### Usage

```bash
# Make executable (Linux/macOS/Git Bash)
chmod +x scripts/simulate_alu.sh

# Run with auto-detection
./scripts/simulate_alu.sh

# Force specific simulator
./scripts/simulate_alu.sh modelsim
./scripts/simulate_alu.sh iverilog

# Skip waveform viewer
./scripts/simulate_alu.sh --no-waveform

# Clean before simulation
./scripts/simulate_alu.sh --clean

# Show help
./scripts/simulate_alu.sh --help
```

### Features

- **Auto-detection**: Automatically detects available simulator
- **Dual support**: Works with both ModelSim and Icarus Verilog
- **Waveform generation**: Creates VCD files for waveform analysis
- **Auto-open viewer**: Opens GTKWave or ModelSim waveform viewer
- **Clean option**: Removes previous simulation files
- **Colored output**: Easy-to-read colored terminal output
- **Error handling**: Comprehensive error checking and reporting

### Output Files

Simulation files are stored in `simulation/` directory:
- `simulation.log`: Simulation output log
- `alu_tb.vcd`: VCD waveform file (Icarus Verilog)
- `alu_tb.wlf`: ModelSim waveform file (ModelSim)

### Troubleshooting

**Script not executable:**
```bash
chmod +x scripts/simulate_alu.sh
```

**Simulator not found:**
- Ensure simulator is installed and in PATH
- Check with: `which iverilog` or `which vsim`

**Waveform viewer not opening:**
- Install GTKWave: `sudo apt-get install gtkwave` (Linux)
- Or use `--no-waveform` flag to skip viewer

**Compilation errors:**
- Check that source files exist in `src/` and `tb/` directories
- Verify SystemVerilog syntax compatibility

## Performance Log Parser

### `parse_performance_logs.py`

A Python script to parse simulation logs and calculate RISC-V processor performance metrics.

### Features

- Parses simulation logs and extracts performance metrics
- Calculates CPI, stall percentage, flush percentage
- Analyzes instruction mix (arithmetic, logic, memory, control flow)
- Generates formatted text reports
- Creates visualization charts (bar charts, line graphs, pie charts)

### Prerequisites

- Python 3.6 or higher
- matplotlib (optional, for visualizations): `pip install matplotlib`

### Usage

```bash
# Parse log file and display report
python scripts/parse_performance_logs.py simulation.log

# Save report to file
python scripts/parse_performance_logs.py simulation.log --report report.txt

# Generate visualizations
python scripts/parse_performance_logs.py simulation.log --visualize

# Generate visualizations in custom directory
python scripts/parse_performance_logs.py simulation.log --visualize --output-dir reports/
```

### Output

- **Text Report**: Formatted performance metrics report
- **Visualizations** (with `--visualize`):
  - Instruction mix bar chart
  - CPI over time line graph
  - Pipeline efficiency pie chart
  - Performance summary bar chart

### Example

```bash
# Run simulation and save log
vsim -c -do "run -all; quit" > simulation.log 2>&1

# Parse and visualize
python scripts/parse_performance_logs.py simulation.log --visualize --output-dir reports/
```

For detailed documentation, see `docs/PERFORMANCE_LOG_PARSER.md`.

## Test Generation Scripts

### `generate_random_test.py`

Generates random RISC-V instruction sequences for processor testing.

See `docs/PERFORMANCE_LOG_PARSER.md` for detailed documentation.

### `generate_golden_test.py`

Generates RISC-V assembly tests with golden reference results.

See `scripts/GOLDEN_TEST_README.md` for detailed documentation.

