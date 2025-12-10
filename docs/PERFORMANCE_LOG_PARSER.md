# Performance Log Parser

## Overview

The `parse_performance_logs.py` script parses RISC-V processor simulation logs and extracts comprehensive performance metrics. It generates formatted text reports and optional visualizations to analyze processor performance.

## Features

### Metrics Extracted

1. **Overall Performance**:
   - Total cycles executed
   - Total instructions completed
   - CPI (Cycles Per Instruction)

2. **Pipeline Efficiency**:
   - Pipeline stall count and percentage
   - Pipeline flush count and percentage
   - Overall pipeline efficiency

3. **Memory Operations**:
   - Load instruction count
   - Store instruction count
   - Total memory operations

4. **Control Flow**:
   - Branch instruction count

5. **Instruction Mix**:
   - Arithmetic instructions (ADD, SUB, ADDI, etc.)
   - Logical instructions (AND, OR, XOR, shifts, etc.)
   - Memory instructions (LW, SW, etc.)
   - Control flow instructions (branches, jumps)
   - Other instructions

### Visualizations

1. **Instruction Mix Bar Chart**: Shows distribution of instruction types
2. **CPI Over Time Line Graph**: Tracks CPI changes during execution
3. **Pipeline Efficiency Pie Chart**: Shows cycle distribution (useful work vs stalls/flushes)
4. **Performance Summary Bar Chart**: Overview of key metrics

## Installation

### Requirements

- Python 3.6 or higher
- matplotlib (optional, for visualizations)

```bash
# Install matplotlib for visualizations
pip install matplotlib
```

## Usage

### Basic Usage

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

### Command-Line Options

| Option | Short | Description |
|--------|-------|-------------|
| `--report` | `-r` | Output file for text report |
| `--visualize` | `-v` | Generate visualization charts |
| `--output-dir` | `-o` | Output directory for visualizations (default: `reports/`) |

## Log File Format

The parser recognizes various log formats and extracts metrics from patterns like:

### Performance Monitor Output

```
Total Cycles: 1250
Instructions Completed: 1000
Pipeline Stalls: 150
Pipeline Flushes: 100
CPI: 1.25
Load Instructions: 200
Store Instructions: 150
Branch Instructions: 100
```

### Instruction Execution Logs

The parser recognizes instruction patterns in logs:

- **Arithmetic**: ADD, SUB, ADDI, SUBI, MUL, DIV, REM
- **Logical**: AND, OR, XOR, ANDI, ORI, XORI, SLL, SRL, SRA
- **Memory**: LW, LH, LB, LHU, LBU, SW, SH, SB
- **Control Flow**: BEQ, BNE, BLT, BGE, BLTU, BGEU, JAL, JALR

### Cycle Information

The parser extracts cycle counts from patterns like:
- `Cycle: 1234`
- `Cycle 1234`
- `Time: 1234 ns | Cycle: 1234`

## Output Format

### Text Report

```
======================================================================
RISC-V Processor Performance Report
======================================================================

Overall Performance Metrics:
----------------------------------------------------------------------
  Total Cycles:                        1,250
  Instructions Completed:              1,000
  CPI (Cycles Per Instruction):        1.2500

Pipeline Efficiency:
----------------------------------------------------------------------
  Pipeline Stalls:                       150
  Stall Percentage:                  12.00%
  Pipeline Flushes:                      100
  Flush Percentage:                   8.00%
  Pipeline Efficiency:                80.00%

Memory Operations:
----------------------------------------------------------------------
  Load Instructions:                     200
  Store Instructions:                    150
  Total Memory Operations:               350

Control Flow:
----------------------------------------------------------------------
  Branch Instructions:                   100

Instruction Mix:
----------------------------------------------------------------------
  Arithmetic Instructions:               400  (40.00%)
  Logical Instructions:                 200  (20.00%)
  Memory Instructions:                   350  (35.00%)
  Control Flow Instructions:             100  (10.00%)
  Other Instructions:                     50  ( 5.00%)
======================================================================
```

### Visualization Files

When using `--visualize`, the script generates:

1. **`instruction_mix.png`**: Bar chart showing instruction type distribution
2. **`cpi_over_time.png`**: Line graph showing CPI changes during execution
3. **`pipeline_efficiency.png`**: Pie chart showing cycle distribution
4. **`performance_summary.png`**: Bar chart with key performance metrics

## Integration with Testbenches

### Example Testbench Integration

Add performance monitoring output to your testbench:

```systemverilog
// In testbench
initial begin
    // ... simulation code ...
    
    // At end of simulation, display performance metrics
    $display("=== Performance Metrics ===");
    $display("Total Cycles: %d", perf_total_cycles);
    $display("Instructions Completed: %d", perf_instructions_completed);
    $display("Pipeline Stalls: %d", perf_pipeline_stalls);
    $display("Pipeline Flushes: %d", perf_pipeline_flushes);
    $display("Load Instructions: %d", perf_load_instructions);
    $display("Store Instructions: %d", perf_store_instructions);
    $display("Branch Instructions: %d", perf_branch_instructions);
    
    real cpi;
    cpi = $itor(perf_cpi_value) / $itor(65536.0);
    $display("CPI: %.4f", cpi);
end
```

### Redirecting Log Output

```bash
# Run simulation and save log
vsim -c -do "run -all; quit" > simulation.log 2>&1

# Parse log file
python scripts/parse_performance_logs.py simulation.log --visualize
```

## Performance Analysis

### Interpreting Metrics

1. **CPI (Cycles Per Instruction)**:
   - Ideal CPI = 1.0 (one instruction per cycle)
   - CPI > 1.0 indicates performance loss
   - Lower CPI = better performance

2. **Stall Percentage**:
   - Percentage of cycles lost to data hazards
   - High stall rate may indicate memory-bound workload
   - Target: < 10%

3. **Flush Percentage**:
   - Percentage of cycles lost to control hazards
   - High flush rate indicates poor branch prediction
   - Target: < 5%

4. **Pipeline Efficiency**:
   - Efficiency = 1.0 / CPI
   - Efficiency = 100% when CPI = 1.0
   - Target: > 80%

5. **Instruction Mix**:
   - Balanced mix indicates diverse workload
   - High memory percentage may indicate memory-bound workload
   - High control flow percentage indicates complex control flow

### Optimization Tips

1. **Reduce CPI**:
   - Minimize stalls and flushes
   - Optimize instruction scheduling
   - Improve branch prediction

2. **Reduce Stalls**:
   - Optimize code to minimize load-use hazards
   - Use register allocation to avoid data dependencies
   - Consider instruction reordering

3. **Reduce Flushes**:
   - Optimize branch patterns
   - Use conditional moves instead of branches
   - Consider branch prediction improvements

## Examples

### Example 1: Basic Report

```bash
python scripts/parse_performance_logs.py simulation.log
```

Output: Text report displayed to console

### Example 2: Report with Visualizations

```bash
python scripts/parse_performance_logs.py simulation.log \
    --visualize \
    --output-dir performance_reports/
```

Output: Text report + 4 PNG charts in `performance_reports/` directory

### Example 3: Save Report to File

```bash
python scripts/parse_performance_logs.py simulation.log \
    --report performance_report.txt \
    --visualize
```

Output: Text report saved to file + visualizations

## Troubleshooting

### No Metrics Found

If the parser doesn't find metrics:
- Check log file format matches expected patterns
- Verify performance monitor is enabled in testbench
- Check log file encoding (should be UTF-8)

### Missing Visualizations

If visualizations are not generated:
- Install matplotlib: `pip install matplotlib`
- Check output directory permissions
- Verify log file contains sufficient data

### Incorrect Instruction Counts

If instruction counts seem incorrect:
- Check log file contains instruction execution traces
- Verify instruction patterns match your log format
- Consider customizing instruction patterns in script

## Customization

### Adding Custom Instruction Patterns

Edit `parse_performance_logs.py` to add custom patterns:

```python
# Add to ARITHMETIC_PATTERNS list
ARITHMETIC_PATTERNS = [
    r'\bADD\b', r'\bSUB\b',
    r'\bCUSTOM_INSTR\b',  # Add custom instruction
    # ...
]
```

### Customizing Visualization Styles

Modify visualization functions in `generate_visualizations()`:
- Change colors
- Adjust chart sizes
- Modify labels and formatting

## See Also

- `scripts/parse_performance_logs.py`: Parser script source code
- `src/performance_monitor.sv`: Performance monitor module
- `docs/PERFORMANCE_MONITOR.md`: Performance monitor documentation

