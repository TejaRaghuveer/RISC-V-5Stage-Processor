# Performance Monitor Module

## Overview

The `performance_monitor` module provides comprehensive performance monitoring capabilities for the RISC-V 5-stage pipeline processor. It tracks various performance metrics including cycle counts, instruction completion, pipeline stalls, flushes, and calculates CPI (Cycles Per Instruction).

## Features

### Performance Metrics Tracked

1. **Total Cycles**: Total clock cycles elapsed during execution
2. **Instructions Completed**: Number of instructions that completed writeback (excluding x0 writes)
3. **Pipeline Stalls**: Cycles lost due to load-use hazards
4. **Pipeline Flushes**: Cycles lost due to branch/jump misprediction
5. **Load Instructions**: Count of load instructions executed
6. **Store Instructions**: Count of store instructions executed
7. **Branch Instructions**: Count of branch instructions executed
8. **CPI (Cycles Per Instruction)**: Average cycles per instruction in fixed-point format

## Module Interface

### Parameters

- `COUNTER_WIDTH` (default: 32): Width of performance counters
- `CPI_WIDTH` (default: 16): Width of CPI fractional part (for fixed-point representation)

### Inputs

- `clk`: Clock signal
- `rst_n`: Active-low reset
- `enable`: Enable performance monitoring (active high)
- `wb_RegWrite`: Register write enable (indicates instruction completion)
- `wb_rd_addr`: Destination register address (to exclude x0)
- `pipeline_stall`: Pipeline stall signal
- `pipeline_flush`: Pipeline flush signal
- `mem_MemRead`: Memory read enable (load instruction)
- `mem_MemWrite`: Memory write enable (store instruction)
- `mem_Branch`: Branch instruction indicator

### Outputs

- `total_cycles`: Total clock cycles counted
- `instructions_completed`: Instructions completed (WB commits)
- `pipeline_stalls`: Pipeline stall cycles
- `pipeline_flushes`: Pipeline flush cycles
- `load_instructions`: Load instruction count
- `store_instructions`: Store instruction count
- `branch_instructions`: Branch instruction count
- `cpi_value`: CPI in fixed-point format (CPI * 2^CPI_WIDTH)

## Integration Example

### Basic Integration in Pipeline

```systemverilog
module riscv_pipeline_with_monitor #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    // ... other parameters
) (
    input  logic clk,
    input  logic rst_n,
    // ... other inputs
    // Performance monitor outputs (optional)
    output logic [31:0] perf_total_cycles,
    output logic [31:0] perf_instructions_completed,
    output logic [31:0] perf_cpi_value
);

    // ... existing pipeline signals ...
    
    // Performance Monitor Instantiation
    performance_monitor #(
        .COUNTER_WIDTH(32),
        .CPI_WIDTH(16)
    ) perf_monitor_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(1'b1),                      // Enable monitoring
        .wb_RegWrite(wb_RegWrite),          // From MEM/WB register
        .wb_rd_addr(wb_rd_addr),            // From MEM/WB register
        .pipeline_stall(pipeline_stall_internal),  // Combined stall signal
        .pipeline_flush(pipeline_flush_internal),  // Combined flush signal
        .mem_MemRead(mem_MemRead),          // From EX/MEM register
        .mem_MemWrite(mem_MemWrite),        // From EX/MEM register
        .mem_Branch(mem_Branch),            // From EX/MEM register
        .total_cycles(perf_total_cycles),
        .instructions_completed(perf_instructions_completed),
        .pipeline_stalls(),
        .pipeline_flushes(),
        .load_instructions(),
        .store_instructions(),
        .branch_instructions(),
        .cpi_value(perf_cpi_value)
    );
    
    // ... rest of pipeline ...
    
endmodule
```

### Signal Connections

The performance monitor requires connections to various pipeline stages:

1. **WB Stage Signals** (for instruction completion):
   - `wb_RegWrite`: From MEM/WB pipeline register
   - `wb_rd_addr`: From MEM/WB pipeline register

2. **Pipeline Control Signals** (for stall/flush counting):
   - `pipeline_stall`: Combined stall signal (`hazard_stall || pipeline_stall`)
   - `pipeline_flush`: Combined flush signal (`mem_branch_flush || pipeline_flush`)

3. **MEM Stage Signals** (for load/store/branch counting):
   - `mem_MemRead`: From EX/MEM pipeline register
   - `mem_MemWrite`: From EX/MEM pipeline register
   - `mem_Branch`: From EX/MEM pipeline register

## CPI Calculation

### Fixed-Point Format

CPI is calculated using fixed-point arithmetic to avoid floating-point division:

```
CPI = (total_cycles << CPI_WIDTH) / instructions_completed
```

With `CPI_WIDTH = 16`:
- CPI = 1.0 → `cpi_value = 65536` (0x10000)
- CPI = 1.5 → `cpi_value = 98304` (0x18000)
- CPI = 2.0 → `cpi_value = 131072` (0x20000)

### Converting CPI Value

To get the actual CPI value:

```systemverilog
real cpi_real;
assign cpi_real = $itor(cpi_value) / $itor(1 << CPI_WIDTH);
```

Or in C/software:

```c
double cpi = (double)cpi_value / (1 << CPI_WIDTH);
```

## Usage in Testbench

### Example Testbench Integration

```systemverilog
module pipeline_tb;

    // ... testbench signals ...
    
    // Performance monitor outputs
    logic [31:0] perf_total_cycles;
    logic [31:0] perf_instructions_completed;
    logic [31:0] perf_pipeline_stalls;
    logic [31:0] perf_pipeline_flushes;
    logic [31:0] perf_load_instructions;
    logic [31:0] perf_store_instructions;
    logic [31:0] perf_branch_instructions;
    logic [47:0] perf_cpi_value;  // 32 + 16 = 48 bits
    
    // Instantiate pipeline with performance monitor
    riscv_pipeline_with_monitor uut (
        .clk(clk),
        .rst_n(rst_n),
        // ... other connections ...
        .perf_total_cycles(perf_total_cycles),
        .perf_instructions_completed(perf_instructions_completed),
        .perf_cpi_value(perf_cpi_value)
    );
    
    // Monitor performance metrics
    initial begin
        wait(perf_instructions_completed > 0);
        $display("=== Performance Metrics ===");
        $display("Total Cycles: %d", perf_total_cycles);
        $display("Instructions Completed: %d", perf_instructions_completed);
        $display("Pipeline Stalls: %d", perf_pipeline_stalls);
        $display("Pipeline Flushes: %d", perf_pipeline_flushes);
        $display("Load Instructions: %d", perf_load_instructions);
        $display("Store Instructions: %d", perf_store_instructions);
        $display("Branch Instructions: %d", perf_branch_instructions);
        
        // Calculate and display CPI
        real cpi;
        cpi = $itor(perf_cpi_value) / $itor(65536.0);
        $display("CPI: %.2f", cpi);
        
        // Calculate efficiency metrics
        real stall_rate, flush_rate, efficiency;
        stall_rate = $itor(perf_pipeline_stalls) / $itor(perf_total_cycles);
        flush_rate = $itor(perf_pipeline_flushes) / $itor(perf_total_cycles);
        efficiency = 1.0 / cpi;
        
        $display("Stall Rate: %.2f%%", stall_rate * 100.0);
        $display("Flush Rate: %.2f%%", flush_rate * 100.0);
        $display("Pipeline Efficiency: %.2f%%", efficiency * 100.0);
    end
    
endmodule
```

## Performance Analysis

### Interpreting Metrics

1. **CPI (Cycles Per Instruction)**:
   - Ideal CPI = 1.0 (one instruction per cycle)
   - CPI > 1.0 indicates performance loss due to stalls/flushes
   - Lower CPI = better performance

2. **Pipeline Efficiency**:
   - Efficiency = 1.0 / CPI
   - Efficiency = 100% when CPI = 1.0
   - Efficiency decreases with stalls and flushes

3. **Stall Rate**:
   - Stall Rate = pipeline_stalls / total_cycles
   - Indicates percentage of cycles lost to data hazards
   - High stall rate may indicate memory-bound workload

4. **Flush Rate**:
   - Flush Rate = pipeline_flushes / total_cycles
   - Indicates percentage of cycles lost to control hazards
   - High flush rate may indicate complex control flow

5. **Instruction Mix**:
   - Load/Store ratio indicates memory access patterns
   - Branch instruction count indicates control flow complexity
   - High branch count with high flush rate indicates poor branch prediction

### Performance Optimization Tips

1. **Reduce Stalls**:
   - Optimize code to minimize load-use hazards
   - Use register allocation to avoid data dependencies
   - Consider instruction scheduling

2. **Reduce Flushes**:
   - Optimize branch patterns
   - Use conditional moves instead of branches when possible
   - Consider branch prediction improvements

3. **Improve CPI**:
   - Minimize stalls and flushes
   - Optimize instruction mix
   - Consider pipeline depth optimization

## Limitations

1. **Counter Overflow**: Counters are 32-bit by default. For very long simulations, consider increasing `COUNTER_WIDTH`.

2. **CPI Precision**: CPI is calculated using fixed-point arithmetic with 16-bit fractional part. Precision is limited to ~0.000015 (1/65536).

3. **Division by Zero**: CPI calculation handles division by zero (returns 0 when no instructions completed).

4. **Enable Signal**: Monitoring only occurs when `enable` is high. Ensure enable is asserted during measurement period.

## Example Output

```
=== Performance Metrics ===
Total Cycles: 1250
Instructions Completed: 1000
Pipeline Stalls: 150
Pipeline Flushes: 100
Load Instructions: 200
Store Instructions: 150
Branch Instructions: 100
CPI: 1.25
Stall Rate: 12.00%
Flush Rate: 8.00%
Pipeline Efficiency: 80.00%
```

## See Also

- `src/performance_monitor.sv`: Performance monitor module source code
- `src/riscv_pipeline.sv`: Pipeline top-level module
- `docs/ARCHITECTURE.md`: Pipeline architecture documentation

