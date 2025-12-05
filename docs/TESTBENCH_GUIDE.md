# RISC-V Pipeline Testbench Guide

## Overview

The `riscv_pipeline_tb.sv` testbench provides a comprehensive testing framework for the RISC-V 5-stage pipelined processor. It includes clock generation, reset sequences, memory initialization, register monitoring, and result verification capabilities.

## Features

- **50MHz Clock Generation**: 20ns clock period for simulation
- **Reset Sequence**: Proper reset timing with configurable cycles
- **Memory Initialization**: Instruction memory loaded from hex file
- **Register Monitoring**: Display and dump register file contents
- **Data Memory Monitoring**: Display and dump data memory contents
- **Simulation Control**: Configurable cycle count and timeout protection
- **VCD Waveform Dump**: Automatic waveform generation for debugging
- **Self-Checking Framework**: Pass/fail reporting for test verification

## File Structure

```
tb/
├── riscv_pipeline_tb.sv    # Main testbench file
└── ...

mem/
├── inst_mem.hex             # Instruction memory initialization file
└── ...

docs/
└── TESTBENCH_GUIDE.md      # This file
```

## Quick Start

### 1. Prepare Test Program

Create a hex file containing your RISC-V instructions in `mem/inst_mem.hex`:

```
00000013  // ADDI x0, x0, 0 (NOP)
00100093  // ADDI x1, x0, 1
00200113  // ADDI x2, x0, 2
00300193  // ADDI x3, x0, 3
```

**Format**: Each line contains one 32-bit instruction in hex (8 hex digits).

### 2. Configure Testbench Parameters

Edit `riscv_pipeline_tb.sv` and adjust parameters as needed:

```systemverilog
parameter SIM_CYCLES = 1000;              // Number of cycles to simulate
parameter IMEM_INIT_FILE = "mem/inst_mem.hex";  // Your hex file path
```

### 3. Set Expected Results (Optional)

In the main test procedure, set expected register values:

```systemverilog
// Set expected results based on your test program
expected_regs[1] = 32'h00000001;  // x1 should be 1
expected_regs[2] = 32'h00000002;  // x2 should be 2
expected_regs[3] = 32'h00000003;  // x3 should be 3
```

### 4. Run Simulation

Using ModelSim:
```bash
vlog -sv tb/riscv_pipeline_tb.sv src/*.sv
vsim -c riscv_pipeline_tb -do "run -all; quit"
```

Using Icarus Verilog:
```bash
iverilog -g2012 -o riscv_pipeline_tb tb/riscv_pipeline_tb.sv src/*.sv
vvp riscv_pipeline_tb
```

### 5. View Results

- **Console Output**: Test results and register states printed to console
- **VCD Waveform**: Open `riscv_pipeline_tb.vcd` in GTKWave or ModelSim
- **Register Dump**: Check `reg_dump.txt` for register contents
- **Memory Dump**: Check `dmem_dump.txt` for data memory contents

## Testbench Tasks

### `reset_sequence(int cycles)`

Performs a proper reset sequence:

```systemverilog
reset_sequence(5);  // Reset for 5 clock cycles
```

### `load_test_program(string hex_file)`

Loads a test program (displays information):

```systemverilog
load_test_program("mem/my_program.hex");
```

### `display_register_state()`

Displays formatted register file contents:

```systemverilog
display_register_state();
```

Output example:
```
========================================
Register File State
========================================
Reg | Name | Hex Value      | Decimal Value
----------------------------------------
x0  | zero | 0x00000000 | 0
x1  | ra   | 0x00000001 | 1 <--
x2  | sp   | 0x00000002 | 2 <--
...
```

### `dump_registers(string file_name)`

Dumps register contents to a file:

```systemverilog
dump_registers("my_reg_dump.txt");
```

### `dump_data_memory(string file_name, int start_addr, int end_addr)`

Dumps data memory contents to a file:

```systemverilog
dump_data_memory("my_dmem_dump.txt", 0, 255);  // Dump addresses 0-255
```

### `check_expected_results(logic [31:0] expected_regs [31:0], logic [31:0] reg_mask)`

Compares actual register values with expected values:

```systemverilog
// Set expected values
expected_regs[1] = 32'h00000001;
expected_regs[2] = 32'h00000002;

// Check all registers
check_expected_results(expected_regs);

// Or check specific registers using mask
check_expected_results(expected_regs, 32'h00000007);  // Check x0, x1, x2
```

### `run_simulation(int cycles)`

Runs simulation for specified number of cycles:

```systemverilog
run_simulation(500);  // Run for 500 clock cycles
```

### `monitor_pipeline_activity()`

Monitors pipeline activity (requires debug ports):

```systemverilog
monitor_pipeline_activity();
```

### `print_test_summary()`

Prints test summary with pass/fail counts:

```systemverilog
print_test_summary();
```

## Customizing the Testbench

### Adding Debug Ports

To access register file and memory contents directly, add debug ports to your modules:

**Register File Debug Port** (`src/reg_file.sv`):
```systemverilog
// Add to module port list
output logic [DATA_WIDTH-1:0] debug_regs [NUM_REGS-1:0]  // Debug port

// Add to module body
assign debug_regs = registers;
```

**Memory Debug Port** (`src/dmem.sv`):
```systemverilog
// Add to module port list
output logic [DATA_WIDTH-1:0] debug_memory [MEM_DEPTH-1:0]  // Debug port

// Add to module body
assign debug_memory = memory;
```

Then connect these through the pipeline hierarchy and access in testbench:
```systemverilog
// In testbench
reg_value = dut.debug_regs[i];
mem_value = dut.debug_memory[i];
```

### Periodic Monitoring

Add periodic monitoring during simulation:

```systemverilog
for (int i = 0; i < SIM_CYCLES; i++) begin
    @(posedge clk);
    
    // Monitor every 50 cycles
    if (i % 50 == 0 && i > 0) begin
        display_register_state();
    end
    
    // Check for specific conditions
    if (cycle_count == 100) begin
        $display("Reached cycle 100 - checking intermediate results");
        check_expected_results(expected_regs, 32'h00000007);
    end
end
```

### Custom Test Sequences

Create custom test sequences:

```systemverilog
task my_custom_test();
    $display("Starting custom test...");
    
    // Load specific program
    load_test_program("mem/custom_test.hex");
    
    // Reset
    reset_sequence(5);
    
    // Run for specific cycles
    run_simulation(200);
    
    // Check results
    expected_regs[10] = 32'h00000042;  // Expected value in a0
    check_expected_results(expected_regs, 32'h00000400);  // Check a0 only
    
    // Dump everything
    dump_registers("custom_test_regs.txt");
    dump_data_memory("custom_test_dmem.txt");
    
    $display("Custom test complete!");
endtask
```

## Example Test Scenarios

### Basic Arithmetic Test

```systemverilog
// Test: ADD x1, x2, x3  (x1 = x2 + x3)
// Expected: x1 = 5 (if x2=2, x3=3)

expected_regs[1] = 32'h00000005;  // Expected result in x1
expected_regs[2] = 32'h00000002;  // x2 = 2
expected_regs[3] = 32'h00000003;  // x3 = 3

run_simulation(20);  // Run enough cycles for instruction to complete
check_expected_results(expected_regs, 32'h0000000E);  // Check x1, x2, x3
```

### Load/Store Test

```systemverilog
// Test: Load from memory, store to memory
// Expected: Data loaded into register, then stored back

run_simulation(50);  // Run enough cycles

// Check register has loaded value
expected_regs[5] = 32'hDEADBEEF;  // Expected loaded value
check_expected_results(expected_regs, 32'h00000020);  // Check x5

// Dump memory to verify store
dump_data_memory("load_store_dmem.txt");
```

### Branch Test

```systemverilog
// Test: Branch instruction (BEQ, BNE, etc.)
// Expected: PC updates correctly, pipeline flushes

run_simulation(30);  // Run enough cycles

// Check that branch was taken (PC should be at target)
// Note: Requires PC monitoring via debug ports
display_register_state();
```

## Troubleshooting

### Issue: Register values show as 0x00000000

**Solution**: Add debug ports to register file and connect through pipeline hierarchy. See "Adding Debug Ports" section above.

### Issue: Memory contents not accessible

**Solution**: Add debug ports to memory modules or use hierarchical access. See "Adding Debug Ports" section above.

### Issue: Simulation runs too long

**Solution**: Adjust `SIM_CYCLES` parameter or add timeout conditions:

```systemverilog
if (cycle_count > MAX_CYCLES) begin
    $display("Maximum cycles reached, stopping simulation");
    $finish;
end
```

### Issue: VCD file too large

**Solution**: Reduce dump scope or use selective dumping:

```systemverilog
// Dump only specific signals
$dumpvars(0, riscv_pipeline_tb.dut.if_stage_inst);
$dumpvars(0, riscv_pipeline_tb.dut.id_stage_inst);
```

## Best Practices

1. **Start Small**: Begin with simple test programs (NOPs, basic arithmetic)
2. **Incremental Testing**: Test one instruction type at a time
3. **Use Assertions**: Add assertions for illegal states
4. **Monitor Pipeline**: Use periodic monitoring to track instruction flow
5. **Document Expected Results**: Always document what your test program should produce
6. **Use Waveforms**: View VCD waveforms to understand pipeline behavior
7. **Check Timing**: Verify that results appear at correct cycle counts

## Advanced Usage

### Conditional Test Execution

```systemverilog
if (ENABLE_BRANCH_TEST) begin
    load_test_program("mem/branch_test.hex");
    run_simulation(100);
    // Check branch results
end
```

### Multiple Test Programs

```systemverilog
string test_programs [3] = {
    "mem/test1.hex",
    "mem/test2.hex",
    "mem/test3.hex"
};

for (int i = 0; i < 3; i++) begin
    $display("Running test %0d: %s", i, test_programs[i]);
    load_test_program(test_programs[i]);
    reset_sequence(5);
    run_simulation(200);
    dump_registers($sformatf("test%0d_regs.txt", i));
end
```

### Performance Monitoring

```systemverilog
real start_time, end_time;
int start_cycle, end_cycle;

start_time = $realtime;
start_cycle = cycle_count;

// Run test
run_simulation(1000);

end_time = $realtime;
end_cycle = cycle_count;

$display("Test Performance:");
$display("  Cycles: %0d", end_cycle - start_cycle);
$display("  Time: %0.2f ns", end_time - start_time);
$display("  CPI: %0.2f", (end_time - start_time) / ((end_cycle - start_cycle) * CLK_PERIOD));
```

## References

- RISC-V Instruction Set Manual: https://riscv.org/specifications/
- SystemVerilog LRM: IEEE 1800-2017
- Testbench examples in `tb/` directory

