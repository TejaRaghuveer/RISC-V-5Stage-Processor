# Golden Reference Test Generator

A Python script that generates RISC-V instruction sequences with simulated execution results (golden reference). The script tracks register file and memory state changes to produce expected outputs that can be compared against processor simulation results.

## Features

- **Instruction Simulation**: Simulates RISC-V instruction execution
- **State Tracking**: Tracks register file and memory state changes
- **Golden Reference**: Generates expected final state for comparison
- **Multiple Output Formats**: Hex machine code, assembly with comments, SystemVerilog reference
- **Execution Trace**: Detailed trace of instruction execution flow

## Usage

### Basic Usage

```bash
# Generate test with default settings
python scripts/generate_golden_test.py --length 50 --seed 42

# Specify output files
python scripts/generate_golden_test.py \
    --length 100 \
    --seed 123 \
    --hex mem/test.hex \
    --ref mem/test_ref.sv \
    --asm mem/test.s
```

### Command-Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `--length N` | Number of instructions to generate | 20 |
| `--seed N` | Random seed for reproducibility | Random |
| `--hex FILE` | Output hex file path | `mem/golden_test.hex` |
| `--ref FILE` | Output reference file path | `mem/golden_test_ref.sv` |
| `--asm FILE` | Output assembly file path | `mem/golden_test.s` |

## Output Files

### 1. Hex File (`*.hex`)

Machine code in hex format suitable for loading into instruction memory:

```
// RISC-V Test Program with Golden Reference
// Generated with seed: 42
// Format: One 32-bit instruction per line (8 hex digits)

00A00093  // ADDI - x1 = x0 + 10 = 0 + 10 = 10
01400113  // ADDI - x2 = x0 + 20 = 0 + 20 = 20
002081B3  // ADD - x3 = x1 + x2 = 10 + 20 = 30
...
```

### 2. Assembly File (`*.s`)

Assembly source with detailed execution comments:

```assembly
# RISC-V Assembly Test Program
# Generated with seed: 42
# Expected execution flow and results:

    ADDI x1, x0, 10  # PC=0x0: x1 = x0 + 10 = 0 + 10 = 10
    ADDI x2, x0, 20  # PC=0x4: x2 = x0 + 20 = 0 + 20 = 20
    ADD x3, x1, x2   # PC=0x8: x3 = x1 + x2 = 10 + 20 = 30
    ...
```

### 3. Reference File (`*_ref.sv`)

SystemVerilog-compatible golden reference:

```systemverilog
// Golden Reference Results
// Generated with seed: 42
// Format: SystemVerilog compatible for testbench comparison

// Expected Final Register File State
// x0 = 0 (0x00000000)
// x1 = 10 (0x0000000A)
// x2 = 20 (0x00000014)
// x3 = 30 (0x0000001E)
...

// SystemVerilog Format (for testbench)
logic [31:0] expected_registers [0:31];
assign expected_registers[0] = 32'h00000000;  // x0
assign expected_registers[1] = 32'h0000000A;  // x1
assign expected_registers[2] = 32'h00000014;  // x2
...
```

## Testbench Integration

### Example SystemVerilog Testbench

```systemverilog
module riscv_golden_test_tb;
    // Include golden reference
    `include "mem/golden_test_ref.sv"
    
    // Testbench signals
    logic clk, rst_n;
    logic [31:0] registers [0:31];
    logic [31:0] memory [0:1023];
    
    // DUT instantiation
    riscv_pipeline uut (
        .clk(clk),
        .rst_n(rst_n),
        // ... other signals
    );
    
    // Load instruction memory
    initial begin
        $readmemh("mem/golden_test.hex", uut.if_stage_inst.imem_inst.memory);
    end
    
    // Compare results
    task check_results();
        int errors = 0;
        
        // Check registers
        for (int i = 0; i < 32; i++) begin
            if (uut.register_file_inst.registers[i] != expected_registers[i]) begin
                $error("Register x%d mismatch: expected 0x%08X, got 0x%08X",
                       i, expected_registers[i], uut.register_file_inst.registers[i]);
                errors++;
            end
        end
        
        // Check memory
        for (int i = 0; i < 1024; i++) begin
            if (uut.dmem_inst.memory[i] != expected_memory[i]) begin
                $error("Memory[%d] mismatch: expected 0x%08X, got 0x%08X",
                       i, expected_memory[i], uut.dmem_inst.memory[i]);
                errors++;
            end
        end
        
        if (errors == 0) begin
            $display("PASS: All comparisons match golden reference");
        end else begin
            $error("FAIL: %d mismatches found", errors);
        end
    endtask
    
    // Run test
    initial begin
        // Reset
        rst_n = 0;
        #100;
        rst_n = 1;
        
        // Run simulation
        #10000;
        
        // Check results
        check_results();
        
        $finish;
    end
endmodule
```

## Supported Instructions

### Arithmetic (R-type)
- ADD, SUB

### Logical (R-type)
- AND, OR, XOR

### Immediate (I-type)
- ADDI, ANDI, ORI, XORI

### Memory Operations
- LW (Load Word)
- SW (Store Word)

### Control Flow
- BEQ, BNE, BLT, BGE, BLTU, BGEU (Branch)
- JAL (Jump and Link)
- JALR (Jump and Link Register)

## Execution Flow Comments

Each instruction includes a comment showing:
- Program counter value
- Instruction execution result
- Register/memory state changes
- Branch/jump outcomes

Example:
```assembly
ADD x3, x1, x2  # PC=0x8: x3 = x1 + x2 = 10 + 20 = 30
BEQ x1, x2, 16  # PC=0xC: Branch NOT TAKEN, target=0x1C
```

## Limitations

1. **Simple Simulation**: Does not model pipeline effects (hazards, forwarding)
2. **Sequential Execution**: Assumes instructions execute sequentially
3. **No Structural Hazards**: Assumes no resource conflicts
4. **Limited Instruction Set**: Supports subset of RV32I instructions

## Use Cases

1. **Functional Verification**: Verify processor executes instructions correctly
2. **Regression Testing**: Generate reproducible test sequences
3. **Debugging**: Compare processor output against known-good reference
4. **Documentation**: Generate annotated assembly showing expected behavior

## Example Workflow

1. **Generate Test**:
   ```bash
   python scripts/generate_golden_test.py --length 100 --seed 42
   ```

2. **Load in Testbench**:
   ```systemverilog
   $readmemh("mem/golden_test.hex", imem.memory);
   `include "mem/golden_test_ref.sv"
   ```

3. **Run Simulation**:
   ```bash
   vsim -do "run -all; check_results; quit"
   ```

4. **Compare Results**:
   - Testbench automatically compares register and memory values
   - Reports mismatches with detailed error messages

## Requirements

- Python 3.6 or higher
- Standard library only (no external dependencies)

