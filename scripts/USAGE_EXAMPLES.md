# Random Instruction Generator Usage Examples

## Quick Start

### Generate Basic Test Sequence

```bash
# Generate 100 random instructions
python scripts/generate_random_test.py --length 100 --output mem/random_test.hex
```

### Generate Reproducible Test

```bash
# Use seed for reproducibility
python scripts/generate_random_test.py --length 200 --seed 12345 --output mem/test.hex
```

## Example Configurations

### 1. Arithmetic-Heavy Test

Focuses on arithmetic and logical operations:

```bash
python scripts/generate_random_test.py \
    --length 500 \
    --r-type 35 \
    --i-type 35 \
    --load 8 \
    --store 8 \
    --branch 10 \
    --jal 2 \
    --jalr 2 \
    --seed 100 \
    --output mem/arithmetic_heavy.hex
```

**Use Case**: Test ALU operations, forwarding, and arithmetic hazards

### 2. Control-Flow Heavy Test

Emphasizes branches and jumps:

```bash
python scripts/generate_random_test.py \
    --length 500 \
    --r-type 10 \
    --i-type 15 \
    --load 5 \
    --store 5 \
    --branch 45 \
    --jal 15 \
    --jalr 5 \
    --seed 200 \
    --output mem/controlflow_heavy.hex
```

**Use Case**: Test branch prediction, pipeline flushing, and control hazards

### 3. Memory-Intensive Test

High percentage of load/store operations:

```bash
python scripts/generate_random_test.py \
    --length 500 \
    --r-type 10 \
    --i-type 10 \
    --load 35 \
    --store 35 \
    --branch 5 \
    --jal 3 \
    --jalr 2 \
    --seed 300 \
    --output mem/memory_heavy.hex
```

**Use Case**: Test memory operations, load-use hazards, and data memory

### 4. Balanced Test

Even distribution of instruction types:

```bash
python scripts/generate_random_test.py \
    --length 1000 \
    --r-type 15 \
    --i-type 20 \
    --load 15 \
    --store 15 \
    --branch 20 \
    --jal 8 \
    --jalr 7 \
    --seed 400 \
    --output mem/balanced_test.hex
```

**Use Case**: General processor stress testing

### 5. Using Configuration File

Create a config file `my_config.txt`:

```
R_TYPE: 25
I_TYPE: 30
LOAD: 12
STORE: 12
BRANCH: 15
JAL: 3
JALR: 3
```

Then run:

```bash
python scripts/generate_random_test.py \
    --length 500 \
    --config my_config.txt \
    --seed 500 \
    --output mem/custom_test.hex
```

## Integration Examples

### SystemVerilog Testbench

```systemverilog
module riscv_pipeline_tb;
    // ... testbench code ...
    
    initial begin
        // Load random instruction sequence
        $readmemh("mem/random_test.hex", uut.if_stage_inst.imem_inst.memory);
        
        // Run simulation
        #1000;
        $finish;
    end
endmodule
```

### Batch Generation Script

Create `generate_tests.sh`:

```bash
#!/bin/bash

# Generate multiple test sequences
for seed in {1..10}; do
    python scripts/generate_random_test.py \
        --length 200 \
        --seed $seed \
        --output mem/random_test_${seed}.hex
done
```

### Python Test Script

```python
import subprocess
import os

# Generate test suite
test_configs = [
    {"name": "arithmetic", "r_type": 40, "i_type": 30, "length": 200},
    {"name": "controlflow", "branch": 40, "jal": 15, "length": 200},
    {"name": "memory", "load": 30, "store": 30, "length": 200},
]

for config in test_configs:
    cmd = ["python", "scripts/generate_random_test.py"]
    cmd.extend(["--length", str(config["length"])])
    cmd.extend(["--output", f"mem/{config['name']}_test.hex"])
    
    for key, value in config.items():
        if key not in ["name", "length"]:
            cmd.extend([f"--{key.replace('_', '-')}", str(value)])
    
    subprocess.run(cmd)
```

## Expected Output

The script generates output like:

```
Generating 100 random RISC-V instructions...
Instruction mix: {'R_TYPE': 20, 'I_TYPE': 25, 'LOAD': 10, 'STORE': 10, 'BRANCH': 20, 'JAL': 5, 'JALR': 10}
Random seed: 42
Generated 100 instructions
Output written to: mem/random_test.hex

Instruction Statistics:
  ADD     :   12 ( 12.0%)
  ADDI    :   15 ( 15.0%)
  AND     :    4 (  4.0%)
  ANDI    :    6 (  6.0%)
  BEQ     :    5 (  5.0%)
  BGE     :    3 (  3.0%)
  BGEU    :    2 (  2.0%)
  BLT     :    4 (  4.0%)
  BLTU    :    3 (  3.0%)
  BNE     :    3 (  3.0%)
  JAL     :    5 (  5.0%)
  JALR    :   10 ( 10.0%)
  LW      :   10 ( 10.0%)
  OR      :    2 (  2.0%)
  ORI     :    4 (  4.0%)
  SUB     :    4 (  4.0%)
  SW      :   10 ( 10.0%)
  XOR     :    2 (  2.0%)
  XORI    :    1 (  1.0%)
```

## Tips

1. **Start Small**: Generate short sequences (50-100 instructions) first to verify output
2. **Use Seeds**: Always use `--seed` for reproducible tests
3. **Check Statistics**: Review the instruction statistics to verify mix percentages
4. **Validate Output**: Load the hex file into your simulator to verify it works
5. **Iterate**: Adjust percentages based on what you want to test

## Troubleshooting

### Script Not Found
```bash
# Make sure you're in the project root directory
cd "path/to/RISC V 5 stage pipeline"
python scripts/generate_random_test.py --help
```

### Import Errors
The script uses only Python standard library. If you get import errors, check your Python version (requires 3.6+).

### Output File Not Created
Check that the output directory exists:
```bash
mkdir -p mem
python scripts/generate_random_test.py --output mem/test.hex
```

