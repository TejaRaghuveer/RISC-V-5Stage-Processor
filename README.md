# RISC-V 5-stage pipelined processor (RV32I) in SystemVerilog

A 32-bit RV32I core I built to learn how pipelining, hazards and forwarding work in hardware. Classic five stages: fetch, decode, execute, memory, write-back.

## What is implemented

- Five stages with pipeline registers between them (`if_stage`, `id_stage`, `ex_stage`, `mem_stage`, `wb_stage`, plus the `*_reg` modules)
- ALU, immediate generator, register file, control unit
- Data hazards: forwarding from EX/MEM and MEM/WB, and a stall for load-use (`forwarding_unit`, `hazard_detection_unit`)
- Control hazards: branch and jump evaluation in the execute stage with a pipeline flush (`branch_jump_control`). Branches are treated as not-taken until resolved.
- Instruction memory loaded from a hex file, data memory, and a set of small assembly test programs in `mem/`

Modules that exist in `src/` but are not wired into `riscv_pipeline.sv` yet: `branch_predictor`, `csr_file`, `performance_monitor`. Each has its own code and (for the predictor) a testbench, but the top-level pipeline does not use them. `fpga_top.sv` and the board constraint files (Basys3, DE10-Lite) are a start on an FPGA demo and still have TODOs. I have not put this on a board.

## Layout

```
src/          RTL
tb/           testbenches (pipeline, ALU, register file, branch predictor)
mem/          test programs: .s source and .hex for the instruction memory
scripts/      Python/shell helpers to generate tests and run them
constraints/  FPGA constraint files
docs/         notes on the pipeline, hazards and testing
```

## Running it

With Icarus Verilog:

```
iverilog -g2012 -o riscv_pipeline -I src src/*.sv tb/riscv_pipeline_tb.sv
vvp riscv_pipeline
```

The instruction memory loads `mem/inst_mem.hex` by default. To run another program, set the `IMEM_INIT_FILE` parameter in the testbench to one of the hex files in `mem/` (for example `mem/raw_hazard_test.hex`). More detail is in `docs/SIMULATION.md`.

What I have checked on this repo: the RTL and `tb/riscv_pipeline_tb.sv` elaborate with the slang parser. I do not have pass/fail logs or waveforms committed, so I am not quoting CPI figures or a pass rate here.

## Test programs

`mem/` has programs for add/sub, addi, logical ops, slt/sltu, memory ops, branches, jumps and RAW hazards. Each `.s` file explains what it expects. Right now the testbench prints the pipeline state, so you read the results yourself rather than getting an automatic pass/fail.

## Known gaps

- No automatic self-checking for the test programs yet
- Not run against the official RISC-V compliance suite (scripts for it are in `scripts/`, untested)
- Branch predictor, CSR file and performance monitor not integrated
- No FPGA run
