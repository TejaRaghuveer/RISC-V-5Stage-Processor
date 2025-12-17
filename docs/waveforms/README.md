# Waveform Screenshots

This directory contains waveform screenshots demonstrating key pipeline behaviors.

## Expected Waveform Files

The following waveform screenshots should be added to this directory:

1. **`pipeline_progression.png`** - Shows multiple instructions in pipeline simultaneously
2. **`forwarding_ex_mem.png`** - Demonstrates EX/MEM forwarding resolving RAW hazard
3. **`load_use_stall.png`** - Shows pipeline stall for load-use hazard
4. **`branch_taken_flush.png`** - Demonstrates branch taken causing pipeline flush

## How to Generate Waveforms

1. Run a simulation using your preferred simulator (Icarus Verilog, ModelSim, etc.)
2. Generate a VCD or waveform file
3. Open in GTKWave or simulator's waveform viewer
4. Capture screenshots of key scenarios
5. Save screenshots to this directory

## Waveform Analysis Guide

See [`../SIMULATION.md`](../SIMULATION.md) for detailed instructions on:
- Running simulations
- Viewing waveforms
- Interpreting results
- Key signals to monitor

See [`../TEST_REPORT.md`](../TEST_REPORT.md) for waveform analysis examples and expected behaviors.

