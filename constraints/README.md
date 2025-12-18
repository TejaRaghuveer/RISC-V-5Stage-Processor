# FPGA Synthesis Constraints

This directory contains constraint files for FPGA synthesis of the RISC-V 5-stage pipeline processor.

## Files

### `riscv_pipeline.xdc`
**Xilinx Vivado Constraint File**

Contains timing and physical constraints for Xilinx FPGAs:
- Clock constraints (period, uncertainty)
- Reset constraints (false paths, delays)
- Input/Output delays
- Memory constraints (BRAM usage)
- Pin assignments (commented - modify for your board)

**Usage**:
1. Add to Vivado project: `add_files -fileset constrs_1 constraints/riscv_pipeline.xdc`
2. Modify clock period for your target frequency
3. Uncomment and modify pin assignments for your FPGA board

### `riscv_pipeline.sdc`
**Intel Quartus Prime SDC Constraint File**

Contains timing constraints for Intel FPGAs:
- Clock constraints (period, uncertainty)
- Reset constraints (false paths, delays)
- Input/Output delays
- Timing exceptions

**Usage**:
1. Add to Quartus project: `set_global_assignment -name SDC_FILE constraints/riscv_pipeline.sdc`
2. Modify clock period for your target frequency
3. Pin assignments are typically done in Pin Planner or QSF file

## Quick Start

### For Xilinx Vivado

1. **Modify Clock Frequency**:
   ```tcl
   # In riscv_pipeline.xdc, change:
   create_clock -period 10.000 -name clk [get_ports clk]
   # To your target frequency, e.g., 50 MHz = 20.000 ns
   ```

2. **Set Pin Assignments**:
   - Uncomment pin assignment lines
   - Modify pin numbers for your FPGA board
   - Check your board schematic for correct pins

3. **Add to Project**:
   ```tcl
   add_files -fileset constrs_1 constraints/riscv_pipeline.xdc
   ```

### For Intel Quartus

1. **Modify Clock Frequency**:
   ```tcl
   # In riscv_pipeline.sdc, change:
   create_clock -name clk -period 10.000 [get_ports {clk}]
   # To your target frequency, e.g., 50 MHz = 20.000 ns
   ```

2. **Set Pin Assignments**:
   - Use Pin Planner (Assignments → Pin Planner)
   - Or add to QSF file:
     ```tcl
     set_location_assignment PIN_R8 -to clk
     set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to clk
     ```

3. **Add to Project**:
   ```tcl
   set_global_assignment -name SDC_FILE constraints/riscv_pipeline.sdc
   ```

## Common FPGA Board Pin Assignments

### Xilinx Boards

#### Basys 3 (Artix-7)
- Clock: `W5` (100 MHz)
- Reset: `T18` (BTNC button)
- I/O Standard: `LVCMOS33`

#### Zybo Z7 (Zynq-7000)
- Clock: `Y9` (125 MHz)
- Reset: `K17` (CPU_RESET button)
- I/O Standard: `LVCMOS33`

### Intel Boards

#### DE10-Lite (Cyclone V)
- Clock: `PIN_R8` (50 MHz)
- Reset: `PIN_N9` (KEY[0])
- I/O Standard: `3.3-V LVTTL`

#### DE2-115 (Cyclone IV)
- Clock: `PIN_Y2` (50 MHz)
- Reset: `PIN_M23` (KEY[0])
- I/O Standard: `3.3-V LVTTL`

## Constraint File Structure

### Clock Constraints
- Clock period (based on target frequency)
- Clock uncertainty (jitter and skew)
- Clock groups (if multiple clocks)

### Reset Constraints
- False paths (reset doesn't need timing)
- Reset delays (stability requirements)
- Async reset handling

### I/O Constraints
- Input delays (external signals)
- Output delays (if outputs exist)
- Drive strength and slew rate

### Timing Exceptions
- False paths (asynchronous signals)
- Multi-cycle paths (if any)
- Maximum/minimum delays

## Notes

1. **Clock Frequency**: Modify based on your requirements and FPGA capabilities
2. **Pin Assignments**: Always check your board schematic for correct pin numbers
3. **I/O Standards**: Match your board's voltage levels (usually 3.3V)
4. **False Paths**: Use sparingly - only for truly asynchronous signals
5. **Memory Constraints**: Ensure memories use BRAM (already configured)

## Troubleshooting

### Timing Violations
- Reduce clock frequency
- Check critical paths (see `docs/TIMING_ANALYSIS.md`)
- Apply optimizations (see `docs/OPTIMIZATION_GUIDE.md`)

### Pin Assignment Errors
- Verify pin numbers in board schematic
- Check I/O standards match board voltage
- Ensure pins are not already assigned

### Constraint File Not Found
- Check file paths are relative to project directory
- Verify file exists in `constraints/` directory
- Use absolute paths if needed

## References

- [Vivado Constraints User Guide](https://www.xilinx.com/support/documentation/sw_manuals/xilinx2021_1/ug949-vivado-design-suite-user-guide.pdf)
- [Quartus Prime Timing Analyzer](https://www.intel.com/content/www/us/en/programmable/documentation/)
- See `docs/SYNTHESIS.md` for complete synthesis guide

