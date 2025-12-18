# FPGA Demonstration Constraint Files

This directory contains constraint files for running the RISC-V processor demonstration on FPGA boards.

## Files

### `basys3.xdc`
**Xilinx Vivado constraint file for Basys3 board**

- **Board**: Basys3 Artix-7 FPGA Board
- **FPGA**: XC7A35TCPG236-1
- **Clock**: 100 MHz
- **LEDs**: 16 LEDs (LD0-LD15)
- **Switches**: 16 switches (SW0-SW15, use 3 for display mode)
- **Reset**: BTNC button

**Usage**:
```tcl
add_files -fileset constrs_1 constraints/basys3.xdc
```

### `de10_lite.sdc`
**Intel Quartus SDC constraint file for DE10-Lite board**

- **Board**: DE10-Lite FPGA Board
- **FPGA**: 5CEBA4F23C7 (Cyclone V)
- **Clock**: 50 MHz
- **LEDs**: 10 LEDs (LEDR[0:9])
- **Switches**: 10 switches (SW[0:9], use 3 for display mode)
- **Reset**: KEY[0] button

**Usage**:
```tcl
set_global_assignment -name SDC_FILE constraints/de10_lite.sdc
# Also copy pin assignments to QSF file
```

## Pin Assignments Summary

### Basys3

| Signal | Pin | Description |
|--------|-----|-------------|
| clk_board | W5 | 100 MHz clock |
| btn_reset | U18 | BTNC button |
| sw_display_mode[0] | V17 | SW0 |
| sw_display_mode[1] | V16 | SW1 |
| sw_display_mode[2] | W16 | SW2 |
| leds[0:15] | Various | LD0-LD15 |

### DE10-Lite

| Signal | Pin | Description |
|--------|-----|-------------|
| clk_board | PIN_R8 | 50 MHz clock |
| btn_reset | PIN_N9 | KEY[0] |
| sw_display_mode[0] | PIN_U11 | SW[0] |
| sw_display_mode[1] | PIN_V11 | SW[1] |
| sw_display_mode[2] | PIN_M9 | SW[2] |
| leds[0:9] | Various | LEDR[0:9] |

## Modifying for Other Boards

To adapt for other FPGA boards:

1. **Find pin assignments** from board schematic/reference manual
2. **Create new constraint file** (`.xdc` for Vivado, `.sdc` for Quartus)
3. **Update pin assignments** with correct pin numbers
4. **Adjust clock frequency** if board clock differs
5. **Modify LED/switch assignments** based on available I/O

## Common Issues

### Pin Assignment Errors

**Error**: Pin already assigned or invalid pin number

**Solution**: 
- Verify pin numbers in board schematic
- Check pins aren't used by other signals
- Ensure I/O standard matches board voltage

### Clock Frequency Mismatch

**Error**: Clock constraint doesn't match board clock

**Solution**:
- Check board clock frequency (usually 50 MHz or 100 MHz)
- Update `create_clock` period in constraint file
- Adjust clock divider divisor if needed

### LED/Switch Not Working

**Error**: LEDs don't light up or switches don't change display

**Solution**:
- Verify pin assignments are correct
- Check I/O standards match board voltage
- Ensure signals are connected in top-level module

## See Also

- [`docs/FPGA_DEMO.md`](../docs/FPGA_DEMO.md) - Complete FPGA demonstration guide
- [`docs/SYNTHESIS.md`](../docs/SYNTHESIS.md) - General synthesis guide

