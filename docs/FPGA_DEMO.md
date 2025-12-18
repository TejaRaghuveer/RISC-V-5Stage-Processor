# FPGA Demonstration Guide for RISC-V Processor

This guide explains how to use the FPGA demonstration top-level module to run the RISC-V processor on FPGA boards.

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Board Support](#board-support)
4. [Quick Start](#quick-start)
5. [Module Descriptions](#module-descriptions)
6. [Display Modes](#display-modes)
7. [Synthesis Instructions](#synthesis-instructions)
8. [Troubleshooting](#troubleshooting)

---

## Overview

The FPGA demonstration module (`fpga_top.sv`) wraps the RISC-V processor for FPGA implementation. It provides:

- **Clock divider**: Slows down processor clock for LED observation
- **Reset debouncing**: Filters button bounces for reliable reset
- **LED display**: Shows processor state (PC, registers, status)
- **Display mode selection**: Switch between different display modes
- **Optional UART**: Interface for program loading (future enhancement)

## Features

### Clock Divider

- Divides board clock (100 MHz) to slower frequency (100 Hz)
- Makes processor operation visible on LEDs
- Can be bypassed for full-speed operation

### Reset Debouncing

- Filters mechanical button bounces
- Prevents multiple resets from single button press
- Configurable debounce time (default: 10 ms)

### LED Display

- 16 LEDs show processor state
- Multiple display modes selectable via switches
- Shows PC value, register values, or processor status

### Display Modes

| Mode | Switch Setting | Display |
|------|----------------|---------|
| 0 | 000 | PC[15:0] (lower 16 bits) |
| 1 | 001 | PC[31:16] (upper 16 bits) |
| 2 | 010 | Register x1[15:0] |
| 3 | 011 | Register x2[15:0] |
| 4 | 100 | Processor status bits |
| 5-7 | 101-111 | Default to PC[15:0] |

---

## Board Support

### Supported Boards

#### Basys3 (Xilinx Artix-7)
- **FPGA**: XC7A35TCPG236-1
- **Clock**: 100 MHz
- **LEDs**: 16 LEDs available
- **Switches**: 16 switches (use 3 for display mode)
- **Reset**: BTNC button
- **Constraint File**: `constraints/basys3.xdc`

#### DE10-Lite (Intel Cyclone V)
- **FPGA**: 5CEBA4F23C7
- **Clock**: 50 MHz
- **LEDs**: 10 LEDs available (LEDR[0:9])
- **Switches**: 10 switches (use 3 for display mode)
- **Reset**: KEY[0] button
- **Constraint File**: `constraints/de10_lite.sdc`

### Other Boards

The design can be adapted to other FPGA boards by:
1. Creating a new constraint file with correct pin assignments
2. Modifying clock divider divisor if board clock differs
3. Adjusting LED/switch assignments

---

## Quick Start

### Step 1: Prepare Project

1. **Add FPGA top module to project**:
   ```tcl
   # In Vivado or Quartus, add:
   # - src/fpga_top.sv
   # - src/clock_divider.sv
   # - src/debounce.sv
   # - src/led_display.sv
   # - All existing processor source files
   ```

2. **Set top-level module**:
   ```tcl
   # Vivado
   set_property top fpga_top [current_fileset]
   
   # Quartus
   set_global_assignment -name TOP_LEVEL_ENTITY fpga_top
   ```

### Step 2: Add Constraint File

#### For Basys3:
```tcl
# Vivado
add_files -fileset constrs_1 constraints/basys3.xdc
```

#### For DE10-Lite:
```tcl
# Quartus
set_global_assignment -name SDC_FILE constraints/de10_lite.sdc
# Also add pin assignments from de10_lite.sdc to QSF file
```

### Step 3: Configure Memory Initialization

Ensure instruction memory file exists:
```
mem/inst_mem.hex
```

Or modify parameter in `fpga_top.sv`:
```systemverilog
riscv_pipeline #(
    .IMEM_INIT_FILE("path/to/your/program.hex"),
    // ... other parameters
) processor_inst (...);
```

### Step 4: Synthesize and Program

#### Vivado:
```tcl
# Run synthesis
launch_runs synth_1
wait_on_run synth_1

# Run implementation
launch_runs impl_1
wait_on_run impl_1

# Generate bitstream
launch_runs impl_1 -to_step write_bitstream
wait_on_run impl_1

# Program FPGA
open_hw_manager
connect_hw_server
open_hw_target
program_hw_devices [get_hw_devices]
```

#### Quartus:
```tcl
# Run compilation
load_package flow
execute_module -tool map
execute_module -tool fit
execute_module -tool sta
execute_module -tool asm

# Program FPGA (use Programmer GUI or command line)
```

### Step 5: Test on Board

1. **Power on FPGA board**
2. **Program FPGA** with generated bitstream
3. **Set display mode** using switches:
   - SW[2:0] = 000: Show PC[15:0]
   - SW[2:0] = 001: Show PC[31:16]
   - etc.
4. **Press reset button** to reset processor
5. **Observe LEDs** showing processor state

---

## Module Descriptions

### `fpga_top.sv`

**Top-level wrapper module** that connects processor to FPGA I/O.

**Ports**:
- `clk_board`: Board clock input (100 MHz for Basys3, 50 MHz for DE10-Lite)
- `btn_reset`: Reset button (active high, debounced)
- `sw_display_mode[2:0]`: Display mode selection switches
- `leds[15:0]`: LED outputs showing processor state
- `uart_rx/tx`: Optional UART interface (not implemented yet)

**Key Features**:
- Instantiates clock divider, debouncer, processor, and LED display
- Connects all modules together
- Provides placeholder signals for processor state (can be connected to actual signals)

### `clock_divider.sv`

**Divides input clock** by configurable divisor.

**Parameters**:
- `DIVISOR`: Clock division factor (default: 1,000,000)

**Example**:
- Input: 100 MHz → Output: 100 Hz (with DIVISOR = 1,000,000)

**Usage**:
```systemverilog
clock_divider #(
    .DIVISOR(1000000)  // Divide by 1 million
) clock_divider_inst (
    .clk_in(clk_board),
    .rst_n(1'b1),
    .clk_out(clk_slow)
);
```

### `debounce.sv`

**Debounces mechanical button inputs**.

**Parameters**:
- `DEBOUNCE_CYCLES`: Number of cycles for debounce (default: 1,000,000 = 10 ms @ 100 MHz)

**Features**:
- Double-flop synchronizer for metastability prevention
- Counter-based debounce algorithm
- Stable output after debounce period

**Usage**:
```systemverilog
debounce #(
    .DEBOUNCE_CYCLES(1000000)
) debounce_inst (
    .clk(clk_board),
    .rst_n(1'b1),
    .button_in(btn_raw),
    .button_out(btn_debounced)
);
```

### `led_display.sv`

**Displays processor state on LEDs** based on mode selection.

**Display Modes**:
- Mode 0: PC[15:0]
- Mode 1: PC[31:16]
- Mode 2: Register x1[15:0]
- Mode 3: Register x2[15:0]
- Mode 4: Processor status (running/stalled/flushed)

**Usage**:
```systemverilog
led_display #(
    .DATA_WIDTH(32),
    .LED_WIDTH(16)
) led_display_inst (
    .clk(clk_board),
    .rst_n(rst_n),
    .display_mode(sw_display_mode),
    .pc_value(pc_value),
    .reg_x1_value(reg_x1_value),
    .reg_x2_value(reg_x2_value),
    .processor_running(processor_running),
    .processor_stalled(processor_stalled),
    .processor_flushed(processor_flushed),
    .leds(leds)
);
```

---

## Display Modes

### Mode 0: PC[15:0] (Lower 16 bits)

Shows the lower 16 bits of the program counter.

**Switch Setting**: SW[2:0] = 000

**Example**:
- PC = 0x00001234
- LEDs show: 0x1234 (binary: 0001 0010 0011 0100)

### Mode 1: PC[31:16] (Upper 16 bits)

Shows the upper 16 bits of the program counter.

**Switch Setting**: SW[2:0] = 001

**Example**:
- PC = 0x12345678
- LEDs show: 0x1234 (binary: 0001 0010 0011 0100)

### Mode 2: Register x1[15:0]

Shows the lower 16 bits of register x1.

**Switch Setting**: SW[2:0] = 010

**Example**:
- x1 = 0xDEADBEEF
- LEDs show: 0xBEEF (binary: 1011 1110 1110 1111)

### Mode 3: Register x2[15:0]

Shows the lower 16 bits of register x2.

**Switch Setting**: SW[2:0] = 011

**Example**:
- x2 = 0xCAFEBABE
- LEDs show: 0xBABE (binary: 1011 1010 1011 1110)

### Mode 4: Processor Status

Shows processor status bits.

**Switch Setting**: SW[2:0] = 100

**LED Mapping**:
- LED[12]: Processor running
- LED[11]: Processor stalled
- LED[10]: Processor flushed
- LED[9:0]: Reserved (0)

**Example**:
- Running: LED[12] = ON
- Stalled: LED[11] = OFF
- Flushed: LED[10] = OFF

---

## Synthesis Instructions

### For Basys3 (Vivado)

1. **Create Project**:
   ```tcl
   create_project riscv_demo ./vivado_project -part xc7a35tcpg236-1
   ```

2. **Add Source Files**:
   ```tcl
   add_files src/fpga_top.sv
   add_files src/clock_divider.sv
   add_files src/debounce.sv
   add_files src/led_display.sv
   add_files src/riscv_pipeline.sv
   # ... add all other processor source files
   ```

3. **Add Constraints**:
   ```tcl
   add_files -fileset constrs_1 constraints/basys3.xdc
   ```

4. **Set Top Module**:
   ```tcl
   set_property top fpga_top [current_fileset]
   ```

5. **Run Synthesis**:
   ```tcl
   launch_runs synth_1
   wait_on_run synth_1
   ```

6. **Run Implementation**:
   ```tcl
   launch_runs impl_1
   wait_on_run impl_1
   ```

7. **Generate Bitstream**:
   ```tcl
   launch_runs impl_1 -to_step write_bitstream
   wait_on_run impl_1
   ```

### For DE10-Lite (Quartus)

1. **Create Project**:
   ```tcl
   project_new riscv_demo -overwrite
   set_global_assignment -name FAMILY "Cyclone V"
   set_global_assignment -name DEVICE 5CEBA4F23C7
   ```

2. **Add Source Files**:
   ```tcl
   set_global_assignment -name SYSTEMVERILOG_FILE src/fpga_top.sv
   set_global_assignment -name SYSTEMVERILOG_FILE src/clock_divider.sv
   set_global_assignment -name SYSTEMVERILOG_FILE src/debounce.sv
   set_global_assignment -name SYSTEMVERILOG_FILE src/led_display.sv
   set_global_assignment -name SYSTEMVERILOG_FILE src/riscv_pipeline.sv
   # ... add all other processor source files
   ```

3. **Add Constraints**:
   ```tcl
   set_global_assignment -name SDC_FILE constraints/de10_lite.sdc
   # Copy pin assignments from de10_lite.sdc to QSF file
   ```

4. **Set Top Module**:
   ```tcl
   set_global_assignment -name TOP_LEVEL_ENTITY fpga_top
   ```

5. **Run Compilation**:
   ```tcl
   load_package flow
   execute_module -tool map
   execute_module -tool fit
   execute_module -tool sta
   execute_module -tool asm
   ```

---

## Troubleshooting

### LEDs Not Updating

**Problem**: LEDs show static values or don't change.

**Solutions**:
1. Check clock divider is working (verify `clk_processor` is toggling)
2. Verify processor is running (check reset is released)
3. Ensure memory initialization file exists and is loaded
4. Check display mode switches are set correctly

### Reset Button Not Working

**Problem**: Pressing reset button doesn't reset processor.

**Solutions**:
1. Verify button pin assignment in constraint file
2. Check debounce module is working (may need longer debounce time)
3. Verify reset polarity (button may be active low instead of high)
4. Check reset signal reaches processor

### Wrong Display Mode

**Problem**: LEDs show wrong data for selected mode.

**Solutions**:
1. Verify switch pin assignments in constraint file
2. Check `sw_display_mode` signals reach LED display module
3. Verify display mode logic in `led_display.sv`

### Clock Too Fast/Slow

**Problem**: Processor runs too fast or too slow for observation.

**Solutions**:
1. Adjust `CLOCK_DIVISOR` parameter in `fpga_top.sv`
2. For faster: Reduce divisor (e.g., 100000 → 10000)
3. For slower: Increase divisor (e.g., 1000000 → 10000000)
4. Or bypass divider: `assign clk_processor = clk_board;`

### Memory Not Initialized

**Problem**: Processor executes NOPs (all zeros).

**Solutions**:
1. Verify `IMEM_INIT_FILE` path is correct
2. Check hex file exists and is readable
3. Verify hex file format (one 32-bit instruction per line)
4. Check file path is relative to project directory

### Constraint File Errors

**Problem**: Pin assignment errors during synthesis.

**Solutions**:
1. Verify pin numbers match your FPGA board schematic
2. Check I/O standards match board voltage (usually 3.3V)
3. Ensure pins aren't already assigned to other signals
4. Verify constraint file syntax (XDC for Vivado, SDC/QSF for Quartus)

---

## Advanced Usage

### Connecting Real Processor Signals

Currently, `fpga_top.sv` uses placeholder signals for PC and register values. To display actual processor state:

1. **Modify `riscv_pipeline.sv`** to expose debug signals:
   ```systemverilog
   // Add output ports
   output logic [ADDR_WIDTH-1:0] debug_pc,
   output logic [DATA_WIDTH-1:0] debug_reg_x1,
   output logic [DATA_WIDTH-1:0] debug_reg_x2
   ```

2. **Connect internal signals**:
   ```systemverilog
   assign debug_pc = if_PC;
   assign debug_reg_x1 = reg_file[1];  // x1 is register 1
   assign debug_reg_x2 = reg_file[2];  // x2 is register 2
   ```

3. **Connect in `fpga_top.sv`**:
   ```systemverilog
   riscv_pipeline processor_inst (
       // ... other ports
       .debug_pc(pc_value),
       .debug_reg_x1(reg_x1_value),
       .debug_reg_x2(reg_x2_value)
   );
   ```

### Adding UART Interface

To add UART for program loading:

1. Create UART module (`uart_loader.sv`)
2. Instantiate in `fpga_top.sv`
3. Connect to instruction memory write interface
4. Add UART pin assignments to constraint file

### Using 7-Segment Displays

For DE10-Lite, can use HEX displays instead of LEDs:

1. Create 7-segment decoder module
2. Connect to `leds` signals
3. Add HEX display pin assignments to constraint file

---

## References

- [Basys3 Reference Manual](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual)
- [DE10-Lite User Manual](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=165&No=1021&PartNo=4)
- [Vivado User Guide](https://www.xilinx.com/support/documentation/sw_manuals/xilinx2021_1/ug910-vivado-getting-started.pdf)
- [Quartus Prime Handbook](https://www.intel.com/content/www/us/en/programmable/documentation/)

---

## Example Programs

### Simple Counter Program

Create `mem/inst_mem.hex`:
```
00100093  // ADDI x1, x0, 1
00200113  // ADDI x2, x0, 2
002081B3  // ADD x3, x1, x2
00310213  // ADDI x4, x2, 3
00418293  // ADDI x5, x3, 4
00000013  // NOP
00000013  // NOP
```

This program:
- Loads 1 into x1
- Loads 2 into x2
- Adds x1 + x2 → x3 (result: 3)
- Adds 3 to x2 → x4 (result: 5)
- Adds 4 to x3 → x5 (result: 7)

Observe on LEDs:
- Mode 0: PC increments
- Mode 2: x1 = 1
- Mode 3: x2 = 2

---

## Notes

- **Clock Divider**: Default divides 100 MHz to 100 Hz. Adjust `CLOCK_DIVISOR` for different speeds.
- **Debounce Time**: Default 10 ms. Increase if button still bounces.
- **Display Modes**: Only 5 modes implemented. Can add more by modifying `led_display.sv`.
- **Processor State**: Currently uses placeholder signals. Connect real signals for actual state display.

