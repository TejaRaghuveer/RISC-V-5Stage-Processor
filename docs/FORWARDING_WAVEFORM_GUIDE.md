# Data Forwarding Waveform Verification Guide

## Overview

This guide explains how to verify data forwarding (bypassing) is working correctly in the RISC-V 5-stage pipeline by examining simulation waveforms. Data forwarding eliminates most Read-After-Write (RAW) data hazards without requiring pipeline stalls.

## Key Signals to Monitor

### 1. Forwarding Control Signals

These signals control which data source is selected for ALU operands:

#### ForwardA[1:0] - Forwarding Control for rs1 (ALU Operand A)
- **Location**: `forwarding_unit_inst.ForwardA`
- **Encoding**:
  - `2'b00`: No forwarding (use register file data from ID/EX)
  - `2'b01`: Forward from MEM/WB stage
  - `2'b10`: Forward from EX/MEM stage (highest priority)
  - `2'b11`: Reserved

#### ForwardB[1:0] - Forwarding Control for rs2 (ALU Operand B)
- **Location**: `forwarding_unit_inst.ForwardB`
- **Encoding**: Same as ForwardA
  - `2'b00`: No forwarding (use register file data from ID/EX)
  - `2'b01`: Forward from MEM/WB stage
  - `2'b10`: Forward from EX/MEM stage (highest priority)

### 2. Register Address Signals

These signals identify which registers are being read/written:

#### Source Register Addresses (from ID/EX Pipeline Register)
- **id_ex_rs1_addr[4:0]**: Source register 1 address (instruction in EX stage)
- **id_ex_rs2_addr[4:0]**: Source register 2 address (instruction in EX stage)

#### Destination Register Addresses
- **ex_mem_rd_addr[4:0]**: Destination register (instruction in MEM stage)
- **mem_wb_rd_addr[4:0]**: Destination register (instruction in WB stage)

#### Register Write Enable Signals
- **ex_mem_reg_write**: Register write enable (instruction in MEM stage)
- **mem_wb_reg_write**: Register write enable (instruction in WB stage)

### 3. Data Signals

These signals show the actual data values being forwarded:

#### Register File Data (from ID/EX Pipeline Register)
- **id_ex_rs1_data[31:0]**: rs1 data read from register file
- **id_ex_rs2_data[31:0]**: rs2 data read from register file

#### Forwarded Data Sources
- **ex_mem_alu_result[31:0]**: ALU result from EX/MEM stage (for EX/MEM forwarding)
- **mem_wb_write_data[31:0]**: Write data from MEM/WB stage (for MEM/WB forwarding)

#### Forwarded Operands (after forwarding muxes)
- **rs1_data_forwarded[31:0]**: Selected rs1 data after forwarding mux
- **rs2_data_forwarded[31:0]**: Selected rs2 data after forwarding mux

#### Final ALU Operands
- **alu_operand_a[31:0]**: Final ALU operand A (rs1_data_forwarded or PC)
- **alu_operand_b[31:0]**: Final ALU operand B (rs2_data_forwarded or immediate)

### 4. ALU Result Signal

- **ex_mem_alu_result[31:0]**: ALU computation result (used for forwarding)

## Expected Signal Patterns for Different Scenarios

### Scenario 1: No Forwarding (Normal Case)

**Example Instructions**:
```assembly
ADDI x1, x0, 10    # I1: x1 = 10
ADDI x2, x0, 20    # I2: x2 = 20
ADD x3, x1, x2     # I3: x3 = x1 + x2 (no hazard)
```

**Pipeline Timeline** (when I3 is in EX stage):
- I1 is in WB stage (x1 already written to register file)
- I2 is in MEM stage (x2 already written to register file)
- I3 is in EX stage (reads x1 and x2)

**Expected Waveform Values**:
```
Cycle N:
  ForwardA = 2'b00          # No forwarding needed
  ForwardB = 2'b00          # No forwarding needed
  
  id_ex_rs1_addr = 5'd1     # x1
  id_ex_rs2_addr = 5'd2     # x2
  
  ex_mem_rd_addr = 5'd2     # I2 writes x2 (not matching rs1 or rs2)
  mem_wb_rd_addr = 5'd1     # I1 writes x1 (already in register file)
  
  id_ex_rs1_data = 32'd10   # x1 value from register file
  id_ex_rs2_data = 32'd20   # x2 value from register file
  
  rs1_data_forwarded = 32'd10  # Selected from register file (ForwardA=00)
  rs2_data_forwarded = 32'd20  # Selected from register file (ForwardB=00)
  
  alu_operand_a = 32'd10    # Final ALU operand A
  alu_operand_b = 32'd20    # Final ALU operand B
  ex_mem_alu_result = 32'd30 # ALU result: 10 + 20 = 30
```

**Key Observations**:
- ForwardA and ForwardB are both `00` (no forwarding)
- Register file data is used directly
- No address matches between source and destination registers

---

### Scenario 2: EX/MEM Forwarding (ForwardA = 10)

**Example Instructions**:
```assembly
ADDI x5, x0, 5     # I1: x5 = 5
ADD x6, x5, x2     # I2: x6 = x5 + x2 (EX hazard on x5)
```

**Pipeline Timeline** (when I2 is in EX stage):
- I1 is in MEM stage (x5 being written)
- I2 is in EX stage (needs x5)

**Expected Waveform Values**:
```
Cycle N:
  ForwardA = 2'b10          # Forward from EX/MEM (x5)
  ForwardB = 2'b00          # No forwarding (x2 from register file)
  
  id_ex_rs1_addr = 5'd5     # x5 (source register)
  id_ex_rs2_addr = 5'd2     # x2 (source register)
  
  ex_mem_rd_addr = 5'd5     # I1 writes x5 (MATCHES rs1!)
  ex_mem_reg_write = 1'b1   # I1 writes to register
  mem_wb_rd_addr = 5'd0     # No relevant write in WB
  
  id_ex_rs1_data = 32'd0     # x5 value from register file (STALE, not yet written)
  ex_mem_alu_result = 32'd5  # I1's ALU result (x5 = 5) - CORRECT VALUE
  
  rs1_data_forwarded = 32'd5  # Selected from EX/MEM (ForwardA=10)
  rs2_data_forwarded = 32'd20 # Selected from register file (ForwardB=00)
  
  alu_operand_a = 32'd5     # Final ALU operand A (forwarded x5)
  alu_operand_b = 32'd20    # Final ALU operand B (x2 from register file)
  ex_mem_alu_result = 32'd25 # ALU result: 5 + 20 = 25
```

**Key Observations**:
- ForwardA = `10` (EX/MEM forwarding active)
- `ex_mem_rd_addr == id_ex_rs1_addr` (address match detected)
- `rs1_data_forwarded == ex_mem_alu_result` (forwarded data matches EX/MEM result)
- `id_ex_rs1_data` may be stale (not yet written to register file)

---

### Scenario 3: MEM/WB Forwarding (ForwardA = 01)

**Example Instructions**:
```assembly
ADDI x16, x0, 16   # I1: x16 = 16
ADDI x17, x0, 17   # I2: x17 = 17 (independent)
ADD x18, x16, x2   # I3: x18 = x16 + x2 (MEM hazard on x16)
```

**Pipeline Timeline** (when I3 is in EX stage):
- I1 is in WB stage (x16 being written)
- I2 is in MEM stage (x17 being written)
- I3 is in EX stage (needs x16)

**Expected Waveform Values**:
```
Cycle N:
  ForwardA = 2'b01          # Forward from MEM/WB (x16)
  ForwardB = 2'b00          # No forwarding (x2 from register file)
  
  id_ex_rs1_addr = 5'd16    # x16 (source register)
  id_ex_rs2_addr = 5'd2     # x2 (source register)
  
  ex_mem_rd_addr = 5'd17    # I2 writes x17 (does NOT match rs1)
  ex_mem_reg_write = 1'b1   # I2 writes to register
  mem_wb_rd_addr = 5'd16    # I1 writes x16 (MATCHES rs1!)
  mem_wb_reg_write = 1'b1   # I1 writes to register
  
  id_ex_rs1_data = 32'd0     # x16 value from register file (STALE)
  mem_wb_write_data = 32'd16 # I1's write data (x16 = 16) - CORRECT VALUE
  
  rs1_data_forwarded = 32'd16 # Selected from MEM/WB (ForwardA=01)
  rs2_data_forwarded = 32'd20 # Selected from register file (ForwardB=00)
  
  alu_operand_a = 32'd16    # Final ALU operand A (forwarded x16)
  alu_operand_b = 32'd20    # Final ALU operand B (x2 from register file)
  ex_mem_alu_result = 32'd36 # ALU result: 16 + 20 = 36
```

**Key Observations**:
- ForwardA = `01` (MEM/WB forwarding active)
- `mem_wb_rd_addr == id_ex_rs1_addr` (address match detected)
- `ex_mem_rd_addr != id_ex_rs1_addr` (no EX/MEM match, so MEM/WB is used)
- `rs1_data_forwarded == mem_wb_write_data` (forwarded data matches MEM/WB data)

---

### Scenario 4: Both Operands Forwarded (EX/MEM and MEM/WB)

**Example Instructions**:
```assembly
ADDI x9, x0, 9     # I1: x9 = 9
ADDI x10, x0, 11   # I2: x10 = 11
ADD x11, x9, x10   # I3: x11 = x9 + x10 (both forwarded)
```

**Pipeline Timeline** (when I3 is in EX stage):
- I1 is in MEM stage (x9 being written)
- I2 is in EX stage (x10 being computed)
- I3 is in EX stage (needs both x9 and x10)

**Expected Waveform Values**:
```
Cycle N:
  ForwardA = 2'b10          # Forward from EX/MEM (x9)
  ForwardB = 2'b10          # Forward from EX/MEM (x10)
  
  id_ex_rs1_addr = 5'd9     # x9 (source register)
  id_ex_rs2_addr = 5'd10    # x10 (source register)
  
  ex_mem_rd_addr = 5'd9     # I1 writes x9 (MATCHES rs1!)
  ex_mem_reg_write = 1'b1   # I1 writes to register
  mem_wb_rd_addr = 5'd0     # No relevant write in WB
  
  # Note: x10 is being computed in current cycle, so it's in ID/EX
  # Actually, wait - let me reconsider the pipeline timing...
  
  # Correct timing:
  # Cycle N-1: I2 in EX, computes x10 = 11
  # Cycle N:   I2 in MEM (x10 in EX/MEM), I3 in EX (needs x10)
  #            I1 in MEM (x9 in EX/MEM), I3 needs x9
  
  ex_mem_alu_result = 32'd9  # I1's result (x9 = 9) - for ForwardA
  # Actually, x10 would be in EX/MEM from I2, but I2 is one cycle ahead
  
  # Let me correct: When I3 is in EX:
  # - I2 is in MEM (x10 in EX/MEM register)
  # - I1 is in WB (x9 in MEM/WB register)
  
  ForwardA = 2'b01          # Forward from MEM/WB (x9 from I1)
  ForwardB = 2'b10          # Forward from EX/MEM (x10 from I2)
  
  ex_mem_rd_addr = 5'd10    # I2 writes x10 (MATCHES rs2!)
  mem_wb_rd_addr = 5'd9     # I1 writes x9 (MATCHES rs1!)
  
  rs1_data_forwarded = 32'd9   # From MEM/WB (ForwardA=01)
  rs2_data_forwarded = 32'd11  # From EX/MEM (ForwardB=10)
  
  alu_operand_a = 32'd9     # Final ALU operand A
  alu_operand_b = 32'd11    # Final ALU operand B
  ex_mem_alu_result = 32'd20 # ALU result: 9 + 11 = 20
```

**Key Observations**:
- ForwardA and ForwardB can have different values
- Both operands are forwarded from different pipeline stages
- Address matches occur for both source registers

---

### Scenario 5: Forwarding Priority (EX/MEM over MEM/WB)

**Example Instructions**:
```assembly
ADDI x26, x0, 26   # I1: x26 = 26
ADDI x27, x0, 27   # I2: x27 = 27
ADD x28, x26, x27  # I3: x28 = x26 + x27
```

**Pipeline Timeline** (when I3 is in EX stage):
- I1 is in WB stage (x26 being written)
- I2 is in MEM stage (x27 being written)
- I3 is in EX stage (needs both x26 and x27)

**Expected Waveform Values**:
```
Cycle N:
  ForwardA = 2'b01          # Forward from MEM/WB (x26 from I1)
  ForwardB = 2'b10          # Forward from EX/MEM (x27 from I2)
  
  id_ex_rs1_addr = 5'd26    # x26 (source register)
  id_ex_rs2_addr = 5'd27    # x27 (source register)
  
  ex_mem_rd_addr = 5'd27    # I2 writes x27 (MATCHES rs2!)
  ex_mem_reg_write = 1'b1   # I2 writes to register
  mem_wb_rd_addr = 5'd26    # I1 writes x26 (MATCHES rs1!)
  mem_wb_reg_write = 1'b1   # I1 writes to register
  
  # Priority check: For rs2, EX/MEM match exists, so ForwardB = 10
  # For rs1, no EX/MEM match, so ForwardA = 01 (MEM/WB)
  
  rs1_data_forwarded = 32'd26 # From MEM/WB (ForwardA=01)
  rs2_data_forwarded = 32'd27 # From EX/MEM (ForwardB=10)
  
  alu_operand_a = 32'd26    # Final ALU operand A
  alu_operand_b = 32'd27    # Final ALU operand B
  ex_mem_alu_result = 32'd53 # ALU result: 26 + 27 = 53
```

**Key Observations**:
- EX/MEM forwarding has priority over MEM/WB forwarding
- If both stages write to the same register, EX/MEM is used
- Different operands can forward from different stages

---

## Verification Checklist

### Step 1: Identify Hazard Scenarios
1. Find instructions with RAW dependencies in your test program
2. Identify which pipeline stages the producer and consumer are in
3. Determine expected forwarding type (EX/MEM or MEM/WB)

### Step 2: Check Forwarding Control Signals
1. **ForwardA/ForwardB Values**:
   - Should be `00` when no forwarding needed
   - Should be `10` when EX/MEM forwarding active
   - Should be `01` when MEM/WB forwarding active

2. **Address Matching**:
   - Verify `ex_mem_rd_addr == id_ex_rs1_addr` for EX/MEM forwarding on rs1
   - Verify `ex_mem_rd_addr == id_ex_rs2_addr` for EX/MEM forwarding on rs2
   - Verify `mem_wb_rd_addr == id_ex_rs1_addr` for MEM/WB forwarding on rs1
   - Verify `mem_wb_rd_addr == id_ex_rs2_addr` for MEM/WB forwarding on rs2

3. **Write Enable Signals**:
   - Verify `ex_mem_reg_write == 1` when EX/MEM forwarding occurs
   - Verify `mem_wb_reg_write == 1` when MEM/WB forwarding occurs
   - Verify `rd_addr != 5'd0` (x0 cannot be written)

### Step 3: Verify Forwarded Data Values
1. **EX/MEM Forwarding**:
   - `rs1_data_forwarded == ex_mem_alu_result` (when ForwardA = 10)
   - `rs2_data_forwarded == ex_mem_alu_result` (when ForwardB = 10)

2. **MEM/WB Forwarding**:
   - `rs1_data_forwarded == mem_wb_write_data` (when ForwardA = 01)
   - `rs2_data_forwarded == mem_wb_write_data` (when ForwardB = 01)

3. **No Forwarding**:
   - `rs1_data_forwarded == id_ex_rs1_data` (when ForwardA = 00)
   - `rs2_data_forwarded == id_ex_rs2_data` (when ForwardB = 00)

### Step 4: Verify ALU Operands
1. **ALU Operand A**:
   - Should equal `rs1_data_forwarded` (for most instructions)
   - Should equal PC (for AUIPC instruction)

2. **ALU Operand B**:
   - Should equal `rs2_data_forwarded` (when ALUSrc = 0, R-type)
   - Should equal immediate (when ALUSrc = 1, I-type)

3. **ALU Result**:
   - Should match expected computation result
   - Verify correctness of forwarded operands

### Step 5: Check Timing Alignment
1. **Pipeline Stage Alignment**:
   - Forwarding signals should be stable when instruction is in EX stage
   - Forwarded data should be available in same cycle as forwarding control

2. **Data Availability**:
   - EX/MEM data available when instruction is in MEM stage
   - MEM/WB data available when instruction is in WB stage
   - Register file data available when instruction is in ID stage

## Common Issues and Debugging

### Issue 1: Forwarding Not Occurring When Expected

**Symptoms**:
- ForwardA/ForwardB remain `00` when they should be `10` or `01`
- ALU operands use stale register file data

**Debugging Steps**:
1. Check register address matches:
   - Verify `ex_mem_rd_addr == id_ex_rs1_addr` (for rs1 forwarding)
   - Verify `mem_wb_rd_addr == id_ex_rs1_addr` (for rs1 forwarding)
2. Check write enable signals:
   - Verify `ex_mem_reg_write == 1` (for EX/MEM forwarding)
   - Verify `mem_wb_reg_write == 1` (for MEM/WB forwarding)
3. Check x0 handling:
   - Verify `rd_addr != 5'd0` (x0 writes are ignored)

### Issue 2: Wrong Forwarding Source Selected

**Symptoms**:
- ForwardA/ForwardB have wrong value (e.g., `01` instead of `10`)
- ALU operands use wrong data source

**Debugging Steps**:
1. Check forwarding priority:
   - EX/MEM forwarding should have priority over MEM/WB
   - Verify `ex_mem_rd_addr` comparison happens before `mem_wb_rd_addr`
2. Check address matching logic:
   - Verify comparison logic in forwarding unit
   - Check for correct `else if` ordering

### Issue 3: Forwarded Data Incorrect

**Symptoms**:
- Forwarding control signals are correct
- But forwarded data values don't match expected sources

**Debugging Steps**:
1. Check forwarding mux selection:
   - Verify `case` statement in EX stage matches ForwardA/ForwardB values
   - Check default case handling
2. Check data source signals:
   - Verify `ex_mem_alu_result` contains correct value
   - Verify `mem_wb_write_data` contains correct value
3. Check timing:
   - Ensure forwarded data is available when needed
   - Verify pipeline register updates occur correctly

## Waveform Display Tips

### Recommended Signal Groups

**Group 1: Forwarding Control**
- `ForwardA[1:0]`
- `ForwardB[1:0]`

**Group 2: Register Addresses**
- `id_ex_rs1_addr[4:0]`
- `id_ex_rs2_addr[4:0]`
- `ex_mem_rd_addr[4:0]`
- `mem_wb_rd_addr[4:0]`

**Group 3: Write Enables**
- `ex_mem_reg_write`
- `mem_wb_reg_write`

**Group 4: Data Sources**
- `id_ex_rs1_data[31:0]`
- `id_ex_rs2_data[31:0]`
- `ex_mem_alu_result[31:0]`
- `mem_wb_write_data[31:0]`

**Group 5: Forwarded Operands**
- `rs1_data_forwarded[31:0]`
- `rs2_data_forwarded[31:0]`

**Group 6: ALU Operands**
- `alu_operand_a[31:0]`
- `alu_operand_b[31:0]`
- `ex_mem_alu_result[31:0]`

### Display Format Recommendations

- **Addresses**: Display in decimal (e.g., `x5` instead of `5'd5`)
- **Control Signals**: Display in binary or hex
- **Data Values**: Display in hex for easier reading
- **Use Radix**: Set appropriate radix for each signal type

## Example Test Cases

Use the `raw_hazard_test.hex` program to verify forwarding:

1. **EX Hazard Test** (Section 2):
   - Monitor ForwardA/ForwardB during EX hazard scenarios
   - Verify ForwardA = `10` or ForwardB = `10` when expected

2. **MEM Hazard Test** (Section 3):
   - Monitor ForwardA/ForwardB during MEM hazard scenarios
   - Verify ForwardA = `01` or ForwardB = `01` when expected

3. **Complex Scenarios** (Section 5):
   - Monitor both ForwardA and ForwardB simultaneously
   - Verify mixed forwarding scenarios (different sources for each operand)

## Related Documentation

- `docs/RAW_HAZARD_TEST.md` - Test program documentation
- `docs/ARCHITECTURE.md` - Pipeline architecture details
- `src/forwarding_unit.sv` - Forwarding unit implementation
- `src/ex_stage.sv` - Execute stage implementation

