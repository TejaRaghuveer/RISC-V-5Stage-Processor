# RISC-V Memory Operations Debugging Guide

This document identifies common issues with data memory read/write operations and provides debugging steps with waveform signals to check.

## Critical Issues Found

### Issue 1: Missing Signal Declarations in EX/MEM Register (CRITICAL BUG)

**Problem**: The `ex_mem_reg.sv` module uses `PCSrc_reg` and `branch_flush_reg` signals but they are not declared in the register declarations section.

**Location**: `src/ex_mem_reg.sv` lines 149, 150, 166, 167, 182, 183, 206, 207

**Fix Required**: Add these signal declarations:
```systemverilog
logic PCSrc_reg;
logic branch_flush_reg;
```

### Issue 2: Memory Timing - Two-Cycle Latency

**Problem**: The DMEM module registers both address and control signals, creating a two-cycle latency:
- **Cycle N**: Address and MemRead/MemWrite arrive at DMEM
- **Cycle N**: DMEM registers them (`addr_reg`, `MemRead_reg`, `MemWrite_reg`)
- **Cycle N+1**: DMEM uses registered signals for memory access
- **Cycle N+2**: Read data available (`read_data`)

**Impact**: This adds an extra cycle of latency compared to typical single-cycle memory designs.

**Current Behavior**:
```
LW x1, 0(x2):
  Cycle 1: EX stage computes address (x2 + 0)
  Cycle 2: Address in EX/MEM register → DMEM input
  Cycle 3: DMEM registers address → Memory read starts
  Cycle 4: DMEM outputs read_data → MEM/WB register
  Cycle 5: WB stage writes to register file
```

**Expected Behavior** (if single-cycle memory):
```
LW x1, 0(x2):
  Cycle 1: EX stage computes address
  Cycle 2: Address in EX/MEM → DMEM reads immediately
  Cycle 3: Read data in MEM/WB register
  Cycle 4: WB stage writes to register file
```

## Common Issues Checklist

### 1. Address Alignment

**Issue**: Memory addresses must be word-aligned (lower 2 bits = 00).

**Check**:
- Verify `ex_mem_alu_result[1:0] == 2'b00` for all load/store instructions
- Check `addr_aligned_reg` signal in DMEM

**Waveform Signals to Monitor**:
```
- ex_mem_alu_result[1:0]  (should be 00)
- dmem.addr[1:0]          (should be 00)
- dmem.addr_aligned_reg   (should be 1)
```

**Debug Steps**:
1. Add `$display` in EX stage: `$display("MEM ADDR: %h, ALIGNED: %b", alu_result, alu_result[1:0] == 2'b00);`
2. Check if ALU correctly computes addresses
3. Verify immediate values are correctly sign-extended

### 2. Byte vs Word Addressing

**Issue**: RISC-V uses byte addresses, but DMEM internally uses word addresses.

**Check**:
- Byte address from ALU: `ex_mem_alu_result` (32-bit byte address)
- Word address in DMEM: `word_addr_reg = addr_reg[MEM_ADDR_WIDTH+1:2]`
- For MEM_DEPTH=1024, MEM_ADDR_WIDTH=10, extracts bits [11:2]

**Waveform Signals to Monitor**:
```
- ex_mem_alu_result        (byte address, e.g., 0x00000000, 0x00000004, 0x00000008)
- dmem.addr_reg           (registered byte address)
- dmem.word_addr_reg       (word address, e.g., 0, 1, 2)
- dmem.addr[MEM_ADDR_WIDTH+1:2]  (word address extraction)
```

**Debug Steps**:
1. Verify word address calculation: `word_addr = byte_addr >> 2`
2. Check address range: `word_addr < MEM_DEPTH` (1024)
3. Monitor address conversion at each stage

**Example**:
```
Byte Address: 0x00000008 → Word Address: 0x00000002 (8 >> 2 = 2)
Byte Address: 0x0000000C → Word Address: 0x00000003 (12 >> 2 = 3)
```

### 3. Memory Initialization

**Issue**: Memory may not be initialized correctly, or initialization file not loaded.

**Check**:
- Verify `INIT_FILE` parameter is set correctly
- Check if `$readmemh` successfully loads data
- Verify memory array contents after initialization

**Waveform Signals to Monitor**:
```
- dmem.memory[0] through dmem.memory[MEM_DEPTH-1]  (initial values)
```

**Debug Steps**:
1. Add initialization debug output:
```systemverilog
initial begin
    $readmemh(INIT_FILE, memory);
    for (int i = 0; i < 16; i++) begin
        $display("DMEM[%0d] = %h", i, memory[i]);
    end
end
```

2. Check if test program initializes memory correctly
3. Verify memory contents match expected values

### 4. MemRead/MemWrite Control Signal Timing

**Issue**: Control signals may not be asserted at the correct time or may be corrupted.

**Check**:
- Verify `ex_mem_MemRead` and `ex_mem_MemWrite` from EX/MEM register
- Check registered signals `MemRead_reg` and `MemWrite_reg` in DMEM
- Ensure signals are not flushed or stalled incorrectly

**Waveform Signals to Monitor**:
```
- id_ex_MemRead           (from ID/EX register)
- id_ex_MemWrite          (from ID/EX register)
- ex_mem_MemRead          (from EX/MEM register)
- ex_mem_MemWrite         (from EX/MEM register)
- dmem.MemRead            (DMEM input)
- dmem.MemRead_reg        (DMEM registered)
- dmem.MemWrite           (DMEM input)
- dmem.MemWrite_reg       (DMEM registered)
```

**Debug Steps**:
1. Trace control signals through pipeline:
   - ID stage: Control unit generates MemRead/MemWrite
   - ID/EX register: Stores control signals
   - EX stage: Passes through (no modification)
   - EX/MEM register: Stores control signals
   - MEM stage: Passes to DMEM
   - DMEM: Registers and uses

2. Check for signal corruption:
   - Verify signals not affected by flush/stall
   - Check if hazard detection incorrectly modifies signals

3. Add debug output:
```systemverilog
always_ff @(posedge clk) begin
    if (MemRead_reg) $display("MEM READ: addr=%h, word_addr=%h", addr_reg, word_addr_reg);
    if (MemWrite_reg) $display("MEM WRITE: addr=%h, word_addr=%h, data=%h", addr_reg, word_addr_reg, write_data_reg);
end
```

### 5. Data Forwarding from MEM Stage

**Issue**: Store instructions may use incorrect rs2_data due to forwarding issues.

**Check**:
- Verify `ex_mem_rs2_data` contains correct value
- Check forwarding unit selects correct source
- Verify rs2_data is not corrupted by pipeline stalls/flushes

**Waveform Signals to Monitor**:
```
- id_ex_rs2_data          (from ID/EX register)
- ex_rs2_data_forwarded   (after forwarding in EX stage)
- ex_mem_rs2_data         (from EX/MEM register)
- dmem.write_data         (DMEM input)
- dmem.write_data_reg     (DMEM registered)
```

**Debug Steps**:
1. Trace rs2_data through pipeline:
   - ID stage: Read from register file
   - ID/EX register: Store rs2_data
   - EX stage: Apply forwarding if needed
   - EX/MEM register: Store forwarded rs2_data
   - MEM stage: Pass to DMEM

2. Verify forwarding logic:
   - Check ForwardB signal from forwarding unit
   - Verify correct source selected (ID/EX, EX/MEM, or MEM/WB)

3. Add debug output:
```systemverilog
always_ff @(posedge clk) begin
    if (MemWrite_reg) begin
        $display("STORE: rs2_data=%h, forwarded=%h", id_ex_rs2_data, ex_rs2_data_forwarded);
    end
end
```

### 6. Address Range Validation

**Issue**: Addresses may be out of memory bounds.

**Check**:
- Verify `word_addr_reg < MEM_DEPTH` (1024)
- Check `addr_in_range_reg` signal
- Ensure invalid addresses return zero (read) or are ignored (write)

**Waveform Signals to Monitor**:
```
- dmem.addr_reg           (byte address)
- dmem.word_addr_reg      (word address)
- dmem.addr_in_range_reg  (should be 1 for valid addresses)
- dmem.addr_valid         (combinational, for external use)
```

**Debug Steps**:
1. Check address calculation:
   - Maximum valid byte address: `MEM_DEPTH * 4 - 4 = 4092` (0xFFC)
   - Maximum valid word address: `MEM_DEPTH - 1 = 1023`

2. Add boundary checking:
```systemverilog
always_ff @(posedge clk) begin
    if (MemRead_reg || MemWrite_reg) begin
        if (!addr_in_range_reg) begin
            $display("WARNING: Address out of range: %h (word: %h)", addr_reg, word_addr_reg);
        end
    end
end
```

## Debugging Waveform Checklist

### Essential Signals to Monitor

#### Pipeline Control
```
- clk
- rst_n
- pipeline_stall_internal
- pipeline_flush_internal
```

#### Instruction Flow
```
- if_id_instruction
- id_ex_instruction
- ex_mem_alu_result (address for memory ops)
```

#### Memory Control Signals
```
- id_ex_MemRead
- id_ex_MemWrite
- ex_mem_MemRead
- ex_mem_MemWrite
- dmem.MemRead_reg
- dmem.MemWrite_reg
```

#### Memory Addresses
```
- ex_mem_alu_result        (byte address from ALU)
- dmem.addr                (DMEM input)
- dmem.addr_reg            (DMEM registered)
- dmem.addr[1:0]           (alignment check)
- dmem.word_addr_reg       (word address)
- dmem.addr_in_range_reg   (range check)
- dmem.addr_aligned_reg    (alignment check)
```

#### Memory Data
```
- id_ex_rs2_data           (store data source)
- ex_rs2_data_forwarded    (after forwarding)
- ex_mem_rs2_data          (to DMEM)
- dmem.write_data           (DMEM input)
- dmem.write_data_reg      (DMEM registered)
- dmem.read_data           (DMEM output)
- mem_read_data            (MEM stage output)
- wb_mem_read_data         (MEM/WB register)
```

#### Register File
```
- reg_file.write_addr
- reg_file.write_data
- reg_file.write_enable
```

## Test Cases for Debugging

### Test 1: Simple Load
```assembly
LW x1, 0(x0)    # Load from address 0
```
**Expected**:
- Address: 0x00000000
- Word address: 0
- MemRead: 1 for one cycle
- Read data: Memory[0]

### Test 2: Simple Store
```assembly
ADDI x2, x0, 0x12345678
SW x2, 0(x0)    # Store to address 0
```
**Expected**:
- Address: 0x00000000
- Word address: 0
- MemWrite: 1 for one cycle
- Write data: 0x12345678
- Memory[0] = 0x12345678 after write

### Test 3: Load-Use Hazard
```assembly
LW x1, 0(x0)    # Load
ADD x2, x1, x0  # Use loaded value immediately
```
**Expected**:
- Pipeline stalls for 1 cycle after load
- Hazard detection unit asserts stall
- Load completes before ADD uses x1

### Test 4: Store-Load Sequence
```assembly
ADDI x2, x0, 0xDEADBEEF
SW x2, 4(x0)    # Store to address 4
LW x3, 4(x0)    # Load from address 4
```
**Expected**:
- Store writes 0xDEADBEEF to Memory[1]
- Load reads 0xDEADBEEF from Memory[1]
- x3 = 0xDEADBEEF

## Common Error Patterns

### Pattern 1: Read Always Returns Zero
**Symptoms**: All loads return 0x00000000
**Possible Causes**:
- Memory not initialized
- Address out of range
- MemRead not asserted
- Address misalignment

**Debug**: Check `MemRead_reg`, `addr_in_range_reg`, `addr_aligned_reg`, `memory[word_addr_reg]`

### Pattern 2: Write Doesn't Persist
**Symptoms**: Store appears to work, but subsequent load returns old value
**Possible Causes**:
- Write enable not asserted
- Address misalignment
- Write to wrong address
- Memory array not updated

**Debug**: Check `MemWrite_reg`, `addr_aligned_reg`, `word_addr_reg`, `memory[word_addr_reg]` after write

### Pattern 3: Wrong Data Read
**Symptoms**: Load returns incorrect value
**Possible Causes**:
- Address calculation error
- Word address conversion error
- Memory initialization error
- Data forwarding issue (for stores)

**Debug**: Check address calculation, word address conversion, memory contents

### Pattern 4: Pipeline Stall Issues
**Symptoms**: Memory operations cause incorrect pipeline behavior
**Possible Causes**:
- Hazard detection not working
- Control signals corrupted during stall
- Pipeline flush affecting memory ops

**Debug**: Check `pipeline_stall_internal`, hazard detection unit, control signal propagation

## Quick Fix Checklist

1. ✅ Fix missing signal declarations in `ex_mem_reg.sv`
2. ⚠️ Review memory timing (two-cycle latency may be intentional)
3. ✅ Verify address alignment checks
4. ✅ Verify word address calculation
5. ✅ Check memory initialization
6. ✅ Verify control signal propagation
7. ✅ Check data forwarding for stores
8. ✅ Verify address range validation

## Next Steps

1. Fix the missing signal declarations in `ex_mem_reg.sv`
2. Run simulation with waveform viewer
3. Monitor all signals listed in "Essential Signals to Monitor"
4. Run test cases and compare with expected behavior
5. Add debug `$display` statements at critical points
6. Verify memory contents match expected values

