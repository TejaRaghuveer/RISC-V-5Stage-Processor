# RISC-V Pipeline Top Module Review

## Issues Found and Fixes

### Critical Issues

#### 1. **Branch/Jump Timing Issue** ⚠️ CRITICAL
**Problem**: Branch/jump target selection uses combinational signals from EX stage (`ex_Jump`, `ex_Branch`, `ex_branch_taken`), causing timing violations. PC update should use registered signals from EX/MEM register.

**Current Code** (lines 424-438):
```systemverilog
always_comb begin
    if (ex_Jump) begin
        branch_target = ex_jump_target;
        branch_taken = 1'b1;
    end else if (ex_Branch && ex_branch_taken) begin
        branch_target = ex_branch_target;
        branch_taken = 1'b1;
    end else begin
        branch_target = {ADDR_WIDTH{1'b0}};
        branch_taken = 1'b0;
    end
end
```

**Issue**: Uses combinational signals `ex_Jump`, `ex_Branch`, `ex_branch_taken` directly from EX stage.

**Fix**: Use registered signals from EX/MEM register:
```systemverilog
always_comb begin
    // Use registered signals from EX/MEM register for proper timing
    if (mem_Jump) begin
        branch_target = mem_jump_target;
        branch_taken = mem_branch_taken;  // Always 1 for jumps
    end else if (mem_Branch && mem_branch_taken) begin
        branch_target = mem_branch_target;
        branch_taken = 1'b1;
    end else begin
        branch_target = {ADDR_WIDTH{1'b0}};
        branch_taken = 1'b0;
    end
end
```

**Note**: Need to add `mem_Jump` and `mem_Branch` signals from EX/MEM register.

---

#### 2. **Missing Branch/Jump Control Signals in EX/MEM Register** ⚠️ CRITICAL
**Problem**: EX/MEM register stores branch/jump targets but doesn't store `Jump` and `Branch` control signals needed for PC update logic.

**Fix**: Add `Jump` and `Branch` signals to EX/MEM register:
- Add inputs: `ex_Jump`, `ex_Branch` to EX/MEM register
- Add outputs: `mem_Jump`, `mem_Branch` from EX/MEM register
- Connect in top-level module

---

#### 3. **Pipeline Flush Generation** ⚠️ IMPORTANT
**Problem**: Flush signal is passed as input but should be generated internally when branch/jump is taken.

**Current**: `pipeline_flush` is an input port.

**Fix**: Generate flush signal internally:
```systemverilog
// Generate flush signal when branch/jump is taken
assign pipeline_flush_internal = branch_taken;
```

**Note**: Keep `pipeline_flush` as input for external control (exceptions), but also generate internally.

---

### Moderate Issues

#### 4. **IMEM Address Width Consistency** ⚠️ MODERATE
**Problem**: IF stage outputs `imem_addr[IMEM_ADDR_WIDTH-1:0]` but IMEM expects `addr[ADDR_WIDTH-1:0]` where `ADDR_WIDTH` parameter name conflicts with pipeline `ADDR_WIDTH`.

**Current**: 
- IF stage: `output logic [IMEM_ADDR_WIDTH-1:0] imem_addr`
- IMEM: `input logic [ADDR_WIDTH-1:0] addr` (where ADDR_WIDTH = 10 for memory)

**Status**: Actually correct - IMEM's `ADDR_WIDTH` parameter is the memory address width (10 bits), which matches `IMEM_ADDR_WIDTH`. The connection is correct.

---

#### 5. **Forwarding Data Path Timing** ✅ CORRECT
**Status**: Forwarding paths are correctly connected:
- EX/MEM → EX: `mem_alu_result` (registered)
- MEM/WB → EX: `wb_write_data` (registered)
- Forwarding unit uses registered addresses and control signals

---

#### 6. **Write-back Path** ✅ CORRECT
**Status**: Write-back path is correctly connected:
- WB stage → ID stage register file
- Signals: `wb_RegWrite`, `wb_rd_addr`, `wb_write_data`
- All properly connected

---

### Minor Issues / Improvements

#### 7. **Signal Naming Consistency**
**Suggestion**: Use consistent naming for pipeline register signals:
- Current: Mix of `ex_*` and `mem_*` prefixes
- Better: Use stage prefixes consistently (e.g., `ex_mem_*` for EX/MEM register outputs)

#### 8. **Comments Enhancement**
**Suggestion**: Add timing diagrams or more detailed comments explaining:
- Pipeline register enable/flush behavior
- Forwarding path timing
- Branch/jump feedback timing

#### 9. **Default Values**
**Suggestion**: Initialize all signals explicitly to avoid X-propagation:
```systemverilog
logic [ADDR_WIDTH-1:0] branch_target = {ADDR_WIDTH{1'b0}};
logic branch_taken = 1'b0;
```

---

## Required Fixes Summary

1. ✅ **Fix Branch/Jump Timing**: Use EX/MEM registered signals instead of EX combinational signals - **FIXED**
2. ✅ **Add Jump/Branch to EX/MEM Register**: Store Jump and Branch control signals - **FIXED**
3. ✅ **Generate Flush Signal**: Create internal flush signal from branch_taken - **FIXED**
4. ✅ **Update EX/MEM Register**: Add Jump and Branch inputs/outputs - **FIXED**

---

## Fixes Applied

### 1. EX/MEM Register Updates
- Added `ex_Branch` and `ex_Jump` inputs
- Added `mem_Branch` and `mem_Jump` outputs
- Added internal registers `Branch_reg` and `Jump_reg`
- Updated register update logic to store Branch and Jump signals

### 2. Top-Level Module Updates
- Changed branch/jump target selection to use `mem_Jump`, `mem_Branch`, `mem_branch_taken` (registered signals)
- Added `pipeline_flush_internal` signal generation: `branch_taken || pipeline_flush`
- Updated all pipeline register flush connections to use `pipeline_flush_internal`
- Added `mem_Branch` and `mem_Jump` signal declarations
- Updated EX/MEM register instantiation to include Branch and Jump signals

### 3. Timing Improvements
- Branch/jump decisions now use registered signals from EX/MEM register
- Ensures proper timing alignment - PC update happens one cycle after branch evaluation
- Flush signal properly generated when branch/jump is taken

---

## Verification Checklist

- [x] All signal widths match
- [x] Write-back path connected correctly
- [x] Forwarding paths connected correctly
- [x] Branch/jump timing fixed - **NOW USES REGISTERED SIGNALS**
- [x] Pipeline flush logic correct - **NOW GENERATES INTERNAL FLUSH**
- [x] Pipeline register enable/flush logic correct
- [x] Control signal routing correct
- [x] EX/MEM register includes Branch and Jump signals

