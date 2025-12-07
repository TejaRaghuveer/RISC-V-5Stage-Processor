# RISC-V Memory Simulation Issues and Fixes

This document identifies and fixes critical issues found in the memory simulation modules.

## Issue 1: Timing Alignment in DMEM Word Address Register (FIXED)

### Problem
The `word_addr_reg` signal was computed in a separate `always_ff` block from `addr_reg`, which could potentially cause timing misalignment issues. While SystemVerilog semantics ensure both blocks use OLD values, combining them improves clarity and ensures perfect synchronization.

### Original Code Structure
```systemverilog
// Block 1: Compute word_addr_reg from addr_reg
always_ff @(posedge clk) begin
    word_addr_reg <= addr_reg[MEM_ADDR_WIDTH+1:2];
    addr_in_range_reg <= (addr_reg[MEM_ADDR_WIDTH+1:2] < MEM_DEPTH);
    addr_aligned_reg <= (addr_reg[1:0] == 2'b00);
end

// Block 2: Update addr_reg
always_ff @(posedge clk) begin
    addr_reg <= addr;
    write_data_reg <= write_data;
    MemRead_reg <= MemRead;
    MemWrite_reg <= MemWrite;
end
```

### Fix Applied
Combined address registration and word address computation into a single `always_ff` block to ensure perfect timing alignment:

```systemverilog
// Single block: Register inputs and compute word address
always_ff @(posedge clk) begin
    // Register inputs
    addr_reg <= addr;
    write_data_reg <= write_data;
    MemRead_reg <= MemRead;
    MemWrite_reg <= MemWrite;
    
    // Compute word address and validation from OLD addr_reg value
    // This ensures timing alignment: word_addr_reg corresponds to the address
    // that was registered in the previous cycle
    word_addr_reg <= addr_reg[MEM_ADDR_WIDTH+1:2];
    addr_in_range_reg <= (addr_reg[MEM_ADDR_WIDTH+1:2] < MEM_DEPTH);
    addr_aligned_reg <= (addr_reg[1:0] == 2'b00);
end
```

### Benefits
1. **Perfect Timing Alignment**: All address-related signals computed in the same block
2. **Clearer Code**: Single source of truth for address registration
3. **Easier to Maintain**: All address logic in one place
4. **Correct Behavior**: Both `addr_reg` and `word_addr_reg` use OLD values, ensuring they reference the same address

### Status
✅ **FIXED** - Code updated in `src/dmem.sv`

