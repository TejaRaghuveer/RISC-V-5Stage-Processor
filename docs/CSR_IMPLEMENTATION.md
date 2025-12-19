# CSR (Control and Status Register) Implementation Summary

This document summarizes the CSR implementation for the RISC-V processor.

## Files Created

1. **`src/csr_file.sv`**: CSR register file module
   - Implements mstatus, mtvec, mcause, mepc registers
   - Supports CSRRW, CSRRS, CSRRC operations
   - Provides CSR read/write interface

## Files Modified

1. **`src/control_unit.sv`**: Added CSR instruction detection
   - Added `CSRRead` and `CSRWrite` output signals
   - Detects CSR instructions (opcode 7'b1110011)

2. **`src/id_stage.sv`**: Added CSR interface
   - Extracts CSR address from instruction [31:20]
   - Connects to CSR file for read operations
   - Handles ECALL/EBREAK detection
   - Disables CSR signals for ECALL/EBREAK

3. **`src/id_ex_reg.sv`**: Added CSR signal propagation
   - Added CSR address, read data, and control signals
   - Passes CSR signals from ID to EX stage

## Remaining Integration Steps

### 1. Modify EX Stage (`src/ex_stage.sv`)

Add CSR operation logic:

```systemverilog
// Add inputs
input  logic [11:0]       ex_csr_addr,
input  logic [31:0]       ex_csr_read_data,
input  logic             ex_CSRRead,
input  logic             ex_CSRWrite,

// CSR operation result
logic [31:0] csr_result;
logic [1:0] csr_write_type;

always_comb begin
    if (ex_CSRRead) begin
        case (ex_funct3)
            3'b000: begin  // CSRRW
                csr_result = ex_rs1_data;
                csr_write_type = 2'b00;
            end
            3'b001: begin  // CSRRS
                csr_result = ex_csr_read_data | ex_rs1_data;
                csr_write_type = 2'b01;
            end
            3'b010: begin  // CSRRC
                csr_result = ex_csr_read_data & ~ex_rs1_data;
                csr_write_type = 2'b10;
            end
            default: begin
                csr_result = ex_csr_read_data;
                csr_write_type = 2'b00;
            end
        endcase
    end else begin
        csr_result = 32'b0;
        csr_write_type = 2'b00;
    end
end

// ALU result selection: Use CSR read data for writeback
assign alu_result = ex_CSRRead ? ex_csr_read_data : alu_result_normal;
```

### 2. Modify EX/MEM Register (`src/ex_mem_reg.sv`)

Add CSR signals to EX/MEM register:

```systemverilog
// Add inputs
input  logic [11:0]       ex_csr_addr,
input  logic [31:0]       ex_csr_result,
input  logic [1:0]        ex_csr_write_type,
input  logic             ex_CSRWrite,
input  logic [4:0]       ex_rs1_addr,

// Add outputs
output logic [11:0]      mem_csr_addr,
output logic [31:0]      mem_csr_result,
output logic [1:0]       mem_csr_write_type,
output logic             mem_CSRWrite,
output logic [4:0]       mem_rs1_addr
```

### 3. Modify MEM/WB Register (`src/mem_wb_reg.sv`)

Add CSR signals to MEM/WB register:

```systemverilog
// Add inputs
input  logic [11:0]      mem_csr_addr,
input  logic [31:0]     mem_csr_result,
input  logic [1:0]      mem_csr_write_type,
input  logic            mem_CSRWrite,
input  logic [4:0]      mem_rs1_addr,

// Add outputs
output logic [11:0]     wb_csr_addr,
output logic [31:0]     wb_csr_result,
output logic [1:0]     wb_csr_write_type,
output logic           wb_CSRWrite,
output logic [4:0]     wb_rs1_addr
```

### 4. Modify WB Stage (`src/wb_stage.sv`)

Add CSR write interface:

```systemverilog
// Add inputs
input  logic [11:0]     wb_csr_addr,
input  logic [31:0]    wb_csr_result,
input  logic [1:0]     wb_csr_write_type,
input  logic           wb_CSRWrite,
input  logic [4:0]     wb_rs1_addr,

// Add outputs
output logic           csr_write_en,
output logic [11:0]   csr_write_addr,
output logic [31:0]   csr_write_data,
output logic [1:0]    csr_write_type,

// CSR write logic
assign csr_write_en = wb_CSRWrite && (wb_rs1_addr != 5'b00000);
assign csr_write_addr = wb_csr_addr;
assign csr_write_data = wb_csr_result;
assign csr_write_type = wb_csr_write_type;

// Writeback data selection (add CSR read data)
assign wb_write_data = wb_CSRRead ? wb_csr_read_data : 
                      (mem_wb_MemToReg ? mem_wb_mem_read_data : mem_wb_alu_result);
```

### 5. Modify Main Pipeline (`src/riscv_pipeline.sv`)

Add CSR file instance and connect signals:

```systemverilog
// CSR File Instance
csr_file #(
    .DATA_WIDTH(32)
) csr_file_inst (
    .clk(clk),
    .rst_n(rst_n),
    .csr_addr(id_csr_addr),              // From ID stage
    .csr_read_data(csr_read_data),       // To ID stage
    .csr_write_en(wb_csr_write_en),      // From WB stage
    .csr_write_addr(wb_csr_write_addr),  // From WB stage
    .csr_write_data(wb_csr_write_data), // From WB stage
    .csr_write_type(wb_csr_write_type),  // From WB stage
    .mstatus(mstatus),
    .mtvec(mtvec),
    .mcause(mcause),
    .mepc(mepc)
);

// Connect CSR read data to ID stage
// (Add csr_read_data input to id_stage)
```

## CSR Instruction Flow

### CSRRW Example

```
Cycle 1 (ID): 
  - Extract CSR address from instruction [31:20]
  - Read CSR value (combinational)
  - Pass CSR read data to EX stage

Cycle 2 (EX):
  - Perform CSR operation: csr_result = rs1_data
  - Pass CSR read data as ALU result (for writeback to rd)
  - Pass CSR write data and type to MEM stage

Cycle 3 (MEM):
  - Pass CSR signals through (no memory operation)

Cycle 4 (WB):
  - Write CSR: CSR[csr_addr] = csr_result
  - Write register: x[rd] = csr_read_data
```

### CSRRS Example

```
Cycle 1 (ID):
  - Extract CSR address
  - Read CSR value

Cycle 2 (EX):
  - Perform CSR operation: csr_result = csr_read_data | rs1_data
  - Pass CSR read data for writeback

Cycle 3 (MEM):
  - Pass through

Cycle 4 (WB):
  - Write CSR: CSR[csr_addr] = csr_result (if rs1 != 0)
  - Write register: x[rd] = csr_read_data
```

## Testing

### Basic Test Cases

1. **CSRRW Test**:
   ```assembly
   ADDI x1, x0, 0x12345678
   CSRRW x2, mstatus, x1    # Write x1 to mstatus, read old value to x2
   ```

2. **CSRRS Test**:
   ```assembly
   ADDI x1, x0, 0x00000008  # Set bit 3
   CSRRS x2, mstatus, x1    # Set bit 3 in mstatus, read old value to x2
   ```

3. **CSRRC Test**:
   ```assembly
   ADDI x1, x0, 0x00000008  # Clear bit 3
   CSRRC x2, mstatus, x1    # Clear bit 3 in mstatus, read old value to x2
   ```

4. **Read-Only Test**:
   ```assembly
   CSRRS x1, mstatus, x0    # Read mstatus, no write (rs1=0)
   ```

## Compliance

This implementation provides:
- ✅ Basic CSRs (mstatus, mtvec, mcause, mepc)
- ✅ CSRRW instruction
- ✅ CSRRS instruction
- ✅ CSRRC instruction
- ✅ ECALL/EBREAK detection
- ✅ Register x0 handling

For full RV32I compliance, additional CSRs may be needed based on specific requirements.

