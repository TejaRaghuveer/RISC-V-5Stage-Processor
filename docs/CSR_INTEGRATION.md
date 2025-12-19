# CSR (Control and Status Register) Integration Guide

This document describes the integration of CSR instructions into the RISC-V processor pipeline.

## Overview

CSR instructions allow reading and writing control and status registers, which are essential for:
- Exception handling
- Interrupt control
- Privilege mode management
- System configuration

## Supported CSRs

For RV32I compliance, the following CSRs are implemented:

| CSR Name | Address | Description |
|----------|---------|-------------|
| mstatus  | 0x300   | Machine status register |
| mtvec    | 0x305   | Machine trap vector base address |
| mepc     | 0x341   | Machine exception program counter |
| mcause   | 0x342   | Machine cause register |

## Supported Instructions

| Instruction | Encoding | Description |
|-------------|----------|-------------|
| CSRRW  | funct3=000 | Atomic read/write CSR |
| CSRRS  | funct3=001 | Atomic read and set bits in CSR |
| CSRRC  | funct3=010 | Atomic read and clear bits in CSR |

## Pipeline Modifications

### 1. ID Stage Modifications

**Add CSR Interface**:
```systemverilog
// CSR Interface
output logic [11:0]  csr_addr,         // CSR address (12 bits)
input  logic [31:0]  csr_read_data,    // CSR read data
output logic         CSRRead,          // CSR read instruction
output logic         CSRWrite          // CSR write instruction
```

**Extract CSR Address**:
```systemverilog
assign csr_addr = instruction[31:20];  // CSR address from instruction
```

**CSR Read Operation**:
- CSR read is combinational (happens in ID stage)
- CSR value is read immediately when CSRRead is asserted
- CSR read data is passed to EX stage through pipeline register

### 2. EX Stage Modifications

**Add CSR Operation Logic**:
```systemverilog
// CSR operation: CSRRW, CSRRS, CSRRC
logic [31:0] csr_result;
logic [1:0] csr_write_type;  // 00=CSRRW, 01=CSRRS, 10=CSRRC

always_comb begin
    if (ex_CSRRead) begin
        case (ex_funct3)
            3'b000: begin  // CSRRW
                csr_result = ex_rs1_data;  // Write rs1 to CSR
                csr_write_type = 2'b00;
            end
            3'b001: begin  // CSRRS
                csr_result = ex_csr_read_data | ex_rs1_data;  // Set bits
                csr_write_type = 2'b01;
            end
            3'b010: begin  // CSRRC
                csr_result = ex_csr_read_data & ~ex_rs1_data;  // Clear bits
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

### 3. WB Stage Modifications

**Add CSR Write Interface**:
```systemverilog
// CSR Write Interface
output logic        csr_write_en,
output logic [11:0] csr_write_addr,
output logic [31:0] csr_write_data,
output logic [1:0]  csr_write_type
```

**CSR Write Operation**:
```systemverilog
// CSR write occurs in WB stage
assign csr_write_en = mem_wb_CSRWrite && (mem_wb_rs1_addr != 5'b00000);
assign csr_write_addr = mem_wb_csr_addr;
assign csr_write_data = mem_wb_csr_result;
assign csr_write_type = mem_wb_csr_write_type;
```

### 4. Pipeline Register Modifications

**ID/EX Register** - Add CSR signals:
```systemverilog
// CSR signals
input  logic [11:0]  id_csr_addr,
input  logic [31:0]  id_csr_read_data,
input  logic         id_CSRRead,
input  logic         id_CSRWrite,

output logic [11:0]  ex_csr_addr,
output logic [31:0]  ex_csr_read_data,
output logic         ex_CSRRead,
output logic         ex_CSRWrite
```

**EX/MEM Register** - Add CSR signals:
```systemverilog
// CSR signals
input  logic [11:0]  ex_csr_addr,
input  logic [31:0]  ex_csr_result,
input  logic [1:0]  ex_csr_write_type,
input  logic         ex_CSRWrite,
input  logic [4:0]  ex_rs1_addr,

output logic [11:0]  mem_csr_addr,
output logic [31:0]  mem_csr_result,
output logic [1:0]  mem_csr_write_type,
output logic         mem_CSRWrite,
output logic [4:0]  mem_rs1_addr
```

**MEM/WB Register** - Add CSR signals:
```systemverilog
// CSR signals
input  logic [11:0]  mem_csr_addr,
input  logic [31:0]  mem_csr_result,
input  logic [1:0]  mem_csr_write_type,
input  logic         mem_CSRWrite,
input  logic [4:0]  mem_rs1_addr,

output logic [11:0]  wb_csr_addr,
output logic [31:0]  wb_csr_result,
output logic [1:0]  wb_csr_write_type,
output logic         wb_CSRWrite,
output logic [4:0]  wb_rs1_addr
```

## Integration Steps

### Step 1: Instantiate CSR File

Add CSR file instance to main pipeline module:

```systemverilog
// CSR File Instance
csr_file #(
    .DATA_WIDTH(32)
) csr_file_inst (
    .clk(clk),
    .rst_n(rst_n),
    .csr_addr(csr_addr),
    .csr_read_data(csr_read_data),
    .csr_write_en(csr_write_en),
    .csr_write_addr(csr_write_addr),
    .csr_write_data(csr_write_data),
    .csr_write_type(csr_write_type),
    .mstatus(mstatus),
    .mtvec(mtvec),
    .mcause(mcause),
    .mepc(mepc)
);
```

### Step 2: Connect CSR Read in ID Stage

```systemverilog
// In id_stage.sv
assign csr_addr = instruction[31:20];  // Extract CSR address

// CSR read is combinational
// csr_read_data comes from csr_file
```

### Step 3: Connect CSR Operation in EX Stage

```systemverilog
// In ex_stage.sv
// Perform CSR operation based on funct3
// Pass CSR read data as ALU result for writeback
```

### Step 4: Connect CSR Write in WB Stage

```systemverilog
// In wb_stage.sv
// Generate CSR write signals
// Only write if rs1 != 0 (for CSRRS/CSRRC)
```

## CSR Operation Details

### CSRRW (Atomic Read/Write)

```
CSRRW rd, csr, rs1
  t = CSR[csr]
  CSR[csr] = x[rs1]
  x[rd] = t
```

- Reads old CSR value
- Writes rs1 value to CSR
- Writes old CSR value to rd

### CSRRS (Atomic Read and Set Bits)

```
CSRRS rd, csr, rs1
  t = CSR[csr]
  CSR[csr] = t | x[rs1]
  x[rd] = t
```

- Reads old CSR value
- Sets bits in CSR (OR operation)
- Writes old CSR value to rd
- If rs1 = 0, no write occurs (read-only)

### CSRRC (Atomic Read and Clear Bits)

```
CSRRC rd, csr, rs1
  t = CSR[csr]
  CSR[csr] = t & ~x[rs1]
  x[rd] = t
```

- Reads old CSR value
- Clears bits in CSR (AND with NOT)
- Writes old CSR value to rd
- If rs1 = 0, no write occurs (read-only)

## Special Cases

### ECALL/EBREAK Detection

ECALL and EBREAK use the same opcode (7'b1110011) but are not CSR instructions:

- **ECALL**: opcode=1110011, funct3=000, rs1=0, rd=0
- **EBREAK**: opcode=1110011, funct3=000, rs1=0, rd=1

These should disable CSR signals and trigger exception handling.

### Register x0 Handling

- If rd = 0, CSR read value is discarded (no register write)
- If rs1 = 0 for CSRRS/CSRRC, CSR is not modified (read-only operation)

## Testing

### Test Cases

1. **CSRRW Test**:
   ```assembly
   CSRRW x1, mstatus, x2  # Read mstatus, write x2 to mstatus
   ```

2. **CSRRS Test**:
   ```assembly
   CSRRS x1, mstatus, x2  # Read mstatus, set bits from x2
   ```

3. **CSRRC Test**:
   ```assembly
   CSRRC x1, mstatus, x2  # Read mstatus, clear bits from x2
   ```

4. **Read-Only Test**:
   ```assembly
   CSRRS x1, mstatus, x0  # Read mstatus, no write (rs1=0)
   ```

5. **Discard Result Test**:
   ```assembly
   CSRRW x0, mstatus, x2  # Write x2 to mstatus, discard read value
   ```

## Performance Impact

CSR instructions add minimal overhead:
- **ID Stage**: CSR read is combinational (no additional delay)
- **EX Stage**: CSR operation is combinational (no additional delay)
- **WB Stage**: CSR write is synchronous (same as register write)

No pipeline stalls or flushes required for CSR instructions.

## Compliance Notes

This implementation provides basic CSR support for RV32I compliance:
- Implements minimum required CSRs (mstatus, mtvec, mcause, mepc)
- Supports all three CSR write types (CSRRW, CSRRS, CSRRC)
- Handles special cases (ECALL/EBREAK, register x0)

For full compliance, additional CSRs may be needed:
- mscratch (0x340): Machine scratch register
- mtval (0x343): Machine trap value
- mie (0x304): Machine interrupt enable
- mip (0x344): Machine interrupt pending

