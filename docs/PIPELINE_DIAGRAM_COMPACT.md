# RISC-V 5-Stage Pipeline - Compact Block Diagram

## Compact Pipeline Diagram (For README/Documentation)

```
┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                         RISC-V 5-STAGE PIPELINE PROCESSOR                                  │
└────────────────────────────────────────────────────────────────────────────────────────────┘

    ┌──────────┐         ┌──────────┐         ┌──────────┐         ┌──────────┐         ┌──────────┐
    │    IF    │────────▶│    ID    │────────▶│    EX    │────────▶│   MEM   │────────▶│    WB    │
    │          │         │          │         │          │         │          │         │          │
    │  ┌────┐  │         │  ┌────┐  │         │  ┌────┐  │         │  ┌────┐  │         │  ┌────┐  │
    │  │ PC │  │         │  │ RF │  │         │  │ALU │  │         │  │DMEM│  │         │  │ WB │  │
    │  └──┬─┘  │         │  └─┬──┘  │         │  └─┬──┘  │         │  └─┬──┘  │         │  └─┬──┘  │
    └─────┼────┘         └────┼────┘         └────┼────┘         └────┼────┘         └────┼────┘
          │                    │                   │                   │                   │
    ┌─────▼─────┐        ┌─────▼─────┐      ┌─────▼─────┐      ┌─────▼─────┐      ┌─────▼─────┐
    │  IF/ID    │        │  ID/EX    │      │  EX/MEM   │      │  MEM/WB   │      │   RF     │
    │ Register  │        │ Register  │      │ Register  │      │ Register  │      │ Write    │
    └───────────┘        └───────────┘      └───────────┘      └───────────┘      └───────────┘

┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    FORWARDING PATHS                                        │
│                                                                                            │
│   EX/MEM.ALU_Result ────────────────┐                                                     │
│                                       │                                                     │
│   MEM/WB.WriteData ──────────────────┼───▶ ForwardA/ForwardB Muxes ───▶ ALU Inputs      │
│                                       │                                                     │
│   Register File ─────────────────────┘                                                     │
└────────────────────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                                 HAZARD DETECTION UNIT                                      │
│                                                                                            │
│   Detects Load-Use Hazards:                                                               │
│   - Monitors ID/EX.MemRead and ID/EX.rd                                                   │
│   - Compares with IF/ID.rs1 and IF/ID.rs2                                                │
│   - Generates: Stall, PCWrite, IF/ID_Enable, ID/EX_Flush                                 │
└────────────────────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                                 BRANCH/JUMP CONTROL                                        │
│                                                                                            │
│   - Evaluates branch conditions in EX stage                                               │
│   - Computes branch/jump targets                                                          │
│   - Generates: PCSrc, Flush signals                                                      │
│   - Updates PC: branch/jump target or PC+4                                              │
└────────────────────────────────────────────────────────────────────────────────────────────┘

Legend:
  ───▶  Data flow (forward)
  ────  Control signals
  PC   = Program Counter
  RF   = Register File  
  ALU  = Arithmetic Logic Unit
  DMEM = Data Memory
  WB   = Write-Back Mux
```

## Key Components

- **IF Stage**: PC, Instruction Memory (IMEM)
- **ID Stage**: Register File, Immediate Generator, Control Unit
- **EX Stage**: ALU, Forwarding Unit, Forwarding Muxes
- **MEM Stage**: Data Memory (DMEM)
- **WB Stage**: Write-Back Mux

## Pipeline Registers

- **IF/ID**: PC+4, Instruction
- **ID/EX**: PC+4, ReadData1, ReadData2, Immediate, rs1, rs2, rd, Control Signals
- **EX/MEM**: ALU_Result, ForwardedData2, rd, Control Signals
- **MEM/WB**: ALU_Result, ReadData, rd, Control Signals

## Forwarding Paths

1. **EX/MEM → EX**: Forward ALU result (priority)
2. **MEM/WB → EX**: Forward write-back data (if no EX/MEM hazard)
3. **Register File**: Normal path when no forwarding needed
