# RISC-V Assembly Test Program: RAW Data Hazard Tests
# This program tests Read-After-Write (RAW) data hazards and forwarding mechanisms
# 
# Pipeline Stages:
#   IF: Instruction Fetch
#   ID: Instruction Decode
#   EX: Execute
#   MEM: Memory Access
#   WB: Write Back
#
# Hazard Types Tested:
#   1. EX Hazard: Forward from EX/MEM stage
#   2. MEM Hazard: Forward from MEM/WB stage
#   3. Load-Use Hazard: Pipeline stall required

# ============================================
# Section 1: Initialize Test Values
# ============================================
    ADDI x1, x0, 10        # x1 = 10 (base value)
    ADDI x2, x0, 20        # x2 = 20 (base value)
    ADDI x3, x0, 30        # x3 = 30 (base value)
    ADDI x4, x0, 0         # x4 = 0 (clear for results)

# ============================================
# Section 2: EX Hazard Test (Forward from EX/MEM)
# ============================================
# EX Hazard: Instruction uses result from immediately previous instruction
# The result is available in EX/MEM pipeline register
# Expected: ForwardA/ForwardB = 2'b10 (forward from EX/MEM)
#
# Pipeline Timeline:
#   Cycle N:   I1 in EX, I2 in ID
#   Cycle N+1: I1 in MEM, I2 in EX (I2 uses forwarded data from I1's EX/MEM)
#
# Test Case 2.1: rs1 dependency (ForwardA)
    ADDI x5, x0, 5         # x5 = 5
    ADD x6, x5, x2         # [EX HAZARD] x6 = x5 + x2 = 5 + 20 = 25
                           # x5 is written in previous instruction, read here
                           # ForwardA = 10: Forward x5 from EX/MEM to ALU operand A
                           # Expected: x6 = 25

# Test Case 2.2: rs2 dependency (ForwardB)
    ADDI x7, x0, 7         # x7 = 7
    ADD x8, x2, x7         # [EX HAZARD] x8 = x2 + x7 = 20 + 7 = 27
                           # x7 is written in previous instruction, read here
                           # ForwardB = 10: Forward x7 from EX/MEM to ALU operand B
                           # Expected: x8 = 27

# Test Case 2.3: Both rs1 and rs2 dependencies
    ADDI x9, x0, 9         # x9 = 9
    ADDI x10, x0, 11       # x10 = 11
    ADD x11, x9, x10       # [EX HAZARD] x11 = x9 + x10 = 9 + 11 = 20
                           # Both x9 and x10 are written in previous instructions
                           # ForwardA = 10: Forward x9 from EX/MEM
                           # ForwardB = 10: Forward x10 from EX/MEM
                           # Expected: x11 = 20

# Test Case 2.4: Chain of EX hazards
    ADDI x12, x0, 12       # x12 = 12
    ADD x13, x12, x0       # [EX HAZARD] x13 = x12 + 0 = 12 (ForwardA = 10)
    ADD x14, x13, x0       # [EX HAZARD] x14 = x13 + 0 = 12 (ForwardA = 10)
    ADD x15, x14, x0       # [EX HAZARD] x15 = x14 + 0 = 12 (ForwardA = 10)
                           # Each instruction depends on previous result
                           # Expected: x13 = 12, x14 = 12, x15 = 12

# ============================================
# Section 3: MEM Hazard Test (Forward from MEM/WB)
# ============================================
# MEM Hazard: Instruction uses result from 2 instructions back
# The result is available in MEM/WB pipeline register
# Expected: ForwardA/ForwardB = 2'b01 (forward from MEM/WB)
#
# Pipeline Timeline:
#   Cycle N:   I1 in MEM, I2 in EX, I3 in ID
#   Cycle N+1: I1 in WB, I2 in MEM, I3 in EX (I3 uses forwarded data from I1's MEM/WB)
#
# Test Case 3.1: rs1 dependency (ForwardA)
    ADDI x16, x0, 16       # x16 = 16
    ADDI x17, x0, 17       # x17 = 17 (independent instruction between hazard)
    ADD x18, x16, x2       # [MEM HAZARD] x18 = x16 + x2 = 16 + 20 = 36
                           # x16 is written 2 instructions back, read here
                           # ForwardA = 01: Forward x16 from MEM/WB to ALU operand A
                           # Expected: x18 = 36

# Test Case 3.2: rs2 dependency (ForwardB)
    ADDI x19, x0, 19       # x19 = 19
    ADDI x20, x0, 20       # x20 = 20 (independent instruction between hazard)
    ADD x21, x2, x19       # [MEM HAZARD] x21 = x2 + x19 = 20 + 19 = 39
                           # x19 is written 2 instructions back, read here
                           # ForwardB = 01: Forward x19 from MEM/WB to ALU operand B
                           # Expected: x21 = 39

# Test Case 3.3: Both rs1 and rs2 dependencies (different distances)
    ADDI x22, x0, 22       # x22 = 22
    ADDI x23, x0, 23       # x23 = 23
    ADDI x24, x0, 24       # x24 = 24 (independent instruction)
    ADD x25, x22, x23      # [MEM HAZARD] x25 = x22 + x23 = 22 + 23 = 45
                           # Both x22 and x23 are written 2 instructions back
                           # ForwardA = 01: Forward x22 from MEM/WB
                           # ForwardB = 01: Forward x23 from MEM/WB
                           # Expected: x25 = 45

# Test Case 3.4: Mixed EX and MEM hazards
    ADDI x26, x0, 26       # x26 = 26
    ADDI x27, x0, 27       # x27 = 27
    ADD x28, x26, x27      # [MIXED HAZARD] x28 = x26 + x27 = 26 + 27 = 53
                           # x26: MEM hazard (2 instructions back) → ForwardA = 01
                           # x27: EX hazard (1 instruction back) → ForwardB = 10
                           # Expected: x28 = 53

# ============================================
# Section 4: Load-Use Hazard Test (Pipeline Stall)
# ============================================
# Load-Use Hazard: Load instruction followed by instruction that uses loaded data
# Data is not available until after MEM stage completes
# Expected: Pipeline stall for 1 cycle (insert bubble in ID/EX)
#
# Pipeline Timeline:
#   Cycle N:   LW in EX, ADD in ID (hazard detected)
#   Cycle N+1: LW in MEM, ADD stalled in ID (bubble inserted in ID/EX)
#   Cycle N+2: LW in WB, ADD in EX (uses forwarded data from MEM/WB)
#
# Initialize memory base address
    ADDI x29, x0, 0        # x29 = 0 (memory base address)

# Test Case 4.1: Load-use hazard with rs1 dependency
    SW x1, 0(x29)          # Store x1 (10) to memory[0]
    LW x30, 0(x29)         # Load from memory[0] into x30
    ADD x31, x30, x2       # [LOAD-USE HAZARD] x31 = x30 + x2
                           # x30 is loaded from memory, immediately used
                           # Hazard detected: id_ex_MemRead=1, id_ex_rd=x30, if_id_rs1=x30
                           # Pipeline stalls: PC and IF/ID held, bubble inserted in ID/EX
                           # After stall: Forward x30 from MEM/WB
                           # Expected: x31 = 10 + 20 = 30

# Test Case 4.2: Load-use hazard with rs2 dependency
    SW x2, 4(x29)          # Store x2 (20) to memory[1]
    LW x1, 4(x29)          # Load from memory[1] into x1
    ADD x2, x3, x1         # [LOAD-USE HAZARD] x2 = x3 + x1
                           # x1 is loaded from memory, immediately used
                           # Hazard detected: id_ex_MemRead=1, id_ex_rd=x1, if_id_rs2=x1
                           # Pipeline stalls for 1 cycle
                           # Expected: x2 = 30 + 20 = 50

# Test Case 4.3: Load-use hazard with both rs1 and rs2 dependencies
    SW x3, 8(x29)          # Store x3 (30) to memory[2]
    ADDI x4, x0, 40        # x4 = 40 (independent instruction)
    LW x5, 8(x29)          # Load from memory[2] into x5
    ADD x6, x5, x5         # [LOAD-USE HAZARD] x6 = x5 + x5
                           # x5 is loaded from memory, used in both operands
                           # Hazard detected: id_ex_MemRead=1, id_ex_rd=x5, if_id_rs1=x5, if_id_rs2=x5
                           # Pipeline stalls for 1 cycle
                           # Expected: x6 = 30 + 30 = 60

# Test Case 4.4: Load-use hazard followed by normal forwarding
    SW x1, 12(x29)         # Store x1 (10) to memory[3]
    LW x7, 12(x29)         # Load from memory[3] into x7
    ADD x8, x7, x0         # [LOAD-USE HAZARD] x8 = x7 + 0 (stall occurs)
    ADD x9, x8, x0         # [EX HAZARD] x9 = x8 + 0 (forward from EX/MEM)
                           # First ADD stalls due to load-use hazard
                           # Second ADD uses EX forwarding (no stall)
                           # Expected: x8 = 10, x9 = 10

# ============================================
# Section 5: Complex Hazard Scenarios
# ============================================
# Test Case 5.1: Multiple consecutive EX hazards
    ADDI x10, x0, 100      # x10 = 100
    ADD x11, x10, x0       # [EX HAZARD] x11 = 100 (ForwardA = 10)
    ADD x12, x11, x0       # [EX HAZARD] x12 = 100 (ForwardA = 10)
    ADD x13, x12, x0       # [EX HAZARD] x13 = 100 (ForwardA = 10)
    ADD x14, x13, x0       # [EX HAZARD] x14 = 100 (ForwardA = 10)
                           # Chain of EX hazards, each forwarding from previous
                           # Expected: x11 = 100, x12 = 100, x13 = 100, x14 = 100

# Test Case 5.2: Arithmetic operations with hazards
    ADDI x15, x0, 15       # x15 = 15
    ADDI x16, x0, 16       # x16 = 16
    ADD x17, x15, x16      # [EX HAZARD] x17 = 15 + 16 = 31 (both ForwardA=10, ForwardB=10)
    SUB x18, x17, x15     # [EX HAZARD] x18 = 31 - 15 = 16 (ForwardA=10)
    AND x19, x17, x18     # [MEM HAZARD] x19 = 31 & 16 = 16 (ForwardA=01, ForwardB=10)
    OR x20, x19, x17      # [MEM HAZARD] x20 = 16 | 31 = 31 (ForwardA=01, ForwardB=01)
                           # Mix of arithmetic and logical operations
                           # Expected: x17 = 31, x18 = 16, x19 = 16, x20 = 31

# Test Case 5.3: Store instruction with hazard (no stall needed)
    ADDI x21, x0, 21       # x21 = 21
    SW x21, 16(x29)        # Store x21 to memory[4]
                           # Store uses x21, but x21 is already in register file
                           # No hazard: Store reads register in ID stage, value available
                           # Expected: memory[4] = 21

# ============================================
# Section 6: Verification and Results
# ============================================
# Store final results to memory for verification
    SW x6, 20(x29)         # Store EX hazard test result (x6 = 25)
    SW x8, 24(x29)         # Store EX hazard test result (x8 = 27)
    SW x11, 28(x29)        # Store EX hazard test result (x11 = 20)
    SW x18, 32(x29)        # Store MEM hazard test result (x18 = 36)
    SW x21, 36(x29)        # Store MEM hazard test result (x21 = 39)
    SW x25, 40(x29)        # Store MEM hazard test result (x25 = 45)
    SW x31, 44(x29)        # Store load-use hazard test result (x31 = 30)
    SW x2, 48(x29)         # Store load-use hazard test result (x2 = 50)
    SW x6, 52(x29)         # Store load-use hazard test result (x6 = 60)
    SW x17, 56(x29)        # Store complex hazard test result (x17 = 31)
    SW x20, 60(x29)        # Store complex hazard test result (x20 = 31)

# ============================================
# Section 7: End Marker
# ============================================
    NOP                    # ADDI x0, x0, 0
    NOP                    # ADDI x0, x0, 0
    NOP                    # ADDI x0, x0, 0

# ============================================
# Expected Final Register State:
# ============================================
# x1 = 20 (overwritten by load-use test)
# x2 = 50 (overwritten by load-use test)
# x3 = 30
# x4 = 40
# x5 = 30 (from load)
# x6 = 60 (from load-use test)
# x7 = 10 (from load)
# x8 = 10 (from load-use test)
# x9 = 10 (from EX hazard after load-use)
# x10 = 100
# x11 = 100 (EX hazard chain)
# x12 = 100 (EX hazard chain)
# x13 = 100 (EX hazard chain)
# x14 = 100 (EX hazard chain)
# x15 = 15
# x16 = 16
# x17 = 31 (EX hazard)
# x18 = 16 (EX hazard)
# x19 = 16 (MEM hazard)
# x20 = 31 (MEM hazard)
# x21 = 21
# x22 = 22
# x23 = 23
# x24 = 24
# x25 = 45 (MEM hazard)
# x26 = 26
# x27 = 27
# x28 = 53 (mixed hazard)
# x29 = 0 (memory base)
# x30 = 10 (from load)
# x31 = 30 (from load-use hazard)

# ============================================
# Expected Final Memory State:
# ============================================
# memory[0] = 10
# memory[1] = 20
# memory[2] = 30
# memory[3] = 10
# memory[4] = 21
# memory[5] = 25 (EX hazard result)
# memory[6] = 27 (EX hazard result)
# memory[7] = 20 (EX hazard result)
# memory[8] = 36 (MEM hazard result)
# memory[9] = 39 (MEM hazard result)
# memory[10] = 45 (MEM hazard result)
# memory[11] = 30 (load-use hazard result)
# memory[12] = 50 (load-use hazard result)
# memory[13] = 60 (load-use hazard result)
# memory[14] = 31 (complex hazard result)
# memory[15] = 31 (complex hazard result)

