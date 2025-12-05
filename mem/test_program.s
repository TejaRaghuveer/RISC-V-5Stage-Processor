# RISC-V Assembly Test Program
# This file contains the assembly source code for reference
# The actual machine code is in test_program.hex

# ============================================
# Section 1: Initialize Registers (ADDI)
# ============================================
    ADDI x1, x0, 5      # x1 = 5
    ADDI x2, x0, 3      # x2 = 3
    ADDI x3, x0, 0      # x3 = 0 (clear for ADD result)
    ADDI x4, x0, 0      # x4 = 0 (clear for SUB result)
    ADDI x5, x0, 0      # x5 = 0 (clear for AND result)
    ADDI x6, x0, 0      # x6 = 0 (clear for OR result)
    ADDI x7, x0, 0      # x7 = 0 (clear for XOR result)
    ADDI x8, x0, 0      # x8 = 0 (clear for load result)
    ADDI x9, x0, 0      # x9 = 0 (clear for branch test)
    ADDI x10, x0, 0     # x10 = 0 (clear for jump test)

# ============================================
# Section 2: Arithmetic Operations (ADD, SUB)
# ============================================
    ADD x3, x1, x2      # x3 = x1 + x2 = 5 + 3 = 8
    SUB x4, x2, x1      # x4 = x2 - x1 = 3 - 5 = -2

# ============================================
# Section 3: Logical Operations (AND, OR, XOR)
# ============================================
    AND x5, x1, x2      # x5 = x1 & x2 = 5 & 3 = 1
    OR x6, x1, x2       # x6 = x1 | x2 = 5 | 3 = 7
    XOR x7, x1, x2      # x7 = x1 ^ x2 = 5 ^ 3 = 6

# ============================================
# Section 4: Store Results to Memory
# ============================================
    SW x3, 0(x0)        # Store x3 (ADD result = 8) to memory[0]
    SW x4, 4(x0)        # Store x4 (SUB result) to memory[1]
    SW x5, 8(x0)        # Store x5 (AND result) to memory[2]
    SW x6, 12(x0)       # Store x6 (OR result) to memory[3]
    SW x7, 16(x0)       # Store x7 (XOR result) to memory[4]

# ============================================
# Section 5: Load from Memory (LW)
# ============================================
    LW x8, 0(x0)        # Load from memory[0] into x8, should be 8

# ============================================
# Section 6: Branch Test (BEQ)
# ============================================
    ADDI x9, x0, 1      # x9 = 1 (before branch)
    BEQ x1, x1, branch_target  # Branch if x1 == x1 (always true)
    ADDI x9, x0, 2      # x9 = 2 (skipped if branch taken)
branch_target:
    ADDI x9, x0, 3      # x9 = 3 (branch target - should execute)
    ADDI x9, x0, 4      # x9 = 4 (after branch - should execute)

# ============================================
# Section 7: Jump Test (JAL)
# ============================================
    JAL x10, jump_target  # Jump forward, save PC+4 in x10
    ADDI x10, x0, 5     # x10 = 5 (skipped if jump taken)
    ADDI x10, x0, 6     # x10 = 6 (skipped if jump taken)
jump_target:
    ADDI x10, x0, 66    # x10 = 66 = 0x42 (jump target - should execute)

# ============================================
# Section 8: Final NOPs and End Marker
# ============================================
    NOP                 # ADDI x0, x0, 0 (end marker)
    NOP                 # ADDI x0, x0, 0
    NOP                 # ADDI x0, x0, 0

# Expected Final Register State:
# x1 = 5
# x2 = 3
# x3 = 8 (ADD result)
# x4 = -2 = 0xFFFFFFFE (SUB result)
# x5 = 1 (AND result)
# x6 = 7 (OR result)
# x7 = 6 (XOR result)
# x8 = 8 (loaded from memory)
# x9 = 4 (branch test result)
# x10 = 66 = 0x42 (jump test result)

# Expected Final Memory State:
# memory[0] = 8
# memory[1] = -2 = 0xFFFFFFFE
# memory[2] = 1
# memory[3] = 7
# memory[4] = 6

