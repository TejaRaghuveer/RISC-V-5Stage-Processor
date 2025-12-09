# RISC-V Branch Instructions Test Program
# Tests all branch types: BEQ, BNE, BLT, BGE, BLTU, BGEU
# Includes: taken/not-taken cases, forward/backward branches, edge cases

# ============================================
# Section 1: Initialize Test Values
# ============================================
    ADDI x1, x0, 10        # x1 = 10 (positive)
    ADDI x2, x0, 20        # x2 = 20 (positive)
    ADDI x3, x0, 0         # x3 = 0 (zero)
    ADDI x4, x0, -10       # x4 = -10 (negative, 0xFFFFFFF6)
    ADDI x5, x0, -20       # x5 = -20 (negative, 0xFFFFFFEC)
    
    # Large positive values
    LUI x6, 0x7FFFF        # x6 = 0x7FFFF000
    ADDI x6, x6, 0xFFF     # x6 = 0x7FFFFFFF (max positive signed)
    
    # Large negative values (unsigned large positive)
    LUI x7, 0x80000        # x7 = 0x80000000 (min negative signed, max unsigned)
    ADDI x7, x7, 0         # x7 = 0x80000000
    
    # Test result registers
    ADDI x10, x0, 0        # x10 = 0 (BEQ test result)
    ADDI x11, x0, 0        # x11 = 0 (BNE test result)
    ADDI x12, x0, 0        # x12 = 0 (BLT test result)
    ADDI x13, x0, 0        # x13 = 0 (BGE test result)
    ADDI x14, x0, 0        # x14 = 0 (BLTU test result)
    ADDI x15, x0, 0        # x15 = 0 (BGEU test result)

# ============================================
# Section 2: BEQ (Branch if Equal) Tests
# ============================================
# BEQ: Branch if rs1 == rs2 (funct3 = 000)

# Test 2.1: BEQ taken (forward branch)
    ADDI x10, x0, 1        # x10 = 1 (marker before branch)
    BEQ x1, x1, beq_taken_forward  # Branch if x1 == x1 (always true)
    ADDI x10, x0, 2       # x10 = 2 (should be skipped)
beq_taken_forward:
    ADDI x10, x0, 3       # x10 = 3 (branch target - should execute)
    # Expected: x10 = 3 (branch taken)

# Test 2.2: BEQ not taken (forward branch)
    ADDI x10, x0, 10      # x10 = 10 (marker before branch)
    BEQ x1, x2, beq_not_taken_forward  # Branch if x1 == x2 (10 == 20, false)
    ADDI x10, x0, 20      # x10 = 20 (should execute - branch not taken)
beq_not_taken_forward:
    ADDI x10, x0, 30      # x10 = 30 (should execute)
    # Expected: x10 = 30 (branch not taken)

# Test 2.3: BEQ taken with zero (forward branch)
    ADDI x10, x0, 100     # x10 = 100
    BEQ x3, x3, beq_zero_taken  # Branch if x3 == x3 (0 == 0, true)
    ADDI x10, x0, 200     # x10 = 200 (should be skipped)
beq_zero_taken:
    ADDI x10, x0, 300     # x10 = 300 (branch target)
    # Expected: x10 = 300 (branch taken)

# Test 2.4: BEQ backward branch (taken)
beq_backward_start:
    ADDI x10, x0, 1000    # x10 = 1000
    BEQ x1, x1, beq_backward_start  # Branch backward if x1 == x1 (always true)
    # This creates an infinite loop if not handled, but we'll break out
    # Expected: PC jumps back to beq_backward_start

# ============================================
# Section 3: BNE (Branch if Not Equal) Tests
# ============================================
# BNE: Branch if rs1 != rs2 (funct3 = 001)

# Test 3.1: BNE taken (forward branch)
    ADDI x11, x0, 1       # x11 = 1
    BNE x1, x2, bne_taken_forward  # Branch if x1 != x2 (10 != 20, true)
    ADDI x11, x0, 2       # x11 = 2 (should be skipped)
bne_taken_forward:
    ADDI x11, x0, 3       # x11 = 3 (branch target)
    # Expected: x11 = 3 (branch taken)

# Test 3.2: BNE not taken (forward branch)
    ADDI x11, x0, 10      # x11 = 10
    BNE x1, x1, bne_not_taken_forward  # Branch if x1 != x1 (10 != 10, false)
    ADDI x11, x0, 20      # x11 = 20 (should execute)
bne_not_taken_forward:
    ADDI x11, x0, 30      # x11 = 30 (should execute)
    # Expected: x11 = 30 (branch not taken)

# Test 3.3: BNE taken with zero (forward branch)
    ADDI x11, x0, 100     # x11 = 100
    BNE x1, x3, bne_zero_taken  # Branch if x1 != x3 (10 != 0, true)
    ADDI x11, x0, 200     # x11 = 200 (should be skipped)
bne_zero_taken:
    ADDI x11, x0, 300     # x11 = 300 (branch target)
    # Expected: x11 = 300 (branch taken)

# ============================================
# Section 4: BLT (Branch if Less Than, Signed) Tests
# ============================================
# BLT: Branch if rs1 < rs2 (signed comparison, funct3 = 100)

# Test 4.1: BLT taken - positive < positive (forward branch)
    ADDI x12, x0, 1       # x12 = 1
    BLT x1, x2, blt_taken_forward  # Branch if x1 < x2 (10 < 20, true)
    ADDI x12, x0, 2       # x12 = 2 (should be skipped)
blt_taken_forward:
    ADDI x12, x0, 3       # x12 = 3 (branch target)
    # Expected: x12 = 3 (branch taken)

# Test 4.2: BLT not taken - positive >= positive (forward branch)
    ADDI x12, x0, 10      # x12 = 10
    BLT x2, x1, blt_not_taken_forward  # Branch if x2 < x1 (20 < 10, false)
    ADDI x12, x0, 20      # x12 = 20 (should execute)
blt_not_taken_forward:
    ADDI x12, x0, 30      # x12 = 30 (should execute)
    # Expected: x12 = 30 (branch not taken)

# Test 4.3: BLT taken - negative < positive (forward branch)
    ADDI x12, x0, 100     # x12 = 100
    BLT x4, x1, blt_neg_pos_taken  # Branch if x4 < x1 (-10 < 10, true)
    ADDI x12, x0, 200     # x12 = 200 (should be skipped)
blt_neg_pos_taken:
    ADDI x12, x0, 300     # x12 = 300 (branch target)
    # Expected: x12 = 300 (branch taken)

# Test 4.4: BLT taken - negative < negative (forward branch)
    ADDI x12, x0, 1000    # x12 = 1000
    BLT x5, x4, blt_neg_neg_taken  # Branch if x5 < x4 (-20 < -10, true)
    ADDI x12, x0, 2000    # x12 = 2000 (should be skipped)
blt_neg_neg_taken:
    ADDI x12, x0, 3000    # x12 = 3000 (branch target)
    # Expected: x12 = 3000 (branch taken)

# Test 4.5: BLT not taken - positive < negative (forward branch)
    ADDI x12, x0, 10000   # x12 = 10000
    BLT x1, x4, blt_pos_neg_not_taken  # Branch if x1 < x4 (10 < -10, false)
    ADDI x12, x0, 20000   # x12 = 20000 (should execute)
blt_pos_neg_not_taken:
    ADDI x12, x0, 30000   # x12 = 30000 (should execute)
    # Expected: x12 = 30000 (branch not taken)

# Test 4.6: BLT edge case - zero comparison (forward branch)
    ADDI x12, x0, 100000  # x12 = 100000
    BLT x3, x1, blt_zero_taken  # Branch if x3 < x1 (0 < 10, true)
    ADDI x12, x0, 200000  # x12 = 200000 (should be skipped)
blt_zero_taken:
    ADDI x12, x0, 300000  # x12 = 300000 (branch target)
    # Expected: x12 = 300000 (branch taken)

# ============================================
# Section 5: BGE (Branch if Greater or Equal, Signed) Tests
# ============================================
# BGE: Branch if rs1 >= rs2 (signed comparison, funct3 = 101)

# Test 5.1: BGE taken - positive > positive (forward branch)
    ADDI x13, x0, 1       # x13 = 1
    BGE x2, x1, bge_taken_forward  # Branch if x2 >= x1 (20 >= 10, true)
    ADDI x13, x0, 2       # x13 = 2 (should be skipped)
bge_taken_forward:
    ADDI x13, x0, 3       # x13 = 3 (branch target)
    # Expected: x13 = 3 (branch taken)

# Test 5.2: BGE taken - equal values (forward branch)
    ADDI x13, x0, 10      # x13 = 10
    BGE x1, x1, bge_equal_taken  # Branch if x1 >= x1 (10 >= 10, true)
    ADDI x13, x0, 20      # x13 = 20 (should be skipped)
bge_equal_taken:
    ADDI x13, x0, 30      # x13 = 30 (branch target)
    # Expected: x13 = 30 (branch taken)

# Test 5.3: BGE not taken - positive < positive (forward branch)
    ADDI x13, x0, 100     # x13 = 100
    BGE x1, x2, bge_not_taken_forward  # Branch if x1 >= x2 (10 >= 20, false)
    ADDI x13, x0, 200     # x13 = 200 (should execute)
bge_not_taken_forward:
    ADDI x13, x0, 300     # x13 = 300 (should execute)
    # Expected: x13 = 300 (branch not taken)

# Test 5.4: BGE taken - negative >= negative (forward branch)
    ADDI x13, x0, 1000    # x13 = 1000
    BGE x4, x5, bge_neg_neg_taken  # Branch if x4 >= x5 (-10 >= -20, true)
    ADDI x13, x0, 2000    # x13 = 2000 (should be skipped)
bge_neg_neg_taken:
    ADDI x13, x0, 3000    # x13 = 3000 (branch target)
    # Expected: x13 = 3000 (branch taken)

# Test 5.5: BGE taken - positive >= negative (forward branch)
    ADDI x13, x0, 10000   # x13 = 10000
    BGE x1, x4, bge_pos_neg_taken  # Branch if x1 >= x4 (10 >= -10, true)
    ADDI x13, x0, 20000   # x13 = 20000 (should be skipped)
bge_pos_neg_taken:
    ADDI x13, x0, 30000   # x13 = 30000 (branch target)
    # Expected: x13 = 30000 (branch taken)

# ============================================
# Section 6: BLTU (Branch if Less Than, Unsigned) Tests
# ============================================
# BLTU: Branch if rs1 < rs2 (unsigned comparison, funct3 = 110)

# Test 6.1: BLTU taken - positive < positive (forward branch)
    ADDI x14, x0, 1       # x14 = 1
    BLTU x1, x2, bltu_taken_forward  # Branch if x1 < x2 (10 < 20, true)
    ADDI x14, x0, 2       # x14 = 2 (should be skipped)
bltu_taken_forward:
    ADDI x14, x0, 3       # x14 = 3 (branch target)
    # Expected: x14 = 3 (branch taken)

# Test 6.2: BLTU not taken - positive >= positive (forward branch)
    ADDI x14, x0, 10      # x14 = 10
    BLTU x2, x1, bltu_not_taken_forward  # Branch if x2 < x1 (20 < 10, false)
    BLTU x2, x1, bltu_not_taken_forward  # Branch if x2 < x1 (20 < 10, false)
    ADDI x14, x0, 20      # x14 = 20 (should execute)
bltu_not_taken_forward:
    ADDI x14, x0, 30      # x14 = 30 (should execute)
    # Expected: x14 = 30 (branch not taken)

# Test 6.3: BLTU edge case - negative treated as large unsigned (forward branch)
    # x4 = -10 = 0xFFFFFFF6 (unsigned: 4294967286)
    # x1 = 10 (unsigned: 10)
    # 4294967286 > 10, so branch should NOT be taken
    ADDI x14, x0, 100     # x14 = 100
    BLTU x1, x4, bltu_neg_unsigned  # Branch if x1 < x4 (10 < 0xFFFFFFF6, false)
    ADDI x14, x0, 200     # x14 = 200 (should execute)
bltu_neg_unsigned:
    ADDI x14, x0, 300     # x14 = 300 (should execute)
    # Expected: x14 = 300 (branch not taken, unsigned comparison)

# Test 6.4: BLTU taken - negative < negative (as unsigned) (forward branch)
    # x5 = -20 = 0xFFFFFFEC (unsigned: 4294967276)
    # x4 = -10 = 0xFFFFFFF6 (unsigned: 4294967286)
    # 4294967276 < 4294967286, so branch SHOULD be taken
    ADDI x14, x0, 1000    # x14 = 1000
    BLTU x5, x4, bltu_neg_neg_taken  # Branch if x5 < x4 (0xFFFFFFEC < 0xFFFFFFF6, true)
    ADDI x14, x0, 2000    # x14 = 2000 (should be skipped)
bltu_neg_neg_taken:
    ADDI x14, x0, 3000    # x14 = 3000 (branch target)
    # Expected: x14 = 3000 (branch taken, unsigned comparison)

# Test 6.5: BLTU edge case - zero comparison (forward branch)
    ADDI x14, x0, 10000   # x14 = 10000
    BLTU x3, x1, bltu_zero_taken  # Branch if x3 < x1 (0 < 10, true)
    ADDI x14, x0, 20000   # x14 = 20000 (should be skipped)
bltu_zero_taken:
    ADDI x14, x0, 30000   # x14 = 30000 (branch target)
    # Expected: x14 = 30000 (branch taken)

# ============================================
# Section 7: BGEU (Branch if Greater or Equal, Unsigned) Tests
# ============================================
# BGEU: Branch if rs1 >= rs2 (unsigned comparison, funct3 = 111)

# Test 7.1: BGEU taken - positive > positive (forward branch)
    ADDI x15, x0, 1       # x15 = 1
    BGEU x2, x1, bgeu_taken_forward  # Branch if x2 >= x1 (20 >= 10, true)
    ADDI x15, x0, 2       # x15 = 2 (should be skipped)
bgeu_taken_forward:
    ADDI x15, x0, 3       # x15 = 3 (branch target)
    # Expected: x15 = 3 (branch taken)

# Test 7.2: BGEU taken - equal values (forward branch)
    ADDI x15, x0, 10      # x15 = 10
    BGEU x1, x1, bgeu_equal_taken  # Branch if x1 >= x1 (10 >= 10, true)
    ADDI x15, x0, 20      # x15 = 20 (should be skipped)
bgeu_equal_taken:
    ADDI x15, x0, 30      # x15 = 30 (branch target)
    # Expected: x15 = 30 (branch taken)

# Test 7.3: BGEU not taken - positive < positive (forward branch)
    ADDI x15, x0, 100     # x15 = 100
    BGEU x1, x2, bgeu_not_taken_forward  # Branch if x1 >= x2 (10 >= 20, false)
    ADDI x15, x0, 200     # x15 = 200 (should execute)
bgeu_not_taken_forward:
    ADDI x15, x0, 300     # x15 = 300 (should execute)
    # Expected: x15 = 300 (branch not taken)

# Test 7.4: BGEU edge case - negative as treated large unsigned (forward branch)
    # x4 = -10 = 0xFFFFFFF6 (unsigned: 4294967286)
    # x1 = 10 (unsigned: 10)
    # 4294967286 >= 10, so branch SHOULD be taken
    ADDI x15, x0, 1000    # x15 = 1000
    BGEU x4, x1, bgeu_neg_unsigned_taken  # Branch if x4 >= x1 (0xFFFFFFF6 >= 10, true)
    ADDI x15, x0, 2000    # x15 = 2000 (should be skipped)
bgeu_neg_unsigned_taken:
    ADDI x15, x0, 3000    # x15 = 3000 (branch target)
    # Expected: x15 = 3000 (branch taken, unsigned comparison)

# Test 7.5: BGEU taken - negative >= negative (as unsigned) (forward branch)
    # x4 = -10 = 0xFFFFFFF6 (unsigned: 4294967286)
    # x5 = -20 = 0xFFFFFFEC (unsigned: 4294967276)
    # 4294967286 >= 4294967276, so branch SHOULD be taken
    ADDI x15, x0, 10000   # x15 = 10000
    BGEU x4, x5, bgeu_neg_neg_taken  # Branch if x4 >= x5 (0xFFFFFFF6 >= 0xFFFFFFEC, true)
    ADDI x15, x0, 20000   # x15 = 20000 (should be skipped)
bgeu_neg_neg_taken:
    ADDI x15, x0, 30000   # x15 = 30000 (branch target)
    # Expected: x15 = 30000 (branch taken, unsigned comparison)

# Test 7.6: BGEU edge case - zero comparison (forward branch)
    ADDI x15, x0, 100000  # x15 = 100000
    BGEU x1, x3, bgeu_zero_taken  # Branch if x1 >= x3 (10 >= 0, true)
    ADDI x15, x0, 200000  # x15 = 200000 (should be skipped)
bgeu_zero_taken:
    ADDI x15, x0, 300000  # x15 = 300000 (branch target)
    # Expected: x15 = 300000 (branch taken)

# ============================================
# Section 8: Store Results for Verification
# ============================================
    SW x10, 0(x0)         # Store BEQ test result
    SW x11, 4(x0)         # Store BNE test result
    SW x12, 8(x0)         # Store BLT test result
    SW x13, 12(x0)        # Store BGE test result
    SW x14, 16(x0)        # Store BLTU test result
    SW x15, 20(x0)        # Store BGEU test result

# ============================================
# Section 9: End Marker
# ============================================
    NOP                   # ADDI x0, x0, 0
    NOP                   # ADDI x0, x0, 0
    NOP                   # ADDI x0, x0, 0

# ============================================
# Expected Final Register State:
# ============================================
# x1 = 10
# x2 = 20
# x3 = 0
# x4 = -10 (0xFFFFFFF6)
# x5 = -20 (0xFFFFFFEC)
# x6 = 0x7FFFFFFF
# x7 = 0x80000000
# x10 = 300 (BEQ test result)
# x11 = 300 (BNE test result)
# x12 = 300000 (BLT test result)
# x13 = 30000 (BGE test result)
# x14 = 30000 (BLTU test result)
# x15 = 300000 (BGEU test result)

# ============================================
# Expected Final Memory State:
# ============================================
# memory[0] = 300 (BEQ result)
# memory[1] = 300 (BNE result)
# memory[2] = 300000 (BLT result)
# memory[3] = 30000 (BGE result)
# memory[4] = 30000 (BLTU result)
# memory[5] = 300000 (BGEU result)

