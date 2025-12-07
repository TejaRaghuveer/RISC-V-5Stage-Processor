# RISC-V ADD/SUB Test Program
# Tests arithmetic operations with positive/negative numbers and overflow cases
# Self-checking: Results stored in memory for verification

# ============================================
# Section 1: Initialize Test Values
# ============================================
    ADDI x1, x0, 5          # x1 = 5 (positive)
    ADDI x2, x0, 3          # x2 = 3 (positive)
    ADDI x3, x0, -5         # x3 = -5 (negative, using ADDI with sign-extended immediate)
    ADDI x4, x0, -3         # x4 = -3 (negative)
    LUI x5, 0x7FFFF         # x5 = 0x7FFFF000 (large positive, upper bits)
    ADDI x5, x5, 0xFFF      # x5 = 0x7FFFFFFF (max positive 32-bit signed: 2147483647)
    LUI x6, 0x80000         # x6 = 0x80000000 (min negative 32-bit signed: -2147483648)
    ADDI x7, x0, 0          # x7 = 0 (zero)
    ADDI x8, x0, 1          # x8 = 1 (one)

# ============================================
# Section 2: Basic ADD Operations
# ============================================
    ADD x10, x1, x2         # x10 = 5 + 3 = 8 (positive + positive)
    ADD x11, x3, x4         # x11 = -5 + (-3) = -8 (negative + negative)
    ADD x12, x1, x3         # x12 = 5 + (-5) = 0 (positive + negative, cancel)
    ADD x13, x1, x4         # x13 = 5 + (-3) = 2 (positive + negative, positive result)
    ADD x14, x3, x2         # x14 = -5 + 3 = -2 (negative + positive, negative result)
    ADD x15, x1, x7         # x15 = 5 + 0 = 5 (positive + zero)
    ADD x16, x3, x7         # x16 = -5 + 0 = -5 (negative + zero)

# ============================================
# Section 3: Basic SUB Operations
# ============================================
    SUB x20, x1, x2         # x20 = 5 - 3 = 2 (positive - positive, positive result)
    SUB x21, x2, x1         # x21 = 3 - 5 = -2 (positive - positive, negative result)
    SUB x22, x3, x4         # x22 = -5 - (-3) = -2 (negative - negative)
    SUB x23, x1, x3         # x23 = 5 - (-5) = 10 (positive - negative, addition effect)
    SUB x24, x3, x1         # x24 = -5 - 5 = -10 (negative - positive)
    SUB x25, x1, x7         # x25 = 5 - 0 = 5 (positive - zero)
    SUB x26, x2, x7         # x26 = 3 - 0 = 3 (positive - zero)

# ============================================
# Section 4: Overflow Cases (ADD)
# ============================================
    # Test: Max positive + 1 (should wrap to min negative)
    ADD x30, x5, x8         # x30 = 0x7FFFFFFF + 1 = 0x80000000 (overflow: wraps to -2147483648)
    
    # Test: Max positive + Max positive (should wrap)
    ADD x31, x5, x5         # x31 = 0x7FFFFFFF + 0x7FFFFFFF = 0xFFFFFFFE (overflow: wraps to -2)

# ============================================
# Section 5: Overflow Cases (SUB)
# ============================================
    # Test: Min negative - 1 (should wrap to max positive)
    SUB x28, x6, x8         # x28 = 0x80000000 - 1 = 0x7FFFFFFF (underflow: wraps to 2147483647)
    
    # Test: Min negative - Min negative (should be zero)
    SUB x29, x6, x6         # x29 = 0x80000000 - 0x80000000 = 0 (zero result)

# ============================================
# Section 6: Store Results for Verification
# ============================================
    # Store basic ADD results
    SW x10, 0(x0)           # memory[0] = 8 (5+3)
    SW x11, 4(x0)           # memory[1] = -8 (-5+-3)
    SW x12, 8(x0)           # memory[2] = 0 (5+-5)
    SW x13, 12(x0)          # memory[3] = 2 (5+-3)
    SW x14, 16(x0)          # memory[4] = -2 (-5+3)
    
    # Store basic SUB results
    SW x20, 20(x0)          # memory[5] = 2 (5-3)
    SW x21, 24(x0)          # memory[6] = -2 (3-5)
    SW x22, 28(x0)          # memory[7] = -2 (-5--3)
    SW x23, 32(x0)          # memory[8] = 10 (5--5)
    SW x24, 36(x0)          # memory[9] = -10 (-5-5)
    
    # Store overflow results
    SW x30, 40(x0)          # memory[10] = 0x80000000 (max+1 overflow)
    SW x31, 44(x0)          # memory[11] = 0xFFFFFFFE (max+max overflow)
    SW x28, 48(x0)          # memory[12] = 0x7FFFFFFF (min-1 underflow)
    SW x29, 52(x0)          # memory[13] = 0 (min-min)

# ============================================
# Section 7: Self-Checking Verification
# ============================================
    # Load expected values and compare
    # Expected: memory[0] = 8
    LW x1, 0(x0)            # Load result
    ADDI x2, x0, 8          # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct, non-zero if wrong
    SW x3, 56(x0)           # Store comparison result (0 = pass, non-zero = fail)
    
    # Expected: memory[1] = -8 = 0xFFFFFFF8
    LW x1, 4(x0)            # Load result
    ADDI x2, x0, -8         # Expected value (sign-extended)
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 60(x0)           # Store comparison result
    
    # Expected: memory[10] = 0x80000000 (overflow case)
    LW x1, 40(x0)           # Load overflow result
    LUI x2, 0x80000         # Expected: 0x80000000
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 64(x0)           # Store comparison result

# ============================================
# End Marker
# ============================================
    NOP                     # ADDI x0, x0, 0
    NOP                     # ADDI x0, x0, 0

# Expected Results Summary:
# Registers:
#   x10 = 8 (5+3)
#   x11 = -8 = 0xFFFFFFF8 (-5+-3)
#   x12 = 0 (5+-5)
#   x13 = 2 (5+-3)
#   x14 = -2 = 0xFFFFFFFE (-5+3)
#   x20 = 2 (5-3)
#   x21 = -2 = 0xFFFFFFFE (3-5)
#   x22 = -2 = 0xFFFFFFFE (-5--3)
#   x23 = 10 (5--5)
#   x24 = -10 = 0xFFFFFFF6 (-5-5)
#   x30 = 0x80000000 (max+1 overflow)
#   x31 = 0xFFFFFFFE (max+max overflow)
#   x28 = 0x7FFFFFFF (min-1 underflow)
#   x29 = 0 (min-min)
#
# Memory (for verification):
#   memory[0] = 8
#   memory[1] = -8 = 0xFFFFFFF8
#   memory[2] = 0
#   memory[3] = 2
#   memory[4] = -2 = 0xFFFFFFFE
#   memory[5] = 2
#   memory[6] = -2 = 0xFFFFFFFE
#   memory[7] = -2 = 0xFFFFFFFE
#   memory[8] = 10
#   memory[9] = -10 = 0xFFFFFFF6
#   memory[10] = 0x80000000 (overflow)
#   memory[11] = 0xFFFFFFFE (overflow)
#   memory[12] = 0x7FFFFFFF (underflow)
#   memory[13] = 0
#   memory[14] = 0 (comparison pass)
#   memory[15] = 0 (comparison pass)
#   memory[16] = 0 (comparison pass)

