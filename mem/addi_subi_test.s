# RISC-V ADDI Test Program
# Tests immediate arithmetic operations (ADDI with positive/negative immediates)
# Note: RISC-V doesn't have SUBI instruction, but ADDI with negative immediate achieves subtraction
# Self-checking: Results stored in memory for verification

# ============================================
# Section 1: Initialize Base Values
# ============================================
    ADDI x1, x0, 10         # x1 = 10 (base value for testing)
    ADDI x2, x0, 5          # x2 = 5 (base value)
    ADDI x3, x0, 0          # x3 = 0 (zero)
    LUI x4, 0x7FFFF         # x4 = 0x7FFFF000
    ADDI x4, x4, 0xFFF      # x4 = 0x7FFFFFFF (max positive)
    LUI x5, 0x80000         # x5 = 0x80000000 (min negative)

# ============================================
# Section 2: ADDI with Positive Immediates
# ============================================
    ADDI x10, x1, 5         # x10 = 10 + 5 = 15 (positive immediate)
    ADDI x11, x1, 0         # x11 = 10 + 0 = 10 (zero immediate)
    ADDI x12, x0, 42        # x12 = 0 + 42 = 42 (add to zero)
    ADDI x13, x1, 1         # x13 = 10 + 1 = 11 (small positive)
    ADDI x14, x1, 2047      # x14 = 10 + 2047 = 2057 (max positive 12-bit immediate)

# ============================================
# Section 3: ADDI with Negative Immediates (acts as SUBI)
# ============================================
    ADDI x20, x1, -5        # x20 = 10 + (-5) = 5 (negative immediate, acts as subtraction)
    ADDI x21, x1, -10       # x21 = 10 + (-10) = 0 (negative immediate, zero result)
    ADDI x22, x1, -1        # x22 = 10 + (-1) = 9 (negative immediate, decrement)
    ADDI x23, x2, -3        # x23 = 5 + (-3) = 2 (negative immediate)
    ADDI x24, x0, -1        # x24 = 0 + (-1) = -1 = 0xFFFFFFFF (negative immediate from zero)
    ADDI x25, x0, -2048     # x25 = 0 + (-2048) = -2048 = 0xFFFFF800 (min negative 12-bit immediate)

# ============================================
# Section 4: ADDI with Large Positive Values
# ============================================
    ADDI x30, x4, 0         # x30 = 0x7FFFFFFF + 0 = 0x7FFFFFFF (max positive)
    ADDI x31, x4, -1        # x31 = 0x7FFFFFFF + (-1) = 0x7FFFFFFE (max positive - 1)

# ============================================
# Section 5: ADDI with Large Negative Values
# ============================================
    ADDI x28, x5, 0         # x28 = 0x80000000 + 0 = 0x80000000 (min negative)
    ADDI x29, x5, 1         # x29 = 0x80000000 + 1 = 0x80000001 (min negative + 1)

# ============================================
# Section 6: ADDI Overflow Cases
# ============================================
    # Test: Max positive + 1 (should wrap to min negative)
    ADDI x6, x4, 1          # x6 = 0x7FFFFFFF + 1 = 0x80000000 (overflow wrap)
    
    # Test: Max positive + large positive immediate
    ADDI x7, x4, 2047      # x7 = 0x7FFFFFFF + 2047 = 0x800007FE (overflow wrap)
    
    # Test: Min negative + (-1) (should wrap to max positive)
    ADDI x8, x5, -1        # x8 = 0x80000000 + (-1) = 0x7FFFFFFF (underflow wrap)

# ============================================
# Section 7: ADDI Edge Cases
# ============================================
    ADDI x15, x3, 0         # x15 = 0 + 0 = 0 (zero + zero)
    ADDI x16, x3, 1         # x16 = 0 + 1 = 1 (zero + positive)
    ADDI x17, x3, -1        # x17 = 0 + (-1) = -1 = 0xFFFFFFFF (zero + negative)
    ADDI x18, x1, -10       # x18 = 10 + (-10) = 0 (cancellation)
    ADDI x19, x1, -11       # x19 = 10 + (-11) = -1 = 0xFFFFFFFF (negative result)

# ============================================
# Section 8: Store Results for Verification
# ============================================
    # Store positive immediate results
    SW x10, 0(x0)           # memory[0] = 15 (10+5)
    SW x11, 4(x0)           # memory[1] = 10 (10+0)
    SW x12, 8(x0)           # memory[2] = 42 (0+42)
    SW x13, 12(x0)          # memory[3] = 11 (10+1)
    SW x14, 16(x0)          # memory[4] = 2057 (10+2047)
    
    # Store negative immediate results (SUBI-like)
    SW x20, 20(x0)          # memory[5] = 5 (10-5)
    SW x21, 24(x0)          # memory[6] = 0 (10-10)
    SW x22, 28(x0)          # memory[7] = 9 (10-1)
    SW x23, 32(x0)          # memory[8] = 2 (5-3)
    SW x24, 36(x0)          # memory[9] = -1 = 0xFFFFFFFF (0-1)
    SW x25, 40(x0)          # memory[10] = -2048 = 0xFFFFF800 (0-2048)
    
    # Store overflow results
    SW x6, 44(x0)           # memory[11] = 0x80000000 (max+1 overflow)
    SW x7, 48(x0)           # memory[12] = 0x800007FE (max+2047 overflow)
    SW x8, 52(x0)           # memory[13] = 0x7FFFFFFF (min-1 underflow)
    
    # Store edge case results
    SW x15, 56(x0)          # memory[14] = 0 (0+0)
    SW x16, 60(x0)          # memory[15] = 1 (0+1)
    SW x17, 64(x0)          # memory[16] = -1 = 0xFFFFFFFF (0-1)
    SW x18, 68(x0)          # memory[17] = 0 (10-10)
    SW x19, 72(x0)          # memory[18] = -1 = 0xFFFFFFFF (10-11)

# ============================================
# Section 9: Self-Checking Verification
# ============================================
    # Verify: memory[0] should be 15
    LW x1, 0(x0)            # Load result
    ADDI x2, x0, 15         # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 76(x0)           # Store comparison (0 = pass)
    
    # Verify: memory[5] should be 5 (10-5)
    LW x1, 20(x0)           # Load result
    ADDI x2, x0, 5          # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 80(x0)           # Store comparison (0 = pass)
    
    # Verify: memory[9] should be -1 = 0xFFFFFFFF
    LW x1, 36(x0)           # Load result
    ADDI x2, x0, -1         # Expected value (-1)
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 84(x0)           # Store comparison (0 = pass)
    
    # Verify: memory[11] should be 0x80000000 (overflow)
    LW x1, 44(x0)           # Load overflow result
    LUI x2, 0x80000         # Expected: 0x80000000
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 88(x0)           # Store comparison (0 = pass)

# ============================================
# End Marker
# ============================================
    NOP                     # ADDI x0, x0, 0
    NOP                     # ADDI x0, x0, 0

# Expected Results Summary:
# Registers:
#   x10 = 15 (10+5)
#   x11 = 10 (10+0)
#   x12 = 42 (0+42)
#   x13 = 11 (10+1)
#   x14 = 2057 (10+2047)
#   x20 = 5 (10-5)
#   x21 = 0 (10-10)
#   x22 = 9 (10-1)
#   x23 = 2 (5-3)
#   x24 = -1 = 0xFFFFFFFF (0-1)
#   x25 = -2048 = 0xFFFFF800 (0-2048)
#   x6 = 0x80000000 (max+1 overflow)
#   x7 = 0x800007FE (max+2047 overflow)
#   x8 = 0x7FFFFFFF (min-1 underflow)
#   x15 = 0 (0+0)
#   x16 = 1 (0+1)
#   x17 = -1 = 0xFFFFFFFF (0-1)
#   x18 = 0 (10-10)
#   x19 = -1 = 0xFFFFFFFF (10-11)
#
# Memory (for verification):
#   memory[0] = 15
#   memory[1] = 10
#   memory[2] = 42
#   memory[3] = 11
#   memory[4] = 2057
#   memory[5] = 5
#   memory[6] = 0
#   memory[7] = 9
#   memory[8] = 2
#   memory[9] = -1 = 0xFFFFFFFF
#   memory[10] = -2048 = 0xFFFFF800
#   memory[11] = 0x80000000 (overflow)
#   memory[12] = 0x800007FE (overflow)
#   memory[13] = 0x7FFFFFFF (underflow)
#   memory[14] = 0
#   memory[15] = 1
#   memory[16] = -1 = 0xFFFFFFFF
#   memory[17] = 0
#   memory[18] = -1 = 0xFFFFFFFF
#   memory[19] = 0 (comparison pass)
#   memory[20] = 0 (comparison pass)
#   memory[21] = 0 (comparison pass)
#   memory[22] = 0 (comparison pass)

