# RISC-V SLT/SLTU Test Program
# Tests signed and unsigned comparison operations
# SLT: Set Less Than (signed comparison)
# SLTU: Set Less Than Unsigned (unsigned comparison)
# Self-checking: Results stored in memory for verification

# ============================================
# Section 1: Initialize Test Values
# ============================================
    ADDI x1, x0, 5          # x1 = 5 (positive)
    ADDI x2, x0, 3          # x2 = 3 (positive, less than x1)
    ADDI x3, x0, -5         # x3 = -5 (negative)
    ADDI x4, x0, -3         # x4 = -3 (negative, greater than x3)
    ADDI x5, x0, 0          # x5 = 0 (zero)
    LUI x6, 0x7FFFF         # x6 = 0x7FFFF000
    ADDI x6, x6, 0xFFF      # x6 = 0x7FFFFFFF (max positive signed: 2147483647)
    LUI x7, 0x80000         # x7 = 0x80000000 (min negative signed: -2147483648)
    ADDI x8, x0, 1          # x8 = 1 (one)
    LUI x9, 0xFFFFF         # x9 = 0xFFFFF000
    ADDI x9, x9, 0xFFF      # x9 = 0xFFFFFFFF (max unsigned: 4294967295, signed: -1)

# ============================================
# Section 2: SLT Tests (Signed Comparison)
# ============================================
    # Basic positive comparisons
    SLT x10, x2, x1         # x10 = (3 < 5) = 1 (true, signed)
    SLT x11, x1, x2         # x11 = (5 < 3) = 0 (false, signed)
    SLT x12, x1, x1         # x12 = (5 < 5) = 0 (false, equal)
    
    # Negative comparisons
    SLT x13, x3, x4         # x13 = (-5 < -3) = 1 (true, signed: -5 is less than -3)
    SLT x14, x4, x3         # x14 = (-3 < -5) = 0 (false, signed)
    
    # Mixed positive/negative comparisons
    SLT x15, x3, x1         # x15 = (-5 < 5) = 1 (true, negative < positive)
    SLT x16, x1, x3         # x16 = (5 < -5) = 0 (false, positive > negative)
    
    # Zero comparisons
    SLT x17, x5, x1         # x17 = (0 < 5) = 1 (true, zero < positive)
    SLT x18, x1, x5         # x18 = (5 < 0) = 0 (false, positive > zero)
    SLT x19, x5, x3         # x19 = (0 < -5) = 0 (false, zero > negative)
    SLT x20, x3, x5         # x20 = (-5 < 0) = 1 (true, negative < zero)
    
    # Edge cases: max/min values
    SLT x21, x6, x7         # x21 = (0x7FFFFFFF < 0x80000000) = 0 (false, signed: max positive > min negative)
    SLT x22, x7, x6         # x22 = (0x80000000 < 0x7FFFFFFF) = 1 (true, signed: min negative < max positive)
    SLT x23, x7, x8         # x23 = (0x80000000 < 1) = 1 (true, signed: min negative < 1)
    SLT x24, x6, x9         # x24 = (0x7FFFFFFF < 0xFFFFFFFF) = 0 (false, signed: max positive > -1)

# ============================================
# Section 3: SLTU Tests (Unsigned Comparison)
# ============================================
    # Basic positive comparisons (same as signed)
    SLTU x30, x2, x1        # x30 = (3 < 5) = 1 (true, unsigned)
    SLTU x31, x1, x2        # x31 = (5 < 3) = 0 (false, unsigned)
    SLTU x25, x1, x1        # x25 = (5 < 5) = 0 (false, equal)
    
    # Negative comparisons (different from signed!)
    SLTU x26, x3, x4        # x26 = (0xFFFFFFFB < 0xFFFFFFFD) = 1 (true, unsigned: large numbers)
    SLTU x27, x4, x3        # x27 = (0xFFFFFFFD < 0xFFFFFFFB) = 0 (false, unsigned)
    
    # Mixed positive/negative comparisons (different from signed!)
    SLTU x28, x3, x1        # x28 = (0xFFFFFFFB < 5) = 0 (false, unsigned: large number > small)
    SLTU x29, x1, x3        # x29 = (5 < 0xFFFFFFFB) = 1 (true, unsigned: small < large)
    
    # Zero comparisons
    SLTU x10, x5, x1        # x10 = (0 < 5) = 1 (true, zero < positive, unsigned)
    SLTU x11, x1, x5        # x11 = (5 < 0) = 0 (false, positive > zero, unsigned)
    SLTU x12, x5, x3        # x12 = (0 < 0xFFFFFFFB) = 1 (true, zero < large unsigned, unsigned)
    SLTU x13, x3, x5        # x13 = (0xFFFFFFFB < 0) = 0 (false, large unsigned > zero, unsigned)
    
    # Edge cases: max/min values
    SLTU x14, x6, x7        # x14 = (0x7FFFFFFF < 0x80000000) = 1 (true, unsigned: smaller < larger)
    SLTU x15, x7, x6        # x15 = (0x80000000 < 0x7FFFFFFF) = 0 (false, unsigned: larger > smaller)
    SLTU x16, x7, x8        # x16 = (0x80000000 < 1) = 0 (false, unsigned: large number > small)
    SLTU x17, x6, x9        # x17 = (0x7FFFFFFF < 0xFFFFFFFF) = 1 (true, unsigned: smaller < max)

# ============================================
# Section 4: Store Results for Verification
# ============================================
    # Store SLT results (signed comparisons)
    SW x10, 0(x0)           # memory[0] = 1 (3 < 5, signed)
    SW x11, 4(x0)           # memory[1] = 0 (5 < 3, signed)
    SW x12, 8(x0)           # memory[2] = 0 (5 < 5, signed)
    SW x13, 12(x0)          # memory[3] = 1 (-5 < -3, signed)
    SW x14, 16(x0)          # memory[4] = 0 (-3 < -5, signed)
    SW x15, 20(x0)          # memory[5] = 1 (-5 < 5, signed)
    SW x16, 24(x0)          # memory[6] = 0 (5 < -5, signed)
    SW x17, 28(x0)          # memory[7] = 1 (0 < 5, signed)
    SW x18, 32(x0)          # memory[8] = 0 (5 < 0, signed)
    SW x19, 36(x0)          # memory[9] = 0 (0 < -5, signed)
    SW x20, 40(x0)          # memory[10] = 1 (-5 < 0, signed)
    SW x21, 44(x0)          # memory[11] = 0 (max < min, signed)
    SW x22, 48(x0)          # memory[12] = 1 (min < max, signed)
    SW x23, 52(x0)          # memory[13] = 1 (min < 1, signed)
    SW x24, 56(x0)          # memory[14] = 0 (max < -1, signed)
    
    # Store SLTU results (unsigned comparisons)
    SW x30, 60(x0)          # memory[15] = 1 (3 < 5, unsigned)
    SW x31, 64(x0)          # memory[16] = 0 (5 < 3, unsigned)
    SW x25, 68(x0)          # memory[17] = 0 (5 < 5, unsigned)
    SW x26, 72(x0)          # memory[18] = 1 (0xFFFFFFFB < 0xFFFFFFFD, unsigned)
    SW x27, 76(x0)          # memory[19] = 0 (0xFFFFFFFD < 0xFFFFFFFB, unsigned)
    SW x28, 80(x0)          # memory[20] = 0 (0xFFFFFFFB < 5, unsigned)
    SW x29, 84(x0)          # memory[21] = 1 (5 < 0xFFFFFFFB, unsigned)
    SW x10, 88(x0)          # memory[22] = 1 (0 < 5, unsigned) - note: reused x10
    SW x11, 92(x0)          # memory[23] = 0 (5 < 0, unsigned) - note: reused x11
    SW x12, 96(x0)          # memory[24] = 1 (0 < 0xFFFFFFFB, unsigned) - note: reused x12
    SW x13, 100(x0)         # memory[25] = 0 (0xFFFFFFFB < 0, unsigned) - note: reused x13
    SW x14, 104(x0)         # memory[26] = 1 (0x7FFFFFFF < 0x80000000, unsigned)
    SW x15, 108(x0)         # memory[27] = 0 (0x80000000 < 0x7FFFFFFF, unsigned)
    SW x16, 112(x0)         # memory[28] = 0 (0x80000000 < 1, unsigned)
    SW x17, 116(x0)         # memory[29] = 1 (0x7FFFFFFF < 0xFFFFFFFF, unsigned)

# ============================================
# Section 5: Self-Checking Verification
# ============================================
    # Verify SLT: memory[0] should be 1 (3 < 5, signed)
    LW x1, 0(x0)            # Load result
    ADDI x2, x0, 1          # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 120(x0)          # Store comparison (0 = pass)
    
    # Verify SLT: memory[15] should be 0 (5 < 3, signed)
    LW x1, 16(x0)           # Load result
    ADDI x2, x0, 0          # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 124(x0)          # Store comparison (0 = pass)
    
    # Verify SLTU: memory[20] should be 0 (0xFFFFFFFB < 5, unsigned)
    LW x1, 80(x0)           # Load result
    ADDI x2, x0, 0          # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 128(x0)          # Store comparison (0 = pass)
    
    # Verify SLTU: memory[21] should be 1 (5 < 0xFFFFFFFB, unsigned)
    LW x1, 84(x0)           # Load result
    ADDI x2, x0, 1          # Expected value
    SUB x3, x1, x2          # x3 = 0 if correct
    SW x3, 132(x0)          # Store comparison (0 = pass)

# ============================================
# End Marker
# ============================================
    NOP                     # ADDI x0, x0, 0
    NOP                     # ADDI x0, x0, 0

# Expected Results Summary:
# SLT (Signed Comparison) Results:
#   x10 = 1 (3 < 5, signed: true)
#   x11 = 0 (5 < 3, signed: false)
#   x12 = 0 (5 < 5, signed: false, equal)
#   x13 = 1 (-5 < -3, signed: true, -5 is less than -3)
#   x14 = 0 (-3 < -5, signed: false)
#   x15 = 1 (-5 < 5, signed: true, negative < positive)
#   x16 = 0 (5 < -5, signed: false, positive > negative)
#   x17 = 1 (0 < 5, signed: true)
#   x18 = 0 (5 < 0, signed: false)
#   x19 = 0 (0 < -5, signed: false, zero > negative)
#   x20 = 1 (-5 < 0, signed: true, negative < zero)
#   x21 = 0 (max < min, signed: false, max positive > min negative)
#   x22 = 1 (min < max, signed: true, min negative < max positive)
#   x23 = 1 (min < 1, signed: true)
#   x24 = 0 (max < -1, signed: false)
#
# SLTU (Unsigned Comparison) Results:
#   x30 = 1 (3 < 5, unsigned: true, same as signed)
#   x31 = 0 (5 < 3, unsigned: false, same as signed)
#   x25 = 0 (5 < 5, unsigned: false, equal)
#   x26 = 1 (0xFFFFFFFB < 0xFFFFFFFD, unsigned: true, large numbers)
#   x27 = 0 (0xFFFFFFFD < 0xFFFFFFFB, unsigned: false)
#   x28 = 0 (0xFFFFFFFB < 5, unsigned: false, large unsigned > small)
#   x29 = 1 (5 < 0xFFFFFFFB, unsigned: true, small < large unsigned)
#   x10 = 1 (0 < 5, unsigned: true)
#   x11 = 0 (5 < 0, unsigned: false)
#   x12 = 1 (0 < 0xFFFFFFFB, unsigned: true, zero < large unsigned)
#   x13 = 0 (0xFFFFFFFB < 0, unsigned: false, large unsigned > zero)
#   x14 = 1 (0x7FFFFFFF < 0x80000000, unsigned: true)
#   x15 = 0 (0x80000000 < 0x7FFFFFFF, unsigned: false)
#   x16 = 0 (0x80000000 < 1, unsigned: false, large > small)
#   x17 = 1 (0x7FFFFFFF < 0xFFFFFFFF, unsigned: true)
#
# Key Differences:
# - SLT treats numbers as signed: -5 < 5 (true)
# - SLTU treats numbers as unsigned: 0xFFFFFFFB (4294967291) > 5 (false for "less than")
# - Negative numbers in SLTU are treated as large positive numbers
# - Memory locations store comparison results (1 = true, 0 = false)

