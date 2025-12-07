# RISC-V Logical Operations Test Program
# Tests bitwise logical operations: AND, OR, XOR, ANDI, ORI, XORI
# Includes bit masking, complement operations, and edge cases
# Self-checking: Results stored in memory for verification

# ============================================
# Section 1: Initialize Test Patterns
# ============================================
    ADDI x1, x0, 0xAA        # x1 = 0x000000AA (lower byte pattern)
    SLLI x1, x1, 8           # x1 = 0x0000AA00
    ADDI x2, x0, 0xAA        # x2 = 0x000000AA
    OR x1, x1, x2            # x1 = 0x0000AAAA
    SLLI x1, x1, 16          # x1 = 0xAAAAAAAA (alternating pattern: 1010...)
    
    ADDI x2, x0, 0x55        # x2 = 0x00000055 (lower byte pattern)
    SLLI x2, x2, 8           # x2 = 0x00005500
    ADDI x3, x0, 0x55        # x3 = 0x00000055
    OR x2, x2, x3            # x2 = 0x00005555
    SLLI x2, x2, 16          # x2 = 0x55555555 (alternating pattern: 0101...)
    
    ADDI x3, x0, -1          # x3 = 0xFFFFFFFF (all 1s)
    ADDI x4, x0, 0           # x4 = 0x00000000 (all 0s)
    
    ADDI x5, x0, 0xFF        # x5 = 0x000000FF (lower byte mask)
    ADDI x6, x0, 0xF0        # x6 = 0x000000F0 (nibble mask)
    ADDI x7, x0, 0x0F        # x7 = 0x0000000F (lower nibble mask)
    
    LUI x8, 0xFFFFF          # x8 = 0xFFFFF000
    ADDI x8, x8, 0xFFF       # x8 = 0xFFFFFFFF (all 1s, alternative method)
    
    ADDI x9, x0, 0x1234      # x9 = 0x00001234 (test value)
    SLLI x9, x9, 16          # x9 = 0x12340000
    ADDI x10, x0, 0x5678     # x10 = 0x00005678
    OR x9, x9, x10           # x9 = 0x12345678 (test pattern)

# ============================================
# Section 2: AND Operations (R-Type)
# ============================================
    # Basic AND operations
    AND x11, x1, x2          # x11 = 0xAAAAAAAA & 0x55555555 = 0x00000000 (no common bits)
    AND x12, x1, x1          # x12 = 0xAAAAAAAA & 0xAAAAAAAA = 0xAAAAAAAA (identity)
    AND x13, x1, x3          # x13 = 0xAAAAAAAA & 0xFFFFFFFF = 0xAAAAAAAA (mask with all 1s)
    AND x14, x1, x4          # x14 = 0xAAAAAAAA & 0x00000000 = 0x00000000 (mask with all 0s)
    AND x15, x2, x2          # x15 = 0x55555555 & 0x55555555 = 0x55555555 (identity)
    AND x16, x2, x3          # x16 = 0x55555555 & 0xFFFFFFFF = 0x55555555 (mask with all 1s)
    AND x17, x3, x3          # x17 = 0xFFFFFFFF & 0xFFFFFFFF = 0xFFFFFFFF (all 1s)
    AND x18, x4, x4          # x18 = 0x00000000 & 0x00000000 = 0x00000000 (all 0s)
    
    # Bit masking examples
    AND x19, x9, x5          # x19 = 0x12345678 & 0x000000FF = 0x00000078 (extract lower byte)
    AND x20, x9, x6          # x20 = 0x12345678 & 0x000000F0 = 0x00000070 (extract upper nibble)
    AND x21, x9, x7          # x21 = 0x12345678 & 0x0000000F = 0x00000008 (extract lower nibble)

# ============================================
# Section 3: OR Operations (R-Type)
# ============================================
    # Basic OR operations
    OR x22, x1, x2           # x22 = 0xAAAAAAAA | 0x55555555 = 0xFFFFFFFF (all bits set)
    OR x23, x1, x1           # x23 = 0xAAAAAAAA | 0xAAAAAAAA = 0xAAAAAAAA (identity)
    OR x24, x1, x3           # x24 = 0xAAAAAAAA | 0xFFFFFFFF = 0xFFFFFFFF (OR with all 1s)
    OR x25, x1, x4           # x25 = 0xAAAAAAAA | 0x00000000 = 0xAAAAAAAA (OR with all 0s)
    OR x26, x2, x2           # x26 = 0x55555555 | 0x55555555 = 0x55555555 (identity)
    OR x27, x2, x3           # x27 = 0x55555555 | 0xFFFFFFFF = 0xFFFFFFFF (OR with all 1s)
    OR x28, x3, x3           # x28 = 0xFFFFFFFF | 0xFFFFFFFF = 0xFFFFFFFF (all 1s)
    OR x29, x4, x4           # x29 = 0x00000000 | 0x00000000 = 0x00000000 (all 0s)
    
    # Bit setting examples
    OR x30, x9, x5           # x30 = 0x12345678 | 0x000000FF = 0x123456FF (set lower byte)
    OR x31, x4, x1           # x31 = 0x00000000 | 0xAAAAAAAA = 0xAAAAAAAA (set pattern)

# ============================================
# Section 4: XOR Operations (R-Type)
# ============================================
    # Basic XOR operations
    XOR x10, x1, x2          # x10 = 0xAAAAAAAA ^ 0x55555555 = 0xFFFFFFFF (complement)
    XOR x11, x1, x1          # x11 = 0xAAAAAAAA ^ 0xAAAAAAAA = 0x00000000 (self-XOR = 0)
    XOR x12, x1, x3          # x12 = 0xAAAAAAAA ^ 0xFFFFFFFF = 0x55555555 (complement)
    XOR x13, x1, x4          # x13 = 0xAAAAAAAA ^ 0x00000000 = 0xAAAAAAAA (XOR with 0 = identity)
    XOR x14, x2, x2          # x14 = 0x55555555 ^ 0x55555555 = 0x00000000 (self-XOR = 0)
    XOR x15, x2, x3          # x15 = 0x55555555 ^ 0xFFFFFFFF = 0xAAAAAAAA (complement)
    XOR x16, x3, x3          # x16 = 0xFFFFFFFF ^ 0xFFFFFFFF = 0x00000000 (all 1s XOR = 0)
    XOR x17, x4, x4          # x17 = 0x00000000 ^ 0x00000000 = 0x00000000 (all 0s XOR = 0)
    
    # Complement operations (XOR with all 1s)
    XOR x18, x9, x3          # x18 = 0x12345678 ^ 0xFFFFFFFF = 0xEDCBA987 (bitwise complement)
    XOR x19, x1, x3          # x19 = 0xAAAAAAAA ^ 0xFFFFFFFF = 0x55555555 (complement)
    XOR x20, x2, x3          # x20 = 0x55555555 ^ 0xFFFFFFFF = 0xAAAAAAAA (complement)
    
    # Toggle bits examples
    XOR x21, x9, x5          # x21 = 0x12345678 ^ 0x000000FF = 0x12345687 (toggle lower byte)

# ============================================
# Section 5: ANDI Operations (I-Type)
# ============================================
    # Basic ANDI operations
    ANDI x22, x1, 0x555      # x22 = 0xAAAAAAAA & 0x555 = 0x00000000 (no match)
    ANDI x23, x1, 0xAAA      # x23 = 0xAAAAAAAA & 0xAAA = 0x00000AAA (match pattern)
    ANDI x24, x2, 0x555      # x24 = 0x55555555 & 0x555 = 0x00000555 (match pattern)
    ANDI x25, x2, 0xAAA      # x25 = 0x55555555 & 0xAAA = 0x00000000 (no match)
    ANDI x26, x3, 0xFFF      # x26 = 0xFFFFFFFF & 0xFFF = 0x00000FFF (mask lower 12 bits)
    ANDI x27, x4, 0xFFF      # x27 = 0x00000000 & 0xFFF = 0x00000000 (zero mask)
    
    # Bit masking with immediates
    ANDI x28, x9, 0xFF       # x28 = 0x12345678 & 0xFF = 0x00000078 (extract lower byte)
    ANDI x29, x9, 0xF0       # x29 = 0x12345678 & 0xF0 = 0x00000070 (extract upper nibble)
    ANDI x30, x9, 0x0F       # x30 = 0x12345678 & 0x0F = 0x00000008 (extract lower nibble)
    ANDI x31, x9, 0x7FF      # x31 = 0x12345678 & 0x7FF = 0x00000678 (mask lower 11 bits)

# ============================================
# Section 6: ORI Operations (I-Type)
# ============================================
    # Basic ORI operations
    ORI x10, x1, 0x555       # x10 = 0xAAAAAAAA | 0x555 = 0xAAAAAFFF (set bits)
    ORI x11, x1, 0xAAA       # x11 = 0xAAAAAAAA | 0xAAA = 0xAAAAAABA (set bits)
    ORI x12, x2, 0x555       # x12 = 0x55555555 | 0x555 = 0x55555FFF (set bits)
    ORI x13, x2, 0xAAA       # x13 = 0x55555555 | 0xAAA = 0x55555FFF (set bits)
    ORI x14, x3, 0xFFF      # x14 = 0xFFFFFFFF | 0xFFF = 0xFFFFFFFF (already all 1s)
    ORI x15, x4, 0xFFF      # x15 = 0x00000000 | 0xFFF = 0x00000FFF (set lower 12 bits)
    
    # Bit setting with immediates
    ORI x16, x9, 0xFF       # x16 = 0x12345678 | 0xFF = 0x123456FF (set lower byte)
    ORI x17, x4, 0xAAA      # x17 = 0x00000000 | 0xAAA = 0x00000AAA (set pattern)
    ORI x18, x9, 0xF0       # x18 = 0x12345678 | 0xF0 = 0x123456F8 (set upper nibble)

# ============================================
# Section 7: XORI Operations (I-Type)
# ============================================
    # Basic XORI operations
    XORI x19, x1, 0x555      # x19 = 0xAAAAAAAA ^ 0x555 = 0xAAAAAFFF (toggle bits)
    XORI x20, x1, 0xAAA      # x20 = 0xAAAAAAAA ^ 0xAAA = 0xAAAAA000 (toggle bits)
    XORI x21, x2, 0x555      # x21 = 0x55555555 ^ 0x555 = 0x55555000 (toggle bits)
    XORI x22, x2, 0xAAA      # x22 = 0x55555555 ^ 0xAAA = 0x55555FFF (toggle bits)
    XORI x23, x3, 0xFFF      # x23 = 0xFFFFFFFF ^ 0xFFF = 0xFFFFF000 (complement lower 12 bits)
    XORI x24, x4, 0xFFF      # x24 = 0x00000000 ^ 0xFFF = 0x00000FFF (set lower 12 bits)
    
    # Complement operations with immediates
    XORI x25, x9, 0xFFF      # x25 = 0x12345678 ^ 0xFFF = 0x12345887 (toggle lower 12 bits)
    XORI x26, x1, 0xFFF      # x26 = 0xAAAAAAAA ^ 0xFFF = 0xAAAAA555 (toggle lower 12 bits)
    XORI x27, x9, 0xFF       # x27 = 0x12345678 ^ 0xFF = 0x12345687 (toggle lower byte)

# ============================================
# Section 8: Store Results for Verification
# ============================================
    # Store AND results
    SW x11, 0(x0)            # memory[0] = 0x00000000 (AND: alternating patterns)
    SW x12, 4(x0)            # memory[1] = 0xAAAAAAAA (AND: identity)
    SW x13, 8(x0)            # memory[2] = 0xAAAAAAAA (AND: mask with all 1s)
    SW x14, 12(x0)           # memory[3] = 0x00000000 (AND: mask with all 0s)
    SW x19, 16(x0)           # memory[4] = 0x00000078 (AND: extract lower byte)
    
    # Store OR results
    SW x22, 20(x0)           # memory[5] = 0xFFFFFFFF (OR: alternating patterns)
    SW x23, 24(x0)           # memory[6] = 0xAAAAAAAA (OR: identity)
    SW x24, 28(x0)           # memory[7] = 0xFFFFFFFF (OR: with all 1s)
    SW x25, 32(x0)           # memory[8] = 0xAAAAAAAA (OR: with all 0s)
    SW x30, 36(x0)           # memory[9] = 0x123456FF (OR: set lower byte)
    
    # Store XOR results
    SW x10, 40(x0)           # memory[10] = 0xFFFFFFFF (XOR: alternating patterns)
    SW x11, 44(x0)           # memory[11] = 0x00000000 (XOR: self-XOR)
    SW x12, 48(x0)           # memory[12] = 0x55555555 (XOR: complement)
    SW x18, 52(x0)           # memory[13] = 0xEDCBA987 (XOR: bitwise complement)
    
    # Store ANDI results
    SW x28, 56(x0)           # memory[14] = 0x00000078 (ANDI: extract lower byte)
    SW x29, 60(x0)           # memory[15] = 0x00000070 (ANDI: extract upper nibble)
    SW x30, 64(x0)           # memory[16] = 0x00000008 (ANDI: extract lower nibble)
    
    # Store ORI results
    SW x16, 68(x0)           # memory[17] = 0x123456FF (ORI: set lower byte)
    SW x17, 72(x0)           # memory[18] = 0x00000AAA (ORI: set pattern)
    
    # Store XORI results
    SW x25, 76(x0)           # memory[19] = 0x12345887 (XORI: toggle lower 12 bits)
    SW x27, 80(x0)           # memory[20] = 0x12345687 (XORI: toggle lower byte)

# ============================================
# Section 9: Self-Checking Verification
# ============================================
    # Verify AND: memory[0] should be 0
    LW x1, 0(x0)             # Load result
    ADDI x2, x0, 0           # Expected value
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 84(x0)            # Store comparison (0 = pass)
    
    # Verify OR: memory[5] should be 0xFFFFFFFF
    LW x1, 20(x0)            # Load result
    ADDI x2, x0, -1          # Expected value (-1 = 0xFFFFFFFF)
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 88(x0)            # Store comparison (0 = pass)
    
    # Verify XOR: memory[10] should be 0xFFFFFFFF
    LW x1, 40(x0)            # Load result
    ADDI x2, x0, -1          # Expected value
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 92(x0)            # Store comparison (0 = pass)
    
    # Verify ANDI: memory[14] should be 0x78
    LW x1, 56(x0)            # Load result
    ADDI x2, x0, 0x78        # Expected value
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 96(x0)            # Store comparison (0 = pass)

# ============================================
# End Marker
# ============================================
    NOP                      # ADDI x0, x0, 0
    NOP                      # ADDI x0, x0, 0

# Expected Results Summary:
# Test Patterns:
#   x1 = 0xAAAAAAAA (alternating: 1010...)
#   x2 = 0x55555555 (alternating: 0101...)
#   x3 = 0xFFFFFFFF (all 1s)
#   x4 = 0x00000000 (all 0s)
#   x9 = 0x12345678 (test value)
#
# AND Results:
#   x11 = 0x00000000 (0xAAAAAAAA & 0x55555555)
#   x12 = 0xAAAAAAAA (identity)
#   x13 = 0xAAAAAAAA (mask with all 1s)
#   x14 = 0x00000000 (mask with all 0s)
#   x19 = 0x00000078 (extract lower byte)
#
# OR Results:
#   x22 = 0xFFFFFFFF (0xAAAAAAAA | 0x55555555)
#   x23 = 0xAAAAAAAA (identity)
#   x24 = 0xFFFFFFFF (OR with all 1s)
#   x25 = 0xAAAAAAAA (OR with all 0s)
#   x30 = 0x123456FF (set lower byte)
#
# XOR Results:
#   x10 = 0xFFFFFFFF (0xAAAAAAAA ^ 0x55555555)
#   x11 = 0x00000000 (self-XOR)
#   x12 = 0x55555555 (complement)
#   x18 = 0xEDCBA987 (bitwise complement of 0x12345678)
#
# ANDI Results:
#   x28 = 0x00000078 (extract lower byte)
#   x29 = 0x00000070 (extract upper nibble)
#   x30 = 0x00000008 (extract lower nibble)
#
# ORI Results:
#   x16 = 0x123456FF (set lower byte)
#   x17 = 0x00000AAA (set pattern)
#
# XORI Results:
#   x25 = 0x12345887 (toggle lower 12 bits)
#   x27 = 0x12345687 (toggle lower byte)

