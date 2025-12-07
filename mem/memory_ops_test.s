# RISC-V Memory Operations Test Program
# Tests: Basic load/store, load-use hazards, store-load forwarding, addressing with offsets
# Self-checking: Results stored in memory for verification

# ============================================
# Section 1: Initialize Test Data in Memory
# ============================================
    # Initialize memory with test data using stores
    # Memory layout:
    # memory[0] = 0x12345678
    # memory[1] = 0xABCDEF00
    # memory[2] = 0x00000042
    # memory[3] = 0xFFFFFFFF
    # memory[4] = 0x00000000
    # memory[5] = 0x11111111
    # memory[6] = 0x22222222
    # memory[7] = 0x33333333
    
    LUI x1, 0x12345          # x1 = 0x12345000
    ADDI x1, x1, 0x678       # x1 = 0x12345678
    SW x1, 0(x0)             # memory[0] = 0x12345678
    
    LUI x2, 0xABCDE          # x2 = 0xABCDE000
    ADDI x2, x2, 0xF00       # x2 = 0xABCDEF00
    SW x2, 4(x0)             # memory[1] = 0xABCDEF00
    
    ADDI x3, x0, 0x42        # x3 = 0x00000042
    SW x3, 8(x0)             # memory[2] = 0x00000042
    
    ADDI x4, x0, -1          # x4 = 0xFFFFFFFF
    SW x4, 12(x0)            # memory[3] = 0xFFFFFFFF
    
    ADDI x5, x0, 0           # x5 = 0x00000000
    SW x5, 16(x0)            # memory[4] = 0x00000000
    
    LUI x6, 0x11111          # x6 = 0x11111000
    ADDI x6, x6, 0x111       # x6 = 0x11111111
    SW x6, 20(x0)            # memory[5] = 0x11111111
    
    LUI x7, 0x22222          # x7 = 0x22222000
    ADDI x7, x7, 0x222       # x7 = 0x22222222
    SW x7, 24(x0)            # memory[6] = 0x22222222
    
    LUI x8, 0x33333          # x8 = 0x33333000
    ADDI x8, x8, 0x333       # x8 = 0x33333333
    SW x8, 28(x0)            # memory[7] = 0x33333333

# ============================================
# Section 2: Basic Load Operations (LW)
# ============================================
    # Load from different addresses
    LW x10, 0(x0)            # x10 = memory[0] = 0x12345678
    LW x11, 4(x0)            # x11 = memory[1] = 0xABCDEF00
    LW x12, 8(x0)            # x12 = memory[2] = 0x00000042
    LW x13, 12(x0)           # x13 = memory[3] = 0xFFFFFFFF
    LW x14, 16(x0)           # x14 = memory[4] = 0x00000000
    LW x15, 20(x0)           # x15 = memory[5] = 0x11111111
    LW x16, 24(x0)           # x16 = memory[6] = 0x22222222
    LW x17, 28(x0)           # x17 = memory[7] = 0x33333333

# ============================================
# Section 3: Basic Store Operations (SW)
# ============================================
    # Store to different addresses
    ADDI x18, x0, 0xDEAD     # x18 = 0x0000DEAD
    SLLI x18, x18, 16        # x18 = 0xDEAD0000
    ADDI x19, x0, 0xBEEF     # x19 = 0x0000BEEF
    OR x18, x18, x19         # x18 = 0xDEADBEEF
    SW x18, 32(x0)           # memory[8] = 0xDEADBEEF
    
    ADDI x19, x0, 0xCAFE     # x19 = 0x0000CAFE
    SLLI x19, x19, 16        # x19 = 0xCAFE0000
    ADDI x20, x0, 0xBABE     # x20 = 0x0000BABE
    OR x19, x19, x20         # x19 = 0xCAFEBABE
    SW x19, 36(x0)           # memory[9] = 0xCAFEBABE
    
    ADDI x20, x0, 0x1234     # x20 = 0x00001234
    SW x20, 40(x0)           # memory[10] = 0x00001234
    
    ADDI x21, x0, -1         # x21 = 0xFFFFFFFF
    SW x21, 44(x0)           # memory[11] = 0xFFFFFFFF

# ============================================
# Section 4: Memory Addressing with Offsets
# ============================================
    # Test positive offsets
    ADDI x22, x0, 48         # x22 = base address offset (48 bytes = 12 words)
    LW x23, 0(x22)           # x23 = memory[12] (using base register)
    LW x24, 4(x22)           # x24 = memory[13] (base + 4 bytes)
    LW x25, 8(x22)           # x25 = memory[14] (base + 8 bytes)
    LW x26, 12(x22)          # x26 = memory[15] (base + 12 bytes)
    
    # Test negative offsets (using ADDI to create base)
    ADDI x27, x0, 64         # x27 = 64 (base address)
    LW x28, -4(x27)          # x28 = memory[15] (base - 4 bytes, but RISC-V doesn't support negative immediates)
    # Instead, use subtraction: x27 - 4 = 60 bytes = word 15
    ADDI x29, x27, -4        # x29 = 60
    LW x28, 0(x29)           # x28 = memory[15] (effective base - 4)
    
    # Test large offsets
    ADDI x30, x0, 0           # x30 = base address 0
    LW x31, 28(x30)           # x31 = memory[7] (base + 28 bytes = 7 words)

# ============================================
# Section 5: Load-Use Hazard Test
# ============================================
    # This tests the hazard detection unit
    # Load-use hazard: Load instruction followed immediately by instruction that uses loaded value
    # Expected behavior: Pipeline should stall for one cycle
    
    # Test 1: Load-use hazard (LW followed by ADD using loaded value)
    LW x1, 0(x0)             # x1 = memory[0] = 0x12345678 (LOAD)
    ADD x2, x1, x0           # x2 = x1 + 0 (USE - should stall here due to load-use hazard)
    SW x2, 48(x0)            # memory[12] = x2 (store result)
    
    # Test 2: Load-use hazard (LW followed by SUB using loaded value)
    LW x3, 4(x0)             # x3 = memory[1] = 0xABCDEF00 (LOAD)
    SUB x4, x3, x0           # x4 = x3 - 0 (USE - should stall here)
    SW x4, 52(x0)            # memory[13] = x4
    
    # Test 3: Load-use hazard (LW followed by AND using loaded value)
    LW x5, 8(x0)             # x5 = memory[2] = 0x00000042 (LOAD)
    AND x6, x5, x5           # x6 = x5 & x5 (USE - should stall here)
    SW x6, 56(x0)            # memory[14] = x6
    
    # Test 4: Load-use hazard (LW followed by OR using loaded value)
    LW x7, 12(x0)            # x7 = memory[3] = 0xFFFFFFFF (LOAD)
    OR x8, x7, x0            # x8 = x7 | 0 (USE - should stall here)
    SW x8, 60(x0)            # memory[15] = x8

# ============================================
# Section 6: Store-Load Forwarding Test
# ============================================
    # Test store-load sequence (store then load from same address)
    # This tests if data can be forwarded from store to load
    
    # Test 1: Store then load from same address
    ADDI x9, x0, 0x5555       # x9 = 0x00005555
    SLLI x9, x9, 16          # x9 = 0x55550000
    ADDI x10, x0, 0xAAAA     # x10 = 0x0000AAAA
    OR x9, x9, x10           # x9 = 0x5555AAAA
    SW x9, 64(x0)            # memory[16] = 0x5555AAAA (STORE)
    LW x11, 64(x0)           # x11 = memory[16] (LOAD - should get 0x5555AAAA)
    SW x11, 68(x0)           # memory[17] = x11 (store loaded value)
    
    # Test 2: Store then load with different base
    ADDI x12, x0, 0x7777     # x12 = 0x00007777
    SLLI x12, x12, 16        # x12 = 0x77770000
    ADDI x13, x0, 0x8888     # x13 = 0x00008888
    OR x12, x12, x13         # x12 = 0x77778888
    ADDI x14, x0, 64         # x14 = base address (64 bytes)
    SW x12, 0(x14)           # memory[16] = 0x77778888 (STORE to same location)
    LW x15, 0(x14)           # x15 = memory[16] (LOAD - should get 0x77778888)
    SW x15, 72(x0)           # memory[18] = x15
    
    # Test 3: Store-load with offset
    ADDI x16, x0, 0x9999     # x16 = 0x00009999
    SLLI x16, x16, 16        # x16 = 0x99990000
    ADDI x17, x0, 0xAAAA     # x17 = 0x0000AAAA
    OR x16, x16, x17         # x16 = 0x9999AAAA
    ADDI x18, x0, 80         # x18 = base address (80 bytes)
    SW x16, 0(x18)           # memory[20] = 0x9999AAAA (STORE)
    LW x19, 0(x18)           # x19 = memory[20] (LOAD)
    SW x19, 88(x0)           # memory[22] = x19 (store-load forwarding result)

# ============================================
# Section 7: Store Results for Verification
# ============================================
    # Store loaded values for verification
    SW x10, 96(x0)           # memory[24] = 0x12345678 (loaded from memory[0])
    SW x11, 100(x0)          # memory[25] = 0xABCDEF00 (loaded from memory[1])
    SW x12, 104(x0)          # memory[26] = 0x00000042 (loaded from memory[2])
    SW x13, 108(x0)          # memory[27] = 0xFFFFFFFF (loaded from memory[3])
    
    # Store offset addressing results
    SW x23, 112(x0)          # memory[28] = result from offset addressing
    SW x24, 116(x0)          # memory[29] = result from offset addressing
    SW x28, 120(x0)          # memory[30] = result from negative offset simulation
    SW x31, 124(x0)          # memory[31] = result from large offset

# ============================================
# Section 8: Self-Checking Verification
# ============================================
    # Verify load-use hazard results
    LW x1, 48(x0)            # Load result from load-use test 1
    LUI x2, 0x12345          # Expected: 0x12345000
    ADDI x2, x2, 0x678       # Expected: 0x12345678
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 128(x0)           # memory[32] = comparison result (0 = pass)
    
    # Verify store-load forwarding
    LW x1, 68(x0)            # Load result from store-load test 1
    LUI x2, 0x5555           # Expected: 0x55550000
    ADDI x2, x2, 0xAAAA      # Expected: 0x5555AAAA
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 132(x0)           # memory[33] = comparison result (0 = pass)
    
    # Verify basic load
    LW x1, 96(x0)            # Load verification result
    LUI x2, 0x12345          # Expected: 0x12345000
    ADDI x2, x2, 0x678       # Expected: 0x12345678
    SUB x3, x1, x2           # x3 = 0 if correct
    SW x3, 136(x0)           # memory[34] = comparison result (0 = pass)

# ============================================
# End Marker
# ============================================
    NOP                      # ADDI x0, x0, 0
    NOP                      # ADDI x0, x0, 0

# Expected Memory Contents After Execution:
# memory[0] = 0x12345678 (initialized)
# memory[1] = 0xABCDEF00 (initialized)
# memory[2] = 0x00000042 (initialized)
# memory[3] = 0xFFFFFFFF (initialized)
# memory[4] = 0x00000000 (initialized)
# memory[5] = 0x11111111 (initialized)
# memory[6] = 0x22222222 (initialized)
# memory[7] = 0x33333333 (initialized)
# memory[8] = 0xDEADBEEF (stored)
# memory[9] = 0xCAFEBABE (stored)
# memory[10] = 0x00001234 (stored)
# memory[11] = 0xFFFFFFFF (stored)
# memory[12] = 0x12345678 (load-use hazard test result)
# memory[13] = 0xABCDEF00 (load-use hazard test result)
# memory[14] = 0x00000042 (load-use hazard test result)
# memory[15] = 0xFFFFFFFF (load-use hazard test result)
# memory[16] = 0x77778888 (store-load forwarding, overwritten)
# memory[17] = 0x5555AAAA (store-load forwarding result)
# memory[18] = 0x77778888 (store-load forwarding result)
# memory[19] = (unused)
# memory[20] = 0x9999AAAA (store-load forwarding result)
# memory[21] = (unused)
# memory[22] = 0x9999AAAA (store-load forwarding result)
# memory[23] = (unused)
# memory[24] = 0x12345678 (verification: loaded value)
# memory[25] = 0xABCDEF00 (verification: loaded value)
# memory[26] = 0x00000042 (verification: loaded value)
# memory[27] = 0xFFFFFFFF (verification: loaded value)
# memory[28] = offset addressing result
# memory[29] = offset addressing result
# memory[30] = negative offset simulation result
# memory[31] = large offset result
# memory[32] = 0 (load-use verification pass)
# memory[33] = 0 (store-load verification pass)
# memory[34] = 0 (basic load verification pass)

