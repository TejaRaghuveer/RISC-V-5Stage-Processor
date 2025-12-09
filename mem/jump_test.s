# RISC-V Jump Instructions Test Program
# Tests JAL (Jump and Link) and JALR (Jump and Link Register)
# Includes: procedure calls, returns, nested calls, calculated addresses

# ============================================
# Section 1: Initialize Test Values
# ============================================
    ADDI x1, x0, 0         # x1 = 0 (will be used as ra - return address register)
    ADDI x2, x0, 0         # x2 = 0 (test result register)
    ADDI x3, x0, 0         # x3 = 0 (test result register)
    ADDI x4, x0, 0         # x4 = 0 (test result register)
    ADDI x5, x0, 0         # x5 = 0 (test result register)
    
    # Procedure address calculation base
    LUI x10, 0x00000       # x10 = 0 (base address for procedure calls)
    ADDI x10, x10, 0       # x10 = 0 (will be set to procedure addresses)

# ============================================
# Section 2: Basic JAL Test (Jump and Link)
# ============================================
# JAL: Jump PC-relative, save return address (PC+4) in rd

# Test 2.1: JAL forward jump
    ADDI x2, x0, 1         # x2 = 1 (marker before jump)
    JAL x1, jal_target_forward  # Jump forward, save PC+4 in x1
    ADDI x2, x0, 2         # x2 = 2 (should be skipped)
    ADDI x2, x0, 3         # x2 = 3 (should be skipped)
jal_target_forward:
    ADDI x2, x0, 100       # x2 = 100 (jump target - should execute)
    # Expected: x1 = PC+4 of JAL instruction (return address)
    # Expected: x2 = 100 (jump taken)

# Test 2.2: JAL with return address verification
    ADDI x3, x0, 10        # x3 = 10 (marker)
    JAL x1, jal_test_2     # Jump forward, save PC+4 in x1
    ADDI x3, x0, 20        # x3 = 20 (should be skipped)
jal_test_2:
    ADDI x3, x0, 30        # x3 = 30 (jump target)
    # Expected: x1 = return address (PC+4 of JAL)
    # Expected: x3 = 30

# ============================================
# Section 3: Basic JALR Test (Jump and Link Register)
# ============================================
# JALR: Jump to address (rs1 + immediate), save return address (PC+4) in rd
# Target address = (rs1 + imm) & ~1 (clear LSB for alignment)

# Test 3.1: JALR with base register
    ADDI x4, x0, 1         # x4 = 1 (marker)
    LUI x11, 0x00000       # x11 = 0 (base address)
    # Calculate target address: x11 + 0 = jalr_target_1 address
    # We'll use PC-relative calculation: load address into x11
    AUIPC x11, 0           # x11 = PC (current PC)
    ADDI x11, x11, 16      # x11 = PC + 16 (target address offset)
    JALR x1, 0(x11)        # Jump to x11 + 0, save PC+4 in x1
    ADDI x4, x0, 2         # x4 = 2 (should be skipped)
jalr_target_1:
    ADDI x4, x0, 200       # x4 = 200 (jump target)
    # Expected: x1 = return address (PC+4 of JALR)
    # Expected: x4 = 200

# Test 3.2: JALR with immediate offset
    ADDI x5, x0, 10        # x5 = 10 (marker)
    AUIPC x12, 0           # x12 = PC
    ADDI x12, x12, 12      # x12 = PC + 12 (target address)
    JALR x1, 0(x12)        # Jump to x12 + 0, save PC+4 in x1
    ADDI x5, x0, 20        # x5 = 20 (should be skipped)
jalr_target_2:
    ADDI x5, x0, 300       # x5 = 300 (jump target)
    # Expected: x1 = return address
    # Expected: x5 = 300

# ============================================
# Section 4: Simple Procedure Call and Return
# ============================================
# This demonstrates a basic function call pattern

# Test 4.1: Call procedure using JAL
    ADDI x2, x0, 0         # x2 = 0 (clear result)
    JAL x1, procedure_a    # Call procedure_a, save return address in x1
    ADDI x2, x0, 50        # x2 = 50 (after return - should execute)
    JAL x0, end_procedure_test  # Jump to end (skip procedure definition)
    
procedure_a:
    ADDI x2, x0, 100       # x2 = 100 (procedure body)
    JALR x0, 0(x1)         # Return: jump to return address in x1
    # Expected: x2 = 100 (procedure executed)
    # Expected: After return, x2 = 50

end_procedure_test:
    ADDI x2, x0, 200       # x2 = 200 (after procedure test)

# ============================================
# Section 5: Nested Procedure Calls
# ============================================
# Test nested function calls: main -> func1 -> func2 -> return

# Test 5.1: Nested calls with JAL
    ADDI x3, x0, 0         # x3 = 0 (call depth counter)
    JAL x1, nested_func1   # Call func1, save return address in x1
    ADDI x3, x0, 1000      # x3 = 1000 (after all returns)
    JAL x0, end_nested_test  # Skip to end
    
nested_func1:
    ADDI x3, x0, 100       # x3 = 100 (func1 entry)
    JAL x1, nested_func2   # Call func2, save return address in x1
    ADDI x3, x0, 200       # x3 = 200 (func1 after func2 returns)
    JALR x0, 0(x1)         # Return from func1
    
nested_func2:
    ADDI x3, x0, 300       # x3 = 300 (func2 entry)
    JALR x0, 0(x1)         # Return from func2
    # Expected execution order:
    # 1. nested_func1: x3 = 100
    # 2. nested_func2: x3 = 300
    # 3. Return to nested_func1: x3 = 200
    # 4. Return to main: x3 = 1000

end_nested_test:
    ADDI x3, x0, 2000      # x3 = 2000 (after nested test)

# ============================================
# Section 6: Procedure Call with Calculated Address
# ============================================
# Test calling a procedure whose address is calculated at runtime

# Test 6.1: Calculate procedure address and call with JALR
    ADDI x4, x0, 0         # x4 = 0 (marker)
    # Calculate address of calc_procedure
    AUIPC x13, 0           # x13 = PC
    ADDI x13, x13, 12      # x13 = PC + 12 (offset to calc_procedure)
    JALR x1, 0(x13)        # Call calc_procedure using calculated address
    ADDI x4, x0, 500       # x4 = 500 (after return)
    JAL x0, end_calc_test  # Skip procedure definition
    
calc_procedure:
    ADDI x4, x0, 600       # x4 = 600 (procedure body)
    JALR x0, 0(x1)         # Return using return address in x1
    # Expected: x4 = 600 (procedure executed)
    # Expected: After return, x4 = 500

end_calc_test:
    ADDI x4, x0, 700       # x4 = 700 (after calc test)

# ============================================
# Section 7: Multiple Procedure Calls
# ============================================
# Test calling multiple different procedures

# Test 7.1: Call procedure1
    ADDI x5, x0, 0         # x5 = 0
    JAL x1, proc1          # Call proc1
    ADDI x5, x0, 10        # x5 = 10 (after proc1)
    
    # Test 7.2: Call procedure2
    JAL x1, proc2          # Call proc2
    ADDI x5, x0, 20        # x5 = 20 (after proc2)
    
    # Test 7.3: Call procedure3
    JAL x1, proc3          # Call proc3
    ADDI x5, x0, 30        # x5 = 30 (after proc3)
    
    JAL x0, end_multi_test # Skip procedure definitions

proc1:
    ADDI x5, x0, 100       # x5 = 100 (proc1 body)
    JALR x0, 0(x1)         # Return
    
proc2:
    ADDI x5, x0, 200       # x5 = 200 (proc2 body)
    JALR x0, 0(x1)         # Return
    
proc3:
    ADDI x5, x0, 300       # x5 = 300 (proc3 body)
    JALR x0, 0(x1)         # Return
    # Expected sequence: x5 = 100 -> 10 -> 200 -> 20 -> 300 -> 30

end_multi_test:
    ADDI x5, x0, 1000      # x5 = 1000 (final value)

# ============================================
# Section 8: Return Address Register (x1/ra) Usage
# ============================================
# Test proper use of x1 (ra - return address register)

# Test 8.1: Save return address in x1
    ADDI x2, x0, 0         # x2 = 0
    JAL x1, ra_test_proc   # Call procedure, save return address in x1
    ADDI x2, x0, 100       # x2 = 100 (after return)
    JAL x0, end_ra_test    # Skip procedure
    
ra_test_proc:
    ADDI x2, x0, 200       # x2 = 200 (procedure body)
    # x1 contains return address - use it to return
    JALR x0, 0(x1)         # Return using x1 (ra)
    # Expected: x2 = 200 (procedure executed)
    # Expected: After return, x2 = 100

end_ra_test:
    ADDI x2, x0, 300       # x2 = 300 (after ra test)

# Test 8.2: Nested calls preserving return addresses
    ADDI x3, x0, 0         # x3 = 0
    JAL x1, outer_proc     # Call outer_proc, save return address in x1
    ADDI x3, x0, 1000      # x3 = 1000 (after all returns)
    JAL x0, end_ra_nested  # Skip procedures
    
outer_proc:
    ADDI x3, x0, 100       # x3 = 100 (outer_proc entry)
    # Save current return address (x1) before nested call
    ADDI x6, x1, 0         # x6 = x1 (save outer return address)
    JAL x1, inner_proc     # Call inner_proc, x1 now has inner return address
    # Restore return address
    ADDI x1, x6, 0         # x1 = x6 (restore outer return address)
    ADDI x3, x0, 200       # x3 = 200 (outer_proc after inner returns)
    JALR x0, 0(x1)         # Return from outer_proc
    
inner_proc:
    ADDI x3, x0, 300       # x3 = 300 (inner_proc entry)
    JALR x0, 0(x1)         # Return from inner_proc
    # Expected sequence:
    # 1. outer_proc: x3 = 100
    # 2. inner_proc: x3 = 300
    # 3. Return to outer_proc: x3 = 200
    # 4. Return to main: x3 = 1000

end_ra_nested:
    ADDI x3, x0, 2000      # x3 = 2000 (after nested ra test)

# ============================================
# Section 9: Jump to Calculated Addresses
# ============================================
# Test jumping to addresses computed at runtime

# Test 9.1: Calculate jump target address
    ADDI x4, x0, 0         # x4 = 0
    # Calculate target address
    AUIPC x14, 0           # x14 = PC
    ADDI x14, x14, 12      # x14 = PC + 12 (target offset)
    JALR x1, 0(x14)        # Jump to calculated address
    ADDI x4, x0, 1         # x4 = 1 (should be skipped)
calc_target_1:
    ADDI x4, x0, 2         # x4 = 2 (calculated target)
    # Expected: x4 = 2

# Test 9.2: Jump with offset calculation
    ADDI x5, x0, 0         # x5 = 0
    # Load base address
    LUI x15, 0x00000       # x15 = 0
    ADDI x15, x15, 0       # x15 = 0 (will be set to target)
    AUIPC x15, 0           # x15 = PC
    ADDI x15, x15, 8       # x15 = PC + 8 (target)
    JALR x1, 0(x15)        # Jump to x15 + 0
    ADDI x5, x0, 10        # x5 = 10 (should be skipped)
calc_target_2:
    ADDI x5, x0, 20        # x5 = 20 (calculated target)
    # Expected: x5 = 20

# ============================================
# Section 10: Store Results for Verification
# ============================================
    SW x1, 0(x0)           # Store return address (x1/ra)
    SW x2, 4(x0)           # Store test result x2
    SW x3, 8(x0)           # Store test result x3
    SW x4, 12(x0)          # Store test result x4
    SW x5, 16(x0)          # Store test result x5

# ============================================
# Section 11: End Marker
# ============================================
    NOP                    # ADDI x0, x0, 0
    NOP                    # ADDI x0, x0, 0
    NOP                    # ADDI x0, x0, 0

# ============================================
# Expected Final Register State:
# ============================================
# x1 = Return address from last procedure call (varies)
# x2 = 300 (after return address tests)
# x3 = 2000 (after nested return address tests)
# x4 = 2 (after calculated address jump)
# x5 = 20 (after calculated address jump with offset)

# ============================================
# Expected Final Memory State:
# ============================================
# memory[0] = Return address value (x1)
# memory[1] = 300 (x2)
# memory[2] = 2000 (x3)
# memory[3] = 2 (x4)
# memory[4] = 20 (x5)

