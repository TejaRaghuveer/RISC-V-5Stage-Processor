# Contributing to RISC-V 5-Stage Pipeline Processor

Thank you for your interest in contributing to this RISC-V processor project! This document provides guidelines and instructions for contributing code, tests, documentation, and other improvements.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Code Style Standards](#code-style-standards)
- [Adding New Instructions](#adding-new-instructions)
- [Submitting Test Cases](#submitting-test-cases)
- [Pull Request Process](#pull-request-process)
- [Reporting Issues](#reporting-issues)
- [Documentation Contributions](#documentation-contributions)

---

## Code of Conduct

This project follows a code of conduct based on respect, professionalism, and collaboration:

- **Be respectful**: Treat all contributors with respect and kindness
- **Be constructive**: Provide constructive feedback and suggestions
- **Be patient**: Understand that contributors have different experience levels
- **Be inclusive**: Welcome contributors from all backgrounds

---

## Getting Started

### Prerequisites

Before contributing, ensure you have:

- **SystemVerilog Simulator**: Icarus Verilog, ModelSim/QuestaSim, or Xilinx Vivado
- **Git**: Version control system
- **Basic RISC-V Knowledge**: Understanding of RV32I instruction set
- **SystemVerilog Knowledge**: Familiarity with SystemVerilog syntax and best practices

### Development Setup

1. **Fork the repository** on GitHub
2. **Clone your fork**:
   ```bash
   git clone https://github.com/YOUR_USERNAME/RISC-V-5Stage-Processor.git
   cd RISC-V-5Stage-Processor
   ```
3. **Create a branch** for your changes:
   ```bash
   git checkout -b feature/your-feature-name
   ```
4. **Make your changes** following the guidelines below
5. **Test your changes** thoroughly
6. **Submit a pull request** (see [Pull Request Process](#pull-request-process))

---

## Code Style Standards

### General Guidelines

- **Indentation**: Use **4 spaces** (no tabs)
- **Line Length**: Maximum **100 characters** per line
- **Naming Conventions**:
  - Modules: `PascalCase` (e.g., `riscv_pipeline`, `forwarding_unit`)
  - Signals: `snake_case` (e.g., `rs1_data`, `branch_taken`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `DATA_WIDTH`, `MEM_DEPTH`)
  - Parameters: `PascalCase` (e.g., `DATA_WIDTH`, `ADDR_WIDTH`)

### SystemVerilog Style

#### Module Declaration
```systemverilog
module module_name #(
    parameter DATA_WIDTH = 32,        // Parameter with default value
    parameter ADDR_WIDTH = 32         // Last parameter without comma
) (
    // Clock and Reset
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Inputs
    input  logic [DATA_WIDTH-1:0]       signal_name,
    
    // Outputs
    output logic [DATA_WIDTH-1:0]       output_signal
);
```

#### Signal Declarations
- Group related signals together
- Use descriptive names
- Include width specifications `[WIDTH-1:0]`
- Add comments for complex signals

#### Comments

**Module Header** (required for all modules):
```systemverilog
/**
 * @file module_name.sv
 * @brief Brief description of the module
 * 
 * Detailed description of the module's purpose, functionality,
 * and key features.
 * 
 * @details
 * Additional details about implementation, timing, etc.
 */
```

**Inline Comments**:
```systemverilog
// Single-line comment for simple explanations

/**
 * Multi-line comment for complex logic
 * explaining the reasoning and implementation details
 */
```

#### Always Blocks

**Combinational Logic**:
```systemverilog
always_comb begin
    // Default assignments
    signal = default_value;
    
    // Conditional logic
    if (condition) begin
        signal = new_value;
    end
end
```

**Sequential Logic**:
```systemverilog
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset assignments
        signal <= reset_value;
    end else begin
        // Normal operation
        signal <= next_value;
    end
end
```

### Code Organization

1. **Module Structure**:
   - Module declaration with parameters
   - Port declarations (grouped by type)
   - Internal signal declarations
   - Instantiation declarations
   - Logic implementation
   - Endmodule

2. **File Organization**:
   - One module per file
   - File name matches module name (lowercase with underscores)
   - Place files in appropriate directories (`src/`, `tb/`, etc.)

### Best Practices

- ✅ Use `logic` type instead of `reg`/`wire`
- ✅ Use `always_comb` for combinational logic
- ✅ Use `always_ff` for sequential logic
- ✅ Use non-blocking assignments (`<=`) in sequential blocks
- ✅ Use blocking assignments (`=`) in combinational blocks
- ✅ Include reset handling for all sequential elements
- ✅ Add default cases in case statements
- ✅ Check for x0 register (hardwired zero) in register file operations
- ✅ Validate addresses before memory access
- ✅ Document complex logic and timing relationships

### What NOT to Do

- ❌ Don't use `reg` or `wire` (use `logic`)
- ❌ Don't mix blocking and non-blocking assignments incorrectly
- ❌ Don't create latches unintentionally
- ❌ Don't ignore reset signals
- ❌ Don't use magic numbers (use parameters or constants)
- ❌ Don't create overly complex nested logic
- ❌ Don't forget to handle x0 register special cases

---

## Adding New Instructions

### Overview

This processor currently implements the complete RV32I base instruction set. To add new instructions (e.g., from RV32M extension), follow these steps:

### Step 1: Understand the Instruction

1. **Read the RISC-V specification** for the instruction
2. **Identify instruction format** (R-type, I-type, S-type, B-type, U-type, J-type)
3. **Determine required operations**:
   - ALU operations needed
   - Memory access requirements
   - Control flow changes
   - Register file operations

### Step 2: Update Control Unit

**File**: `src/control_unit.sv`

Add a new case for the instruction opcode:

```systemverilog
case (opcode)
    // Existing cases...
    
    // New instruction opcode
    7'bXXXXXXX: begin
        RegWrite = 1'b1;      // Set based on instruction
        MemRead = 1'b0;
        MemWrite = 1'b0;
        MemToReg = 1'b0;
        ALUSrc = 1'b0;        // Set based on instruction type
        ALUOp = 2'bXX;        // Set based on ALU operation needed
        Branch = 1'b0;
        Jump = 1'b0;
    end
endcase
```

### Step 3: Update ALU (if needed)

**File**: `src/alu.sv`

If the instruction requires a new ALU operation:

1. **Add ALU control encoding** in comments
2. **Add case statement** for new operation:
```systemverilog
case (alu_control)
    // Existing cases...
    
    // New ALU operation
    4'bXXXX: begin
        alu_result = /* operation */;
    end
endcase
```

3. **Update ALU control unit** in `ex_stage.sv` to generate the new control code

### Step 4: Update Immediate Generator (if needed)

**File**: `src/imm_gen.sv`

If the instruction uses a new immediate format:

```systemverilog
case (instruction[6:0])
    // Existing cases...
    
    // New instruction opcode
    7'bXXXXXXX: begin
        immediate = /* extract and sign-extend immediate */;
    end
endcase
```

### Step 5: Update Execute Stage (if needed)

**File**: `src/ex_stage.sv`

If special handling is needed in the EX stage:

- Add logic for operand selection
- Add logic for result calculation
- Update ALU control generation

### Step 6: Create Test Cases

**See**: [Submitting Test Cases](#submitting-test-cases)

Create comprehensive test cases covering:
- Normal operation
- Edge cases
- Boundary conditions
- Integration with existing instructions

### Step 7: Update Documentation

Update the following files:
- `docs/ARCHITECTURE.md`: Add instruction to supported set
- `docs/MODULES.md`: Document any module changes
- `README.md`: Update instruction support list

### Example: Adding MUL Instruction (RV32M)

1. **Opcode**: `7'b0110011` with `funct3 = 3'b000` and `funct7[0] = 1'b1`
2. **Control Unit**: Add case for MUL (similar to ADD but different ALU operation)
3. **ALU**: Add MUL operation (multiplication)
4. **Test**: Create test cases for MUL with various operands
5. **Documentation**: Update architecture docs

---

## Submitting Test Cases

### Test Case Requirements

All test cases must:

1. **Be self-contained**: Include all necessary setup and verification
2. **Have clear documentation**: Explain what is being tested
3. **Include expected results**: Document expected register/memory values
4. **Cover edge cases**: Test boundary conditions and error cases
5. **Follow naming conventions**: Use descriptive names (e.g., `test_add_overflow.sv`)

### Test File Structure

**Location**: `tb/` directory

**Naming**: `*_tb.sv` or `test_*.sv`

**Structure**:
```systemverilog
/**
 * @file test_instruction_name.sv
 * @brief Testbench for [Instruction Name]
 * 
 * Tests [specific functionality] including:
 * - Normal operation
 * - Edge cases
 * - Error conditions
 */

module test_instruction_name;
    // Testbench code
endmodule
```

### Test Program Structure

**Location**: `mem/` directory

**Format**: Assembly (`.s`) and hex (`.hex`) files

**Naming**: `test_program_name.s` and `test_program_name.hex`

**Example**:
```assembly
# test_add_basic.s
# Test basic ADD instruction functionality

addi x1, x0, 5      # x1 = 5
addi x2, x0, 3      # x2 = 3
add  x3, x1, x2     # x3 = x1 + x2 = 8
# Expected: x3 = 8
```

### Test Coverage Guidelines

Test cases should cover:

1. **Functional Correctness**:
   - Normal operation with various inputs
   - All instruction variants (if applicable)
   - Register combinations

2. **Edge Cases**:
   - Zero operands
   - Maximum/minimum values
   - Overflow conditions
   - x0 register handling

3. **Integration**:
   - Interaction with other instructions
   - Pipeline hazards
   - Forwarding scenarios

4. **Error Conditions**:
   - Invalid addresses
   - Boundary conditions

### Submitting Test Cases

1. **Create test files** following the structure above
2. **Add to appropriate directory** (`tb/` or `mem/`)
3. **Update test documentation** if creating a new test suite
4. **Verify tests pass** before submitting
5. **Include in pull request** with clear description

---

## Pull Request Process

### Before Submitting

1. ✅ **Test your changes** thoroughly
2. ✅ **Follow code style** guidelines
3. ✅ **Update documentation** if needed
4. ✅ **Add test cases** for new features
5. ✅ **Check for linter errors**
6. ✅ **Rebase on latest main** branch

### Creating a Pull Request

1. **Push your branch** to your fork:
   ```bash
   git push origin feature/your-feature-name
   ```

2. **Create PR on GitHub**:
   - Go to the repository
   - Click "New Pull Request"
   - Select your branch
   - Fill out the PR template

3. **PR Title Format**:
   ```
   [Type] Brief description
   ```
   Types: `Feature`, `Fix`, `Docs`, `Test`, `Refactor`

4. **PR Description Template**:
   ```markdown
   ## Description
   Brief description of changes
   
   ## Changes Made
   - Change 1
   - Change 2
   - Change 3
   
   ## Testing
   - Test cases added
   - Verification performed
   
   ## Documentation
   - Documentation updated
   
   ## Related Issues
   Closes #issue_number
   ```

### PR Review Process

1. **Automated Checks**: CI/CD will run tests and linting
2. **Code Review**: Maintainers will review your code
3. **Feedback**: Address any requested changes
4. **Approval**: Once approved, PR will be merged

### PR Guidelines

- **Keep PRs focused**: One feature/fix per PR
- **Keep PRs small**: Easier to review and merge
- **Respond to feedback**: Address review comments promptly
- **Update PR**: Rebase if main branch changes
- **Be patient**: Reviews may take time

---

## Reporting Issues

### Bug Reports

When reporting bugs, include:

1. **Description**: Clear description of the issue
2. **Steps to Reproduce**: Detailed steps to reproduce
3. **Expected Behavior**: What should happen
4. **Actual Behavior**: What actually happens
5. **Environment**: Simulator, OS, versions
6. **Test Case**: Minimal test case demonstrating the issue
7. **Screenshots/Logs**: If applicable

### Feature Requests

When requesting features, include:

1. **Description**: Clear description of the feature
2. **Use Case**: Why this feature is needed
3. **Proposed Solution**: How you envision it working
4. **Alternatives**: Other approaches considered

### Issue Template

Use the GitHub issue template or include:

```markdown
**Description**
[Clear description]

**Steps to Reproduce**
1. Step 1
2. Step 2
3. Step 3

**Expected Behavior**
[What should happen]

**Actual Behavior**
[What actually happens]

**Environment**
- Simulator: [Name and version]
- OS: [Operating system]
- Project Version: [Git commit or tag]

**Additional Context**
[Any other relevant information]
```

---

## Documentation Contributions

### Documentation Standards

- **Markdown Format**: All documentation in Markdown (`.md`)
- **Clear Structure**: Use headers, lists, and code blocks
- **Code Examples**: Include working code examples
- **Diagrams**: Use ASCII art or Mermaid diagrams
- **Links**: Link to related documentation

### Documentation Files

- **`README.md`**: Project overview and quick start
- **`docs/ARCHITECTURE.md`**: Detailed architecture documentation
- **`docs/MODULES.md`**: Module-level documentation
- **`docs/HAZARDS.md`**: Hazard handling documentation
- **`docs/TEST_REPORT.md`**: Test results and coverage
- **`docs/BUILD.md`**: Build instructions
- **`docs/SIMULATION.md`**: Simulation guide

### Updating Documentation

When updating documentation:

1. **Keep it current**: Update docs when code changes
2. **Be clear**: Use clear, concise language
3. **Include examples**: Show, don't just tell
4. **Cross-reference**: Link related sections
5. **Review**: Check for typos and clarity

---

## Development Workflow

### Branch Naming

- `feature/description`: New features
- `fix/description`: Bug fixes
- `docs/description`: Documentation updates
- `test/description`: Test additions
- `refactor/description`: Code refactoring

### Commit Messages

Follow conventional commits format:

```
[Type] Brief description

Detailed explanation if needed

- Change 1
- Change 2
```

Types: `feat`, `fix`, `docs`, `test`, `refactor`, `style`, `chore`

### Example Commit Messages

```
feat: Add MUL instruction support

Implements RV32M MUL instruction with:
- ALU multiplication operation
- Control unit updates
- Comprehensive test cases

fix: Correct branch target calculation

Fixed off-by-one error in branch target address calculation
for B-type instructions.

docs: Update architecture documentation

Added detailed explanation of forwarding paths and timing.
```

---

## Questions?

If you have questions about contributing:

1. **Check Documentation**: Review existing docs first
2. **Search Issues**: Check if your question was already asked
3. **Open an Issue**: Use the "Question" label
4. **Contact Maintainers**: Reach out via GitHub

---

## Recognition

Contributors will be recognized in:
- `CONTRIBUTORS.md` (if created)
- GitHub contributors page
- Release notes (for significant contributions)

Thank you for contributing to this project! 🎉

