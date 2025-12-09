#!/usr/bin/env python3
"""
RISC-V RV32I Random Instruction Sequence Generator

Generates random RISC-V instruction sequences for processor testing.
Supports arithmetic, logical, immediate, load/store, branch, and jump instructions.

Usage:
    python generate_random_test.py [options]

Options:
    --length N          Number of instructions to generate (default: 100)
    --output FILE       Output hex file path (default: mem/random_test.hex)
    --seed N            Random seed for reproducibility (default: random)
    --config FILE       Configuration file for instruction mix (optional)
"""

import random
import argparse
import sys
from typing import List, Tuple, Dict

# RISC-V Instruction Encodings
OPCODES = {
    'R_TYPE': 0x33,      # ADD, SUB, AND, OR, XOR
    'I_TYPE': 0x13,      # ADDI, ANDI, ORI, XORI
    'LOAD': 0x03,        # LW
    'STORE': 0x23,       # SW
    'BRANCH': 0x63,      # BEQ, BNE, BLT, BGE, BLTU, BGEU
    'JAL': 0x6F,         # JAL
    'JALR': 0x67,        # JALR
}

FUNCT3 = {
    'ADD': 0x0, 'SUB': 0x0,
    'AND': 0x7, 'OR': 0x6, 'XOR': 0x4,
    'ADDI': 0x0, 'ANDI': 0x7, 'ORI': 0x6, 'XORI': 0x4,
    'LW': 0x2, 'SW': 0x2,
    'BEQ': 0x0, 'BNE': 0x1, 'BLT': 0x4, 'BGE': 0x5, 'BLTU': 0x6, 'BGEU': 0x7,
    'JALR': 0x0,
}

FUNCT7 = {
    'ADD': 0x00, 'SUB': 0x20,
    'AND': 0x00, 'OR': 0x00, 'XOR': 0x00,
}

# Default instruction mix percentages
DEFAULT_MIX = {
    'R_TYPE': 20,    # Arithmetic/logical R-type
    'I_TYPE': 25,    # Immediate instructions
    'LOAD': 10,      # Load instructions
    'STORE': 10,     # Store instructions
    'BRANCH': 20,    # Branch instructions
    'JAL': 5,        # JAL instructions
    'JALR': 10,      # JALR instructions
}


class RISCVRandomGenerator:
    """RISC-V Random Instruction Generator"""
    
    def __init__(self, seed=None, instruction_mix=None):
        """
        Initialize generator
        
        Args:
            seed: Random seed for reproducibility
            instruction_mix: Dictionary of instruction type percentages
        """
        if seed is not None:
            random.seed(seed)
        
        self.instruction_mix = instruction_mix or DEFAULT_MIX.copy()
        self.normalize_mix()
        
        # Track register usage for generating realistic sequences
        self.registers = list(range(1, 32))  # x1 to x31
        self.memory_base = 0x1000  # Base address for memory operations
        
    def normalize_mix(self):
        """Normalize instruction mix percentages to sum to 100"""
        total = sum(self.instruction_mix.values())
        if total != 100:
            # Normalize to percentages
            for key in self.instruction_mix:
                self.instruction_mix[key] = int(self.instruction_mix[key] * 100 / total)
            # Adjust to ensure sum is exactly 100
            diff = 100 - sum(self.instruction_mix.values())
            if diff != 0:
                # Add difference to most common type
                max_key = max(self.instruction_mix, key=self.instruction_mix.get)
                self.instruction_mix[max_key] += diff
    
    def get_random_reg(self, exclude=None):
        """Get random register address (x1-x31)"""
        available = [r for r in self.registers if r != exclude]
        return random.choice(available)
    
    def get_random_regs(self, count, exclude=None):
        """Get multiple random register addresses"""
        available = [r for r in self.registers if r != exclude]
        return random.sample(available, min(count, len(available)))
    
    def get_random_imm12(self):
        """Get random 12-bit signed immediate"""
        return random.randint(-2048, 2047)
    
    def get_random_imm20(self):
        """Get random 20-bit signed immediate (for JAL)"""
        return random.randint(-524288, 524287)
    
    def get_random_mem_offset(self):
        """Get random memory offset (word-aligned)"""
        # Generate word-aligned offset
        offset = random.randint(0, 1023) * 4  # 0 to 4092, step 4
        return offset
    
    def select_instruction_type(self):
        """Select instruction type based on mix percentages"""
        rand = random.randint(1, 100)
        cumulative = 0
        for inst_type, percentage in self.instruction_mix.items():
            cumulative += percentage
            if rand <= cumulative:
                return inst_type
        return 'I_TYPE'  # Default fallback
    
    def encode_r_type(self, op, rd, rs1, rs2):
        """Encode R-type instruction"""
        funct7 = FUNCT7.get(op, 0x00)
        funct3 = FUNCT3[op]
        opcode = OPCODES['R_TYPE']
        
        instruction = (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | \
                     (funct3 << 12) | (rd << 7) | opcode
        return instruction
    
    def encode_i_type(self, op, rd, rs1, imm):
        """Encode I-type instruction"""
        funct3 = FUNCT3[op]
        opcode = OPCODES['I_TYPE']
        
        # Sign-extend immediate to 12 bits
        imm12 = imm & 0xFFF
        if imm < 0:
            imm12 = imm12 | 0xFFFFF000
        
        instruction = (imm12 << 20) | (rs1 << 15) | \
                     (funct3 << 12) | (rd << 7) | opcode
        return instruction
    
    def encode_load(self, rd, rs1, offset):
        """Encode load instruction (LW)"""
        funct3 = FUNCT3['LW']
        opcode = OPCODES['LOAD']
        
        offset12 = offset & 0xFFF
        instruction = (offset12 << 20) | (rs1 << 15) | \
                     (funct3 << 12) | (rd << 7) | opcode
        return instruction
    
    def encode_store(self, rs1, rs2, offset):
        """Encode store instruction (SW)"""
        funct3 = FUNCT3['SW']
        opcode = OPCODES['STORE']
        
        offset12 = offset & 0xFFF
        imm11_5 = (offset12 >> 5) & 0x7F
        imm4_0 = offset12 & 0x1F
        
        instruction = (imm11_5 << 25) | (rs2 << 20) | (rs1 << 15) | \
                     (funct3 << 12) | (imm4_0 << 7) | opcode
        return instruction
    
    def encode_branch(self, op, rs1, rs2, offset):
        """Encode branch instruction (B-type)"""
        funct3 = FUNCT3[op]
        opcode = OPCODES['BRANCH']
        
        # Branch offset is in multiples of 2 bytes
        offset = offset // 2
        
        # Sign-extend to 13 bits
        if offset < 0:
            offset = offset & 0x1FFF | 0xE000
        
        imm12 = (offset >> 11) & 0x1
        imm10_5 = (offset >> 5) & 0x3F
        imm4_1 = (offset >> 1) & 0xF
        imm11 = (offset >> 10) & 0x1
        
        instruction = (imm12 << 31) | (imm10_5 << 25) | (rs2 << 20) | \
                     (rs1 << 15) | (funct3 << 12) | (imm4_1 << 8) | \
                     (imm11 << 7) | opcode
        return instruction
    
    def encode_jal(self, rd, offset):
        """Encode JAL instruction (J-type)"""
        opcode = OPCODES['JAL']
        
        # JAL offset is in multiples of 2 bytes
        offset = offset // 2
        
        # Sign-extend to 21 bits
        if offset < 0:
            offset = offset & 0x1FFFFF | 0xFFE00000
        
        imm20 = (offset >> 20) & 0x1
        imm10_1 = (offset >> 1) & 0x3FF
        imm11 = (offset >> 11) & 0x1
        imm19_12 = (offset >> 12) & 0xFF
        
        instruction = (imm20 << 31) | (imm10_1 << 21) | (imm11 << 20) | \
                     (imm19_12 << 12) | (rd << 7) | opcode
        return instruction
    
    def encode_jalr(self, rd, rs1, offset):
        """Encode JALR instruction (I-type)"""
        funct3 = FUNCT3['JALR']
        opcode = OPCODES['JALR']
        
        offset12 = offset & 0xFFF
        if offset < 0:
            offset12 = offset12 | 0xFFFFF000
        
        instruction = (offset12 << 20) | (rs1 << 15) | \
                     (funct3 << 12) | (rd << 7) | opcode
        return instruction
    
    def generate_r_type(self):
        """Generate random R-type instruction"""
        ops = ['ADD', 'SUB', 'AND', 'OR', 'XOR']
        op = random.choice(ops)
        rd, rs1, rs2 = self.get_random_regs(3)
        return self.encode_r_type(op, rd, rs1, rs2), f"{op} x{rd}, x{rs1}, x{rs2}"
    
    def generate_i_type(self):
        """Generate random I-type instruction"""
        ops = ['ADDI', 'ANDI', 'ORI', 'XORI']
        op = random.choice(ops)
        rd = self.get_random_reg()
        rs1 = self.get_random_reg(exclude=rd)
        imm = self.get_random_imm12()
        return self.encode_i_type(op, rd, rs1, imm), f"{op} x{rd}, x{rs1}, {imm}"
    
    def generate_load(self):
        """Generate random load instruction"""
        rd = self.get_random_reg()
        rs1 = self.get_random_reg(exclude=rd)
        # Use x2 (sp) as common base register, or random register
        base_reg = random.choice([2, rs1]) if rs1 != 2 else rs1
        offset = self.get_random_mem_offset()
        return self.encode_load(rd, base_reg, offset), f"LW x{rd}, {offset}(x{base_reg})"
    
    def generate_store(self):
        """Generate random store instruction"""
        rs2 = self.get_random_reg()
        rs1 = self.get_random_reg(exclude=rs2)
        # Use x2 (sp) as common base register, or random register
        base_reg = random.choice([2, rs1]) if rs1 != 2 else rs1
        offset = self.get_random_mem_offset()
        return self.encode_store(base_reg, rs2, offset), f"SW x{rs2}, {offset}(x{base_reg})"
    
    def generate_branch(self, current_pc, instructions):
        """Generate random branch instruction"""
        ops = ['BEQ', 'BNE', 'BLT', 'BGE', 'BLTU', 'BGEU']
        op = random.choice(ops)
        rs1, rs2 = self.get_random_regs(2)
        
        # Calculate reasonable branch offset
        # Branch forward or backward within reasonable range
        max_offset = min(256, len(instructions) * 4)  # Max 256 bytes or remaining instructions
        offset = random.randint(-max_offset, max_offset)
        offset = (offset // 2) * 2  # Ensure word-aligned
        
        return self.encode_branch(op, rs1, rs2, offset), \
               f"{op} x{rs1}, x{rs2}, {offset}"
    
    def generate_jal(self, current_pc, instructions):
        """Generate random JAL instruction"""
        rd = random.choice([1, self.get_random_reg()])  # Often use x1 (ra)
        
        # Calculate reasonable jump offset
        max_offset = min(1024, len(instructions) * 4)  # Max 1KB or remaining instructions
        offset = random.randint(-max_offset, max_offset)
        offset = (offset // 2) * 2  # Ensure word-aligned
        
        return self.encode_jal(rd, offset), f"JAL x{rd}, {offset}"
    
    def generate_jalr(self):
        """Generate random JALR instruction"""
        rd = random.choice([0, 1, self.get_random_reg()])  # Often use x0 or x1
        rs1 = self.get_random_reg(exclude=rd) if rd != 0 else self.get_random_reg()
        offset = random.randint(-2048, 2047)
        offset = (offset // 2) * 2  # Ensure word-aligned
        
        return self.encode_jalr(rd, rs1, offset), f"JALR x{rd}, {offset}(x{rs1})"
    
    def generate_instruction(self, index, total):
        """Generate a random instruction based on mix"""
        inst_type = self.select_instruction_type()
        
        if inst_type == 'R_TYPE':
            return self.generate_r_type()
        elif inst_type == 'I_TYPE':
            return self.generate_i_type()
        elif inst_type == 'LOAD':
            return self.generate_load()
        elif inst_type == 'STORE':
            return self.generate_store()
        elif inst_type == 'BRANCH':
            return self.generate_branch(index * 4, list(range(total)))
        elif inst_type == 'JAL':
            return self.generate_jal(index * 4, list(range(total)))
        elif inst_type == 'JALR':
            return self.generate_jalr()
        else:
            # Default to ADDI
            rd = self.get_random_reg()
            rs1 = self.get_random_reg(exclude=rd)
            imm = self.get_random_imm12()
            return self.encode_i_type('ADDI', rd, rs1, imm), f"ADDI x{rd}, x{rs1}, {imm}"


def write_hex_file(instructions: List[Tuple[int, str]], output_file: str):
    """Write instructions to hex file"""
    with open(output_file, 'w') as f:
        f.write("// RISC-V Random Instruction Sequence\n")
        f.write("// Generated by generate_random_test.py\n")
        f.write("// Format: One 32-bit instruction per line (8 hex digits)\n\n")
        
        for i, (instr, comment) in enumerate(instructions):
            hex_str = f"{instr:08X}"
            f.write(f"{hex_str}  // {comment}\n")


def load_config(config_file: str) -> Dict[str, int]:
    """Load instruction mix configuration from file"""
    mix = {}
    try:
        with open(config_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if ':' in line:
                    key, value = line.split(':', 1)
                    mix[key.strip()] = int(value.strip())
        return mix
    except FileNotFoundError:
        print(f"Warning: Config file {config_file} not found, using defaults")
        return DEFAULT_MIX.copy()
    except Exception as e:
        print(f"Error loading config: {e}, using defaults")
        return DEFAULT_MIX.copy()


def main():
    parser = argparse.ArgumentParser(
        description='Generate random RISC-V RV32I instruction sequences',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate 100 instructions with default mix
  python generate_random_test.py --length 100 --output mem/test.hex
  
  # Generate with specific seed for reproducibility
  python generate_random_test.py --length 200 --seed 42 --output mem/test.hex
  
  # Use custom instruction mix
  python generate_random_test.py --length 100 --config config.txt
        """
    )
    
    parser.add_argument('--length', type=int, default=100,
                       help='Number of instructions to generate (default: 100)')
    parser.add_argument('--output', type=str, default='mem/random_test.hex',
                       help='Output hex file path (default: mem/random_test.hex)')
    parser.add_argument('--seed', type=int, default=None,
                       help='Random seed for reproducibility (default: random)')
    parser.add_argument('--config', type=str, default=None,
                       help='Configuration file for instruction mix (optional)')
    
    # Instruction mix overrides
    parser.add_argument('--r-type', type=int, default=None,
                       help='Percentage of R-type instructions')
    parser.add_argument('--i-type', type=int, default=None,
                       help='Percentage of I-type instructions')
    parser.add_argument('--load', type=int, default=None,
                       help='Percentage of load instructions')
    parser.add_argument('--store', type=int, default=None,
                       help='Percentage of store instructions')
    parser.add_argument('--branch', type=int, default=None,
                       help='Percentage of branch instructions')
    parser.add_argument('--jal', type=int, default=None,
                       help='Percentage of JAL instructions')
    parser.add_argument('--jalr', type=int, default=None,
                       help='Percentage of JALR instructions')
    
    args = parser.parse_args()
    
    # Load configuration
    if args.config:
        instruction_mix = load_config(args.config)
    else:
        instruction_mix = DEFAULT_MIX.copy()
    
    # Override with command-line arguments
    if args.r_type is not None:
        instruction_mix['R_TYPE'] = args.r_type
    if args.i_type is not None:
        instruction_mix['I_TYPE'] = args.i_type
    if args.load is not None:
        instruction_mix['LOAD'] = args.load
    if args.store is not None:
        instruction_mix['STORE'] = args.store
    if args.branch is not None:
        instruction_mix['BRANCH'] = args.branch
    if args.jal is not None:
        instruction_mix['JAL'] = args.jal
    if args.jalr is not None:
        instruction_mix['JALR'] = args.jalr
    
    # Initialize generator
    generator = RISCVRandomGenerator(seed=args.seed, instruction_mix=instruction_mix)
    
    # Generate instructions
    print(f"Generating {args.length} random RISC-V instructions...")
    print(f"Instruction mix: {instruction_mix}")
    if args.seed is not None:
        print(f"Random seed: {args.seed}")
    
    instructions = []
    for i in range(args.length):
        instr, comment = generator.generate_instruction(i, args.length)
        instructions.append((instr, comment))
    
    # Write output file
    write_hex_file(instructions, args.output)
    print(f"Generated {len(instructions)} instructions")
    print(f"Output written to: {args.output}")
    
    # Print statistics
    print("\nInstruction Statistics:")
    inst_counts = {}
    for _, comment in instructions:
        inst_type = comment.split()[0]
        inst_counts[inst_type] = inst_counts.get(inst_type, 0) + 1
    
    for inst_type in sorted(inst_counts.keys()):
        count = inst_counts[inst_type]
        percentage = (count / len(instructions)) * 100
        print(f"  {inst_type:8s}: {count:4d} ({percentage:5.1f}%)")


if __name__ == '__main__':
    main()

