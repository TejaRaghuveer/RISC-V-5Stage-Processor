#!/usr/bin/env python3
"""
RISC-V Golden Reference Test Generator

Generates RISC-V instruction sequences with simulated execution results.
Tracks register file and memory state changes to produce golden reference outputs.

Usage:
    python generate_golden_test.py [options]
"""

import random
import argparse
import sys
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field
from enum import Enum


class InstructionType(Enum):
    """Instruction type enumeration"""
    R_TYPE = "R_TYPE"
    I_TYPE = "I_TYPE"
    LOAD = "LOAD"
    STORE = "STORE"
    BRANCH = "BRANCH"
    JAL = "JAL"
    JALR = "JALR"


@dataclass
class Instruction:
    """Represents a RISC-V instruction"""
    mnemonic: str
    rd: Optional[int] = None
    rs1: Optional[int] = None
    rs2: Optional[int] = None
    imm: Optional[int] = None
    target_label: Optional[str] = None
    comment: str = ""
    hex_encoding: int = 0
    pc: int = 0  # Program counter when this instruction executes


class RISCVSimulator:
    """RISC-V instruction simulator for golden reference generation"""
    
    def __init__(self, seed=None):
        """Initialize simulator state"""
        if seed is not None:
            random.seed(seed)
        
        # Register file (x0-x31), x0 is hardwired to 0
        self.registers = [0] * 32
        
        # Memory (word-addressable)
        self.memory = {}
        
        # Program counter
        self.pc = 0
        
        # Instruction list
        self.instructions: List[Instruction] = []
        
        # Label to PC mapping
        self.labels: Dict[str, int] = {}
        
        # Execution trace
        self.execution_trace: List[str] = []
    
    def reset(self):
        """Reset simulator state"""
        self.registers = [0] * 32
        self.memory = {}
        self.pc = 0
        self.execution_trace = []
    
    def get_reg(self, reg_num: int) -> int:
        """Get register value (x0 always returns 0)"""
        if reg_num == 0:
            return 0
        return self.registers[reg_num]
    
    def set_reg(self, reg_num: int, value: int):
        """Set register value (x0 is read-only)"""
        if reg_num != 0:
            self.registers[reg_num] = value & 0xFFFFFFFF
    
    def get_mem(self, addr: int) -> int:
        """Get memory word (word address)"""
        return self.memory.get(addr, 0)
    
    def set_mem(self, addr: int, value: int):
        """Set memory word (word address)"""
        self.memory[addr] = value & 0xFFFFFFFF
    
    def sign_extend(self, value: int, bits: int) -> int:
        """Sign extend value to 32 bits"""
        sign_bit = 1 << (bits - 1)
        if value & sign_bit:
            return value | (~((1 << bits) - 1))
        return value & ((1 << bits) - 1)
    
    def execute_add(self, rd: int, rs1: int, rs2: int):
        """Execute ADD instruction"""
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        result = (val1 + val2) & 0xFFFFFFFF
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} + x{rs2} = {val1} + {val2} = {result}"
    
    def execute_sub(self, rd: int, rs1: int, rs2: int):
        """Execute SUB instruction"""
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        result = (val1 - val2) & 0xFFFFFFFF
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} - x{rs2} = {val1} - {val2} = {result}"
    
    def execute_and(self, rd: int, rs1: int, rs2: int):
        """Execute AND instruction"""
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        result = val1 & val2
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} & x{rs2} = {val1} & {val2} = {hex(result)}"
    
    def execute_or(self, rd: int, rs1: int, rs2: int):
        """Execute OR instruction"""
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        result = val1 | val2
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} | x{rs2} = {val1} | {val2} = {hex(result)}"
    
    def execute_xor(self, rd: int, rs1: int, rs2: int):
        """Execute XOR instruction"""
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        result = val1 ^ val2
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} ^ x{rs2} = {val1} ^ {val2} = {hex(result)}"
    
    def execute_addi(self, rd: int, rs1: int, imm: int):
        """Execute ADDI instruction"""
        val1 = self.get_reg(rs1)
        imm_sext = self.sign_extend(imm, 12)
        result = (val1 + imm_sext) & 0xFFFFFFFF
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} + {imm} = {val1} + {imm_sext} = {result}"
    
    def execute_andi(self, rd: int, rs1: int, imm: int):
        """Execute ANDI instruction"""
        val1 = self.get_reg(rs1)
        imm_sext = self.sign_extend(imm, 12)
        result = val1 & imm_sext
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} & {imm} = {val1} & {imm_sext} = {hex(result)}"
    
    def execute_ori(self, rd: int, rs1: int, imm: int):
        """Execute ORI instruction"""
        val1 = self.get_reg(rs1)
        imm_sext = self.sign_extend(imm, 12)
        result = val1 | imm_sext
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} | {imm} = {val1} | {imm_sext} = {hex(result)}"
    
    def execute_xori(self, rd: int, rs1: int, imm: int):
        """Execute XORI instruction"""
        val1 = self.get_reg(rs1)
        imm_sext = self.sign_extend(imm, 12)
        result = val1 ^ imm_sext
        self.set_reg(rd, result)
        return f"x{rd} = x{rs1} ^ {imm} = {val1} ^ {imm_sext} = {hex(result)}"
    
    def execute_lw(self, rd: int, rs1: int, offset: int):
        """Execute LW instruction"""
        base = self.get_reg(rs1)
        offset_sext = self.sign_extend(offset, 12)
        addr = (base + offset_sext) & 0xFFFFFFFF
        word_addr = addr // 4  # Convert byte address to word address
        value = self.get_mem(word_addr)
        self.set_reg(rd, value)
        return f"x{rd} = memory[{word_addr}] = {value} (byte addr: {hex(addr)})"
    
    def execute_sw(self, rs1: int, rs2: int, offset: int):
        """Execute SW instruction"""
        base = self.get_reg(rs1)
        value = self.get_reg(rs2)
        offset_sext = self.sign_extend(offset, 12)
        addr = (base + offset_sext) & 0xFFFFFFFF
        word_addr = addr // 4  # Convert byte address to word address
        self.set_mem(word_addr, value)
        return f"memory[{word_addr}] = x{rs2} = {value} (byte addr: {hex(addr)})"
    
    def evaluate_branch(self, op: str, rs1: int, rs2: int) -> bool:
        """Evaluate branch condition"""
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        
        if op == "BEQ":
            return val1 == val2
        elif op == "BNE":
            return val1 != val2
        elif op == "BLT":
            val1_signed = self.sign_extend(val1, 32)
            val2_signed = self.sign_extend(val2, 32)
            return val1_signed < val2_signed
        elif op == "BGE":
            val1_signed = self.sign_extend(val1, 32)
            val2_signed = self.sign_extend(val2, 32)
            return val1_signed >= val2_signed
        elif op == "BLTU":
            return val1 < val2
        elif op == "BGEU":
            return val1 >= val2
        return False
    
    def execute_branch(self, op: str, rs1: int, rs2: int, offset: int) -> Tuple[bool, int]:
        """Execute branch instruction, return (taken, target_pc)"""
        taken = self.evaluate_branch(op, rs1, rs2)
        val1 = self.get_reg(rs1)
        val2 = self.get_reg(rs2)
        offset_sext = self.sign_extend(offset, 13)
        target_pc = self.pc + offset_sext
        return taken, target_pc
    
    def execute_jal(self, rd: int, offset: int) -> int:
        """Execute JAL instruction, return target PC"""
        self.set_reg(rd, self.pc + 4)
        offset_sext = self.sign_extend(offset, 21)
        target_pc = self.pc + offset_sext
        return target_pc
    
    def execute_jalr(self, rd: int, rs1: int, offset: int) -> int:
        """Execute JALR instruction, return target PC"""
        self.set_reg(rd, self.pc + 4)
        base = self.get_reg(rs1)
        offset_sext = self.sign_extend(offset, 12)
        target_pc = (base + offset_sext) & 0xFFFFFFFE  # Clear LSB
        return target_pc
    
    def simulate_instruction(self, instr: Instruction) -> str:
        """Simulate execution of an instruction"""
        self.pc = instr.pc
        trace = f"PC={hex(self.pc)}: {instr.mnemonic}"
        
        if instr.mnemonic == "ADD":
            result = self.execute_add(instr.rd, instr.rs1, instr.rs2)
        elif instr.mnemonic == "SUB":
            result = self.execute_sub(instr.rd, instr.rs1, instr.rs2)
        elif instr.mnemonic == "AND":
            result = self.execute_and(instr.rd, instr.rs1, instr.rs2)
        elif instr.mnemonic == "OR":
            result = self.execute_or(instr.rd, instr.rs1, instr.rs2)
        elif instr.mnemonic == "XOR":
            result = self.execute_xor(instr.rd, instr.rs1, instr.rs2)
        elif instr.mnemonic == "ADDI":
            result = self.execute_addi(instr.rd, instr.rs1, instr.imm)
        elif instr.mnemonic == "ANDI":
            result = self.execute_andi(instr.rd, instr.rs1, instr.imm)
        elif instr.mnemonic == "ORI":
            result = self.execute_ori(instr.rd, instr.rs1, instr.imm)
        elif instr.mnemonic == "XORI":
            result = self.execute_xori(instr.rd, instr.rs1, instr.imm)
        elif instr.mnemonic == "LW":
            result = self.execute_lw(instr.rd, instr.rs1, instr.imm)
        elif instr.mnemonic == "SW":
            result = self.execute_sw(instr.rs1, instr.rs2, instr.imm)
        elif instr.mnemonic in ["BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU"]:
            taken, target = self.execute_branch(instr.mnemonic, instr.rs1, instr.rs2, instr.imm)
            result = f"Branch {'TAKEN' if taken else 'NOT TAKEN'}, target={hex(target)}"
        elif instr.mnemonic == "JAL":
            target = self.execute_jal(instr.rd, instr.imm)
            result = f"Jump to {hex(target)}, return address saved in x{instr.rd}"
        elif instr.mnemonic == "JALR":
            target = self.execute_jalr(instr.rd, instr.rs1, instr.imm)
            result = f"Jump to {hex(target)}, return address saved in x{instr.rd}"
        else:
            result = "Unknown instruction"
        
        trace += f" -> {result}"
        self.execution_trace.append(trace)
        return result


class InstructionEncoder:
    """Encodes RISC-V instructions to machine code"""
    
    @staticmethod
    def encode_r_type(op: str, rd: int, rs1: int, rs2: int) -> int:
        """Encode R-type instruction"""
        funct7_map = {"ADD": 0x00, "SUB": 0x20, "AND": 0x00, "OR": 0x00, "XOR": 0x00}
        funct3_map = {"ADD": 0x0, "SUB": 0x0, "AND": 0x7, "OR": 0x6, "XOR": 0x4}
        
        funct7 = funct7_map.get(op, 0x00)
        funct3 = funct3_map.get(op, 0x0)
        opcode = 0x33
        
        return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
    
    @staticmethod
    def encode_i_type(op: str, rd: int, rs1: int, imm: int) -> int:
        """Encode I-type instruction"""
        funct3_map = {"ADDI": 0x0, "ANDI": 0x7, "ORI": 0x6, "XORI": 0x4, "LW": 0x2, "JALR": 0x0}
        opcode_map = {"ADDI": 0x13, "ANDI": 0x13, "ORI": 0x13, "XORI": 0x13, "LW": 0x03, "JALR": 0x67}
        
        funct3 = funct3_map.get(op, 0x0)
        opcode = opcode_map.get(op, 0x13)
        
        imm12 = imm & 0xFFF
        if imm < 0:
            imm12 |= 0xFFFFF000
        
        return (imm12 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
    
    @staticmethod
    def encode_s_type(rs1: int, rs2: int, offset: int) -> int:
        """Encode S-type instruction (SW)"""
        opcode = 0x23
        funct3 = 0x2
        
        offset12 = offset & 0xFFF
        if offset < 0:
            offset12 |= 0xFFFFF000
        
        imm11_5 = (offset12 >> 5) & 0x7F
        imm4_0 = offset12 & 0x1F
        
        return (imm11_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (imm4_0 << 7) | opcode
    
    @staticmethod
    def encode_b_type(op: str, rs1: int, rs2: int, offset: int) -> int:
        """Encode B-type instruction"""
        funct3_map = {"BEQ": 0x0, "BNE": 0x1, "BLT": 0x4, "BGE": 0x5, "BLTU": 0x6, "BGEU": 0x7}
        opcode = 0x63
        
        funct3 = funct3_map.get(op, 0x0)
        offset = offset // 2  # Branch offset is in multiples of 2
        
        if offset < 0:
            offset |= 0xE000
        
        imm12 = (offset >> 11) & 0x1
        imm10_5 = (offset >> 5) & 0x3F
        imm4_1 = (offset >> 1) & 0xF
        imm11 = (offset >> 10) & 0x1
        
        return (imm12 << 31) | (imm10_5 << 25) | (rs2 << 20) | (rs1 << 15) | \
               (funct3 << 12) | (imm4_1 << 8) | (imm11 << 7) | opcode
    
    @staticmethod
    def encode_j_type(rd: int, offset: int) -> int:
        """Encode J-type instruction (JAL)"""
        opcode = 0x6F
        
        offset = offset // 2  # JAL offset is in multiples of 2
        
        if offset < 0:
            offset |= 0xFFE00000
        
        imm20 = (offset >> 20) & 0x1
        imm10_1 = (offset >> 1) & 0x3FF
        imm11 = (offset >> 11) & 0x1
        imm19_12 = (offset >> 12) & 0xFF
        
        return (imm20 << 31) | (imm10_1 << 21) | (imm11 << 20) | (imm19_12 << 12) | (rd << 7) | opcode
    
    @staticmethod
    def encode_instruction(instr: Instruction) -> int:
        """Encode instruction based on its type"""
        if instr.mnemonic in ["ADD", "SUB", "AND", "OR", "XOR"]:
            return InstructionEncoder.encode_r_type(instr.mnemonic, instr.rd, instr.rs1, instr.rs2)
        elif instr.mnemonic in ["ADDI", "ANDI", "ORI", "XORI", "LW", "JALR"]:
            return InstructionEncoder.encode_i_type(instr.mnemonic, instr.rd, instr.rs1, instr.imm)
        elif instr.mnemonic == "SW":
            return InstructionEncoder.encode_s_type(instr.rs1, instr.rs2, instr.imm)
        elif instr.mnemonic in ["BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU"]:
            return InstructionEncoder.encode_b_type(instr.mnemonic, instr.rs1, instr.rs2, instr.imm)
        elif instr.mnemonic == "JAL":
            return InstructionEncoder.encode_j_type(instr.rd, instr.imm)
        return 0


class TestGenerator:
    """Generates test sequences with golden reference"""
    
    def __init__(self, seed=None):
        """Initialize test generator"""
        self.simulator = RISCVSimulator(seed)
        self.encoder = InstructionEncoder()
        self.seed = seed
        if seed is not None:
            random.seed(seed)
    
    def generate_simple_sequence(self, length: int) -> List[Instruction]:
        """Generate a simple instruction sequence"""
        instructions = []
        pc = 0
        
        # Initialize some registers
        instructions.append(Instruction("ADDI", rd=1, rs1=0, imm=10, pc=pc, comment="Initialize x1 = 10"))
        pc += 4
        instructions.append(Instruction("ADDI", rd=2, rs1=0, imm=20, pc=pc, comment="Initialize x2 = 20"))
        pc += 4
        
        # Generate random instructions
        for i in range(length - 2):
            inst_type = random.choice(["ADD", "SUB", "ADDI", "AND", "OR", "XOR"])
            
            if inst_type in ["ADD", "SUB", "AND", "OR", "XOR"]:
                rd = random.randint(3, 10)
                rs1 = random.randint(1, 10)
                rs2 = random.randint(1, 10)
                instructions.append(Instruction(inst_type, rd=rd, rs1=rs1, rs2=rs2, pc=pc))
            elif inst_type == "ADDI":
                rd = random.randint(3, 10)
                rs1 = random.randint(1, 10)
                imm = random.randint(-100, 100)
                instructions.append(Instruction(inst_type, rd=rd, rs1=rs1, imm=imm, pc=pc))
            
            pc += 4
        
        return instructions
    
    def simulate_sequence(self, instructions: List[Instruction]) -> Dict:
        """Simulate instruction sequence and return final state"""
        self.simulator.reset()
        
        # First pass: assign PCs and resolve labels
        for i, instr in enumerate(instructions):
            instr.pc = i * 4
        
        # Second pass: encode instructions
        for instr in instructions:
            instr.hex_encoding = self.encoder.encode_instruction(instr)
        
        # Third pass: simulate execution
        pc = 0
        i = 0
        max_iterations = len(instructions) * 10  # Prevent infinite loops
        
        while i < len(instructions) and pc < len(instructions) * 4 and max_iterations > 0:
            instr = instructions[pc // 4]
            instr.pc = pc
            
            # Simulate instruction
            result = self.simulator.simulate_instruction(instr)
            instr.comment = result
            
            # Handle control flow
            if instr.mnemonic in ["BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU"]:
                taken, target = self.simulator.execute_branch(
                    instr.mnemonic, instr.rs1, instr.rs2, instr.imm
                )
                if taken:
                    pc = target
                    continue
            elif instr.mnemonic == "JAL":
                target = self.simulator.execute_jal(instr.rd, instr.imm)
                pc = target
                continue
            elif instr.mnemonic == "JALR":
                target = self.simulator.execute_jalr(instr.rd, instr.rs1, instr.imm)
                pc = target
                continue
            
            pc += 4
            i += 1
            max_iterations -= 1
        
        return {
            'registers': self.simulator.registers.copy(),
            'memory': self.simulator.memory.copy(),
            'trace': self.simulator.execution_trace.copy()
        }
    
    def write_output(self, instructions: List[Instruction], final_state: Dict, 
                     hex_file: str, ref_file: str, asm_file: str):
        """Write output files"""
        # Write hex file
        with open(hex_file, 'w') as f:
            f.write("// RISC-V Test Program with Golden Reference\n")
            f.write(f"// Generated with seed: {self.seed}\n")
            f.write("// Format: One 32-bit instruction per line (8 hex digits)\n\n")
            
            for instr in instructions:
                hex_str = f"{instr.hex_encoding:08X}"
                comment = instr.comment if instr.comment else ""
                f.write(f"{hex_str}  // {instr.mnemonic} - {comment}\n")
        
        # Write assembly file with detailed comments
        with open(asm_file, 'w') as f:
            f.write("# RISC-V Assembly Test Program\n")
            f.write(f"# Generated with seed: {self.seed}\n")
            f.write("# Expected execution flow and results:\n\n")
            
            for instr in instructions:
                # Build instruction string
                if instr.mnemonic in ["ADD", "SUB", "AND", "OR", "XOR"]:
                    inst_str = f"    {instr.mnemonic} x{instr.rd}, x{instr.rs1}, x{instr.rs2}"
                elif instr.mnemonic in ["ADDI", "ANDI", "ORI", "XORI"]:
                    inst_str = f"    {instr.mnemonic} x{instr.rd}, x{instr.rs1}, {instr.imm}"
                elif instr.mnemonic == "LW":
                    inst_str = f"    {instr.mnemonic} x{instr.rd}, {instr.imm}(x{instr.rs1})"
                elif instr.mnemonic == "SW":
                    inst_str = f"    {instr.mnemonic} x{instr.rs2}, {instr.imm}(x{instr.rs1})"
                elif instr.mnemonic in ["BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU"]:
                    inst_str = f"    {instr.mnemonic} x{instr.rs1}, x{instr.rs2}, {instr.imm}"
                elif instr.mnemonic == "JAL":
                    inst_str = f"    {instr.mnemonic} x{instr.rd}, {instr.imm}"
                elif instr.mnemonic == "JALR":
                    inst_str = f"    {instr.mnemonic} x{instr.rd}, {instr.imm}(x{instr.rs1})"
                else:
                    inst_str = f"    {instr.mnemonic}"
                
                f.write(f"{inst_str}  # PC={hex(instr.pc)}: {instr.comment}\n")
        
        # Write golden reference file (SystemVerilog compatible)
        with open(ref_file, 'w') as f:
            f.write("// Golden Reference Results\n")
            f.write(f"// Generated with seed: {self.seed}\n")
            f.write("// Format: SystemVerilog compatible for testbench comparison\n\n")
            
            # Register file state
            f.write("// Expected Final Register File State\n")
            f.write("// Format: register[address] = value\n")
            f.write("// Non-zero registers only\n")
            for i, val in enumerate(final_state['registers']):
                if val != 0 or i == 0:  # Include x0 for completeness
                    f.write(f"// x{i} = {val} (0x{val:08X})\n")
            
            f.write("\n// Expected Final Memory State\n")
            f.write("// Format: memory[word_address] = value\n")
            f.write("// Non-zero memory locations only\n")
            if final_state['memory']:
                for addr in sorted(final_state['memory'].keys()):
                    val = final_state['memory'][addr]
                    f.write(f"// memory[{addr}] = {val} (0x{val:08X})\n")
            else:
                f.write("// No memory writes occurred\n")
            
            f.write("\n// Execution Trace\n")
            f.write("// Instruction execution order and results\n")
            for trace_line in final_state['trace']:
                f.write(f"// {trace_line}\n")
            
            # SystemVerilog format for direct inclusion
            f.write("\n// SystemVerilog Format (for testbench)\n")
            f.write("// Expected register values\n")
            f.write("logic [31:0] expected_registers [0:31];\n")
            for i, val in enumerate(final_state['registers']):
                f.write(f"assign expected_registers[{i}] = 32'h{val:08X};  // x{i}\n")
            
            f.write("\n// Expected memory values\n")
            if final_state['memory']:
                f.write("logic [31:0] expected_memory [0:1023];\n")
                f.write("// Initialize all to zero\n")
                f.write("initial begin\n")
                f.write("    for (int i = 0; i < 1024; i++) begin\n")
                f.write("        expected_memory[i] = 32'h00000000;\n")
                f.write("    end\n")
                f.write("end\n")
                f.write("// Set expected values\n")
                for addr in sorted(final_state['memory'].keys()):
                    val = final_state['memory'][addr]
                    f.write(f"assign expected_memory[{addr}] = 32'h{val:08X};\n")
            else:
                f.write("// No memory writes expected\n")


def main():
    parser = argparse.ArgumentParser(
        description='Generate RISC-V tests with golden reference results',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--length', type=int, default=20,
                       help='Number of instructions to generate (default: 20)')
    parser.add_argument('--seed', type=int, default=None,
                       help='Random seed for reproducibility')
    parser.add_argument('--hex', type=str, default='mem/golden_test.hex',
                       help='Output hex file (default: mem/golden_test.hex)')
    parser.add_argument('--ref', type=str, default='mem/golden_test_ref.sv',
                       help='Output reference file (default: mem/golden_test_ref.sv)')
    parser.add_argument('--asm', type=str, default='mem/golden_test.s',
                       help='Output assembly file (default: mem/golden_test.s)')
    
    args = parser.parse_args()
    
    # Generate test
    generator = TestGenerator(seed=args.seed)
    instructions = generator.generate_simple_sequence(args.length)
    final_state = generator.simulate_sequence(instructions)
    
    # Write output files
    generator.write_output(instructions, final_state, args.hex, args.ref, args.asm)
    
    print(f"Generated {len(instructions)} instructions")
    print(f"Hex file: {args.hex}")
    print(f"Reference file: {args.ref}")
    print(f"Assembly file: {args.asm}")
    print(f"\nFinal state:")
    print(f"  Non-zero registers: {sum(1 for r in final_state['registers'] if r != 0)}")
    print(f"  Memory locations written: {len(final_state['memory'])}")
    print(f"  Instructions executed: {len(final_state['trace'])}")


if __name__ == '__main__':
    main()

