#!/usr/bin/env python3
"""
ELF to Hex Converter for RISC-V Compliance Tests

Converts RISC-V ELF files to hex format compatible with the processor's
instruction memory initialization.

Usage:
    python3 scripts/elf_to_hex.py input.elf output.hex

Output Format:
    One 32-bit instruction per line (8 hex digits)
    Example:
        00100093
        00200113
        002081B3
"""

import sys
import struct
import os

def read_elf_sections(elf_file):
    """
    Read text section from ELF file.
    
    Note: This is a simplified ELF reader. For production use,
    consider using pyelftools library for proper ELF parsing.
    """
    with open(elf_file, 'rb') as f:
        elf_data = f.read()
    
    # Check ELF magic number
    if elf_data[:4] != b'\x7fELF':
        raise ValueError("Not a valid ELF file")
    
    # Determine if 32-bit or 64-bit
    ei_class = elf_data[4]
    if ei_class == 1:  # 32-bit
        is_64bit = False
    elif ei_class == 2:  # 64-bit
        is_64bit = True
    else:
        raise ValueError("Invalid ELF class")
    
    # Determine endianness
    ei_data = elf_data[5]
    if ei_data == 1:  # Little-endian
        endian = '<'
    elif ei_data == 2:  # Big-endian
        endian = '>'
    else:
        raise ValueError("Invalid ELF data encoding")
    
    # Read ELF header
    if is_64bit:
        # 64-bit ELF header
        e_shoff = struct.unpack(endian + 'Q', elf_data[40:48])[0]
        e_shentsize = struct.unpack(endian + 'H', elf_data[58:60])[0]
        e_shnum = struct.unpack(endian + 'H', elf_data[60:62])[0]
        e_shstrndx = struct.unpack(endian + 'H', elf_data[62:64])[0]
    else:
        # 32-bit ELF header
        e_shoff = struct.unpack(endian + 'I', elf_data[32:36])[0]
        e_shentsize = struct.unpack(endian + 'H', elf_data[46:48])[0]
        e_shnum = struct.unpack(endian + 'H', elf_data[48:50])[0]
        e_shstrndx = struct.unpack(endian + 'H', elf_data[50:52])[0]
    
    # Read section headers to find .text section
    text_data = None
    for i in range(e_shnum):
        sh_offset = e_shoff + i * e_shentsize
        
        if is_64bit:
            sh_type = struct.unpack(endian + 'I', elf_data[sh_offset + 4:sh_offset + 8])[0]
            sh_offset_addr = sh_offset + 16
            sh_size_addr = sh_offset + 24
            sh_addr_addr = sh_offset + 16
        else:
            sh_type = struct.unpack(endian + 'I', elf_data[sh_offset + 4:sh_offset + 8])[0]
            sh_offset_addr = sh_offset + 16
            sh_size_addr = sh_offset + 20
            sh_addr_addr = sh_offset + 12
        
        if sh_type == 1:  # SHT_PROGBITS
            sh_offset_val = struct.unpack(endian + ('Q' if is_64bit else 'I'), 
                                         elf_data[sh_offset_addr:sh_offset_addr + (8 if is_64bit else 4)])[0]
            sh_size = struct.unpack(endian + ('Q' if is_64bit else 'I'), 
                                    elf_data[sh_size_addr:sh_size_addr + (8 if is_64bit else 4)])[0]
            
            # Read section data
            section_data = elf_data[sh_offset_val:sh_offset_val + sh_size]
            
            # Check if this looks like code (starts with common RISC-V instructions)
            if len(section_data) >= 4 and section_data[:4] != b'\x00' * 4:
                text_data = section_data
                break
    
    return text_data, endian

def convert_elf_to_hex(elf_file, hex_file):
    """
    Convert ELF file to hex format.
    
    Args:
        elf_file: Path to input ELF file
        hex_file: Path to output hex file
    """
    try:
        # Try using pyelftools if available (more reliable)
        try:
            from elftools.elf.elffile import ELFFile
            from elftools.elf.sections import Section
            
            with open(elf_file, 'rb') as f:
                elf = ELFFile(f)
                
                # Find .text section
                text_section = None
                for section in elf.iter_sections():
                    if section.name == '.text':
                        text_section = section
                        break
                
                if text_section is None:
                    raise ValueError("No .text section found in ELF file")
                
                # Read text section data
                text_data = text_section.data()
                
        except ImportError:
            # Fallback to simple ELF reader
            text_data, endian = read_elf_sections(elf_file)
            if text_data is None:
                raise ValueError("Could not find .text section in ELF file")
        
        # Convert binary data to hex format
        # RISC-V instructions are 32-bit (4 bytes), little-endian
        instructions = []
        for i in range(0, len(text_data), 4):
            if i + 4 <= len(text_data):
                # Read 32-bit instruction (little-endian)
                instruction_bytes = text_data[i:i+4]
                instruction = struct.unpack('<I', instruction_bytes)[0]
                instructions.append(instruction)
        
        # Write hex file (one instruction per line, 8 hex digits)
        with open(hex_file, 'w') as f:
            f.write("// RISC-V Compliance Test - Converted from ELF\n")
            f.write("// Format: One 32-bit instruction per line (8 hex digits)\n")
            f.write("// Instructions: %d\n" % len(instructions))
            f.write("\n")
            
            for instruction in instructions:
                f.write("%08X\n" % instruction)
        
        print(f"Converted {len(instructions)} instructions")
        return True
        
    except Exception as e:
        print(f"Error converting ELF to hex: {e}", file=sys.stderr)
        return False

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 elf_to_hex.py <input.elf> <output.hex>", file=sys.stderr)
        sys.exit(1)
    
    elf_file = sys.argv[1]
    hex_file = sys.argv[2]
    
    if not os.path.exists(elf_file):
        print(f"Error: ELF file not found: {elf_file}", file=sys.stderr)
        sys.exit(1)
    
    if convert_elf_to_hex(elf_file, hex_file):
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()

