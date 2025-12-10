#!/usr/bin/env python3
"""
RISC-V Processor Performance Log Parser

This script parses simulation logs and calculates performance metrics:
- Total instructions executed
- Total cycles
- CPI (Cycles Per Instruction)
- Stall percentage
- Flush percentage
- Instruction mix (arithmetic, logic, memory, control flow)

Generates formatted reports and optional visualizations.
"""

import re
import argparse
import sys
from collections import defaultdict
from typing import Dict, List, Tuple, Optional
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available. Visualization disabled.")


class PerformanceParser:
    """Parser for RISC-V processor simulation logs"""
    
    # Instruction type patterns (opcodes and instruction names)
    ARITHMETIC_PATTERNS = [
        r'\bADD\b', r'\bSUB\b', r'\bADDI\b', r'\bSUBI\b',
        r'\bMUL\b', r'\bDIV\b', r'\bREM\b',
        r'0x[0-9A-Fa-f]{8}.*ADD', r'0x[0-9A-Fa-f]{8}.*SUB'
    ]
    
    LOGICAL_PATTERNS = [
        r'\bAND\b', r'\bOR\b', r'\bXOR\b', r'\bANDI\b', r'\bORI\b', r'\bXORI\b',
        r'\bSLL\b', r'\bSRL\b', r'\bSRA\b', r'\bSLLI\b', r'\bSRLI\b', r'\bSRAI\b',
        r'0x[0-9A-Fa-f]{8}.*AND', r'0x[0-9A-Fa-f]{8}.*OR', r'0x[0-9A-Fa-f]{8}.*XOR'
    ]
    
    MEMORY_PATTERNS = [
        r'\bLW\b', r'\bLH\b', r'\bLB\b', r'\bLHU\b', r'\bLBU\b',
        r'\bSW\b', r'\bSH\b', r'\bSB\b',
        r'0x[0-9A-Fa-f]{8}.*LW', r'0x[0-9A-Fa-f]{8}.*SW'
    ]
    
    CONTROL_FLOW_PATTERNS = [
        r'\bBEQ\b', r'\bBNE\b', r'\bBLT\b', r'\bBGE\b', r'\bBLTU\b', r'\bBGEU\b',
        r'\bJAL\b', r'\bJALR\b',
        r'0x[0-9A-Fa-f]{8}.*BEQ', r'0x[0-9A-Fa-f]{8}.*BNE',
        r'0x[0-9A-Fa-f]{8}.*JAL', r'0x[0-9A-Fa-f]{8}.*JALR'
    ]
    
    def __init__(self):
        self.total_cycles = 0
        self.instructions_completed = 0
        self.pipeline_stalls = 0
        self.pipeline_flushes = 0
        self.load_instructions = 0
        self.store_instructions = 0
        self.branch_instructions = 0
        
        # Instruction mix counters
        self.arithmetic_count = 0
        self.logical_count = 0
        self.memory_count = 0
        self.control_flow_count = 0
        self.other_count = 0
        
        # Time-series data for CPI tracking
        self.cpi_over_time = []  # List of (cycle, cpi) tuples
        self.cycle_instructions = []  # List of (cycle, instruction_count) tuples
        
        # Raw instruction log
        self.instructions = []
        
    def parse_log_file(self, log_file: Path) -> None:
        """Parse a simulation log file"""
        print(f"Parsing log file: {log_file}")
        
        try:
            with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
        except Exception as e:
            print(f"Error reading log file: {e}")
            sys.exit(1)
        
        current_cycle = 0
        
        for line_num, line in enumerate(lines, 1):
            line = line.strip()
            if not line:
                continue
            
            # Parse performance monitor output
            self._parse_performance_metrics(line)
            
            # Parse cycle information
            cycle_match = re.search(r'Cycle[:\s]+(\d+)', line, re.IGNORECASE)
            if cycle_match:
                current_cycle = int(cycle_match.group(1))
            
            # Parse instruction execution
            self._parse_instruction(line, current_cycle)
            
            # Parse stall/flush events
            self._parse_pipeline_events(line)
        
        # Calculate final metrics
        self._calculate_metrics()
    
    def _parse_performance_metrics(self, line: str) -> None:
        """Parse performance monitor output lines"""
        
        # Total cycles
        match = re.search(r'Total\s+Cycles[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.total_cycles = max(self.total_cycles, int(match.group(1)))
        
        # Instructions completed
        match = re.search(r'Instructions\s+Completed[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.instructions_completed = max(self.instructions_completed, int(match.group(1)))
        
        # Pipeline stalls
        match = re.search(r'Pipeline\s+Stalls[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.pipeline_stalls = max(self.pipeline_stalls, int(match.group(1)))
        
        # Pipeline flushes
        match = re.search(r'Pipeline\s+Flushes[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.pipeline_flushes = max(self.pipeline_flushes, int(match.group(1)))
        
        # Load instructions
        match = re.search(r'Load\s+Instructions[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.load_instructions = max(self.load_instructions, int(match.group(1)))
        
        # Store instructions
        match = re.search(r'Store\s+Instructions[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.store_instructions = max(self.store_instructions, int(match.group(1)))
        
        # Branch instructions
        match = re.search(r'Branch\s+Instructions[:\s]+(\d+)', line, re.IGNORECASE)
        if match:
            self.branch_instructions = max(self.branch_instructions, int(match.group(1)))
        
        # CPI value
        match = re.search(r'CPI[:\s]+([\d.]+)', line, re.IGNORECASE)
        if match:
            cpi = float(match.group(1))
            if self.total_cycles > 0:
                self.cpi_over_time.append((self.total_cycles, cpi))
    
    def _parse_instruction(self, line: str, cycle: int) -> None:
        """Parse instruction execution from log line"""
        
        # Look for instruction patterns
        instruction_found = False
        
        # Check arithmetic instructions
        for pattern in self.ARITHMETIC_PATTERNS:
            if re.search(pattern, line, re.IGNORECASE):
                self.arithmetic_count += 1
                self.instructions.append(('arithmetic', cycle, line))
                instruction_found = True
                break
        
        if not instruction_found:
            # Check logical instructions
            for pattern in self.LOGICAL_PATTERNS:
                if re.search(pattern, line, re.IGNORECASE):
                    self.logical_count += 1
                    self.instructions.append(('logical', cycle, line))
                    instruction_found = True
                    break
        
        if not instruction_found:
            # Check memory instructions
            for pattern in self.MEMORY_PATTERNS:
                if re.search(pattern, line, re.IGNORECASE):
                    self.memory_count += 1
                    self.instructions.append(('memory', cycle, line))
                    instruction_found = True
                    break
        
        if not instruction_found:
            # Check control flow instructions
            for pattern in self.CONTROL_FLOW_PATTERNS:
                if re.search(pattern, line, re.IGNORECASE):
                    self.control_flow_count += 1
                    self.instructions.append(('control_flow', cycle, line))
                    instruction_found = True
                    break
        
        # Count as other if it looks like an instruction but doesn't match categories
        if not instruction_found:
            # Look for hex instruction patterns
            if re.search(r'0x[0-9A-Fa-f]{8}', line):
                self.other_count += 1
                self.instructions.append(('other', cycle, line))
        
        # Track instruction count over time
        if instruction_found and cycle > 0:
            self.cycle_instructions.append((cycle, len(self.instructions)))
    
    def _parse_pipeline_events(self, line: str) -> None:
        """Parse pipeline stall and flush events"""
        
        # Stall events
        if re.search(r'stall', line, re.IGNORECASE) and not re.search(r'Pipeline\s+Stalls', line, re.IGNORECASE):
            self.pipeline_stalls += 1
        
        # Flush events
        if re.search(r'flush', line, re.IGNORECASE) and not re.search(r'Pipeline\s+Flushes', line, re.IGNORECASE):
            self.pipeline_flushes += 1
    
    def _calculate_metrics(self) -> None:
        """Calculate derived metrics"""
        
        # If we parsed instructions but not cycles, estimate cycles
        if self.total_cycles == 0 and len(self.instructions) > 0:
            # Estimate cycles from instruction count and CPI
            if self.instructions_completed > 0:
                estimated_cpi = 1.5  # Default estimate
                self.total_cycles = int(self.instructions_completed * estimated_cpi)
            else:
                self.total_cycles = len(self.instructions) * 2  # Conservative estimate
        
        # If we have instruction mix but not total instructions, sum them
        if self.instructions_completed == 0:
            total_instructions = (self.arithmetic_count + self.logical_count + 
                                self.memory_count + self.control_flow_count + 
                                self.other_count)
            if total_instructions > 0:
                self.instructions_completed = total_instructions
    
    def get_metrics(self) -> Dict:
        """Get all calculated metrics"""
        
        # Calculate CPI
        cpi = 0.0
        if self.instructions_completed > 0:
            cpi = self.total_cycles / self.instructions_completed
        
        # Calculate percentages
        stall_percentage = 0.0
        flush_percentage = 0.0
        if self.total_cycles > 0:
            stall_percentage = (self.pipeline_stalls / self.total_cycles) * 100.0
            flush_percentage = (self.pipeline_flushes / self.total_cycles) * 100.0
        
        # Instruction mix percentages
        total_instructions = self.instructions_completed
        instruction_mix = {}
        if total_instructions > 0:
            instruction_mix = {
                'arithmetic': (self.arithmetic_count / total_instructions) * 100.0,
                'logical': (self.logical_count / total_instructions) * 100.0,
                'memory': (self.memory_count / total_instructions) * 100.0,
                'control_flow': (self.control_flow_count / total_instructions) * 100.0,
                'other': (self.other_count / total_instructions) * 100.0
            }
        
        return {
            'total_cycles': self.total_cycles,
            'instructions_completed': self.instructions_completed,
            'cpi': cpi,
            'stall_percentage': stall_percentage,
            'flush_percentage': flush_percentage,
            'pipeline_stalls': self.pipeline_stalls,
            'pipeline_flushes': self.pipeline_flushes,
            'load_instructions': self.load_instructions,
            'store_instructions': self.store_instructions,
            'branch_instructions': self.branch_instructions,
            'instruction_mix': instruction_mix,
            'instruction_counts': {
                'arithmetic': self.arithmetic_count,
                'logical': self.logical_count,
                'memory': self.memory_count,
                'control_flow': self.control_flow_count,
                'other': self.other_count
            },
            'cpi_over_time': self.cpi_over_time,
            'cycle_instructions': self.cycle_instructions
        }


def generate_text_report(metrics: Dict, output_file: Optional[Path] = None) -> None:
    """Generate formatted text report"""
    
    report = []
    report.append("=" * 70)
    report.append("RISC-V Processor Performance Report")
    report.append("=" * 70)
    report.append("")
    
    # Overall metrics
    report.append("Overall Performance Metrics:")
    report.append("-" * 70)
    report.append(f"  Total Cycles:              {metrics['total_cycles']:>10,}")
    report.append(f"  Instructions Completed:    {metrics['instructions_completed']:>10,}")
    report.append(f"  CPI (Cycles Per Instruction): {metrics['cpi']:>10.4f}")
    report.append("")
    
    # Pipeline efficiency
    report.append("Pipeline Efficiency:")
    report.append("-" * 70)
    report.append(f"  Pipeline Stalls:          {metrics['pipeline_stalls']:>10,}")
    report.append(f"  Stall Percentage:          {metrics['stall_percentage']:>10.2f}%")
    report.append(f"  Pipeline Flushes:          {metrics['pipeline_flushes']:>10,}")
    report.append(f"  Flush Percentage:          {metrics['flush_percentage']:>10.2f}%")
    
    efficiency = 1.0 / metrics['cpi'] if metrics['cpi'] > 0 else 0.0
    report.append(f"  Pipeline Efficiency:      {efficiency * 100:>10.2f}%")
    report.append("")
    
    # Memory operations
    report.append("Memory Operations:")
    report.append("-" * 70)
    report.append(f"  Load Instructions:         {metrics['load_instructions']:>10,}")
    report.append(f"  Store Instructions:        {metrics['store_instructions']:>10,}")
    report.append(f"  Total Memory Operations:    {metrics['load_instructions'] + metrics['store_instructions']:>10,}")
    report.append("")
    
    # Control flow
    report.append("Control Flow:")
    report.append("-" * 70)
    report.append(f"  Branch Instructions:       {metrics['branch_instructions']:>10,}")
    report.append("")
    
    # Instruction mix
    report.append("Instruction Mix:")
    report.append("-" * 70)
    counts = metrics['instruction_counts']
    mix = metrics['instruction_mix']
    
    if mix:
        report.append(f"  Arithmetic Instructions:   {counts['arithmetic']:>10,}  ({mix['arithmetic']:>6.2f}%)")
        report.append(f"  Logical Instructions:     {counts['logical']:>10,}  ({mix['logical']:>6.2f}%)")
        report.append(f"  Memory Instructions:      {counts['memory']:>10,}  ({mix['memory']:>6.2f}%)")
        report.append(f"  Control Flow Instructions: {counts['control_flow']:>10,}  ({mix['control_flow']:>6.2f}%)")
        report.append(f"  Other Instructions:       {counts['other']:>10,}  ({mix['other']:>6.2f}%)")
    else:
        report.append("  (Instruction mix data not available)")
    
    report.append("")
    report.append("=" * 70)
    
    # Print report
    report_text = "\n".join(report)
    print(report_text)
    
    # Write to file if specified
    if output_file:
        try:
            with open(output_file, 'w') as f:
                f.write(report_text)
            print(f"\nReport written to: {output_file}")
        except Exception as e:
            print(f"Error writing report file: {e}")


def generate_visualizations(metrics: Dict, output_dir: Path) -> None:
    """Generate visualization plots"""
    
    if not HAS_MATPLOTLIB:
        print("Warning: matplotlib not available. Skipping visualizations.")
        return
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 1. Instruction Mix Bar Chart
    if metrics['instruction_mix']:
        fig, ax = plt.subplots(figsize=(10, 6))
        
        categories = list(metrics['instruction_mix'].keys())
        percentages = [metrics['instruction_mix'][cat] for cat in categories]
        counts = [metrics['instruction_counts'][cat] for cat in categories]
        
        # Create bar chart
        bars = ax.bar(categories, percentages, color=['#3498db', '#2ecc71', '#e74c3c', '#f39c12', '#9b59b6'])
        
        # Add value labels on bars
        for i, (bar, pct, count) in enumerate(zip(bars, percentages, counts)):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{pct:.1f}%\n({count:,})',
                   ha='center', va='bottom', fontsize=10)
        
        ax.set_ylabel('Percentage (%)', fontsize=12)
        ax.set_xlabel('Instruction Type', fontsize=12)
        ax.set_title('Instruction Mix Distribution', fontsize=14, fontweight='bold')
        ax.set_ylim(0, max(percentages) * 1.2 if percentages else 100)
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'instruction_mix.png', dpi=300, bbox_inches='tight')
        print(f"Saved instruction mix chart: {output_dir / 'instruction_mix.png'}")
        plt.close()
    
    # 2. CPI Over Time Line Graph
    if metrics['cpi_over_time']:
        fig, ax = plt.subplots(figsize=(12, 6))
        
        cycles = [point[0] for point in metrics['cpi_over_time']]
        cpi_values = [point[1] for point in metrics['cpi_over_time']]
        
        ax.plot(cycles, cpi_values, 'b-', linewidth=2, marker='o', markersize=4)
        ax.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='Ideal CPI (1.0)')
        
        ax.set_xlabel('Cycle', fontsize=12)
        ax.set_ylabel('CPI (Cycles Per Instruction)', fontsize=12)
        ax.set_title('CPI Over Time', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(output_dir / 'cpi_over_time.png', dpi=300, bbox_inches='tight')
        print(f"Saved CPI over time chart: {output_dir / 'cpi_over_time.png'}")
        plt.close()
    
    # 3. Pipeline Efficiency Pie Chart
    if metrics['total_cycles'] > 0:
        fig, ax = plt.subplots(figsize=(10, 8))
        
        useful_cycles = metrics['instructions_completed']
        stall_cycles = metrics['pipeline_stalls']
        flush_cycles = metrics['pipeline_flushes']
        other_cycles = metrics['total_cycles'] - useful_cycles - stall_cycles - flush_cycles
        
        labels = ['Useful Work', 'Stalls', 'Flushes', 'Other']
        sizes = [useful_cycles, stall_cycles, flush_cycles, max(0, other_cycles)]
        colors = ['#2ecc71', '#e74c3c', '#f39c12', '#95a5a6']
        explode = (0.05, 0.05, 0.05, 0)
        
        # Filter out zero values
        filtered_data = [(label, size, color, exp) for label, size, color, exp 
                        in zip(labels, sizes, colors, explode) if size > 0]
        if filtered_data:
            labels, sizes, colors, explode = zip(*filtered_data)
            
            wedges, texts, autotexts = ax.pie(sizes, explode=explode, labels=labels, colors=colors,
                                              autopct='%1.1f%%', shadow=True, startangle=90)
            
            for autotext in autotexts:
                autotext.set_color('white')
                autotext.set_fontweight('bold')
            
            ax.set_title('Pipeline Cycle Distribution', fontsize=14, fontweight='bold')
            
            plt.tight_layout()
            plt.savefig(output_dir / 'pipeline_efficiency.png', dpi=300, bbox_inches='tight')
            print(f"Saved pipeline efficiency chart: {output_dir / 'pipeline_efficiency.png'}")
        plt.close()
    
    # 4. Performance Summary Bar Chart
    fig, ax = plt.subplots(figsize=(10, 6))
    
    categories = ['CPI', 'Stall %', 'Flush %', 'Efficiency %']
    values = [
        metrics['cpi'],
        metrics['stall_percentage'],
        metrics['flush_percentage'],
        (1.0 / metrics['cpi'] * 100) if metrics['cpi'] > 0 else 0
    ]
    
    colors = ['#3498db' if v <= 1.0 else '#e74c3c' if v > 2.0 else '#f39c12' 
              for v in values]
    
    bars = ax.bar(categories, values, color=colors)
    
    # Add value labels
    for bar, val in zip(bars, values):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
               f'{val:.2f}',
               ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    ax.set_ylabel('Value', fontsize=12)
    ax.set_title('Performance Summary', fontsize=14, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)
    
    # Add ideal CPI reference line
    if categories[0] == 'CPI':
        ax.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, alpha=0.7)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'performance_summary.png', dpi=300, bbox_inches='tight')
    print(f"Saved performance summary chart: {output_dir / 'performance_summary.png'}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Parse RISC-V processor simulation logs and generate performance reports',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Parse log file and generate text report
  python parse_performance_logs.py simulation.log

  # Generate report with visualizations
  python parse_performance_logs.py simulation.log --visualize --output-dir reports/

  # Save text report to file
  python parse_performance_logs.py simulation.log --report report.txt
        """
    )
    
    parser.add_argument('log_file', type=Path, help='Simulation log file to parse')
    parser.add_argument('--report', '-r', type=Path, help='Output file for text report')
    parser.add_argument('--visualize', '-v', action='store_true', 
                       help='Generate visualization charts')
    parser.add_argument('--output-dir', '-o', type=Path, default=Path('reports'),
                       help='Output directory for visualizations (default: reports/)')
    
    args = parser.parse_args()
    
    # Check if log file exists
    if not args.log_file.exists():
        print(f"Error: Log file not found: {args.log_file}")
        sys.exit(1)
    
    # Parse log file
    parser_obj = PerformanceParser()
    parser_obj.parse_log_file(args.log_file)
    
    # Get metrics
    metrics = parser_obj.get_metrics()
    
    # Generate text report
    generate_text_report(metrics, args.report)
    
    # Generate visualizations if requested
    if args.visualize:
        if not HAS_MATPLOTLIB:
            print("\nWarning: matplotlib not available. Install with: pip install matplotlib")
        else:
            print("\nGenerating visualizations...")
            generate_visualizations(metrics, args.output_dir)
            print(f"\nVisualizations saved to: {args.output_dir}")


if __name__ == '__main__':
    main()

