#!/usr/bin/env python3
"""
RISC-V Compliance Test Runner

Automates running RISC-V compliance tests on the processor:
- Iterates through test hex files
- Runs simulation for each test
- Captures results from register x1 (pass/fail indicator)
- Compares against expected values
- Generates summary report

Usage:
    python3 scripts/run_compliance_tests.py [options]

Options:
    --test-dir <dir>      Directory containing test hex files (default: tests/compliance/hex)
    --simulator <sim>     Simulator to use: iverilog, modelsim (default: iverilog)
    --timeout <sec>       Timeout per test in seconds (default: 60)
    --verbose             Enable verbose output
    --debug               Enable debug output
    --report <file>       Generate HTML report (default: compliance_report.html)
    --json <file>         Generate JSON report
    --waveform            Generate waveform files for failed tests
"""

import os
import sys
import subprocess
import re
import json
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import argparse

# Colors for terminal output
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

class ComplianceTestRunner:
    """RISC-V Compliance Test Runner"""
    
    def __init__(self, test_dir: str, simulator: str = "iverilog", 
                 timeout: int = 60, verbose: bool = False, debug: bool = False,
                 waveform: bool = False):
        self.test_dir = Path(test_dir)
        self.simulator = simulator
        self.timeout = timeout
        self.verbose = verbose
        self.debug = debug
        self.waveform = waveform
        
        # Project directories
        self.script_dir = Path(__file__).parent
        self.project_root = self.script_dir.parent
        self.mem_dir = self.project_root / "mem"
        self.src_dir = self.project_root / "src"
        self.tb_dir = self.project_root / "tb"
        self.log_dir = self.project_root / "tests" / "compliance" / "logs"
        
        # Results storage
        self.results: List[Dict] = []
        
        # Ensure log directory exists
        self.log_dir.mkdir(parents=True, exist_ok=True)
    
    def find_test_files(self) -> List[Path]:
        """Find all test hex files in test directory"""
        if not self.test_dir.exists():
            print(f"{Colors.RED}Error: Test directory not found: {self.test_dir}{Colors.RESET}")
            return []
        
        test_files = sorted(self.test_dir.glob("*.hex"))
        if self.verbose:
            print(f"{Colors.CYAN}Found {len(test_files)} test files{Colors.RESET}")
        
        return test_files
    
    def run_simulation_iverilog(self, test_name: str, hex_file: Path) -> Tuple[bool, str, Optional[int]]:
        """
        Run simulation using Icarus Verilog
        
        Returns:
            (success, log_output, x1_value)
        """
        # Copy hex file to instruction memory
        inst_mem_file = self.mem_dir / "inst_mem.hex"
        try:
            import shutil
            shutil.copy(hex_file, inst_mem_file)
        except Exception as e:
            return False, f"Failed to copy hex file: {e}", None
        
        # Compile
        sim_executable = f"{test_name}_sim"
        compile_cmd = [
            "iverilog",
            "-o", sim_executable,
            "-I", str(self.src_dir),
            str(self.src_dir / "*.sv"),
            str(self.tb_dir / "riscv_pipeline_tb.sv")
        ]
        
        if self.debug:
            compile_cmd.append("-DDEBUG")
        
        log_file = self.log_dir / f"{test_name}_compile.log"
        
        try:
            with open(log_file, 'w') as f:
                result = subprocess.run(
                    compile_cmd,
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    timeout=self.timeout,
                    cwd=str(self.project_root)
                )
            
            if result.returncode != 0:
                with open(log_file, 'r') as f:
                    compile_log = f.read()
                return False, f"Compilation failed:\n{compile_log}", None
        except subprocess.TimeoutExpired:
            return False, f"Compilation timeout ({self.timeout}s)", None
        except Exception as e:
            return False, f"Compilation error: {e}", None
        
        # Run simulation
        run_cmd = ["vvp", sim_executable]
        
        if self.waveform:
            vcd_file = self.project_root / f"{test_name}.vcd"
            run_cmd.extend(["+dumpfile", str(vcd_file)])
        
        log_file = self.log_dir / f"{test_name}_run.log"
        
        try:
            with open(log_file, 'w') as f:
                result = subprocess.run(
                    run_cmd,
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    timeout=self.timeout,
                    cwd=str(self.project_root)
                )
            
            # Read log file
            with open(log_file, 'r') as f:
                log_output = f.read()
            
            # Cleanup
            sim_path = self.project_root / sim_executable
            if sim_path.exists():
                sim_path.unlink()
            
            # Parse results
            x1_value = self.parse_results(log_output)
            
            return True, log_output, x1_value
            
        except subprocess.TimeoutExpired:
            return False, f"Simulation timeout ({self.timeout}s)", None
        except Exception as e:
            return False, f"Simulation error: {e}", None
    
    def parse_results(self, log_output: str) -> Optional[int]:
        """
        Parse simulation log to extract register x1 value
        
        RISC-V compliance tests set x1 = 1 for pass, x1 = 0 for fail
        """
        # Look for register x1 value in various formats
        patterns = [
            r'x1\s*[=:]\s*(0x[0-9a-fA-F]+|\d+)',
            r'register\s+1\s*[=:]\s*(0x[0-9a-fA-F]+|\d+)',
            r'x1\s*=\s*(0x[0-9a-fA-F]+|\d+)',
            r'Register\s+x1:\s*(0x[0-9a-fA-F]+|\d+)',
            r'x1:\s*(0x[0-9a-fA-F]+|\d+)',
        ]
        
        # Also look for pass/fail indicators
        if re.search(r'PASS|PASSED|test.*pass', log_output, re.IGNORECASE):
            return 1
        if re.search(r'FAIL|FAILED|test.*fail', log_output, re.IGNORECASE):
            return 0
        
        # Try to find x1 value
        for pattern in patterns:
            match = re.search(pattern, log_output, re.IGNORECASE)
            if match:
                value_str = match.group(1)
                try:
                    if value_str.startswith('0x'):
                        value = int(value_str, 16)
                    else:
                        value = int(value_str)
                    return value
                except ValueError:
                    continue
        
        # Look for final register dump
        # Try to find register values at end of simulation
        lines = log_output.split('\n')
        for line in reversed(lines):
            # Look for register dump format: "x1: 0x00000001"
            match = re.search(r'x1[:\s]+(0x[0-9a-fA-F]+)', line, re.IGNORECASE)
            if match:
                try:
                    return int(match.group(1), 16)
                except ValueError:
                    continue
        
        return None
    
    def run_test(self, test_file: Path) -> Dict:
        """Run a single compliance test"""
        test_name = test_file.stem
        
        if self.verbose:
            print(f"{Colors.CYAN}Running test: {test_name}{Colors.RESET}")
        
        start_time = time.time()
        
        # Run simulation
        if self.simulator == "iverilog":
            success, log_output, x1_value = self.run_simulation_iverilog(test_name, test_file)
        else:
            return {
                "test": test_name,
                "status": "ERROR",
                "message": f"Unsupported simulator: {self.simulator}",
                "x1_value": None,
                "expected": 1,
                "duration": 0
            }
        
        duration = time.time() - start_time
        
        # Determine test result
        expected_x1 = 1  # Compliance tests set x1=1 for pass
        
        if not success:
            status = "ERROR"
            passed = False
        elif x1_value is None:
            status = "UNKNOWN"
            passed = False
        elif x1_value == expected_x1:
            status = "PASSED"
            passed = True
        else:
            status = "FAILED"
            passed = False
        
        result = {
            "test": test_name,
            "status": status,
            "passed": passed,
            "x1_value": x1_value,
            "expected": expected_x1,
            "duration": duration,
            "log_file": str(self.log_dir / f"{test_name}_run.log"),
            "message": "" if success else log_output[:200]  # First 200 chars of error
        }
        
        # Print result
        if passed:
            print(f"  {Colors.GREEN}✓ PASSED{Colors.RESET} (x1=0x{x1_value:08x}, {duration:.2f}s)")
        elif status == "ERROR":
            print(f"  {Colors.RED}✗ ERROR{Colors.RESET} ({duration:.2f}s)")
            if self.verbose:
                print(f"    {log_output[:100]}")
        elif status == "UNKNOWN":
            print(f"  {Colors.YELLOW}? UNKNOWN{Colors.RESET} (x1 not found, {duration:.2f}s)")
        else:
            print(f"  {Colors.RED}✗ FAILED{Colors.RESET} (x1=0x{x1_value:08x}, expected 0x{expected_x1:08x}, {duration:.2f}s)")
        
        return result
    
    def run_all_tests(self) -> Dict:
        """Run all compliance tests"""
        test_files = self.find_test_files()
        
        if not test_files:
            print(f"{Colors.RED}No test files found in {self.test_dir}{Colors.RESET}")
            return {}
        
        print(f"{Colors.BOLD}Running {len(test_files)} compliance tests...{Colors.RESET}\n")
        
        start_time = time.time()
        
        for test_file in test_files:
            result = self.run_test(test_file)
            self.results.append(result)
        
        total_duration = time.time() - start_time
        
        # Calculate statistics
        total = len(self.results)
        passed = sum(1 for r in self.results if r["passed"])
        failed = sum(1 for r in self.results if r["status"] == "FAILED")
        errors = sum(1 for r in self.results if r["status"] == "ERROR")
        unknown = sum(1 for r in self.results if r["status"] == "UNKNOWN")
        
        pass_rate = (passed / total * 100) if total > 0 else 0
        
        summary = {
            "total": total,
            "passed": passed,
            "failed": failed,
            "errors": errors,
            "unknown": unknown,
            "pass_rate": pass_rate,
            "duration": total_duration,
            "timestamp": datetime.now().isoformat()
        }
        
        return summary
    
    def generate_text_report(self, summary: Dict) -> str:
        """Generate text report"""
        report = []
        report.append("=" * 60)
        report.append("RISC-V Compliance Test Report")
        report.append("=" * 60)
        report.append(f"Date: {summary['timestamp']}")
        report.append(f"Processor: RISC-V 5-Stage Pipeline")
        report.append(f"ISA: RV32I")
        report.append("")
        report.append("Summary:")
        report.append("-" * 60)
        report.append(f"Total Tests:  {summary['total']}")
        report.append(f"{Colors.GREEN}Passed:       {summary['passed']}{Colors.RESET}")
        report.append(f"{Colors.RED}Failed:       {summary['failed']}{Colors.RESET}")
        if summary['errors'] > 0:
            report.append(f"{Colors.RED}Errors:       {summary['errors']}{Colors.RESET}")
        if summary['unknown'] > 0:
            report.append(f"{Colors.YELLOW}Unknown:      {summary['unknown']}{Colors.RESET}")
        report.append(f"Pass Rate:    {summary['pass_rate']:.2f}%")
        report.append(f"Duration:     {summary['duration']:.2f}s")
        report.append("")
        
        # Failed tests details
        failed_tests = [r for r in self.results if r["status"] == "FAILED"]
        if failed_tests:
            report.append("Failed Tests:")
            report.append("-" * 60)
            for result in failed_tests:
                report.append(f"  {result['test']}:")
                report.append(f"    x1 = 0x{result['x1_value']:08x} (expected 0x{result['expected']:08x})")
                report.append(f"    Duration: {result['duration']:.2f}s")
                report.append("")
        
        # Error tests details
        error_tests = [r for r in self.results if r["status"] == "ERROR"]
        if error_tests:
            report.append("Error Tests:")
            report.append("-" * 60)
            for result in error_tests:
                report.append(f"  {result['test']}:")
                report.append(f"    {result['message'][:100]}")
                report.append("")
        
        return "\n".join(report)
    
    def generate_html_report(self, summary: Dict, output_file: str):
        """Generate HTML report"""
        html = []
        html.append("<!DOCTYPE html>")
        html.append("<html>")
        html.append("<head>")
        html.append("  <title>RISC-V Compliance Test Report</title>")
        html.append("  <style>")
        html.append("    body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }")
        html.append("    .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }")
        html.append("    h1 { color: #333; border-bottom: 3px solid #4CAF50; padding-bottom: 10px; }")
        html.append("    .summary { background: #f9f9f9; padding: 15px; border-radius: 5px; margin: 20px 0; }")
        html.append("    .summary-item { margin: 5px 0; }")
        html.append("    .passed { color: #4CAF50; font-weight: bold; }")
        html.append("    .failed { color: #f44336; font-weight: bold; }")
        html.append("    .error { color: #ff9800; font-weight: bold; }")
        html.append("    .unknown { color: #9e9e9e; font-weight: bold; }")
        html.append("    table { border-collapse: collapse; width: 100%; margin-top: 20px; }")
        html.append("    th, td { border: 1px solid #ddd; padding: 10px; text-align: left; }")
        html.append("    th { background-color: #4CAF50; color: white; }")
        html.append("    tr:nth-child(even) { background-color: #f9f9f9; }")
        html.append("    .status-passed { color: #4CAF50; font-weight: bold; }")
        html.append("    .status-failed { color: #f44336; font-weight: bold; }")
        html.append("    .status-error { color: #ff9800; font-weight: bold; }")
        html.append("    .status-unknown { color: #9e9e9e; font-weight: bold; }")
        html.append("  </style>")
        html.append("</head>")
        html.append("<body>")
        html.append("  <div class='container'>")
        html.append("    <h1>RISC-V Compliance Test Report</h1>")
        html.append("    <div class='summary'>")
        html.append(f"      <div class='summary-item'><strong>Date:</strong> {summary['timestamp']}</div>")
        html.append(f"      <div class='summary-item'><strong>Processor:</strong> RISC-V 5-Stage Pipeline</div>")
        html.append(f"      <div class='summary-item'><strong>ISA:</strong> RV32I</div>")
        html.append(f"      <div class='summary-item'><strong>Total Tests:</strong> {summary['total']}</div>")
        html.append(f"      <div class='summary-item'><span class='passed'>Passed:</span> {summary['passed']}</div>")
        html.append(f"      <div class='summary-item'><span class='failed'>Failed:</span> {summary['failed']}</div>")
        if summary['errors'] > 0:
            html.append(f"      <div class='summary-item'><span class='error'>Errors:</span> {summary['errors']}</div>")
        if summary['unknown'] > 0:
            html.append(f"      <div class='summary-item'><span class='unknown'>Unknown:</span> {summary['unknown']}</div>")
        html.append(f"      <div class='summary-item'><strong>Pass Rate:</strong> {summary['pass_rate']:.2f}%</div>")
        html.append(f"      <div class='summary-item'><strong>Duration:</strong> {summary['duration']:.2f}s</div>")
        html.append("    </div>")
        html.append("    <h2>Test Results</h2>")
        html.append("    <table>")
        html.append("      <tr>")
        html.append("        <th>Test Name</th>")
        html.append("        <th>Status</th>")
        html.append("        <th>x1 Value</th>")
        html.append("        <th>Expected</th>")
        html.append("        <th>Duration (s)</th>")
        html.append("      </tr>")
        
        for result in self.results:
            status_class = f"status-{result['status'].lower()}"
            x1_display = f"0x{result['x1_value']:08x}" if result['x1_value'] is not None else "N/A"
            expected_display = f"0x{result['expected']:08x}"
            
            html.append("      <tr>")
            html.append(f"        <td>{result['test']}</td>")
            html.append(f"        <td class='{status_class}'>{result['status']}</td>")
            html.append(f"        <td>{x1_display}</td>")
            html.append(f"        <td>{expected_display}</td>")
            html.append(f"        <td>{result['duration']:.2f}</td>")
            html.append("      </tr>")
        
        html.append("    </table>")
        
        # Failed tests details
        failed_tests = [r for r in self.results if r["status"] == "FAILED"]
        if failed_tests:
            html.append("    <h2>Failed Tests Details</h2>")
            html.append("    <table>")
            html.append("      <tr>")
            html.append("        <th>Test Name</th>")
            html.append("        <th>x1 Value</th>")
            html.append("        <th>Expected</th>")
            html.append("        <th>Log File</th>")
            html.append("      </tr>")
            
            for result in failed_tests:
                html.append("      <tr>")
                html.append(f"        <td>{result['test']}</td>")
                html.append(f"        <td>0x{result['x1_value']:08x}</td>")
                html.append(f"        <td>0x{result['expected']:08x}</td>")
                html.append(f"        <td><a href='{result['log_file']}'>View Log</a></td>")
                html.append("      </tr>")
            
            html.append("    </table>")
        
        html.append("  </div>")
        html.append("</body>")
        html.append("</html>")
        
        with open(output_file, 'w') as f:
            f.write('\n'.join(html))
    
    def generate_json_report(self, summary: Dict, output_file: str):
        """Generate JSON report"""
        report = {
            "summary": summary,
            "results": self.results
        }
        
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)

def main():
    parser = argparse.ArgumentParser(
        description="Run RISC-V compliance tests on processor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all tests
  python3 scripts/run_compliance_tests.py

  # Run with verbose output
  python3 scripts/run_compliance_tests.py --verbose

  # Generate HTML report
  python3 scripts/run_compliance_tests.py --report report.html

  # Generate JSON report
  python3 scripts/run_compliance_tests.py --json report.json
        """
    )
    
    parser.add_argument(
        "--test-dir",
        default="tests/compliance/hex",
        help="Directory containing test hex files (default: tests/compliance/hex)"
    )
    parser.add_argument(
        "--simulator",
        choices=["iverilog", "modelsim"],
        default="iverilog",
        help="Simulator to use (default: iverilog)"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="Timeout per test in seconds (default: 60)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output"
    )
    parser.add_argument(
        "--report",
        help="Generate HTML report (default: compliance_report.html)"
    )
    parser.add_argument(
        "--json",
        help="Generate JSON report"
    )
    parser.add_argument(
        "--waveform",
        action="store_true",
        help="Generate waveform files for failed tests"
    )
    
    args = parser.parse_args()
    
    # Create test runner
    runner = ComplianceTestRunner(
        test_dir=args.test_dir,
        simulator=args.simulator,
        timeout=args.timeout,
        verbose=args.verbose,
        debug=args.debug,
        waveform=args.waveform
    )
    
    # Run all tests
    summary = runner.run_all_tests()
    
    if not summary:
        sys.exit(1)
    
    # Print summary
    print("\n" + runner.generate_text_report(summary))
    
    # Generate reports
    if args.report:
        report_file = args.report
    elif args.report is None and not args.json:
        # Default HTML report
        report_file = "compliance_report.html"
    else:
        report_file = None
    
    if report_file:
        runner.generate_html_report(summary, report_file)
        print(f"\n{Colors.GREEN}HTML report generated: {report_file}{Colors.RESET}")
    
    if args.json:
        runner.generate_json_report(summary, args.json)
        print(f"{Colors.GREEN}JSON report generated: {args.json}{Colors.RESET}")
    
    # Exit with error code if tests failed
    if summary['failed'] > 0 or summary['errors'] > 0:
        sys.exit(1)
    elif summary['unknown'] > 0:
        sys.exit(2)
    else:
        sys.exit(0)

if __name__ == "__main__":
    main()

