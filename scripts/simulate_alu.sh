#!/bin/bash

################################################################################
# RISC-V ALU Simulation Script
# 
# This script compiles and simulates the ALU module and testbench using either
# ModelSim/QuestaSim or Icarus Verilog. It generates VCD waveform files and
# optionally opens them in a waveform viewer.
#
# Usage:
#   ./simulate_alu.sh [simulator] [options]
#
# Simulators:
#   modelsim  - Use ModelSim/QuestaSim (default if available)
#   iverilog  - Use Icarus Verilog (default if ModelSim not available)
#
# Options:
#   --no-waveform  - Skip opening waveform viewer
#   --clean        - Clean previous simulation files before running
#   --help         - Display this help message
#
# Examples:
#   ./simulate_alu.sh                    # Auto-detect simulator
#   ./simulate_alu.sh modelsim            # Force ModelSim
#   ./simulate_alu.sh iverilog --no-waveform  # Use Icarus, no viewer
################################################################################

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$PROJECT_ROOT/src"
TB_DIR="$PROJECT_ROOT/tb"
WORK_DIR="$PROJECT_ROOT/work"
SIM_DIR="$PROJECT_ROOT/simulation"

# Default settings
SIMULATOR=""
OPEN_WAVEFORM=true
CLEAN_FIRST=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        modelsim|vsim|questa)
            SIMULATOR="modelsim"
            shift
            ;;
        iverilog|iverilog-vvp)
            SIMULATOR="iverilog"
            shift
            ;;
        --no-waveform)
            OPEN_WAVEFORM=false
            shift
            ;;
        --clean)
            CLEAN_FIRST=true
            shift
            ;;
        --help|-h)
            head -n 30 "$0" | grep -E "^#"
            exit 0
            ;;
        *)
            echo -e "${RED}Error: Unknown option '$1'${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

################################################################################
# Function: Print colored messages
################################################################################
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

################################################################################
# Function: Check if command exists
################################################################################
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

################################################################################
# Function: Detect available simulator
################################################################################
detect_simulator() {
    if [[ -n "$SIMULATOR" ]]; then
        # Simulator explicitly specified
        if [[ "$SIMULATOR" == "modelsim" ]]; then
            if command_exists vlog && command_exists vsim; then
                print_info "Using ModelSim/QuestaSim (explicitly requested)"
                return 0
            else
                print_error "ModelSim/QuestaSim not found!"
                exit 1
            fi
        elif [[ "$SIMULATOR" == "iverilog" ]]; then
            if command_exists iverilog && command_exists vvp; then
                print_info "Using Icarus Verilog (explicitly requested)"
                return 0
            else
                print_error "Icarus Verilog not found!"
                exit 1
            fi
        fi
    fi
    
    # Auto-detect simulator
    if command_exists vlog && command_exists vsim; then
        SIMULATOR="modelsim"
        print_info "Auto-detected: ModelSim/QuestaSim"
        return 0
    elif command_exists iverilog && command_exists vvp; then
        SIMULATOR="iverilog"
        print_info "Auto-detected: Icarus Verilog"
        return 0
    else
        print_error "No suitable simulator found!"
        print_info "Please install ModelSim/QuestaSim or Icarus Verilog"
        print_info "Icarus Verilog: sudo apt-get install iverilog (Linux)"
        print_info "                brew install icarus-verilog (macOS)"
        exit 1
    fi
}

################################################################################
# Function: Clean previous simulation files
################################################################################
clean_simulation() {
    print_info "Cleaning previous simulation files..."
    
    if [[ -d "$WORK_DIR" ]]; then
        rm -rf "$WORK_DIR"
        print_info "Removed work directory"
    fi
    
    if [[ -d "$SIM_DIR" ]]; then
        rm -rf "$SIM_DIR"
        print_info "Removed simulation directory"
    fi
    
    # Clean ModelSim files
    rm -f transcript
    rm -f vsim.wlf
    rm -f modelsim.ini
    
    # Clean Icarus Verilog files
    rm -f alu_tb
    rm -f alu_tb.vcd
    
    print_success "Cleanup complete"
}

################################################################################
# Function: Compile and simulate with ModelSim
################################################################################
simulate_modelsim() {
    print_info "Starting ModelSim/QuestaSim simulation..."
    
    # Create work directory
    mkdir -p "$WORK_DIR"
    mkdir -p "$SIM_DIR"
    cd "$WORK_DIR" || exit 1
    
    # Create work library
    print_info "Creating work library..."
    vlib work
    vmap work work
    
    # Compile source files
    print_info "Compiling ALU module..."
    if ! vlog -work work "$SRC_DIR/alu.sv"; then
        print_error "Failed to compile ALU module"
        exit 1
    fi
    
    print_info "Compiling ALU testbench..."
    if ! vlog -work work "$TB_DIR/alu_tb.sv"; then
        print_error "Failed to compile ALU testbench"
        exit 1
    fi
    
    # Run simulation
    print_info "Running simulation..."
    print_info "========================================"
    
    # Run simulation and capture output
    vsim -voptargs=+acc -do "run -all; quit -f" work.alu_tb > "$SIM_DIR/simulation.log" 2>&1
    
    # Check simulation result
    if [[ $? -eq 0 ]]; then
        print_success "Simulation completed successfully"
    else
        print_error "Simulation failed. Check $SIM_DIR/simulation.log for details"
        exit 1
    fi
    
    # Display simulation log
    print_info "Simulation output:"
    echo "----------------------------------------"
    tail -n 50 "$SIM_DIR/simulation.log"
    echo "----------------------------------------"
    
    # Generate VCD file if needed
    if [[ "$OPEN_WAVEFORM" == true ]]; then
        print_info "Generating waveform file..."
        vsim -voptargs=+acc -do "add wave -radix hex /alu_tb/dut/*; add wave -radix hex /alu_tb/*; run -all; dataset save vsim.wlf $SIM_DIR/alu_tb.wlf; quit -f" work.alu_tb
        
        # Open waveform viewer
        if command_exists vsim; then
            print_info "Opening ModelSim waveform viewer..."
            vsim -view "$SIM_DIR/alu_tb.wlf" -do "add wave -radix hex /alu_tb/dut/*; add wave -radix hex /alu_tb/*" &
        fi
    fi
    
    cd "$PROJECT_ROOT" || exit 1
}

################################################################################
# Function: Compile and simulate with Icarus Verilog
################################################################################
simulate_iverilog() {
    print_info "Starting Icarus Verilog simulation..."
    
    # Create simulation directory
    mkdir -p "$SIM_DIR"
    cd "$SIM_DIR" || exit 1
    
    # Compile source files
    print_info "Compiling ALU module and testbench..."
    
    # Check if files exist
    if [[ ! -f "$SRC_DIR/alu.sv" ]]; then
        print_error "ALU source file not found: $SRC_DIR/alu.sv"
        exit 1
    fi
    
    if [[ ! -f "$TB_DIR/alu_tb.sv" ]]; then
        print_error "ALU testbench file not found: $TB_DIR/alu_tb.sv"
        exit 1
    fi
    
    # Compile with Icarus Verilog
    # -g2012: Use SystemVerilog-2012 standard
    # -o: Output executable name
    # -I: Include directories
    if ! iverilog -g2012 -o alu_tb -I"$SRC_DIR" "$SRC_DIR/alu.sv" "$TB_DIR/alu_tb.sv"; then
        print_error "Compilation failed"
        exit 1
    fi
    
    print_success "Compilation successful"
    
    # Run simulation with VCD generation
    print_info "Running simulation and generating VCD waveform..."
    print_info "========================================"
    
    # Run simulation and capture output
    vvp alu_tb > simulation.log 2>&1
    
    # Check simulation result
    if [[ $? -eq 0 ]]; then
        print_success "Simulation completed successfully"
    else
        print_error "Simulation failed. Check simulation.log for details"
        cat simulation.log
        exit 1
    fi
    
    # Display simulation output
    print_info "Simulation output:"
    echo "----------------------------------------"
    cat simulation.log
    echo "----------------------------------------"
    
    # Check if VCD file was generated
    if [[ -f "alu_tb.vcd" ]]; then
        print_success "VCD waveform file generated: $SIM_DIR/alu_tb.vcd"
        
        # Open waveform viewer if requested
        if [[ "$OPEN_WAVEFORM" == true ]]; then
            if command_exists gtkwave; then
                print_info "Opening GTKWave..."
                gtkwave alu_tb.vcd &
            elif command_exists open; then
                # macOS - try to open with default application
                print_info "Opening waveform file..."
                open alu_tb.vcd
            elif command_exists xdg-open; then
                # Linux - try to open with default application
                print_info "Opening waveform file..."
                xdg-open alu_tb.vcd &
            else
                print_warning "No waveform viewer found. Install GTKWave to view waveforms:"
                print_info "  Linux: sudo apt-get install gtkwave"
                print_info "  macOS: brew install gtkwave"
                print_info "VCD file available at: $SIM_DIR/alu_tb.vcd"
            fi
        fi
    else
        print_warning "VCD file not generated. Check testbench for VCD dump commands."
    fi
    
    cd "$PROJECT_ROOT" || exit 1
}

################################################################################
# Main execution
################################################################################
main() {
    echo ""
    echo "========================================"
    echo "RISC-V ALU Simulation Script"
    echo "========================================"
    echo ""
    
    # Clean if requested
    if [[ "$CLEAN_FIRST" == true ]]; then
        clean_simulation
        echo ""
    fi
    
    # Detect simulator
    detect_simulator
    
    # Check source files exist
    if [[ ! -f "$SRC_DIR/alu.sv" ]]; then
        print_error "ALU source file not found: $SRC_DIR/alu.sv"
        exit 1
    fi
    
    if [[ ! -f "$TB_DIR/alu_tb.sv" ]]; then
        print_error "ALU testbench file not found: $TB_DIR/alu_tb.sv"
        exit 1
    fi
    
    print_info "Source directory: $SRC_DIR"
    print_info "Testbench directory: $TB_DIR"
    echo ""
    
    # Run simulation based on detected simulator
    if [[ "$SIMULATOR" == "modelsim" ]]; then
        simulate_modelsim
    elif [[ "$SIMULATOR" == "iverilog" ]]; then
        simulate_iverilog
    else
        print_error "Unknown simulator: $SIMULATOR"
        exit 1
    fi
    
    echo ""
    print_success "Simulation script completed!"
    echo ""
    print_info "Simulation files location: $SIM_DIR"
    if [[ "$SIMULATOR" == "iverilog" ]]; then
        print_info "VCD waveform file: $SIM_DIR/alu_tb.vcd"
    fi
    echo ""
}

# Run main function
main

