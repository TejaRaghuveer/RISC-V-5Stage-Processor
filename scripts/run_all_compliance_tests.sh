#!/bin/bash
################################################################################
# Run All RISC-V Compliance Tests
#
# This script runs all available compliance tests and generates a summary report.
#
# Usage:
#   ./scripts/run_all_compliance_tests.sh [options]
#
# Options:
#   --category <cat>  Run only tests in category (isa, compliance, etc.)
#   --report <file>   Generate HTML report
#   --verbose         Enable verbose output
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TESTS_DIR="$PROJECT_ROOT/tests/compliance"
HEX_DIR="$TESTS_DIR/hex"
REPORT_DIR="$TESTS_DIR/reports"

# Parse arguments
CATEGORY=""
REPORT_FILE=""
VERBOSE=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --category)
            CATEGORY="$2"
            shift 2
            ;;
        --report)
            REPORT_FILE="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE=1
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Create report directory
mkdir -p "$REPORT_DIR"

echo "=========================================="
echo "RISC-V Compliance Test Suite"
echo "=========================================="
echo "Project: RISC-V 5-Stage Pipeline Processor"
echo "Date: $(date)"
echo ""

# Find all test files
if [ -n "$CATEGORY" ]; then
    TEST_FILES=$(ls "$HEX_DIR"/${CATEGORY}_*.hex 2>/dev/null || echo "")
else
    TEST_FILES=$(ls "$HEX_DIR"/*.hex 2>/dev/null || echo "")
fi

if [ -z "$TEST_FILES" ]; then
    echo -e "${RED}Error: No test files found in $HEX_DIR${NC}"
    echo "Please run: ./scripts/build_compliance_tests.sh"
    exit 1
fi

# Count tests
TOTAL_TESTS=$(echo "$TEST_FILES" | wc -l)
PASSED_TESTS=0
FAILED_TESTS=0
UNKNOWN_TESTS=0

PASSED_LIST=()
FAILED_LIST=()
UNKNOWN_LIST=()

echo "Found $TOTAL_TESTS tests"
echo ""

# Run each test
for test_file in $TEST_FILES; do
    test_name=$(basename "$test_file" .hex)
    
    echo -n "Running $test_name... "
    
    # Run test
    if [ $VERBOSE -eq 1 ]; then
        echo ""
    fi
    
    if ./scripts/run_compliance_test.sh "$test_name" > /tmp/test_output_$$.txt 2>&1; then
        echo -e "${GREEN}PASSED${NC}"
        ((PASSED_TESTS++))
        PASSED_LIST+=("$test_name")
    else
        EXIT_CODE=$?
        if [ $EXIT_CODE -eq 1 ]; then
            echo -e "${RED}FAILED${NC}"
            ((FAILED_TESTS++))
            FAILED_LIST+=("$test_name")
        else
            echo -e "${YELLOW}UNKNOWN${NC}"
            ((UNKNOWN_TESTS++))
            UNKNOWN_LIST+=("$test_name")
        fi
    fi
done

# Calculate pass rate
if [ $TOTAL_TESTS -gt 0 ]; then
    PASS_RATE=$(echo "scale=2; $PASSED_TESTS * 100 / $TOTAL_TESTS" | bc)
else
    PASS_RATE=0
fi

# Generate summary
echo ""
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo "Total Tests: $TOTAL_TESTS"
echo -e "${GREEN}Passed: $PASSED_TESTS${NC}"
echo -e "${RED}Failed: $FAILED_TESTS${NC}"
if [ $UNKNOWN_TESTS -gt 0 ]; then
    echo -e "${YELLOW}Unknown: $UNKNOWN_TESTS${NC}"
fi
echo "Pass Rate: ${PASS_RATE}%"
echo ""

# Display failed tests
if [ ${#FAILED_LIST[@]} -gt 0 ]; then
    echo "Failed Tests:"
    for test in "${FAILED_LIST[@]}"; do
        echo -e "  ${RED}✗${NC} $test"
    done
    echo ""
fi

# Generate report file
if [ -n "$REPORT_FILE" ]; then
    REPORT_PATH="$REPORT_DIR/$REPORT_FILE"
    cat > "$REPORT_PATH" <<EOF
<!DOCTYPE html>
<html>
<head>
    <title>RISC-V Compliance Test Report</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        h1 { color: #333; }
        .summary { background: #f5f5f5; padding: 15px; border-radius: 5px; }
        .passed { color: green; }
        .failed { color: red; }
        .unknown { color: orange; }
        table { border-collapse: collapse; width: 100%; margin-top: 20px; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
        th { background-color: #4CAF50; color: white; }
    </style>
</head>
<body>
    <h1>RISC-V Compliance Test Report</h1>
    <div class="summary">
        <p><strong>Date:</strong> $(date)</p>
        <p><strong>Processor:</strong> RISC-V 5-Stage Pipeline</p>
        <p><strong>ISA:</strong> RV32I</p>
        <p><strong>Total Tests:</strong> $TOTAL_TESTS</p>
        <p class="passed"><strong>Passed:</strong> $PASSED_TESTS</p>
        <p class="failed"><strong>Failed:</strong> $FAILED_TESTS</p>
        <p><strong>Pass Rate:</strong> ${PASS_RATE}%</p>
    </div>
    
    <h2>Test Results</h2>
    <table>
        <tr>
            <th>Test Name</th>
            <th>Status</th>
        </tr>
EOF

    # Add passed tests
    for test in "${PASSED_LIST[@]}"; do
        echo "        <tr><td>$test</td><td class=\"passed\">PASSED</td></tr>" >> "$REPORT_PATH"
    done
    
    # Add failed tests
    for test in "${FAILED_LIST[@]}"; do
        echo "        <tr><td>$test</td><td class=\"failed\">FAILED</td></tr>" >> "$REPORT_PATH"
    done
    
    # Add unknown tests
    for test in "${UNKNOWN_LIST[@]}"; do
        echo "        <tr><td>$test</td><td class=\"unknown\">UNKNOWN</td></tr>" >> "$REPORT_PATH"
    done
    
    cat >> "$REPORT_PATH" <<EOF
    </table>
</body>
</html>
EOF

    echo "Report generated: $REPORT_PATH"
fi

# Exit with error if any tests failed
if [ $FAILED_TESTS -gt 0 ]; then
    exit 1
else
    exit 0
fi

