#!/bin/bash
# ============================================================
# verify_all.sh
#
# Runs ALL test cases against the test_pid binary and
# compares output to expected values.
# Reports PASS/FAIL for each test with a summary at the end.
#
# Usage (from project root):
#   ./scripts/tests/verify_all.sh
#
# Expects:
#   - ./main/test_pid    (compiled binary)
#   - ./tests/           (folder with test .txt files)
#   - ./all_expected/    (folder with *_expected.txt files)
# ============================================================

BINARY="./main/test_pid"
TEST_DIR="./all_tests"
EXPECTED_DIR="./all_expected"
TOLERANCE="0.005"  # degrees — allow ±0.005° for floating-point rounding

PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0
TOTAL=0

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ---- Preflight checks ----
if [ ! -f "$BINARY" ]; then
    echo -e "${RED}ERROR: $BINARY not found.${NC}"
    echo "  Build first:  gcc main/rocket_pid.c main/test_pid.c -o main/test_pid -lm"
    exit 1
fi
if [ ! -d "$TEST_DIR" ]; then
    echo -e "${RED}ERROR: $TEST_DIR directory not found.${NC}"
    exit 1
fi
if [ ! -d "$EXPECTED_DIR" ]; then
    echo -e "${RED}ERROR: $EXPECTED_DIR directory not found.${NC}"
    exit 1
fi

echo ""
echo -e "${BOLD}============================================================${NC}"
echo -e "${BOLD}  Rocket PID Controller — Full Test Suite${NC}"
echo -e "${BOLD}  Tolerance: ±${TOLERANCE}°${NC}"
echo -e "${BOLD}============================================================${NC}"
echo ""

# ---- Group labels for pretty output ----
print_group_header() {
    local num=$1
    case $num in
        1)  echo -e "${CYAN}--- Tests 1-10: Original baseline tests ---${NC}" ;;
        11) echo -e "${CYAN}--- Tests 11-16: Single-axis errors ---${NC}" ;;
        17) echo -e "${CYAN}--- Tests 17-19: Asymmetric gains ---${NC}" ;;
        20) echo -e "${CYAN}--- Tests 20-23: Derivative isolation ---${NC}" ;;
        24) echo -e "${CYAN}--- Tests 24-27: Filter tau sweep ---${NC}" ;;
        28) echo -e "${CYAN}--- Tests 28-30: Integral Ki sweep ---${NC}" ;;
        31) echo -e "${CYAN}--- Tests 31-33: Combined PID ---${NC}" ;;
        34) echo -e "${CYAN}--- Tests 34-36: Reference lookup ---${NC}" ;;
        37) echo -e "${CYAN}--- Tests 37-39: Non-identity state ---${NC}" ;;
        40) echo -e "${CYAN}--- Tests 40-42: Variable timestep ---${NC}" ;;
        43) echo -e "${CYAN}--- Tests 43-45: Filter noise stress ---${NC}" ;;
        46) echo -e "${CYAN}--- Tests 46-47: Large gain stress ---${NC}" ;;
        48) echo -e "${CYAN}--- Test 48: Negative rotation ---${NC}" ;;
        49) echo -e "${CYAN}--- Test 49: Convergence tracking ---${NC}" ;;
        50) echo -e "${CYAN}--- Test 50: Long sequence ---${NC}" ;;
    esac
}

# ---- Track which group headers we've printed ----
last_group=""

# ---- Sort test files: numbered first (by number), custom after (alphabetical) ----
sorted_files=$(ls "$TEST_DIR"/*.txt 2>/dev/null | while read f; do
    base=$(basename "$f" .txt)
    num=$(echo "$base" | grep -o '[0-9]\+' | head -1)
    if [ -n "$num" ]; then
        printf "%04d %s\n" "$((10#$num))" "$f"
    else
        printf "9999_%s %s\n" "$base" "$f"
    fi
done | sort | awk '{print $2}')

if [ -z "$sorted_files" ]; then
    echo -e "${RED}ERROR: No test files found in $TEST_DIR${NC}"
    exit 1
fi

# ---- Run each test ----
for test_file in $sorted_files; do
    test_name=$(basename "$test_file" .txt)
    expected_file="$EXPECTED_DIR/${test_name}_expected.txt"

    # Extract test number for group headers
    test_num=$(echo "$test_name" | grep -o '[0-9]\+' | head -1)

    # Print group header if entering a new group
    current_group=""
    case $test_num in
        [1-9]|10) current_group="1" ;;
        1[1-6])   current_group="11" ;;
        1[7-9])   current_group="17" ;;
        2[0-3])   current_group="20" ;;
        2[4-7])   current_group="24" ;;
        2[8-9]|30) current_group="28" ;;
        3[1-3])   current_group="31" ;;
        3[4-6])   current_group="34" ;;
        3[7-9])   current_group="37" ;;
        4[0-2])   current_group="40" ;;
        4[3-5])   current_group="43" ;;
        4[6-7])   current_group="46" ;;
        48)       current_group="48" ;;
        49)       current_group="49" ;;
        50)       current_group="50" ;;
    esac

    if [ "$current_group" != "$last_group" ] && [ -n "$current_group" ]; then
        echo ""
        print_group_header "$current_group"
        last_group="$current_group"
    fi

    # Check expected file exists
    if [ ! -f "$expected_file" ]; then
        echo -e "  ${YELLOW}SKIP${NC}  $test_name — no expected file"
        SKIP_COUNT=$((SKIP_COUNT + 1))
        continue
    fi

    TOTAL=$((TOTAL + 1))

    # Run the test — strip comments and blank lines before feeding to binary
    actual_output=$(grep -vE '^ *#|^ *$' "$test_file" | $BINARY 2>&1 | grep "Actuator command" | sed 's/.*\[//;s/\].*//;s/  */ /g')
    expected_output=$(cat "$expected_file")

    # Count expected steps
    expected_steps=$(echo "$expected_output" | wc -l)
    actual_steps=$(echo "$actual_output" | wc -l)

    if [ "$expected_steps" != "$actual_steps" ]; then
        echo -e "  ${RED}FAIL${NC}  $test_name — expected $expected_steps steps, got $actual_steps"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        continue
    fi

    # Compare line by line
    test_passed=true
    fail_step=0
    fail_expected=""
    fail_actual=""
    step=0

    while IFS= read -r expected_line && IFS= read -r actual_line <&3; do
        step=$((step + 1))

        # Parse 3 values from each line
        read -r ae1 ae2 ae3 <<< "$expected_line"
        read -r aa1 aa2 aa3 <<< "$actual_line"

        # Compare each axis within tolerance
        for pair in "$ae1 $aa1" "$ae2 $aa2" "$ae3 $aa3"; do
            read -r exp act <<< "$pair"
            result=$(awk -v e="$exp" -v a="$act" -v tol="$TOLERANCE" \
                'BEGIN { diff = (e - a); if (diff < 0) diff = -diff; print (diff <= tol) ? "OK" : "FAIL" }')
            if [ "$result" = "FAIL" ]; then
                test_passed=false
                fail_step=$step
                fail_expected="$expected_line"
                fail_actual="$actual_line"
                break
            fi
        done

        if [ "$test_passed" = false ]; then
            break
        fi
    done <<< "$expected_output" 3<<< "$actual_output"

    if [ "$test_passed" = true ]; then
        echo -e "  ${GREEN}PASS${NC}  $test_name ($expected_steps step(s))"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo -e "  ${RED}FAIL${NC}  $test_name (step $fail_step of $expected_steps)"
        echo -e "         Expected: [$fail_expected]"
        echo -e "         Actual:   [$fail_actual]"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
done

# ---- Summary ----
echo ""
echo -e "${BOLD}============================================================${NC}"
echo -e "${BOLD}  Results: $PASS_COUNT passed, $FAIL_COUNT failed, $SKIP_COUNT skipped, $TOTAL tested${NC}"
echo -e "${BOLD}============================================================${NC}"

if [ "$FAIL_COUNT" -eq 0 ]; then
    echo -e "${GREEN}${BOLD}  ALL $TOTAL TESTS PASSED${NC}"
    echo ""
    exit 0
else
    echo -e "${RED}${BOLD}  $FAIL_COUNT TEST(S) FAILED${NC}"
    echo ""
    exit 1
fi
