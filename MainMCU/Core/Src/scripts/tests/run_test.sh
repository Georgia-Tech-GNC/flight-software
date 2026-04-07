#!/bin/bash
# ============================================================
# run_test.sh
#
# Strips comments/blank lines from a test file and pipes it
# to the test harness binary. Optionally saves the actuator
# output as the expected result.
#
# Usage:
#   ./scripts/tests/run_test.sh <test_name>          # run and print
#   ./scripts/tests/run_test.sh <test_name> --save   # run and save expected
#   ./scripts/tests/run_test.sh path/to/file.txt     # also accepts a path
#
# Env overrides:
#   TEST_DIR      (default: ./all_tests)
#   EXPECTED_DIR  (default: ./all_expected)
#   ROCKET_BIN    (default: ./main/test_pid)
#
# Build the binary first with:
#   gcc main/rocket_pid.c main/test_pid.c -o main/test_pid -lm
# ============================================================
set -euo pipefail

TEST_DIR="${TEST_DIR:-./all_tests}"
EXPECTED_DIR="${EXPECTED_DIR:-./all_expected}"
ROCKET_BIN="${ROCKET_BIN:-./main/test_pid}"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <test_name> [--save]"
    exit 1
fi

ARG="$1"
SAVE=0
if [ "${2:-}" = "--save" ]; then
    SAVE=1
fi

# Accept either a bare name or a path
if [ -f "$ARG" ]; then
    TEST_FILE="$ARG"
    TEST_NAME="$(basename "${ARG%.txt}")"
else
    TEST_NAME="$ARG"
    TEST_FILE="$TEST_DIR/${TEST_NAME}.txt"
fi

if [ ! -f "$TEST_FILE" ]; then
    echo "ERROR: test file not found: $TEST_FILE"
    exit 1
fi

if [ ! -x "$ROCKET_BIN" ]; then
    echo "ERROR: $ROCKET_BIN not found or not executable."
    echo "       Build it with:"
    echo "         gcc main/rocket_pid.c main/test_pid.c -o main/test_pid -lm"
    echo "       Or set ROCKET_BIN=/path/to/binary"
    exit 1
fi

# Strip comment lines and blank lines, then feed to the binary
CLEAN="$(grep -vE '^[[:space:]]*(#|$)' "$TEST_FILE")"

if [ "$SAVE" -eq 1 ]; then
    mkdir -p "$EXPECTED_DIR"
    EXPECTED_FILE="$EXPECTED_DIR/${TEST_NAME}_expected.txt"
    echo "$CLEAN" | "$ROCKET_BIN" \
        | grep 'Actuator command' \
        | sed 's/.*\[//; s/\].*//; s/  */ /g' \
        > "$EXPECTED_FILE"
    echo "Saved: $EXPECTED_FILE"
    echo "--- contents ---"
    cat "$EXPECTED_FILE"
else
    echo "$CLEAN" | "$ROCKET_BIN"
fi