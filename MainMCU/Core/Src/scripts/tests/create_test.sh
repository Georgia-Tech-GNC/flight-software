#!/bin/bash
# ============================================================
# create_test.sh
#
# Creates an annotated test case template. Edit it, then run:
#   ./scripts/run_test.sh <test_name>
#
# Usage:
#   ./scripts/create_test.sh my_test_name
#   ./scripts/create_test.sh               (defaults to "custom_test")
#
# Env overrides:
#   TEST_DIR      (default: ./all_tests)
#   EXPECTED_DIR  (default: ./all_expected)
# ============================================================
set -euo pipefail

TEST_DIR="${TEST_DIR:-./all_tests}"
EXPECTED_DIR="${EXPECTED_DIR:-./all_expected}"
mkdir -p "$TEST_DIR" "$EXPECTED_DIR"

TEST_NAME="${1:-custom_test}"

# Validate test name: alphanumeric, underscore, dash only
if [[ ! "$TEST_NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "ERROR: test name must contain only letters, digits, underscores, or dashes."
    echo "       Got: '$TEST_NAME'"
    exit 1
fi

TEST_FILE="$TEST_DIR/${TEST_NAME}.txt"

if [ -f "$TEST_FILE" ]; then
    echo "ERROR: $TEST_FILE already exists. Pick a different name."
    exit 1
fi

cat > "$TEST_FILE" << 'TEMPLATE'
# ============================================================
# ROCKET PID CONTROLLER — TEST CASE
#
# Edit the values below. Comment lines (starting with #) and
# blank lines are stripped automatically when you run:
#
#     ./scripts/run_test.sh <this_test_name>
#
# You never need to hand-clean this file.
# ============================================================

# ---- PID GAINS (roll pitch yaw) ----
# Kp: proportional (1.0-5.0 typical)
# Ki: integral     (0.0-0.5 typical, start at 0)
# Kd: derivative   (0.1-2.0 typical)
2.0 2.0 2.0
0.0 0.0 0.0
0.5 0.5 0.5

# ---- FILTER TIME CONSTANT (seconds) ----
# 0.0 = disabled (matches original MATLAB)
# 0.02-0.05 = light filtering
# 0.1-0.5 = heavy filtering
0.05

# ---- REFERENCE TRAJECTORY ----
# First line: number of reference points
# Then per point: time, then quaternion (w x y z)
#
# Common quaternions:
#   1 0 0 0             = identity (no rotation)
#   0.9239 0.3827 0 0   = 45 deg roll
#   0.9239 0 0.3827 0   = 45 deg pitch
#   0.7071 0.7071 0 0   = 90 deg roll
#
# Formula for angle A (radians) about axis [ax,ay,az]:
#   w = cos(A/2), x = sin(A/2)*ax, y = sin(A/2)*ay, z = sin(A/2)*az
1
0.0
1 0 0 0

# ---- TIMESTEPS ----
# Per timestep, provide 6 lines:
#   1. current time (seconds)
#   2. cumulative velocity (0, unused)
#   3. position x y z (0 0 0, unused)
#   4. state[3..6] (0 0 0 0, unused)
#   5. quaternion w x y z (current orientation)
#   6. angular velocity rad/s (roll_rate pitch_rate yaw_rate)
#
# End the stream with: q
#
# Example below: rocket starts tilted ~10 deg on roll, commanded
# to identity. You should see actuator output drive roll back.

# -- Step 1: t=0.0, tilted 10 deg on roll (sin(5deg)=0.0872) --
0.0
0.0
0 0 0
0 0 0 0
0.9962 0.0872 0 0
0 0 0

# -- Step 2: t=0.1 --
0.1
0.0
0 0 0
0 0 0 0
0.9962 0.0872 0 0
0 0 0

# -- Step 3: t=0.2 --
0.2
0.0
0 0 0
0 0 0 0
0.9980 0.0628 0 0
0 0 0

# -- Step 4: t=0.3 --
0.3
0.0
0 0 0
0 0 0 0
0.9994 0.0349 0 0
0 0 0

# -- Step 5: t=0.4 --
0.4
0.0
0 0 0
0 0 0 0
1 0 0 0
0 0 0

q
TEMPLATE

echo ""
echo "  Created: $TEST_FILE"
echo ""
echo "  Next:"
echo "    1. Edit $TEST_FILE"
echo "    2. Run it:           ./scripts/run_test.sh $TEST_NAME"
echo "    3. Save as expected: ./scripts/run_test.sh $TEST_NAME --save"
echo ""