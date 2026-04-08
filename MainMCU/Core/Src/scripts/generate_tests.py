#!/usr/bin/env python3
"""
generate_tests.py

Generates 50 test input files in all_tests/ and runs each one against
the test_pid binary to capture ground-truth expected output in all_expected/.

Run once after building:
    make
    python3 generate_tests.py

Test categories:
    1-10   Baseline (tau=0, matches MATLAB exactly)
    11-16  Single-axis errors (roll, pitch, yaw at various angles)
    17-19  Asymmetric gains (different Kp/Ki/Kd per axis)
    20-23  Derivative isolation (Kp=Ki=0, only Kd active)
    24-27  Filter tau sweep (same scenario, increasing tau)
    28-30  Integral Ki sweep (Kp=Kd=0, only Ki active)
    31-33  Combined PID (realistic gain sets)
    34-36  Reference trajectory lookup (multi-point ref)
    37-39  Non-identity initial state + angular velocity
    40-42  Variable timestep (irregular dt)
    43-45  Filter noise stress (noisy omega signals)
    46-47  Large gain stress test
    48     Negative rotation (q_err.w < 0 path)
    49     Convergence tracking (error shrinking over time)
    50     Long sequence (20 steps)
"""

import os
import subprocess
import math

TEST_DIR = "all_tests"
EXPECTED_DIR = "all_expected"
BINARY = "./main/test_pid"

os.makedirs(TEST_DIR, exist_ok=True)
os.makedirs(EXPECTED_DIR, exist_ok=True)


def axis_angle_to_quat(axis, angle_deg):
    """Convert axis-angle to quaternion [w, x, y, z]."""
    a = math.radians(angle_deg) / 2.0
    s = math.sin(a)
    norm = math.sqrt(sum(x*x for x in axis))
    if norm < 1e-30:
        return (1.0, 0.0, 0.0, 0.0)
    ax = [x / norm for x in axis]
    return (math.cos(a), s * ax[0], s * ax[1], s * ax[2])


def fmt_q(q):
    return f"{q[0]:.10f} {q[1]:.10f} {q[2]:.10f} {q[3]:.10f}"


def fmt_v3(v):
    return f"{v[0]:.10f} {v[1]:.10f} {v[2]:.10f}"


def make_test_content(kp, ki, kd, tau, ref_points, steps):
    """
    Build a test input string.
    ref_points: list of (time, (w, x, y, z))
    steps: list of (time, cumvel, quat, omega)
           quat = (w,x,y,z), omega = (r,p,y)
    """
    lines = []
    # Gains
    lines.append(fmt_v3(kp))
    lines.append(fmt_v3(ki))
    lines.append(fmt_v3(kd))
    # Tau
    lines.append(f"{tau:.10f}")
    # Reference trajectory
    lines.append(str(len(ref_points)))
    for t, q in ref_points:
        lines.append(f"{t:.10f}")
        lines.append(fmt_q(q))
    # Steps
    for t, cv, quat, omega in steps:
        lines.append(f"{t:.10f}")     # current time
        lines.append(f"{cv:.10f}")    # cumulative velocity
        lines.append("0 0 0")        # position (unused)
        lines.append("0 0 0 0")      # state[3..6] (unused)
        lines.append(fmt_q(quat))    # current quaternion
        lines.append(fmt_v3(omega))  # angular velocity
    # Quit
    lines.append("q")
    return "\n".join(lines) + "\n"


def write_and_run(test_num, content, description=""):
    """Write test file, run binary, capture expected output."""
    name = f"test{test_num:02d}"
    test_file = os.path.join(TEST_DIR, f"{name}.txt")
    expected_file = os.path.join(EXPECTED_DIR, f"{name}_expected.txt")

    with open(test_file, "w") as f:
        f.write(content)

    # Run binary and extract actuator command values
    result = subprocess.run(
        [BINARY],
        input=content,
        capture_output=True,
        text=True,
        timeout=10
    )

    # Parse actuator commands from output
    commands = []
    for line in result.stdout.splitlines():
        if "Actuator command" in line:
            # Extract the values between [ and ]
            inside = line.split("[")[1].split("]")[0]
            vals = inside.split()
            commands.append(" ".join(vals))

    with open(expected_file, "w") as f:
        f.write("\n".join(commands) + "\n")

    status = f"  [{test_num:02d}] {description} — {len(commands)} step(s)"
    print(status)


# =====================================================================
# Identity quaternion and zero state (shared across many tests)
# =====================================================================
Q_IDENT = (1.0, 0.0, 0.0, 0.0)
OMEGA_ZERO = (0.0, 0.0, 0.0)
REF_IDENT = [(0.0, Q_IDENT)]

# =====================================================================
# Tests 1-10: Baseline (tau=0, MATLAB-equivalent)
# These prove the C port matches MATLAB when the filter is disabled.
# =====================================================================

# Test 1: Zero error, zero omega — output should be exactly 0
write_and_run(1,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(0.5,0.5,0.5), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.0, 0.0, Q_IDENT, OMEGA_ZERO)]
    ),
    "Zero error, zero omega (tau=0)")

# Test 2: 45° roll error, P-only
q_45_roll = axis_angle_to_quat([1,0,0], 45)
write_and_run(2,
    make_test_content(
        kp=(2,2,2), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_45_roll, OMEGA_ZERO)]
    ),
    "45° roll error, P-only (tau=0)")

# Test 3: 45° pitch error, P-only
q_45_pitch = axis_angle_to_quat([0,1,0], 45)
write_and_run(3,
    make_test_content(
        kp=(2,2,2), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_45_pitch, OMEGA_ZERO)]
    ),
    "45° pitch error, P-only (tau=0)")

# Test 4: 45° yaw error, P-only
q_45_yaw = axis_angle_to_quat([0,0,1], 45)
write_and_run(4,
    make_test_content(
        kp=(2,2,2), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_45_yaw, OMEGA_ZERO)]
    ),
    "45° yaw error, P-only (tau=0)")

# Test 5: 90° roll error, P-only
q_90_roll = axis_angle_to_quat([1,0,0], 90)
write_and_run(5,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_90_roll, OMEGA_ZERO)]
    ),
    "90° roll error, P-only (tau=0)")

# Test 6: D-only with angular velocity, no error
write_and_run(6,
    make_test_content(
        kp=(0,0,0), ki=(0,0,0), kd=(1,1,1), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, Q_IDENT, (0.5, -0.3, 0.1))]
    ),
    "D-only, omega=[0.5,-0.3,0.1] (tau=0)")

# Test 7: P+D with 30° pitch error and omega
q_30_pitch = axis_angle_to_quat([0,1,0], 30)
write_and_run(7,
    make_test_content(
        kp=(1.5,1.5,1.5), ki=(0,0,0), kd=(0.5,0.5,0.5), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_30_pitch, (0.0, 0.2, 0.0))]
    ),
    "P+D, 30° pitch + omega (tau=0)")

# Test 8: I-only, two steps (integral accumulates)
write_and_run(8,
    make_test_content(
        kp=(0,0,0), ki=(1,1,1), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[
            (0.1, 0.0, q_45_roll, OMEGA_ZERO),
            (0.2, 0.0, q_45_roll, OMEGA_ZERO),
        ]
    ),
    "I-only, 45° roll, 2 steps (tau=0)")

# Test 9: Full PID, 3 steps
write_and_run(9,
    make_test_content(
        kp=(2,2,2), ki=(0.1,0.1,0.1), kd=(0.5,0.5,0.5), tau=0.0,
        ref_points=REF_IDENT,
        steps=[
            (0.1, 0.0, q_45_yaw, (0.0, 0.0, 0.1)),
            (0.2, 0.0, q_45_yaw, (0.0, 0.0, 0.05)),
            (0.3, 0.0, q_30_pitch, (0.0, -0.1, 0.0)),
        ]
    ),
    "Full PID, 3 steps, mixed axes (tau=0)")

# Test 10: dt guard — same timestamp twice (dt forced to 1e-4)
write_and_run(10,
    make_test_content(
        kp=(1,1,1), ki=(0.5,0.5,0.5), kd=(0.5,0.5,0.5), tau=0.0,
        ref_points=REF_IDENT,
        steps=[
            (1.0, 0.0, q_45_roll, (0.1, 0.0, 0.0)),
            (1.0, 0.0, q_45_roll, (0.1, 0.0, 0.0)),
        ]
    ),
    "Repeated timestamp (dt guard) (tau=0)")

# =====================================================================
# Tests 11-16: Single-axis errors at various angles
# =====================================================================

for i, angle in enumerate([10, 30, 60, 90, 120, 170]):
    q = axis_angle_to_quat([1, 0, 0], angle)
    write_and_run(11 + i,
        make_test_content(
            kp=(1,1,1), ki=(0,0,0), kd=(0,0,0), tau=0.0,
            ref_points=REF_IDENT,
            steps=[(0.1, 0.0, q, OMEGA_ZERO)]
        ),
        f"Roll error {angle}°, P-only (tau=0)")

# =====================================================================
# Tests 17-19: Asymmetric gains
# =====================================================================

# 17: Different Kp per axis
q_compound = axis_angle_to_quat([1, 1, 1], 45)
write_and_run(17,
    make_test_content(
        kp=(1, 3, 5), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_compound, OMEGA_ZERO)]
    ),
    "Asymmetric Kp=[1,3,5], compound 45° (tau=0)")

# 18: Different Kd per axis
write_and_run(18,
    make_test_content(
        kp=(0,0,0), ki=(0,0,0), kd=(0.1, 1.0, 5.0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, Q_IDENT, (1.0, 1.0, 1.0))]
    ),
    "Asymmetric Kd=[0.1,1,5], uniform omega (tau=0)")

# 19: All gains asymmetric
write_and_run(19,
    make_test_content(
        kp=(2, 0.5, 1), ki=(0.1, 0.3, 0), kd=(1, 0, 0.5), tau=0.0,
        ref_points=REF_IDENT,
        steps=[
            (0.1, 0.0, q_compound, (0.5, -0.5, 0.3)),
            (0.2, 0.0, q_compound, (0.4, -0.4, 0.2)),
        ]
    ),
    "All gains asymmetric, 2 steps (tau=0)")

# =====================================================================
# Tests 20-23: Derivative isolation (only Kd, vary omega)
# =====================================================================

omega_sets = [
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.5, 0.5, 0.5),
    (-1.0, 2.0, -0.5),
]
for i, om in enumerate(omega_sets):
    write_and_run(20 + i,
        make_test_content(
            kp=(0,0,0), ki=(0,0,0), kd=(2,2,2), tau=0.0,
            ref_points=REF_IDENT,
            steps=[(0.1, 0.0, Q_IDENT, om)]
        ),
        f"D-only, omega={list(om)} (tau=0)")

# =====================================================================
# Tests 24-27: Filter tau sweep (same input, different tau)
# Same 45° error + omega, only tau changes
# =====================================================================

for i, tau_val in enumerate([0.0, 0.02, 0.1, 0.5]):
    write_and_run(24 + i,
        make_test_content(
            kp=(1,1,1), ki=(0,0,0), kd=(1,1,1), tau=tau_val,
            ref_points=REF_IDENT,
            steps=[
                (0.01, 0.0, q_45_roll, (1.0, 0.0, 0.0)),
                (0.02, 0.0, q_45_roll, (1.5, 0.0, 0.0)),
                (0.03, 0.0, q_45_roll, (0.5, 0.0, 0.0)),
                (0.04, 0.0, q_45_roll, (2.0, 0.0, 0.0)),
            ]
        ),
        f"Tau sweep: tau={tau_val}, 4 steps with omega changes")

# =====================================================================
# Tests 28-30: Integral Ki sweep
# =====================================================================

for i, ki_val in enumerate([0.1, 1.0, 5.0]):
    write_and_run(28 + i,
        make_test_content(
            kp=(0,0,0), ki=(ki_val, ki_val, ki_val), kd=(0,0,0), tau=0.0,
            ref_points=REF_IDENT,
            steps=[
                (0.1, 0.0, q_45_yaw, OMEGA_ZERO),
                (0.2, 0.0, q_45_yaw, OMEGA_ZERO),
                (0.3, 0.0, q_45_yaw, OMEGA_ZERO),
            ]
        ),
        f"I-only sweep: Ki={ki_val}, 3 steps (tau=0)")

# =====================================================================
# Tests 31-33: Combined PID with realistic gains
# =====================================================================

realistic_gains = [
    ((2, 2, 2), (0.1, 0.1, 0.1), (0.5, 0.5, 0.5), 0.05),
    ((3, 3, 3), (0.2, 0.2, 0.2), (1.0, 1.0, 1.0), 0.02),
    ((1, 2, 3), (0.05, 0.1, 0.2), (0.3, 0.6, 1.0), 0.1),
]
for i, (kp, ki, kd, tau_val) in enumerate(realistic_gains):
    write_and_run(31 + i,
        make_test_content(
            kp=kp, ki=ki, kd=kd, tau=tau_val,
            ref_points=REF_IDENT,
            steps=[
                (0.01, 0.0, q_45_pitch, (0.0, 0.5, 0.0)),
                (0.02, 0.0, q_30_pitch, (0.0, 0.3, 0.0)),
                (0.05, 0.0, axis_angle_to_quat([0,1,0], 15), (0.0, 0.1, 0.0)),
                (0.10, 0.0, axis_angle_to_quat([0,1,0], 5), (0.0, 0.02, 0.0)),
            ]
        ),
        f"Combined PID set {i+1}: Kp={list(kp)} tau={tau_val}")

# =====================================================================
# Tests 34-36: Reference trajectory lookup (multi-point)
# =====================================================================

# 34: 2-point ref, time snaps to nearest
q_ref_a = axis_angle_to_quat([0,0,1], 20)
q_ref_b = axis_angle_to_quat([0,0,1], -20)
write_and_run(34,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=[(0.0, q_ref_a), (5.0, q_ref_b)],
        steps=[
            (0.1, 0.0, Q_IDENT, OMEGA_ZERO),   # snaps to ref[0]
            (4.9, 0.0, Q_IDENT, OMEGA_ZERO),   # snaps to ref[1]
        ]
    ),
    "2-point ref, snap to nearest time (tau=0)")

# 35: 5-point ref trajectory (pitchover manoeuvre)
ref_pitchover = []
for j in range(5):
    angle = j * 20.0
    ref_pitchover.append((j * 2.0, axis_angle_to_quat([0,1,0], angle)))
write_and_run(35,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(0.5,0.5,0.5), tau=0.0,
        ref_points=ref_pitchover,
        steps=[
            (0.0, 0.0, Q_IDENT, OMEGA_ZERO),
            (2.0, 0.0, axis_angle_to_quat([0,1,0], 15), (0.0, 0.1, 0.0)),
            (4.0, 0.0, axis_angle_to_quat([0,1,0], 35), (0.0, 0.15, 0.0)),
            (6.0, 0.0, axis_angle_to_quat([0,1,0], 55), (0.0, 0.1, 0.0)),
        ]
    ),
    "5-point pitchover ref trajectory (tau=0)")

# 36: Ref at exact boundary — t=2.5 equidistant between 0 and 5
write_and_run(36,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=[(0.0, Q_IDENT), (5.0, q_45_yaw)],
        steps=[
            (2.4, 0.0, Q_IDENT, OMEGA_ZERO),  # snaps to ref[0]
            (2.6, 0.0, Q_IDENT, OMEGA_ZERO),  # snaps to ref[1]
        ]
    ),
    "Ref boundary snap: t=2.4 vs t=2.6 (tau=0)")

# =====================================================================
# Tests 37-39: Non-identity initial state + angular velocity
# =====================================================================

# 37: Already at ref, spinning fast
write_and_run(37,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(2,2,2), tau=0.05,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, Q_IDENT, (3.0, -2.0, 1.5))]
    ),
    "At ref but spinning fast, D-dominated")

# 38: Large compound error + spin
q_large = axis_angle_to_quat([0.5, 0.7, 0.5], 120)
write_and_run(38,
    make_test_content(
        kp=(1,1,1), ki=(0.1,0.1,0.1), kd=(0.5,0.5,0.5), tau=0.05,
        ref_points=REF_IDENT,
        steps=[
            (0.1, 0.0, q_large, (1.0, -1.0, 0.5)),
            (0.2, 0.0, q_large, (0.8, -0.8, 0.4)),
        ]
    ),
    "120° compound error + spin, 2 steps")

# 39: Near-identity quaternion (very small error)
q_tiny = axis_angle_to_quat([1,0,0], 0.5)  # half a degree
write_and_run(39,
    make_test_content(
        kp=(5,5,5), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_tiny, OMEGA_ZERO)]
    ),
    "Tiny error (0.5°), high Kp (tau=0)")

# =====================================================================
# Tests 40-42: Variable timestep
# =====================================================================

# 40: Increasing dt
write_and_run(40,
    make_test_content(
        kp=(1,1,1), ki=(0.5,0.5,0.5), kd=(0.5,0.5,0.5), tau=0.05,
        ref_points=REF_IDENT,
        steps=[
            (0.001, 0.0, q_45_roll, (0.1, 0.0, 0.0)),
            (0.002, 0.0, q_45_roll, (0.1, 0.0, 0.0)),
            (0.01,  0.0, q_45_roll, (0.1, 0.0, 0.0)),
            (0.1,   0.0, q_45_roll, (0.1, 0.0, 0.0)),
            (1.0,   0.0, q_45_roll, (0.1, 0.0, 0.0)),
        ]
    ),
    "Increasing dt: 1ms to 1s gaps")

# 41: Very small dt (high loop rate)
steps_fast = []
for j in range(10):
    t = j * 0.001
    steps_fast.append((t, 0.0, q_45_pitch, (0.0, 0.5, 0.0)))
write_and_run(41,
    make_test_content(
        kp=(1,1,1), ki=(0.1,0.1,0.1), kd=(1,1,1), tau=0.02,
        ref_points=REF_IDENT,
        steps=steps_fast
    ),
    "1kHz loop rate, 10 steps")

# 42: Backwards timestamp (dt guard triggers)
write_and_run(42,
    make_test_content(
        kp=(1,1,1), ki=(0.5,0.5,0.5), kd=(0.5,0.5,0.5), tau=0.0,
        ref_points=REF_IDENT,
        steps=[
            (1.0, 0.0, q_45_roll, OMEGA_ZERO),
            (0.5, 0.0, q_45_roll, OMEGA_ZERO),  # backwards!
        ]
    ),
    "Backwards timestamp (dt guard) (tau=0)")

# =====================================================================
# Tests 43-45: Filter noise stress (oscillating omega)
# =====================================================================

# 43: Alternating omega sign (simulates noisy gyro)
steps_noisy = []
for j in range(8):
    t = (j + 1) * 0.01
    sign = 1.0 if j % 2 == 0 else -1.0
    steps_noisy.append((t, 0.0, Q_IDENT, (sign * 2.0, 0.0, 0.0)))
write_and_run(43,
    make_test_content(
        kp=(0,0,0), ki=(0,0,0), kd=(1,1,1), tau=0.05,
        ref_points=REF_IDENT,
        steps=steps_noisy
    ),
    "Alternating omega ±2 rad/s, filter tau=0.05")

# 44: Same noise, no filter (tau=0) — shows unfiltered behaviour
write_and_run(44,
    make_test_content(
        kp=(0,0,0), ki=(0,0,0), kd=(1,1,1), tau=0.0,
        ref_points=REF_IDENT,
        steps=steps_noisy
    ),
    "Alternating omega ±2 rad/s, NO filter (tau=0)")

# 45: Spike then settle
steps_spike = [
    (0.01, 0.0, Q_IDENT, (0.1, 0.0, 0.0)),
    (0.02, 0.0, Q_IDENT, (10.0, 0.0, 0.0)),  # spike!
    (0.03, 0.0, Q_IDENT, (0.1, 0.0, 0.0)),
    (0.04, 0.0, Q_IDENT, (0.1, 0.0, 0.0)),
    (0.05, 0.0, Q_IDENT, (0.1, 0.0, 0.0)),
]
write_and_run(45,
    make_test_content(
        kp=(0,0,0), ki=(0,0,0), kd=(1,1,1), tau=0.1,
        ref_points=REF_IDENT,
        steps=steps_spike
    ),
    "Omega spike then settle, filter tau=0.1")

# =====================================================================
# Tests 46-47: Large gain stress
# =====================================================================

write_and_run(46,
    make_test_content(
        kp=(100,100,100), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_45_roll, OMEGA_ZERO)]
    ),
    "Very high Kp=100 (tau=0)")

write_and_run(47,
    make_test_content(
        kp=(0,0,0), ki=(0,0,0), kd=(100,100,100), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, Q_IDENT, (0.01, 0.01, 0.01))]
    ),
    "Very high Kd=100, tiny omega (tau=0)")

# =====================================================================
# Test 48: Negative rotation (forces q_err negate path)
# =====================================================================

# 170° error — q_err.w will be small, close to the negate boundary
q_170 = axis_angle_to_quat([1, 0, 0], 170)
write_and_run(48,
    make_test_content(
        kp=(1,1,1), ki=(0,0,0), kd=(0,0,0), tau=0.0,
        ref_points=REF_IDENT,
        steps=[(0.1, 0.0, q_170, OMEGA_ZERO)]
    ),
    "170° error, near q_err negate boundary (tau=0)")

# =====================================================================
# Test 49: Convergence tracking (error shrinks step by step)
# =====================================================================

convergence_angles = [60, 45, 30, 20, 10, 5, 2, 1, 0.5, 0.1]
steps_converge = []
for j, ang in enumerate(convergence_angles):
    t = (j + 1) * 0.1
    q = axis_angle_to_quat([0, 0, 1], ang)
    omega_val = math.radians(ang) * 0.1  # decreasing spin
    steps_converge.append((t, 0.0, q, (0.0, 0.0, omega_val)))
write_and_run(49,
    make_test_content(
        kp=(2,2,2), ki=(0.1,0.1,0.1), kd=(0.5,0.5,0.5), tau=0.05,
        ref_points=REF_IDENT,
        steps=steps_converge
    ),
    "Convergence: 60° down to 0.1°, 10 steps")

# =====================================================================
# Test 50: Long sequence (20 steps, mixed dynamics)
# =====================================================================

steps_long = []
for j in range(20):
    t = j * 0.05
    angle = 45.0 * math.sin(t * 3.0)  # oscillating error
    q = axis_angle_to_quat([0, 1, 0], angle)
    om_y = 0.5 * math.cos(t * 3.0)
    steps_long.append((t, 0.0, q, (0.0, om_y, 0.0)))
write_and_run(50,
    make_test_content(
        kp=(2,2,2), ki=(0.1,0.1,0.1), kd=(1,1,1), tau=0.05,
        ref_points=REF_IDENT,
        steps=steps_long
    ),
    "20-step oscillating pitch, combined PID")

# =====================================================================
# Done
# =====================================================================

print(f"\n{'='*60}")
print(f"  Generated 50 tests in {TEST_DIR}/")
print(f"  Expected outputs in {EXPECTED_DIR}/")
print(f"  Run:  ./verify_all.sh")
print(f"{'='*60}")
