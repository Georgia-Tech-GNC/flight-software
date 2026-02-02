#include "controls.h"
#include "controls_unittests.h"
#include <math.h>
#include <stdio.h>

void TEST_ASSERT_EQUAL_FLOAT(float expected, float actual) {
    if (fabs(expected - actual) > 0.0001) {
        printf("Test failed: expected %f, got %f\n", expected, actual);
    }
}

void test_initialize_controller() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Check that all values are set properly to default.
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[4]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[5]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[6]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[7]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[8]);

    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[4]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[5]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[6]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[7]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x0[8]);
    
    for (int i = 0; i < 36; i++) {
        TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.K[i]);
    }

    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.M_roll);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.M_pitch);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.M_yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.T);

    TEST_ASSERT_EQUAL_FLOAT(0.967, ctrl.yaw_moment_arm);
    TEST_ASSERT_EQUAL_FLOAT(0.1, ctrl.roll_moment_arm);

    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.forces[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.forces[1]);

    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.vane_deflections[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.vane_deflections[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.vane_deflections[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.vane_deflections[3]);

    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.servo_deflections[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.servo_deflections[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.servo_deflections[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.servo_deflections[3]);

    float32_t thrust_curve[15] = {2125.0, 1650.0, 1530.0, 1520.0, 1490.0, 1350.0, 1285.0, 1150.0, 990.0, 760.0, 610.0, 400.0, 270.0, 150.0, 0.0};
    for (int i = 0; i < 15; i++) {
        TEST_ASSERT_EQUAL_FLOAT(thrust_curve[i], ctrl.thrust_curve[i]);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.current_thrust);

    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.time_since_launch);
}

void test_lqr_gain_selector() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.time_since_launch = 1.2;
    ctrl.x[0] = 100.0;
    ctrl.x[1] = 3.0;
    ctrl.x[2] = 4.0;
    ctrl.x[8] = 0.1;

    // Call the LQR gain selector
    LQR_gain_selector(&ctrl);

    // Check that the gains are set properly
    // Change the expected gains if the test values are changed
    float expected_gains[] = { -0.000000, 0.000000, 0.000000, 31.727774, 0.000000, 0.000000, 31.622777, 0.000000, -0.000000, -0.000000, 0.000000, 0.000316, 0.000000, 2.270675, 0.000000, 0.000000, 0.181417, -0.000000, -0.000000, -0.356285, 0.000000, -0.000000, 0.000000, 51.997442, -0.000000, 0.000000, 8.507669, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000 };
    for (int i = 0; i < 36; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_gains[i], ctrl.K[i]);
    }
}

void test_reference_selector() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.time_since_launch = 5.3;

    // Call the reference selector
    reference_selector(&ctrl);

    // Check that the reference state is set properly
    // Change the expected reference state if the test values are changed
    float expected_ref_state[] = { 142.482196, -0.001701624, -0.002190393, -0.002206419, 0.000670038, 0.1745, -0.001469671, 0.005009632, 0.084768004 };
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_ref_state[i], ctrl.x0[i]);
    }
}

void test_compute_controls() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.x[0] = 100.0;
    ctrl.x[1] = 3.0;
    ctrl.x[2] = 4.0;
    ctrl.x[8] = 0.1;
    ctrl.x0[0] = 142.482196;
    ctrl.x0[1] = -0.001701624;
    ctrl.x0[2] = -0.002190393;
    ctrl.x0[3] = -0.002206419;
    ctrl.x0[4] = 0.000670038;
    ctrl.x0[5] = 0.1745;
    ctrl.x0[6] = -0.001469671;
    ctrl.x0[7] = 0.005009632;
    ctrl.x0[8] = 0.084768004;
    ctrl.K[0] = -0.000000;
    ctrl.K[1] = 0.000000;
    ctrl.K[2] = 0.000000;
    ctrl.K[3] = 31.727774;
    ctrl.K[4] = 0.000000;
    ctrl.K[5] = 0.000000;
    ctrl.K[6] = 31.622777;
    ctrl.K[7] = 0.000000;
    ctrl.K[8] = -0.000000;
    ctrl.K[9] = -0.000000;
    ctrl.K[10] = 0.000000;
    ctrl.K[11] = 0.000316;
    ctrl.K[12] = 0.000000;
    ctrl.K[13] = 2.270675;
    ctrl.K[14] = 0.000000;
    ctrl.K[15] = 0.000000;
    ctrl.K[16] = 0.181417;
    ctrl.K[17] = -0.000000;
    ctrl.K[18] = -0.000000;
    ctrl.K[19] = -0.356285;
    ctrl.K[20] = 0.000000;
    ctrl.K[21] = -0.000000;
    ctrl.K[22] = 0.000000;

    // Call the compute controls function
    compute_controls(&ctrl);

    // Check that the control moments are set properly
    // Change the expected control moments if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(0.116480, ctrl.M_roll);
    TEST_ASSERT_EQUAL_FLOAT(-0.001166, ctrl.M_pitch);
    TEST_ASSERT_EQUAL_FLOAT(-1.069461, ctrl.M_yaw);
}

void test_update_yaw_moment_arm() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.time_since_launch = 5.3;

    // Call the update yaw moment arm function
    update_yaw_moment_arm(&ctrl);

    // Check that the yaw moment arm is set properly
    // Change the expected yaw moment arm if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(0.967 - ((0.967 - 1.06)/13.1)*5.3, ctrl.yaw_moment_arm);
}

void test_moment_to_sideforce() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.M_roll = 0.116480;
    ctrl.M_yaw = -1.069461;
    ctrl.roll_moment_arm = 0.1;
    ctrl.yaw_moment_arm = 0.967;

    // Call the moment to sideforce function
    moment_to_sideforce(&ctrl);

    // Check that the forces are set properly
    // Change the expected forces if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(1.164800, ctrl.forces[0]);
    TEST_ASSERT_EQUAL_FLOAT(-1.105958, ctrl.forces[1]);
}

void test_sideforce_to_vane_angle() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.forces[0] = 1.164800;
    ctrl.forces[1] = -1.104610;

    // Call the sideforce to vane angle function
    sideforce_to_vane_angle(&ctrl);

    // Check that the vane deflections are set properly
    // Change the expected vane deflections if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(-0.155945, ctrl.vane_deflections[0]);
    TEST_ASSERT_EQUAL_FLOAT(-0.155945, ctrl.vane_deflections[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.164442, ctrl.vane_deflections[2]);
    TEST_ASSERT_EQUAL_FLOAT(-0.164442, ctrl.vane_deflections[3]);
}

void test_vane_angle_to_servo_angle() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    ctrl.vane_deflections[0] = 0.164442;
    ctrl.vane_deflections[1] = 0.164442;
    ctrl.vane_deflections[2] = 0.164442;
    ctrl.vane_deflections[3] = -0.164442;

    // Call the vane angle to servo angle function
    vane_angle_to_servo_angle(&ctrl);

    // Check that the servo deflections are set properly
    // Change the expected servo deflections if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(0.263107, ctrl.servo_deflections[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.263107, ctrl.servo_deflections[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.263107, ctrl.servo_deflections[2]);
    TEST_ASSERT_EQUAL_FLOAT(-0.263107, ctrl.servo_deflections[3]);
}

void test_run_controls() {
    // Make a controller struct
    controller ctrl;
    initialize_controls(&ctrl);

    // Set the test values
    float state[9] = { 100.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3 };
    float elapsed_time = 1.2;

    // Call the run controls function
    run_controls(&ctrl, state, elapsed_time);

    // Check that the controller struct is updated properly
    // Change the expected values if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(1.2, ctrl.time_since_launch);
    TEST_ASSERT_EQUAL_FLOAT(100.0, ctrl.x[0]);
    TEST_ASSERT_EQUAL_FLOAT(3.0, ctrl.x[1]);
    TEST_ASSERT_EQUAL_FLOAT(4.0, ctrl.x[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[4]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[5]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[6]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, ctrl.x[7]);
    TEST_ASSERT_EQUAL_FLOAT(3, ctrl.x[8]);

    // Check that the reference state is set properly
    // Change the expected reference state if the test values are changed
    float expected_ref_state[] = { 33.41983, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_ref_state[i], ctrl.x0[i]);
    }

    // Check that the gains are set properly
    // Change the expected gains if the test values are changed
    float expected_gains[] = { -0.000000, 0.000000, 0.000000, 31.727774, 0.000000, 0.000000, 31.622777, 0.000000, -0.000000, -0.000000, 0.000000, 0.000316, 0.000000, 2.270675, 0.000000, 0.000000, 0.181417, -0.000000, -0.000000, -0.356285, 0.000000, -0.000000, 0.000000, 51.997442, -0.000000, 0.000000, 8.507669, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000 };
    for (int i = 0; i < 36; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_gains[i], ctrl.K[i]);
    }

    // Check that the control moments are set properly
    // Change the expected control moments if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(0.000000, ctrl.M_roll);
    TEST_ASSERT_EQUAL_FLOAT(0.001264, ctrl.M_pitch);
    TEST_ASSERT_EQUAL_FLOAT(24.454153, ctrl.M_yaw);

    // Check that the yaw moment arm is set properly
    // Change the expected yaw moment arm if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(0.967 - ((0.967 - 1.06)/13.1)*1.2, ctrl.yaw_moment_arm);

    // Check that the forces are set properly
    // Change the expected forces if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(0.000000, ctrl.forces[0]);
    TEST_ASSERT_EQUAL_FLOAT(25.067837, ctrl.forces[1]);

    // Check that the vane deflections are set properly
    // Change the expected vane deflections if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(4.625062, ctrl.vane_deflections[0]);
    TEST_ASSERT_EQUAL_FLOAT(4.625062, ctrl.vane_deflections[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.000000, ctrl.vane_deflections[2]);
    TEST_ASSERT_EQUAL_FLOAT(-0.000000, ctrl.vane_deflections[3]);

    // Check that the servo deflections are set properly
    // Change the expected servo deflections if the test values are changed
    TEST_ASSERT_EQUAL_FLOAT(7.400099, ctrl.servo_deflections[0]);
    TEST_ASSERT_EQUAL_FLOAT(7.400099, ctrl.servo_deflections[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.000000, ctrl.servo_deflections[2]);
    TEST_ASSERT_EQUAL_FLOAT(-0.000000, ctrl.servo_deflections[3]);
}

// Driver code
int main() {
    test_initialize_controller();
    printf("test_initialize_controller passed\n");
    test_lqr_gain_selector();
    printf("test_lqr_gain_selector passed\n");
    test_reference_selector();
    printf("test_reference_selector passed\n");
    test_compute_controls();
    printf("test_compute_controls passed\n");
    test_update_yaw_moment_arm();
    printf("test_update_yaw_moment_arm passed\n");
    test_moment_to_sideforce();
    printf("test_moment_to_sideforce passed\n");
    test_sideforce_to_vane_angle();
    printf("test_sideforce_to_vane_angle passed\n");
    test_vane_angle_to_servo_angle();
    printf("test_vane_angle_to_servo_angle passed\n");
    test_run_controls();
    printf("test_run_controls passed\n");
    return 0;
}