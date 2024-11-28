#ifndef CONTROLS_UNITTESTS_H
#define CONTROLS_UNITTESTS_H

// #include <gtest/gtest.h>

// Function declarations for unit tests
void TEST_ASSERT_EQUAL_FLOAT(float expected, float actual);
void test_initialize_controller();
void test_lqr_gain_selector();
void test_reference_selector();
void test_compute_controls();
void test_sideforce_to_vane_angle();
void test_vane_angle_to_servo_angle();
void test_moment_to_sideforce();
void test_sideforce_to_vane_angle();
void test_vane_angle_to_servo_angle();
void test_run_controls();
int main();

#endif // CONTROLS_UNITTESTS_H