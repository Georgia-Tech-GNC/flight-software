/**
 * @file controls.h
 * @author Patrick Barry
 * @brief This contains the definitions of functions needed for in-flight controls on the jet vanes rocket.
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * The materials provided are for the use of the students.
 * Copyrighted course materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __CONTROLS_H__
#define __CONTROLS_H__

typedef struct { 
 
    float time_since_launch; //Update at every time step

    float32_t x[9]; //State estimate
    float32_t x0[9]; //Reference state [u, v, w, p, q, r, q0, q1, q2, q3]
    float32_t u0[4]; //Reference control inputs [M_roll, M_pitch, M_yaw, T]

    float32_t K[9*4]; //Controller gian matrix K, is size 4x9 (mxn)

    float M_roll; //desired roll moment from controller
    float M_pitch; //desired pitch moment from controller
    float M_yaw; //desired yaw moment from controller
    float T; //thrust reported by controller

    float yaw_moment_arm; //distance between jet vanes and CG at any given moment of time
    float roll_moment_arm = 0.1; //TODO: CONFIRM THIS VALUE IS CORRECT. Distance between vanes and central axis of rocket

    float32_t forces[2]; //Desired forces according to moment arm [F_roll, F_yaw]

    float32_t vane_deflections[4]; //Desired vane deflections according to forces [Roll Vane 1, Roll Vane 2, Yaw Vane 1, Yaw Vane 2], degrees
    float32_t servo_deflections[4]; //Desired servo deflections according to servo drivetrain gear ratio, degrees

    float32_t thrust_curve[14] = {2125.0, 1650.0, 1530.0, 1520.0, 1490.0, 1350.0, 1285.0, 1150.0, 990.0, 760.0, 610.0, 400.0, 270.0, 150.0, 0.0};
        //Thrust at each second from t = 0 to t = 14, i.e., thrust_curve = {T(0.0), T(1.0), T(2.0), T(3.0), etc.}
    float current_thrust;
        


} controller;

void LQR_gain_selector(controller *ctrl);
void reference_selector(controller *ctrl);
void compute_controls(controller *ctrl);
void update_yaw_Smoment_arm(controller *ctrl);
void moment_to_sideforce(controller *ctrl);
void sideforce_to_vane_angle(controller *ctrl);
void vane_angle_to_servo_angle(controller *ctrl);
void run_controls(controller *ctrl, float *state, float elapsed_time);

#endif