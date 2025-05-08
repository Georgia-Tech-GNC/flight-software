/**
 * @file controls.h
 * @author Patrick Barry, Karsten Caillet
 * @brief This contains the definitions of functions needed for in-flight controls on the jet vanes rocket.
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * The materials provided are for the use of the students.
 * Copyrighted course materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __CONTROLS_H__
#define __CONTROLS_H__

#include "arm_math.h"

typedef struct { 

    float time_since_launch; //Update at every time step

    float32_t x[9]; //State estimate, assuming we are NOT estimating 10
    float32_t x0[9]; //Reference state [u, v, w, p, q, r, q1, q2, q3]
    //float32_t u0[4]; //Reference control inputs [M_roll, M_pitch, M_yaw, T]

    float32_t K[9*4]; //Controller gain matrix K, is size 4x9 (mxn)

    float M_roll; //desired roll moment from controller
    float M_pitch; //desired pitch moment from controller
    float M_yaw; //desired yaw moment from controller
    float T; //thrust reported by controller

    float yaw_moment_arm; //distance between jet vanes and CG at any given moment of time
    float roll_moment_arm; //TODO: CONFIRM THIS VALUE IS CORRECT. Distance between vanes and central axis of rocket

    float32_t forces[2]; //Desired forces according to moment arm [F_roll, F_yaw]

    float32_t vane_deflections[4]; //Desired vane deflections according to forces [Roll Vane 1, Roll Vane 2, Yaw Vane 1, Yaw Vane 2], degrees
    float32_t servo_deflections[4]; //Desired servo deflections according to servo drivetrain gear ratio, degrees
    
    float32_t thrust_curve[15];
        //Thrust at each second from t = 0 to t = 14, i.e., thrust_curve = {T(0.0), T(1.0), T(2.0), T(3.0), etc.}
    float current_thrust;

} controller;

// Structure to hold LQR gain data for a given time and state
#ifndef LQRGainSet_DEFINED
#define LQRGainSet_DEFINED
typedef struct {
    float time;       // Time for this LQR gain set
    float state[9];   // State vector (e.g., 9 states in the system)
    float gains[36];  // Flattened LQR gain matrix (4x9 matrix flattened into 1x36 array)
} LQRGainSet;
#endif

extern const LQRGainSet lqr_data[]; // Declare the array of LQR gains (this is defined in the .c file)

extern const int num_lqr_entries; // Declare the number of entries in the LQR gain array

// Structure to hold reference state data for a given time and state
#ifndef ReferenceState_DEFINED
#define ReferenceState_DEFINED
typedef struct {
    float time;       // Time for this reference state
    float state[10];  // 10-variable state vector
} ReferenceState;
#endif

extern const ReferenceState ref_state_data[]; // Declare the array of reference states (this is defined in the .c file)

extern const int num_ref_state_entries; // Declare the number of entries in the reference state array

void LQR_gain_selector(controller *ctrl);
void reference_selector(controller *ctrl);
void compute_controls(controller *ctrl);
void update_yaw_moment_arm(controller *ctrl);
void moment_to_sideforce(controller *ctrl);
void sideforce_to_vane_angle(controller *ctrl);
void vane_angle_to_servo_angle(controller *ctrl);
void initialize_controls(controller *ctrl);
void run_controls(controller *ctrl, float *state, float elapsed_time);

#endif