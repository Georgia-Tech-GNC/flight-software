/**
 * @file controls.c
 * @author Patrick Barry, Karsten Caillet
 * @brief Source file for in-flight control algorithm of the jet vanes rocket
 *  
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "controls.h"
#include "reference_data.h"
#include <math.h>
#include <stdio.h>

/**
 * @brief Takes into account current state estimate and time since launch and returns closest matching LQR Gain
 * @param ctrl Takes controller struct, but uses current state and time
 * @return closest matching LQR gain
 * @note
*/
void LQR_gain_selector(controller *ctrl){
    int t = (int)round(ctrl->time_since_launch);
    float q3 = ctrl->x[8];
    float airspeed = sqrt(ctrl->x[0] * ctrl->x[0] + ctrl->x[1] * ctrl->x[1] + ctrl->x[2] * ctrl->x[2]);
    float min_distance = 1e9;
    float distance = 0.0;
    int best_index = -1;

    if (t < 1 || t > 12) {
        for (int i = 0; i < 36; i++) {
            ctrl->K[i] = 0.0;
        }
    } else {
        for (int i = 0; i < num_lqr_entries; i++) {
            if (t == lqr_data[i].time) {
                if (best_index == -1) {
                    best_index = i;
                }
                distance = fabs(airspeed - lqr_data[i].state[0]) + fabs(q3 - lqr_data[i].state[1]);
                if (distance < min_distance) {
                    min_distance = distance;
                    best_index = i;
                }
            } else if (lqr_data[i].time > t) {
                break;
            }  
        }
        for (int i = 0; i < 36; i++) {
            ctrl->K[i] = lqr_data[best_index].gains[i];
        }
    }
}

/**
 * @brief Takes in account current state estiate and time since launch and returns reference state
 * @param ctrl Takes controller struct, but uses current state and time
 * @return correct reference state
 * @note
*/
void reference_selector(controller *ctrl){
    int t = (int)round(ctrl->time_since_launch);
    float base_x0[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    if (t > 13) {
        for (int i = 0; i < 9; i++) {
            ctrl->x0[i] = base_x0[i];
        }
    } else {
        for (int i = 0; i < 9; i++) {
            ctrl->x0[i] = ref_state_data[t].state[i]; // time corresponds 1:1 to index
        }
    } 
}

/**
 * @brief Initializes the control struct with all default values
 * @param ctrl Takes controller struct, but uses all variables in the struct
 * @return Initializes all variables in the controller struct
 * @note 
*/
void initialize_controls(controller *ctrl) {
    for (int i = 0; i < 9; i++) {
        ctrl->x[i] = 0.0;
        ctrl->x0[i] = 0.0;
    }

    for (int i = 0; i < 36; i++) {
        ctrl->K[i] = 0.0;
    }

    ctrl->M_roll = 0.0;
    ctrl->M_pitch = 0.0;
    ctrl->M_yaw = 0.0;
    ctrl->T = 0.0;

    ctrl->yaw_moment_arm = 0.967;
    ctrl->roll_moment_arm = 0.1;

    ctrl->forces[0] = 0.0;
    ctrl->forces[1] = 0.0;

    ctrl->vane_deflections[0] = 0.0;
    ctrl->vane_deflections[1] = 0.0;
    ctrl->vane_deflections[2] = 0.0;
    ctrl->vane_deflections[3] = 0.0;

    ctrl->servo_deflections[0] = 0.0;
    ctrl->servo_deflections[1] = 0.0;
    ctrl->servo_deflections[2] = 0.0;
    ctrl->servo_deflections[3] = 0.0;

    ctrl->current_thrust = 0.0;

    float32_t thrust_curve[15] = {2125.0, 1650.0, 1530.0, 1520.0, 1490.0, 1350.0, 1285.0, 1150.0, 990.0, 760.0, 610.0, 400.0, 270.0, 150.0, 0.0};
    for (int i = 0; i < 15; i++) {
        ctrl->thrust_curve[i] = thrust_curve[i];
    }

    ctrl->time_since_launch = 0.0;
}

/**
 * @brief Computes the control moments based on the LQR gain matrix and the difference between the current state and the reference state
 * @param ctrl Takes controller struct, but uses LQR gain matrix and current state
 * @return Updates the control moments in the controller struct
 * @note 
*/
void compute_controls(controller *ctrl) {
    float del_x[9] = {0};
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    float thrust = 0.0;

    for (int i = 0; i < 9; i++) {
        del_x[i] = ctrl->x[i] - ctrl->x0[i];
    }

    for (int j = 0; j < 4; j++) {
        if (j == 0) {
            for (int i = 0; i < 9; i++) {
                roll += ctrl->K[j*9 + i] * del_x[i];
            }
        } else if (j == 1) {
            for (int i = 0; i < 9; i++) {
                pitch += ctrl->K[j*9 + i] * del_x[i];
            }
        } else if (j == 2) {
            for (int i = 0; i < 9; i++) {
                yaw += ctrl->K[j*9 + i] * del_x[i];
            }
        } else if (j == 3) {
            for (int i = 0; i < 9; i++) {
                thrust += ctrl->K[j*9 + i] * del_x[i];
            }
        }
    }

    ctrl->M_roll = roll;
    ctrl->M_pitch = pitch;
    ctrl->M_yaw = yaw;
    ctrl->T = thrust;
} 

/**
 * @brief Function that updates the yaw moment arm in the struct since it shifts due to propellant burning
 * @param ctrl Takes controller struct, but uses initial and final moment arms plus burn time
 * @return Updates yaw_moment_arm variable in controller struct
 * @note Uses linear interpolation
*/
void update_yaw_moment_arm(controller *ctrl){
    float yaw_moment_arm_0 = 0.967; //initial yaw moment arm, m
    float yaw_moment_arm_f = 1.06; //final yaw moment arm, m
    
    float burn_time = 13.1;

    ctrl->yaw_moment_arm = yaw_moment_arm_0 - ((yaw_moment_arm_0 - yaw_moment_arm_f)/burn_time)*ctrl->time_since_launch;
}


/**
 * @brief Converts desired control moments of roll and yaw into desired roll and yaw forces
 * @param ctrl Takes controller struct, uses desired roll and yaw moments, and current roll and yaw moment arms
 * @return Updates values of forces 1-D array, forces = [F_roll, F_pitch]
 * @note Uses the fact that M = F*d
*/
void moment_to_sideforce(controller *ctrl){
    ctrl->forces[0] = ctrl->M_roll/ctrl->roll_moment_arm;
    ctrl->forces[1] = ctrl->M_yaw/ctrl->yaw_moment_arm;
}


/**
 * @brief Finds current thrust value, computes max sideforce for that thrust, and then computes vane deflection angles
 * @param ctrl Takes controller struct, but uses thrust curve, time since launch, and forces 1-D array
 * @return Updates vane_deflections 1-D array [roll1, roll2, yaw1, yaw2]
 * @note Sign conventions for vanes are important here!
*/
void sideforce_to_vane_angle(controller *ctrl){ //TODO: UPDATE BASED ON ACTUAL SIDEFORCE MODEL
    int rounded_down_second = (int)floor(ctrl->time_since_launch);

    if (ctrl->time_since_launch >= 13.8){ //If more than 13.8 seconds has passed since launch, assume no thrust
        ctrl->current_thrust = 0;
    } else { //Use linear interpolation with thrust curve
        float T_1 = ctrl->thrust_curve[rounded_down_second];
        float T_2 = ctrl->thrust_curve[rounded_down_second + 1];

        ctrl->current_thrust = T_1 + (T_2 - T_1)*(ctrl->time_since_launch - floor(ctrl->time_since_launch)); 
    } //Current thrust has been obtained

    float max_sideforce = 0.05*ctrl->current_thrust; //max sideforce per vane

    float yaw_force_per_vane = ctrl->forces[0]/2.0;
    float roll_force_per_vane = ctrl->forces[1]/2.0;

    float yaw_vane_angle; //degrees
    float roll_vane_angle; //degrees

    //Find vane angle for yaw
    if (yaw_force_per_vane >= max_sideforce){
        yaw_vane_angle = 30.0; //degrees
    } else {
        yaw_vane_angle = (yaw_force_per_vane/max_sideforce)*30.0;
    }

    //Find vane angle to roll
    if (roll_force_per_vane >= max_sideforce){
        roll_vane_angle = 30.0; //degrees - 30 degrees is the maximum deflection
    } else {
        roll_vane_angle = (roll_force_per_vane/max_sideforce)*30.0;
    }

    ctrl->vane_deflections[0] = roll_vane_angle;
    ctrl->vane_deflections[1] = roll_vane_angle; //Rolling vanes turn in same direction
    ctrl->vane_deflections[2] = yaw_vane_angle;
    ctrl->vane_deflections[3] = -yaw_vane_angle; //Because the yawing vanes turn in opposite directions
}


/**
 * @brief Turns vane angles until
 * @param ctrl Takes the controller struct, but uses the vane deflections 1-D array
 * @return Updates the servo_deflections 1-D array
 * @note Turn a vane by x degrees --> Turn the servo by (8/5)*x degrees according to the gear ratio in servo drivetrain
*/
void vane_angle_to_servo_angle(controller *ctrl){

    ctrl->servo_deflections[0] = ctrl->vane_deflections[0]*(8.0/5.0); //roll servo 1
    ctrl->servo_deflections[1] = ctrl->vane_deflections[1]*(8.0/5.0); //roll servo 2
    ctrl->servo_deflections[2] = ctrl->vane_deflections[2]*(8.0/5.0); //yaw servo 1
    ctrl->servo_deflections[3] = ctrl->vane_deflections[3]*(8.0/5.0); //yaw servo 2
}

/**
 * @brief Runs the control algorithm for the jet vanes rocket for a given time step
 * @param ctrl Takes the controller struct, the current state estimate, and the elapsed time since launch
 * @param state 1-D array of current state estimate
 * @param elapsed_time Time since launch
 * @return
 * @note
*/
void run_controls(controller *ctrl, float *state, float elapsed_time){
    // Update time since launch
    ctrl->time_since_launch = elapsed_time;

    // Update state estimate
    for (int i = 0; i < 9; i++) {
        ctrl->x[i] = state[i];
    }

    reference_selector(ctrl); // Update the reference state
    LQR_gain_selector(ctrl); // Update the LQR gain matrix
    compute_controls(ctrl); // Compute the control moments
    update_yaw_moment_arm(ctrl); // Update the yaw moment arm
    moment_to_sideforce(ctrl); // Convert moments to side forces
    sideforce_to_vane_angle(ctrl); // Convert side forces to vane angles
    vane_angle_to_servo_angle(ctrl); // Convert vane angles to servo angles

    // TODO: Send servo_deflections to the servo controller
}