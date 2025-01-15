/**
 * @file flight_ekf.c
 * @author Patrick Barry, Kanav Chugh
 * @brief Source file for revised in-flight EKF for position, velocity using GPS and IMU
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "flight_ekf.h"
#include "ekf_constants.h"

/**
 * @brief This function should only be called once at the beginning of flight; it initializes the ekf, putting all the matrices and vectors
 * into arm matrix instances so matrix operations can be performed from arm_math.h
 * @param ekf, the struct representing the Extended Kalman Filter which contains translational state estimates
 * @return
 * @note See https://github.com/ramblinrocketclub/flight-computer/blob/master/Core/Src/rocket.c for initializing process
 * This also may be rearranged to define initial matrices and vectors in another file to make the code cleaner. That is left
 * up to the software team.
*/
//See https://github.com/ramblinrocketclub/flight-computer/blob/master/Core/Src/rocket.c for initializing process
void initialize_ekf(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart, Sensors *sensors, uint16_t nz){
    //EKF state vector is x, vx, y, vy, z, vz

    ekf->time_step = 0.02;

    ekf->nx = 6;
    ekf->nu = 0;
    ekf->nz = nz;

    //arm_mat_init_f32(&ekf->G, ekf->nu, ekf->nu, G_f32);

    arm_mat_init_f32(&ekf->R, ekf->nz, ekf->nz, R_f32);
    arm_mat_init_f32(&ekf->dhdx, ekf->nz, ekf->nx, dhdx_f32);
    arm_mat_init_f32(&ekf->dfdx, ekf->nx, ekf->nx, dfdx_f32);
    arm_mat_init_f32(&ekf->Q, ekf->nx, ekf->nx, Q_f32);

    arm_mat_init_f32(&ekf->K_n, ekf->nx, ekf->nz, K_f32);

    arm_mat_init_f32(&ekf->x_n, ekf->nx, 1, x_init);
    arm_mat_init_f32(&ekf->P_n, ekf->nx, ekf->nx, P_init);

    arm_mat_init_f32(&ekf->f, ekf->nx, 1, f_f32);
    arm_mat_init_f32(&ekf->h, ekf->nz, 1, h_f32);
    //arm_mat_init_f32(&ekf->z, ekf->nz, 1, z_f32);
    //Compute process noise matrix 
    //print_matrix("Process Noise Covariance", &ekf->Q, huart);

    ekf->c[0] = -0.3; //vector from the IMU to the center of mass
    ekf->c[1] = 0.0;
    ekf->c[2] = 0.0;
    //TODO: Confirm with structures that the above is correct in terms of IMU placement in the rocket.

    ekf->gps[0] = 0.0;
    ekf->gps[1] = 0.0;
    ekf->gps[2] = 0.0;

    ekf->gps_origin[0] = ekf->gps[0];
    ekf->gps_origin[1] = ekf->gps[1];
    ekf->gps_origin[2] = ekf->gps[2];

    ekf->accelerometer[0] = 0;
    ekf->accelerometer[1] = 0.0;
    ekf->accelerometer[2] = 0.0;

    ekf->gyro[0] = 0.0;
    ekf->gyro[1] = 0.0;
    ekf->gyro[2] = 0.0;

    ekf->magneto[0] = 0.0;
    ekf->magneto[1] = 0.0;
    ekf->magneto[2] = 0.0;

    ekf->barometer = 0.0;
    ekf->accel_offset[0] = sensors->accel_x;
    ekf->accel_offset[1] = sensors->accel_y;
    ekf->accel_offset[2] = sensors->accel_z;
}

/**
 * @brief This function takes in measurements for the "Update" step of the EKF; the measurements are GPS.
 * @param ekf, the EKF struct
 * @return
 * @note
*/
void make_measurement(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    //HAL_UART_Transmit(huart, (uint8_t*)"Starting make_measurement...\r\n", 30, HAL_MAX_DELAY);
    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "GPS Measurements: [%.4f, %.4f, %.4f]\r\n", 
                       ekf->gps[0], ekf->gps[1], ekf->gps[2]);
    //HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    float32_t z_new_f32[ekf->nz];
    z_new_f32[0] = ekf->gps[0];
    z_new_f32[1] = ekf->gps[1];
    z_new_f32[2] = ekf->gps[2];

    arm_matrix_instance_f32 z_new;
    arm_mat_init_f32(&z_new, ekf->nz, 1, z_new_f32);
    //print_matrix("New measurement (z)", &z_new, huart);
    memcpy(ekf->z_data, z_new_f32, sizeof(float32_t) * ekf->nz);
    arm_mat_init_f32(&ekf->z, ekf->nz, 1, ekf->z_data);
    //print_matrix("Final measurement (z) in EKF", &ekf->z, huart);
    //HAL_UART_Transmit(huart, (uint8_t*)"make_measurement completed.\r\n", 29, HAL_MAX_DELAY);
}

/**
 * @brief This function establishes the EKF's "observation" function,
 *      which takes in the state estimate and converts it to the form of the measurements.
 * @param ekf, the rocket's EKF struct
*/
void observation_function(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    if (ekf == NULL || ekf->h.pData == NULL || ekf->x_n.pData == NULL) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error: Null pointer in observation_function\r\n", 45, HAL_MAX_DELAY);
        return;
    }

    //HAL_UART_Transmit(huart, (uint8_t*)"Observation Function:\r\n", 23, HAL_MAX_DELAY);

    // Print current state
    char buffer[200];
    int len = snprintf(buffer, sizeof(buffer), "Current state (x_n): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\r\n",
                       ekf->x_n.pData[0], ekf->x_n.pData[1], ekf->x_n.pData[2],
                       ekf->x_n.pData[3], ekf->x_n.pData[4], ekf->x_n.pData[5]);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    len = snprintf(buffer, sizeof(buffer), 
        "GPS origin: [%.4f, %.4f, %.4f]\r\n",
        ekf->gps_origin[0], ekf->gps_origin[1], ekf->gps_origin[2]);
    //HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    
    ekf->h.pData[0] = ekf->x_n.pData[0];

    ekf->h.pData[1] = ekf->x_n.pData[2];

    ekf->h.pData[2] = ekf->x_n.pData[4];

    len = snprintf(buffer, sizeof(buffer), "Observation (h): [%.4f, %.4f, %.4f]\r\n",
                   ekf->h.pData[0], ekf->h.pData[1], ekf->h.pData[2]);
    //HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

void observation_jacobian(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, (uint8_t*)"Observation Jacobian:\r\n", 23, HAL_MAX_DELAY);

    for (int i = 0; i < 18; i++) {
        ekf->dhdx.pData[i] = 0.0f;
    }
    ekf->dhdx.pData[0] = 1.0f;               // dh1/dx
    ekf->dhdx.pData[1 * 6 + 2] = 1.0f;       // dh2/dy
    ekf->dhdx.pData[2 * 6 + 4] = 1.0f;       // dh3/dz

    // Print Jacobian matrix
    char buffer[300];
    int len = snprintf(buffer, sizeof(buffer),
        "H matrix:\r\n"
        "[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\r\n"
        "[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\r\n"
        "[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\r\n",
        ekf->dhdx.pData[0], ekf->dhdx.pData[1], ekf->dhdx.pData[2], ekf->dhdx.pData[3], ekf->dhdx.pData[4], ekf->dhdx.pData[5],
        ekf->dhdx.pData[6], ekf->dhdx.pData[7], ekf->dhdx.pData[8], ekf->dhdx.pData[9], ekf->dhdx.pData[10], ekf->dhdx.pData[11],
        ekf->dhdx.pData[12], ekf->dhdx.pData[13], ekf->dhdx.pData[14], ekf->dhdx.pData[15], ekf->dhdx.pData[16], ekf->dhdx.pData[17]);
    //HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

/**
 * @brief This function computes the Kalman gain, which is a relative measure of trust between the measurements and the dynamics;
 *  in this case, the Kalman gain weighs the accelerometer against the GPS
 * @param ekf, the rocket's EKF struct
 * @return
 * @note A more robust equation for the Kalman gain is used per the suggestion of the rlabbe Kalman-and-Bayesian-Filters-in-Python GitHub tutorial
*/
arm_status kalman_gain(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    
    arm_status result = ARM_MATH_SUCCESS;
    arm_matrix_instance_f32 R_mat = ekf->R;

    arm_mat_init_f32(&HP, ekf->nz, ekf->nx, HP_f32);
    arm_mat_init_f32(&Ht, ekf->nx, ekf->nz, Ht_f32);
    arm_mat_init_f32(&HPHt, ekf->nz, ekf->nz, HPHt_f32);
    arm_mat_init_f32(&HPHtR, ekf->nz, ekf->nz, HPHtR_f32);
    arm_mat_init_f32(&HPHtRi, ekf->nz, ekf->nz, HPHtRi_f32);
    arm_mat_init_f32(&PHt, ekf->nx, ekf->nz, PHt_f32);
    arm_mat_init_f32(&K_new, ekf->nx, ekf->nz, K_f32);
    
    arm_matrix_instance_f32 H = ekf->dhdx;
    arm_matrix_instance_f32 P = ekf->P_n;

    HAL_UART_Transmit(huart, (uint8_t*)"Starting Kalman gain calculation...\r\n", 37, HAL_MAX_DELAY);

    //print_matrix("H matrix", &H, huart);
    //print_matrix("P matrix", &P, huart);
    //print_matrix("R matrix", &R_mat, huart);

    // Compute HP
    result = arm_mat_mult_f32(&H, &P, &HP);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in HP calculation\r\n", 25, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("HP matrix", &HP, huart);
    check_for_nan("HP matrix", &HP, huart);

    // Compute Ht
    result = arm_mat_trans_f32(&H, &Ht);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in Ht calculation\r\n", 25, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("Ht matrix", &Ht, huart);

    // Compute HPHt
    result = arm_mat_mult_f32(&HP, &Ht, &HPHt);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in HPHt calculation\r\n", 27, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("HPHt matrix", &HPHt, huart);

    // Compute HPHt + R
    result = arm_mat_add_f32(&HPHt, &R_mat, &HPHtR);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in HPHt + R calculation\r\n", 31, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("HPHt + R matrix", &HPHtR, huart);

    // Compute (HPHt + R)^-1
    result = arm_mat_inverse_f32(&HPHtR, &HPHtRi);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in (HPHt + R)^-1 calculation\r\n", 37, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("(HPHt + R)^-1 matrix", &HPHtRi, huart);

    // Compute PHt
    result = arm_mat_mult_f32(&P, &Ht, &PHt);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in PHt calculation\r\n", 26, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("PHt matrix", &PHt, huart);

    // Compute K
    result = arm_mat_mult_f32(&PHt, &HPHtRi, &K_new);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in K calculation\r\n", 24, HAL_MAX_DELAY);
        return result;
    }
    memcpy(ekf->K_n_data, K_new.pData, sizeof(float32_t) * ekf->nx * ekf->nz);
    arm_mat_init_f32(&ekf->K_n, ekf->nx, ekf->nz, ekf->K_n_data);
    //print_matrix("K (Kalman gain) matrix", &ekf->K_n, huart);
    //HAL_UART_Transmit(huart, (uint8_t*)"Kalman gain calculation complete.\r\n", 35, HAL_MAX_DELAY);
    return result;
}

/**
 * @brief Form a new state estimate in the "Update" step of the Kalman filter
 * @param ekf, the EKF struct
 * @return
 * @note
*/
// Updated update_state function with debug prints
void update_state(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    //HAL_UART_Transmit(huart, (uint8_t*)"Starting state update...\r\n", 26, HAL_MAX_DELAY);
    
    //print_matrix("Kalman gain K at start of update_state", &ekf->K_n, huart);
    check_for_nan("Kalman gain K at start", &ekf->K_n, huart);

    arm_status result = ARM_MATH_SUCCESS;
    float32_t innovation[ekf->nz];
    float32_t state_correction[ekf->nx];
    arm_matrix_instance_f32 innovation_mat = {ekf->nz, 1, innovation};
    arm_matrix_instance_f32 state_correction_mat = {ekf->nx, 1, state_correction};

    // Print H, P_, and R
    //print_matrix("Observation matrix H", &ekf->dhdx, huart);
    //print_matrix("Previous covariance P_", &ekf->P_prev, huart);
    //print_matrix("Measurement noise covariance R", &ekf->R, huart);

    // Print z, h, and x_prev
    //print_matrix("Measurement z", &ekf->z, huart);
    //print_matrix("Observation h", &ekf->h, huart);
    //print_matrix("Previous state x_prev", &ekf->x_prev, huart);

    // Compute innovation (z - h)
    result |= arm_mat_sub_f32(&ekf->z, &ekf->h, &innovation_mat);
    //print_matrix("Innovation (z - h)", &innovation_mat, huart);

    // Compute state correction K * (z - h)
    result |= arm_mat_mult_f32(&ekf->K_n, &innovation_mat, &state_correction_mat);
    //print_matrix("State correction K*(z - h)", &state_correction_mat, huart);

    // Update state: x_n = x_n + K * (z - h)
    result |= arm_mat_add_f32(&ekf->x_n, &state_correction_mat, &ekf->x_n);
    //print_matrix("Updated state x_n", &ekf->x_n, huart);

    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in state update\r\n", 23, HAL_MAX_DELAY);
    } else {
        //HAL_UART_Transmit(huart, (uint8_t*)"State update completed successfully\r\n", 37, HAL_MAX_DELAY);
    }

    //print_matrix("Kalman gain K at end of update_state", &ekf->K_n, huart);
    check_for_nan("Kalman gain K at end", &ekf->K_n, huart);
}

void update_covariance(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, (uint8_t*)"Starting covariance update...\r\n", 31, HAL_MAX_DELAY);

    //print_matrix("Kalman gain K at start of update_covariance", &ekf->K_n, huart);
    check_for_nan("Kalman gain K at start", &ekf->K_n, huart);

    arm_status result = ARM_MATH_SUCCESS;
    float32_t I_KH[ekf->nx * ekf->nx];
    float32_t temp[ekf->nx * ekf->nx];
    arm_matrix_instance_f32 I_KH_mat = {ekf->nx, ekf->nx, I_KH};
    arm_matrix_instance_f32 temp_mat = {ekf->nx, ekf->nx, temp};

    // Print H, P_, and R
    //print_matrix("Observation matrix H", &ekf->dhdx, huart);
    //print_matrix("Previous covariance P_", &ekf->P_prev, huart);
    //print_matrix("Measurement noise covariance R", &ekf->R, huart);

    // Compute KH
    result |= arm_mat_mult_f32(&ekf->K_n, &ekf->dhdx, &I_KH_mat);
    //print_matrix("KH", &I_KH_mat, huart);

    // Compute I - KH
    for (int i = 0; i < ekf->nx * ekf->nx; i++) {
        I_KH[i] = (i % (ekf->nx + 1) == 0) ? 1.0f - I_KH[i] : -I_KH[i];
    }
    //print_matrix("I - KH", &I_KH_mat, huart);

    // Compute (I - KH) * P_prev
    result |= arm_mat_mult_f32(&I_KH_mat, &ekf->P_n, &temp_mat);
    //print_matrix("(I - KH) * P_prev", &temp_mat, huart);

    // Compute P_n = (I - KH) * P_prev * (I - KH)'
    result |= arm_mat_mult_f32(&temp_mat, &I_KH_mat, &ekf->P_n);
    //print_matrix("P_n before adding KRK'", &ekf->P_n, huart);

    // Compute K * R
    float32_t KR[ekf->nx * ekf->nz];
    arm_matrix_instance_f32 KR_mat = {ekf->nx, ekf->nz, KR};
    result |= arm_mat_mult_f32(&ekf->K_n, &ekf->R, &KR_mat);
    //print_matrix("K * R", &KR_mat, huart);

    // Compute K * R * K'
    result |= arm_mat_mult_f32(&KR_mat, &(arm_matrix_instance_f32){ekf->nz, ekf->nx, ekf->K_n.pData}, &temp_mat);
    //print_matrix("K * R * K'", &temp_mat, huart);

    // Add K * R * K' to P_n
    result |= arm_mat_add_f32(&ekf->P_n, &temp_mat, &ekf->P_n);
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j <= i; j++) {
            float value = (ekf->P_n.pData[i * ekf->nx + j] + ekf->P_n.pData[j * ekf->nx + i]) * 0.5;
            ekf->P_n.pData[i * ekf->nx + j] = ekf->P_n.pData[j * ekf->nx + i] = value;
        }
    }
    //print_matrix("Final P_n", &ekf->P_n, huart);

    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in covariance update\r\n", 29, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(huart, (uint8_t*)"Covariance update completed successfully\r\n", 42, HAL_MAX_DELAY);
    }

    //print_matrix("Kalman gain K at end of update_covariance", &ekf->K_n, huart);
    check_for_nan("Kalman gain K at end", &ekf->K_n, huart);
}

/**
 * @brief Form the state transition function for the EKF, which propagates the states forward in time based on measurements.
 * This implementation takes the velocity in the body frame and rotates it into the velocity in the flat Earth frame to integrate the
 * flat Earth position
 * @param ekf, the EKF struct
*/
void state_transition_function(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, UART_HandleTypeDef *huart) {
    float dt = ekf->time_step;

    //Extract the quaternion and convert it to the quaternion that rotates body frame into the flat Earth frame
    float32_t q0 = rocket_atd->q_current_s;
    float32_t q1 = -1.0*rocket_atd->q_current_x;
    float32_t q2 = -1.0*rocket_atd->q_current_y;
    float32_t q3 = -1.0*rocket_atd->q_current_z; 

    float32_t q_rot_mat[3][3] = {{(2*(q0*q0 + q1*q1) - 1), (2*(q1*q2 - q0*q3)), (2*(q1*q3 + q0*q2))},
                                                  {(2*(q1*q2 + q0*q3)), (2*(q0*q0 + q2*q2) - 1), (2*(q2*q3 - q0*q1))},
                                                  {(2*(q1*q3 - q0*q2)), (2*(q2*q3 + q0*q1)), (2*(q0*q0 + q3*q3) - 1)}};

    float32_t vel_body_x = ekf->x_n.pData[1];
    float32_t vel_body_y = ekf->x_n.pData[3];
    float32_t vel_body_z = ekf->x_n.pData[5];

    float32_t vel_flat_x = q_rot_mat[0][0]*vel_body_x + q_rot_mat[0][1]*vel_body_y + q_rot_mat[0][2]*vel_body_z;
    float32_t vel_flat_y = q_rot_mat[1][0]*vel_body_x + q_rot_mat[1][1]*vel_body_y + q_rot_mat[1][2]*vel_body_z;
    float32_t vel_flat_z = q_rot_mat[2][0]*vel_body_x + q_rot_mat[2][1]*vel_body_y + q_rot_mat[2][2]*vel_body_z;

    ekf->f.pData[0] = vel_flat_x * dt + ekf->x_n.pData[0];
    ekf->f.pData[1] = ekf->accelerometer[0] * dt + ekf->x_n.pData[1];
    ekf->f.pData[2] = vel_flat_y * dt + ekf->x_n.pData[2];
    ekf->f.pData[3] = ekf->accelerometer[1] * dt + ekf->x_n.pData[3];
    ekf->f.pData[4] = vel_flat_z * dt + ekf->x_n.pData[4];
    ekf->f.pData[5] = ekf->accelerometer[2] * dt + ekf->x_n.pData[5];
}


void state_transition_jacobian(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, UART_HandleTypeDef *huart) {
    //HAL_UART_Transmit(huart, (uint8_t*)"Starting state transition Jacobian calculation...\r\n", 52, HAL_MAX_DELAY);

    float32_t dfdx_new[ekf->nx * ekf->nx];
    memset(dfdx_new, 0.0, sizeof(dfdx_new));

    //Extract the quaternion and convert it to the quaternion that rotates body frame into the flat Earth frame
    float32_t q0 = rocket_atd->q_current_s;
    float32_t q1 = -1.0*rocket_atd->q_current_x;
    float32_t q2 = -1.0*rocket_atd->q_current_y;
    float32_t q3 = -1.0*rocket_atd->q_current_z; 

    float dt = ekf->time_step;

    // x position, velocity
    dfdx_new[0 * ekf->nx + 0] = 1.0;
    dfdx_new[0 * ekf->nx + 1] = dt*(2*q0*q0 + 2*q1*q1 - 1);
    dfdx_new[0 * ekf->nx + 3] = dt*(2*q0*q3 + 2*q1*q2);
    dfdx_new[0 * ekf->nx + 5] = -dt*(2*q0*q2 - 2*q1*q3);

    dfdx_new[1 * ekf->nx + 1] = 1.0;


    // y position, velocity
    dfdx_new[2 * ekf->nx + 1] = -dt*(2*q0*q3 - 2*q1*q2);
    dfdx_new[2 * ekf->nx + 2] = 1.0;
    dfdx_new[2 * ekf->nx + 3] = dt*(2*q0*q0 + 2*q2*q2 - 1);
    dfdx_new[2 * ekf->nx + 5] = dt*(2*q0*q1 + 2*q2*q3);

    dfdx_new[3 * ekf->nx + 3] = 1.0;

    // z position, velocity
    dfdx_new[4 * ekf->nx + 1] = dt*(2*q0*q2 + 2*q1*q3);
    dfdx_new[4 * ekf->nx + 3] = -dt*(2*q0*q1 - 2*q2*q3);
    dfdx_new[4 * ekf->nx + 4] = 1.0;
    dfdx_new[4 * ekf->nx + 5] = dt*(2*q0*q0 + 2*q3*q3 - 1);

    dfdx_new[5 * ekf->nx + 5] = 1.0;

    // Copy the new Jacobian data to ekf->dfdx_data
    memcpy(ekf->dfdx_data, dfdx_new, sizeof(float32_t) * ekf->nx * ekf->nx);
    
    // Reinitialize dfdx with the updated data
    arm_mat_init_f32(&ekf->dfdx, ekf->nx, ekf->nx, ekf->dfdx_data);

    //HAL_UART_Transmit(huart, (uint8_t*)"State transition Jacobian:\r\n", 29, HAL_MAX_DELAY);
    //print_matrix("F (State Transition Jacobian)", &ekf->dfdx, huart);

    //HAL_UART_Transmit(huart, (uint8_t*)"State transition Jacobian calculation completed.\r\n", 51, HAL_MAX_DELAY);
}

/**
 * @brief Predicts next state based on current state and dynamics
 * @param ekf Pointer to the flight EKF structure
 * @param huart Pointer to UART handle for debug output
 * @details Applies state transition function to predict next state
 */
void predict_state(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    //HAL_UART_Transmit(huart, (uint8_t*)"Starting state prediction...\r\n", 30, HAL_MAX_DELAY);

    //print_matrix("Current state (x_n)", &ekf->x_n, huart);

    // Use state_transition_function to update x_next
    ekf->x_n = ekf->f;

    //print_matrix("Predicted state (x_next)", &ekf->x_next, huart);

    //HAL_UART_Transmit(huart, (uint8_t*)"State prediction completed.\r\n", 29, HAL_MAX_DELAY);
}

void predict_covariance(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    //HAL_UART_Transmit(huart, (uint8_t*)"Starting covariance prediction...\r\n", 35, HAL_MAX_DELAY);

    arm_status result = ARM_MATH_SUCCESS;

    arm_matrix_instance_f32 P = ekf->P_n;
    arm_matrix_instance_f32 F = ekf->dfdx;
    arm_matrix_instance_f32 Q_mat = ekf->Q;

    //print_matrix("Current covariance (P)", &P, huart);
    //print_matrix("State transition Jacobian (F)", &F, huart);
    //print_matrix("Process noise covariance (Q)", &Q_mat, huart);

    float32_t Ft_f32[ekf->nx * ekf->nx];
    float32_t FP_f32[ekf->nx * ekf->nx];
    float32_t FPFt_f32[ekf->nx * ekf->nx];
    float32_t P_future_f32[ekf->nx * ekf->nx];

    arm_matrix_instance_f32 Ft, FP, FPFt, P_future;

    arm_mat_init_f32(&Ft, ekf->nx, ekf->nx, Ft_f32);
    arm_mat_init_f32(&FP, ekf->nx, ekf->nx, FP_f32);
    arm_mat_init_f32(&FPFt, ekf->nx, ekf->nx, FPFt_f32);
    arm_mat_init_f32(&P_future, ekf->nx, ekf->nx, P_future_f32);

    // Compute FP
    result |= arm_mat_mult_f32(&F, &P, &FP);
    //print_matrix("FP", &FP, huart);

    // Compute Ft
    result |= arm_mat_trans_f32(&F, &Ft);
    //print_matrix("F transpose (Ft)", &Ft, huart);

    // Compute FPFt
    result |= arm_mat_mult_f32(&FP, &Ft, &FPFt);
    //print_matrix("FPFt", &FPFt, huart);

    // Compute P_next = FPFt + Q
    result |= arm_mat_add_f32(&FPFt, &Q_mat, &P_future);
    //print_matrix("P_future before regularization", &P_future, huart);

    // Copy P_future data to P_next_data
    memcpy(ekf->P_n_data, P_future.pData, sizeof(float32_t) * ekf->nx * ekf->nx);
    
    // Reinitialize P_next with the updated data
    arm_mat_init_f32(&ekf->P_n, ekf->nx, ekf->nx, ekf->P_n_data);

    //print_matrix("Predicted covariance (P_next)", &ekf->P_next, huart);

    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in covariance prediction calculations\r\n", 45, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(huart, (uint8_t*)"Covariance prediction completed successfully.\r\n", 47, HAL_MAX_DELAY);
    }
}

///**
// * @brief This function represents the passage of one (variable) time step.
// * @param ekf, the EKF struct
// * @return
// * @note
//*/
//void acknowledge_time_passed(ExtKalmanFilter *ekf){
//    ekf->x_prev = ekf->x_n;
//   ekf->P_prev = ekf->P_next;
//}

/**
 * @brief Converts GPS coordinates from WGS84 to a local flat Earth frame
 * @param sensors Pointer to sensors structure containing GPS readings
 * @param ekf Pointer to the flight EKF structure
 * @param ground Flag indicating if conversion is for ground reference (1) or flight (0)
 * @details Transforms GPS coordinates through ECEF to ENU frame, stores results in ekf->gps_flat
 */
void GPS2Flat(Sensors *sensors, ExtKalmanFilter *ekf, uint8_t ground) {
    // Converts from WGS84 to ECEF

    // Define intermediate variables
    double WGS84_E = 0.08181;

    double clat = cos(sensors->gps_x * (PI / 180.0));
    double slat = sin(sensors->gps_x * (PI / 180.0));
    double clon = cos(sensors->gps_y * (PI / 180.0));
    double slon = sin(sensors->gps_y * (PI / 180.0));

    double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);
    double x_ecef = (N + sensors->gps_z) * clat * clon;
    double y_ecef = (N + sensors->gps_z) * clat * slon;
    double z_ecef = (N * (1.0 - WGS84_E * WGS84_E) + sensors->gps_z) * slat;
    if (ground) {
        sensors->gps_offset_x = x_ecef;
        sensors->gps_offset_y = y_ecef;
        sensors->gps_offset_z = z_ecef;
    }
    double dx = x_ecef - sensors->gps_offset_x;
    double dy = y_ecef - sensors->gps_offset_y;
    double dz = z_ecef - sensors->gps_offset_z;
    double x_enu = -slon*dx  + clon*dy;
    double y_enu = -slat*clon*dx - slat*slon*dy + clat*dz;
    double z_enu = clat*clon*dx + clat*slon*dy + slat*dz;
    if (ground) {
        ekf->gps_flat[0] = sensors->gps_offset_x;
        ekf->gps_flat[1] = sensors->gps_offset_y;
        ekf->gps_flat[2] = sensors->gps_offset_z;
    } else {
        ekf->gps_flat[0] = z_enu - ekf->launch_gps[0];  // Subtract launch position
        ekf->gps_flat[1] = y_enu - ekf->launch_gps[1];
        ekf->gps_flat[2] = -1.0 * x_enu - ekf->launch_gps[2];
    }
}

/**
 * @brief Updates the extended Kalman filter with current sensor readings and attitude
 * @param ekf Pointer to the flight EKF structure
 * @param rocket_atd Pointer to rocket attitude structure
 * @param sensors Pointer to sensors structure with current readings
 * @details Transforms sensor readings into body frame, computes gravity vector, 
 *          and applies coriolis corrections for accelerometer readings
 */
void update_ekf(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors* sensors) {
    ekf->gps[0] = ekf->gps_flat[0];
    ekf->gps[1] = ekf->gps_flat[1];
    ekf->gps[2] = ekf->gps_flat[2];
    ekf->gyro[0] = (sensors->gyro_x - sensors->gyro_bias_x);
    ekf->gyro[1] = (sensors->gyro_y - sensors->gyro_bias_y);
    ekf->gyro[2] = (sensors->gyro_z - sensors->gyro_bias_z);

    //Extract the quaternion that rotates flat Earth frame to body frame
    float32_t q0 = rocket_atd->q_current_s;
    float32_t q1 = rocket_atd->q_current_x;
    float32_t q2 = rocket_atd->q_current_y;
    float32_t q3 = rocket_atd->q_current_z; 

    float32_t q_rot_mat[3][3] = {{(2*(q0*q0 + q1*q1) - 1), (2*(q1*q2 - q0*q3)), (2*(q1*q3 + q0*q2))},
                                                  {(2*(q1*q2 + q0*q3)), (2*(q0*q0 + q2*q2) - 1), (2*(q2*q3 - q0*q1))},
                                                  {(2*(q1*q3 - q0*q2)), (2*(q2*q3 + q0*q1)), (2*(q0*q0 + q3*q3) - 1)}};

    //Compute the gravity vector in the body frame
    float32_t g_bx = q_rot_mat[0][0]*(-9.81);
    float32_t g_by = q_rot_mat[1][0]*(-9.81);
    float32_t g_bz = q_rot_mat[2][0]*(-9.81);

    float32_t wx = ekf->gyro[0];
    float32_t wy = ekf->gyro[1];
    float32_t wz = ekf->gyro[2];

    //Assumes no angular acceleration
    float32_t coriolis_mat[3][3] = {{(-1*(wy*wy) - wz*wz), (wx*wy), (wx*wz)},
                                    {(wx*wy), (-1*(wx*wx) - wz*wz), (wy*wz)},
                                    {(wx*wz), (wy*wz), (-1*(wx*wx) - wy*wy)}};


    //Accelerometer readings: bias removed, transformed to be about center of mass, with gravity in body frame removed
    ekf->accelerometer[0] = (sensors->accel_x - sensors->accel_bias_x) + coriolis_mat[0][0]*ekf->c[0] + coriolis_mat[0][1]*ekf->c[1] + coriolis_mat[0][2]*ekf->c[2] + g_bx;
    ekf->accelerometer[1] = (sensors->accel_y - sensors->accel_bias_y) + coriolis_mat[1][0]*ekf->c[0] + coriolis_mat[1][1]*ekf->c[1] + coriolis_mat[1][2]*ekf->c[2] + g_by;
    ekf->accelerometer[2] = (sensors->accel_z - sensors->accel_bias_z) + coriolis_mat[2][0]*ekf->c[0] + coriolis_mat[2][1]*ekf->c[1] + coriolis_mat[2][2]*ekf->c[2] + g_bz;

    //ekf->accelerometer[0] = -9.81 + ((sensors->accel_x + sensors->accel_bias_x));
    //ekf->accelerometer[1] = (sensors->accel_y - sensors->accel_bias_y);
    //ekf->accelerometer[2] = (sensors->accel_z - sensors->accel_bias_z);
}

/**
 * @brief Runs one complete iteration of the flight EKF
 * @param ekf Pointer to the flight EKF structure
 * @param rocket_atd Pointer to rocket attitude structure
 * @param sensors Pointer to sensors structure
 * @param huart Pointer to UART handle for debug output
 * @param ekf_initialized Flag indicating if EKF has been initialized
 * @details Performs prediction and update steps of the EKF, processes GPS measurements
 */
void run_ekf(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors *sensors, UART_HandleTypeDef *huart, int ekf_initialized) {
    char buffer[256];
    int len;
    len = snprintf(buffer, sizeof(buffer), "State before update:\r\n");
    //HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    for (int i = 0; i < 6; i++) {
        len = snprintf(buffer, sizeof(buffer), "x[%d]: %f\r\n", i, ekf->x_n.pData[i]);
        //HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }

    predict_step(ekf, rocket_atd, huart);

    make_measurement(ekf, huart);
    GPS2Flat(sensors, ekf, 0);
    //if (GPS measurement is valid) {
    update_step(ekf, huart);
    //}

    //acknowledge_time_passed(ekf);
    //arm_matrix_instance_f32 curr_state = ekf->x_n;
}

/**
 * @brief Performs the prediction step of the EKF
 * @param ekf Pointer to the flight EKF structure
 * @param rocket_atd Pointer to rocket attitude structure
 * @param huart Pointer to UART handle for debug output
 * @details Executes state transition, Jacobian computation, and covariance prediction
 */
void predict_step(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, UART_HandleTypeDef *huart) {
    state_transition_function(ekf, rocket_atd, huart);
    state_transition_jacobian(ekf, rocket_atd, huart);
    predict_state(ekf, huart);
    predict_covariance(ekf, huart);
}

/**
 * @brief Performs the update step of the EKF
 * @param ekf Pointer to the flight EKF structure
 * @param huart Pointer to UART handle for debug output
 * @details Computes Kalman gain and updates state and covariance estimates using measurements
 */
void update_step(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart){
    observation_function(ekf, huart);
    observation_jacobian(ekf, huart);
    kalman_gain(ekf, huart);
    update_state(ekf, huart);
    update_covariance(ekf, huart);
}