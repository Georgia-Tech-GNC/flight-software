/**
 * @file ekf.c
 * @author Patrick Barry, Kanav Chugh
 * @brief Source file for in-flight EKF for position, velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "ground_ekf.h"
#include "gekf_constants.h"

/**
 * @brief Converts GPS coordinates to flat Earth frame for ground-based navigation
 * @param sensors Pointer to sensors structure containing GPS readings
 * @param ekf Pointer to the ground EKF structure
 * @param ground Flag indicating if conversion is for ground reference
 * @details Similar to GPS2Flat but specifically for ground operations
 */
void GPS2FlatGround(Sensors *sensors, GroundExtKalmanFilter *ekf, uint8_t ground) {
    // Converts from WGS84 to ECEF

    // Define intermediate variables
    double WGS84_E = 0.08181;

    // Convert latitude and longitude from degrees to radians
    double clat = cos(sensors->gps_x * (PI / 180.0));
    double slat = sin(sensors->gps_x * (PI / 180.0));
    double clon = cos(sensors->gps_y * (PI / 180.0));
    double slon = sin(sensors->gps_y * (PI / 180.0));

    // Convert LLA to ECEF
    double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);
    double x_ecef = (N + sensors->gps_z) * clat * clon;
    double y_ecef = (N + sensors->gps_z) * clat * slon;
    double z_ecef = (N * (1.0 - WGS84_E * WGS84_E) + sensors->gps_z) * slat;
    if (ground) {
        sensors->gps_offset_x = x_ecef;
        sensors->gps_offset_y = y_ecef;
        sensors->gps_offset_z = z_ecef;
    }

    // Convert ECEF to ENU
    // xr, yr, zr - ECEF coordinates of your ENU origin
    // x_ecef, y_ecef, z_ecef - rocket's position in ENU
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
        ekf->gps_flat[0] = z_enu;
        ekf->gps_flat[1] = y_enu;
        ekf->gps_flat[2] = -1.0 * x_enu;
    }
}


void initialize_ekf_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart, Sensors *sensors, uint16_t nz){
    //EKF state vector is x, vx, y, vy, z, vz

    ekf->time_step = 0.02;

    ekf->nx = 6;
    ekf->nu = 0;
    ekf->nz = nz;

    arm_mat_init_f32(&ekf->G, ekf->nu, ekf->nu, G_f32);
    if (ekf->nz == 6) {
        arm_mat_init_f32(&ekf->R, ekf->nz, ekf->nz, R_f32_ground);
        arm_mat_init_f32(&ekf->dhdx, ekf->nz, ekf->nx, dhdx_f32_ground);
        arm_mat_init_f32(&ekf->dfdx, ekf->nx, ekf->nx, dfdx_f32_ground);
        arm_mat_init_f32(&ekf->Q, ekf->nx, ekf->nx, Q_f32_ground);
        arm_mat_init_f32(&ekf->P_prev, ekf->nx, ekf->nx, P_init_ground);
    }

    arm_mat_init_f32(&ekf->K_n, ekf->nx, ekf->nz, K_f32_ground);

    arm_mat_init_f32(&ekf->x_prev, ekf->nx, 1, x_p);
    arm_mat_init_f32(&ekf->x_n, ekf->nx, 1, x_init_ground);
    arm_mat_init_f32(&ekf->x_next, ekf->nx, 1, x_f);

    arm_mat_init_f32(&ekf->P_n, ekf->nx, ekf->nx, P_init_ground);
    arm_mat_init_f32(&ekf->P_next, ekf->nx, ekf->nx, P_f);

    arm_mat_init_f32(&ekf->f, ekf->nx, 1, f_f32_ground);
    arm_mat_init_f32(&ekf->h, ekf->nz, 1, h_f32_ground);
    //arm_mat_init_f32(&ekf->z, ekf->nz, 1, z_f32);
    //Compute process noise matrix 
    //print_matrix("Process Noise Covariance", &ekf->Q, huart);

    ekf->gps[0] = 0.0;
    ekf->gps[1] = 0.0;
    ekf->gps[2] = 0.0;

    ekf->gps_origin[0] = ekf->gps[0];
    ekf->gps_origin[1] = ekf->gps[1];
    ekf->gps_origin[2] = ekf->gps[2];

    ekf->accelerometer[0] = 0;
    ekf->accelerometer[1] = 0.0;
    ekf->accelerometer[2] = 9.8;

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
void make_measurement_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, (uint8_t*)"Starting make_measurement...\r\n", 30, HAL_MAX_DELAY);
    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "GPS Measurements: [%.4f, %.4f, %.4f]\r\n", 
                       ekf->gps[0], ekf->gps[1], ekf->gps[2]);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    float32_t z_new_f32[ekf->nz];
    if (ekf->nz == 6) {
        z_new_f32[0] = ekf->accelerometer[0];
        z_new_f32[1] = ekf->accelerometer[1];
        z_new_f32[2] = ekf->accelerometer[2];
        z_new_f32[3] = ekf->gyro[0];
        z_new_f32[4] = ekf->gyro[1];
        z_new_f32[5] = ekf->gyro[2];
    } else {
        z_new_f32[0] = ekf->gps[0];
        z_new_f32[1] = ekf->gps[1];
        z_new_f32[2] = ekf->gps[2];
    }
    arm_matrix_instance_f32 z_new;
    arm_mat_init_f32(&z_new, ekf->nz, 1, z_new_f32);
    //print_matrix("New measurement (z)", &z_new, huart);
    memcpy(ekf->z_data, z_new_f32, sizeof(float32_t) * ekf->nz);
    arm_mat_init_f32(&ekf->z, ekf->nz, 1, ekf->z_data);
    //print_matrix("Final measurement (z) in EKF", &ekf->z, huart);
    HAL_UART_Transmit(huart, (uint8_t*)"make_measurement completed.\r\n", 29, HAL_MAX_DELAY);
}




/**
 * @brief This function computes the Kalman gain, which is a relative measure of trust between the measurements and the dynamics;
 *  in this case, the Kalman gain weighs the accelerometer against the GPS
 * @param ekf, the rocket's EKF struct
 * @return
 * @note A more robust equation for the Kalman gain is used per the suggestion of the rlabbe Kalman-and-Bayesian-Filters-in-Python GitHub tutorial
*/
arm_status kalman_gain_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    arm_status result = ARM_MATH_SUCCESS;
    
    arm_matrix_instance_f32 R_mat = ekf->R;
    arm_mat_init_f32(&HP_ground, ekf->nz, ekf->nx, HP_f32_ground);
    arm_mat_init_f32(&Ht_ground, ekf->nx, ekf->nz, Ht_f32_ground);
    arm_mat_init_f32(&HPHt_ground, ekf->nz, ekf->nz, HPHt_f32_ground);
    arm_mat_init_f32(&HPHtR_ground, ekf->nz, ekf->nz, HPHtR_f32_ground);
    arm_mat_init_f32(&HPHtRi_ground, ekf->nz, ekf->nz, HPHtRi_f32_ground);
    arm_mat_init_f32(&PHt_ground, ekf->nx, ekf->nz, PHt_f32_ground);
    arm_mat_init_f32(&K_new_ground, ekf->nx, ekf->nz, K_f32_ground);


    arm_matrix_instance_f32 H = ekf->dhdx;
    arm_matrix_instance_f32 P = ekf->P_prev;

    HAL_UART_Transmit(huart, (uint8_t*)"Starting Kalman gain calculation...\r\n", 37, HAL_MAX_DELAY);

    //print_matrix("H matrix", &H, huart);
    //print_matrix("P matrix", &P, huart);
    //print_matrix("R matrix", &R_mat, huart);

    // Compute HP
    result = arm_mat_mult_f32(&H, &P, &HP_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in HP calculation\r\n", 25, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("HP matrix", &HP, huart);
    check_for_nan("HP matrix", &HP_ground, huart);

    // Compute Ht
    result = arm_mat_trans_f32(&H, &Ht_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in Ht calculation\r\n", 25, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("Ht matrix", &Ht, huart);

    // Compute HPHt
    result = arm_mat_mult_f32(&HP_ground, &Ht_ground, &HPHt_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in HPHt calculation\r\n", 27, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("HPHt matrix", &HPHt, huart);

    // Compute HPHt + R
    result = arm_mat_add_f32(&HPHt_ground, &R_mat, &HPHtR_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in HPHt + R calculation\r\n", 31, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("HPHt + R matrix", &HPHtR, huart);

    // Compute (HPHt + R)^-1
    result = arm_mat_inverse_f32(&HPHtR_ground, &HPHtRi_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in (HPHt + R)^-1 calculation\r\n", 37, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("(HPHt + R)^-1 matrix", &HPHtRi, huart);

    // Compute PHt
    result = arm_mat_mult_f32(&P, &Ht_ground, &PHt_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in PHt calculation\r\n", 26, HAL_MAX_DELAY);
        return result;
    }
    //print_matrix("PHt matrix", &PHt, huart);

    // Compute K
    result = arm_mat_mult_f32(&PHt_ground, &HPHtRi_ground, &K_new_ground);
    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in K calculation\r\n", 24, HAL_MAX_DELAY);
        return result;
    }
    memcpy(ekf->K_n_data, K_new_ground.pData, sizeof(float32_t) * ekf->nx * ekf->nz);
    arm_mat_init_f32(&ekf->K_n, ekf->nx, ekf->nz, ekf->K_n_data);
    //print_matrix("K (Kalman gain) matrix", &ekf->K_n, huart);
    HAL_UART_Transmit(huart, (uint8_t*)"Kalman gain calculation complete.\r\n", 35, HAL_MAX_DELAY);
    return result;
}
/**
 * @brief Form a new state estimate in the "Update" step of the Kalman filter
 * @param ekf, the EKF struct
 * @return
 * @note
*/
// Updated update_state function with debug prints
void update_state_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, (uint8_t*)"Starting state update...\r\n", 26, HAL_MAX_DELAY);
    
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

    // Update state: x_n = x_prev + K * (z - h)
    result |= arm_mat_add_f32(&ekf->x_prev, &state_correction_mat, &ekf->x_n);
    //print_matrix("Updated state x_n", &ekf->x_n, huart);

    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in state update\r\n", 23, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(huart, (uint8_t*)"State update completed successfully\r\n", 37, HAL_MAX_DELAY);
    }

    //print_matrix("Kalman gain K at end of update_state", &ekf->K_n, huart);
    check_for_nan("Kalman gain K at end", &ekf->K_n, huart);
}

/**
 * @brief Updates error covariance for ground EKF
 * @param ekf Pointer to the ground EKF structure
 * @param huart Pointer to UART handle for debug output
 * @details Implements covariance update using Joseph form: P = (I-KH)P(I-KH)' + KRK'
 */
void update_covariance_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
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
    result |= arm_mat_mult_f32(&I_KH_mat, &ekf->P_prev, &temp_mat);
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
        // Ensure positive diagonal elements
        if (ekf->P_n.pData[i * ekf->nx + i] <= 0) {
            ekf->P_n.pData[i * ekf->nx + i] = 1e-6;
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
 * @brief Predicts error covariance for ground EKF
 * @param ekf Pointer to the ground EKF structure
 * @param huart Pointer to UART handle for debug output
 * @details Implements covariance prediction: P = FPF' + Q
 */
void predict_covariance_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, (uint8_t*)"Starting covariance prediction...\r\n", 35, HAL_MAX_DELAY);

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
    memcpy(ekf->P_next_data, P_future.pData, sizeof(float32_t) * ekf->nx * ekf->nx);
    
    // Reinitialize P_next with the updated data
    arm_mat_init_f32(&ekf->P_next, ekf->nx, ekf->nx, ekf->P_next_data);

    //print_matrix("Predicted covariance (P_next)", &ekf->P_next, huart);

    if (result != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Error in covariance prediction calculations\r\n", 45, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(huart, (uint8_t*)"Covariance prediction completed successfully.\r\n", 47, HAL_MAX_DELAY);
    }
}

/**
 * @brief This function represents the passage of one (variable) time step.
 * @param ekf, the EKF struct
 * @return
 * @note
*/
void acknowledge_time_passed_ground(GroundExtKalmanFilter *ekf){
    ekf->x_prev = ekf->x_next;
    ekf->P_prev = ekf->P_n;
}

/**
 * @brief Updates the ground EKF with current sensor readings
 * @param ekf Pointer to the ground EKF structure
 * @param sensors Pointer to sensors structure
 * @details Updates internal state with current sensor readings, applies bias corrections
 */
void update_ekf_ground(GroundExtKalmanFilter *ekf, Sensors* sensors) {
    ekf->gps[0] = ekf->gps_flat[0];
    ekf->gps[1] = ekf->gps_flat[1];
    ekf->gps[2] = ekf->gps_flat[2];
    ekf->accelerometer[0] = -9.81 + (sensors->accel_x + sensors->accel_bias_x);
    ekf->accelerometer[1] = sensors->accel_y - sensors->accel_bias_y;
    ekf->accelerometer[2] = sensors->accel_z - sensors->accel_bias_z;
    ekf->gyro[0] = sensors->gyro_x - sensors->gyro_bias_x;
    ekf->gyro[1] = sensors->gyro_y - sensors->gyro_bias_y;
    ekf->gyro[2] = sensors->gyro_z - sensors->gyro_bias_z;
}
