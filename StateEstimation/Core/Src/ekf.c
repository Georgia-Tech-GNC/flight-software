/**
 * @file ekf.c
 * @author Patrick Barry
 * @brief Source file for in-flight EKF for position, velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "ekf.h"

//See https://github.com/ramblinrocketclub/flight-computer/blob/master/Core/Src/rocket.c for initializing process
arm_status initialize_ekf(ExtKalmanFilter *ekf, uint16_t num_states, uint16_t num_inputs, uint16_t num_measurements, 
float32_t *dfdx_f32, float32_t *dhdx_f32, float32_t *G_f32, float32_t *Q_f32, float32_t *K_f32, float32_t *R_f32,
float32_t *x_p, float32_t *P_p, float32_t *x_init, float32_t *P_init, float32_t *x_f, float32_t *P_f,
float32_t *f_f32, float32_t *h_f32, float32_t *z_f32, float32_t *state_stddevs){

    arm_status result = ARM_MATH_SUCCESS;

    float32_t wnT_f32[num_states];

    ekf->nx = num_states;
    ekf->nu = num_inputs;
    ekf->nz = num_measurements;

    arm_mat_init_f32(&ekf->dfdx, ekf->nx, ekf->nx, dfdx_f32);
    arm_mat_init_f32(&ekf->G, ekf->nu, ekf->nu, G_f32);
    arm_mat_init_f32(&ekf->Q, ekf->nx, ekf->nx, Q_f32);
    arm_mat_init_f32(&ekf->R, ekf->nz, ekf->nz, R_f32);
    arm_mat_init_f32(&ekf->dhdx, ekf->nz, ekf->nx, dhdx_f32);
    arm_mat_init_f32(&ekf->K_n, ekf->nx, ekf->nz, K_f32);

    arm_mat_init_f32(&ekf->x_prev, ekf->nx, 1, x_p);
    arm_mat_init_f32(&ekf->x_n, ekf->nx, 1, x_init);
    arm_mat_init_f32(&ekf->x_next, ekf->nx, 1, x_f);

    arm_mat_init_f32(&ekf->P_prev, ekf->nx, ekf->nx, P_p);
    arm_mat_init_f32(&ekf->P_n, ekf->nx, ekf->nx, P_init);
    arm_mat_init_f32(&ekf->P_next, ekf->nx, ekf->nx, P_f);

    arm_mat_init_f32(&ekf->f, ekf->nx, 1, f_f32);
    arm_mat_init_f32(&ekf->h, ekf->nz, 1, h_f32);
    arm_mat_init_f32(&ekf->z, ekf->nz, 1, z_f32);

    arm_matrix_instance_f32 wn;
    arm_matrix_instance_f32 wnT;

    arm_mat_init_f32(&wn, ekf->nx, 1, state_stddevs);
    arm_mat_init_f32(&wnT, 1, ekf->nx, wnT_f32);

    result |= arm_mat_trans_f32(&wn, &wnT);
    result |= arm_mat_mult_f32(&wn, &wnT, &ekf->Q); //Compute process noise matrix

    ekf->gps[0] = 0.0;
    ekf->gps[1] = 0.0;
    ekf->gps[2] = 0.0;

    ekf->accelerometer[0] = 0.0;
    ekf->accelerometer[1] = 0.0;
    ekf->accelerometer[2] = 0.0;

    ekf->barometer = 0.0;

    ekf->time_step = 0.02; //initial time step

    return result;
}

int64_t currentTimeMillis(){
    struct timeval time;
    gettimeofday(&time, NULL);
    int64_t s1 = (int64_t)(time.tv_sec) * 1000;
    int64_t s2 = (time.tv_usec / 1000);
    return s1 + s2;
}

void update_time_step(ExtKalmanFilter *ekf){

    int64_t curr_time_millis = currentTimeMillis();
    float32_t dt_millis = (float32_t)(curr_time_millis - ekf->prev_time_millis);
    ekf->time_step = dt_millis / 1000.0;
    ekf->prev_time_millis = curr_time_millis;

}

void observation_function(ExtKalmanFilter *ekf){

    float h_new_data[ekf->nz];
    
    h_new_data[0] = ekf->x_n.pData[0];
    h_new_data[1] = ekf->x_n.pData[2];
    h_new_data[2] = ekf->x_n.pData[4];

    arm_matrix_instance_f32 h_new = {ekf->nz, 1, h_new_data};
    ekf->h = h_new;

}

void observation_jacobian(ExtKalmanFilter *ekf){

    float32_t dhdx_new[ekf->nz][ekf->nx];
    memset(dhdx_new, 0.0, sizeof dhdx_new);

    //x position, velocity
    dhdx_new[0][0] = 1.0;
    dhdx_new[1][2] = 1.0;
    dhdx_new[2][4] = 1.0;

    arm_mat_init_f32(&ekf->dhdx, ekf->nx, ekf->nx, dhdx_new);

}

void kalman_gain(ExtKalmanFilter *ekf){

    //K = PHt(HPHt + R)i

    //1. Find HP
    //2. Find Ht
    //3. Find HPHt
    //4. Find HPHt + R
    //5. Find (HPHt + R)^-1
    //6. Find PHt
    //7. Find K = step6*step5;

    arm_status result = ARM_MATH_SUCCESS;

    float32_t HP_f32[ekf->nz * ekf->nx];
    float32_t Ht_f32[ekf->nx * ekf->nz];
    float32_t HPHt_f32[ekf->nz * ekf->nz];
    
    arm_matrix_instance_f32 R_mat = ekf->R;
    float32_t HPHtR_f32[ekf->nz * ekf->nz];
    float32_t HPHtRi_f32[ekf->nz * ekf->nz];

    float32_t PHt_f32[ekf->nx * ekf->nz];
    float32_t K_f32[ekf->nx * ekf->nz];

    arm_matrix_instance_f32 HP;
    arm_matrix_instance_f32 Ht;
    arm_matrix_instance_f32 HPHt;
    arm_matrix_instance_f32 HPHtR;
    arm_matrix_instance_f32 HPHtRi;
    arm_matrix_instance_f32 PHt;
    arm_matrix_instance_f32 K_new;

    arm_mat_init_f32(&HP, ekf->nz, ekf->nx, HP_f32);
    arm_mat_init_f32(&Ht, ekf->nx, ekf->nz, Ht_f32);
    arm_mat_init_f32(&HPHt, ekf->nz, ekf->nz, HPHt_f32);
    arm_mat_init_f32(&HPHtR, ekf->nz, ekf->nz, HPHtR_f32);
    arm_mat_init_f32(&HPHtRi, ekf->nz, ekf->nz, HPHtRi_f32);
    arm_mat_init_f32(&PHt, ekf->nx, ekf->nz, PHt_f32);
    arm_mat_init_f32(&K_new, ekf->nx, ekf->nz, K_f32);

    arm_matrix_instance_f32 H = ekf->dhdx;
    arm_matrix_instance_f32 P = ekf->P_prev;
    
    //Compute HP
    result |= arm_mat_mult_f32(&H, &P, &HP);

    //Compute Ht
    result |= arm_mat_trans_f32(&H, &Ht);

    //Compute HPHt
    result |= arm_mat_mult_f32(&HP, &Ht, &HPHt);

    //Compute HPHt + R
    result |= arm_mat_add_f32(&HPHt, &R_mat, &HPHtR);

    //Compute (HPHt + R)^-1
    result |= arm_mat_inverse_f32(&HPHtR, &HPHtRi);

    //Compute PHt
    result |= arm_mat_mult_f32(&P, &Ht, &PHt);

    //Compute K
    result |= arm_mat_mult_f32(&PHt, &HPHtRi, &K_new);

    //Set K to new value in the ekf struct
    ekf->K_n = K_new;

}
void update_state(ExtKalmanFilter *ekf){

    //x_n = x_prev + K*(z-h)

    arm_status result = ARM_MATH_SUCCESS;
    float32_t zh_f32[ekf->nz];
    float32_t Kzh_f32[ekf->nx];
    float32_t x_updated_f32[ekf->nx];

    arm_matrix_instance_f32 zh;
    arm_matrix_instance_f32 Kzh;
    arm_matrix_instance_f32 x_updated;

    arm_mat_init_f32(&zh, ekf->nz, 1, zh_f32);
    arm_mat_init_f32(&Kzh, ekf->nx, 1, Kzh_f32);
    arm_mat_init_f32(&x_updated, ekf->nx, 1, x_updated_f32);

    arm_matrix_instance_f32 z_vec = ekf->z;
    arm_matrix_instance_f32 h_vec = ekf->h;
    arm_matrix_instance_f32 x_vec = ekf->x_prev;
    arm_matrix_instance_f32 K = ekf->K_n;

    //Compute (z-h)
    result |= arm_mat_sub_f32(&z_vec, &h_vec, &zh);
    
    //Compute K(z-h)
    result |= arm_mat_mult_f32(&K, &zh, &Kzh);

    //Compute x_n = x_prev + K*(z-h)
    result |= arm_mat_add_f32(&x_vec, &Kzh, &x_updated);

    //Update x_n in ekf struct
    ekf->x_n = x_updated;
}

void update_covariance(ExtKalmanFilter *ekf){

    //P_n = (I - KH)P_prev(I - KH)t + KRKt

    //1. Find I - KH
    //2. Find (I - KH)t
    //3. Find (I - KH)P_prev(I - KH)t
    //4. Find KRKt
    //5. Find Pn

    arm_status result = ARM_MATH_SUCCESS;

    float32_t I_f32[ekf->nx * ekf->nx];
    for (uint16_t y = 0; y < ekf->nx; y++){ //Construct identity matrix
        for (uint16_t x = 0; x < ekf->nx; x++){
            if (x == y){
                I_f32[x + y*ekf->nx] = 1.0;
            }
            else{
                I_f32[x + y*ekf->nx] = 0.0;
            }
        }
    }
    arm_matrix_instance_f32 I;
    arm_mat_init_f32(&I, ekf->nx, ekf->nx, I_f32);

    arm_matrix_instance_f32 K = ekf->K_n;
    arm_matrix_instance_f32 H = ekf->dhdx;
    arm_matrix_instance_f32 P_ = ekf->P_prev;
    arm_matrix_instance_f32 R = ekf->R;

    float32_t KH_f32[ekf->nx * ekf->nx];
    float32_t IKH_f32[ekf->nx * ekf->nx];
    float32_t IKHt_f32[ekf->nx * ekf->nx];
    float32_t IKHP_f32[ekf->nx * ekf->nx];
    float32_t IKHPIKHt_f32[ekf->nx * ekf->nx];
    float32_t KR_f32[ekf->nx * ekf->nz];
    float32_t KRKt_f32[ekf->nx * ekf->nx];
    float32_t P_result_f32[ekf->nx * ekf->nx];
    float32_t Kt_f32[ekf->nz * ekf->nx];

    arm_matrix_instance_f32 KH;
    arm_matrix_instance_f32 IKH;
    arm_matrix_instance_f32 IKHt;
    arm_matrix_instance_f32 IKHP;
    arm_matrix_instance_f32 IKHPIKHt;
    arm_matrix_instance_f32 KR;
    arm_matrix_instance_f32 KRKt;
    arm_matrix_instance_f32 P_result;
    arm_matrix_instance_f32 Kt;

    arm_mat_init(&KH, ekf->nx, ekf->nx, KH_f32);
    arm_mat_init(&IKH, ekf->nx, ekf->nx, IKH_f32);
    arm_mat_init(&IKHt, ekf->nx, ekf->nx, IKHt_f32);
    arm_mat_init(&IKHP, ekf->nx, ekf->nx, IKHP_f32);
    arm_mat_init(&IKHPIKHt, ekf->nx, ekf->nx, IKHPIKHt_f32);
    arm_mat_init(&KR, ekf->nx, ekf->nz, KR_f32);
    arm_mat_init(&KRKt, ekf->nx, ekf->nx, KRKt_f32);
    arm_mat_init(&P_result, ekf->nx, ekf->nx, P_result_f32);
    arm_mat_init(&Kt, ekf->nz, ekf->nx, Kt_f32);

    //Compute KH
    result |= arm_mat_mult_f32(&K, &H, &KH);

    //Compute (I - KH)
    result |= arm_mat_sub_f32(&I, &KH, &IKH);

    //Compute (I - KH)P_
    result |= arm_mat_mult_f32(&IKH, &P_, &IKHP);

    //Compute (I - KH)t
    result |= arm_mat_trans_f32(&IKH, &IKHt);

    //Compute (I - KH)P_(I - KH)t
    result |= arm_mat_mult_f32(&IKHP, &IKHt, &IKHPIKHt);

    //Compute KR
    result |= arm_mat_mult_f32(&K, &R, &KR);

    //Compute Kt
    result |= arm_mat_trans_f32(&K, &Kt);

    //Compute KRKt
    result |= arm_mat_mult_f32(&KR, &Kt, &KRKt);

    //Compute P_n
    result |= arm_mat_add_f32(&IKHPIKHt, &KRKt, &P_result);

    //Update P_n in ekf struct
    ekf->P_n = P_result;
}

void state_transition_function(ExtKalmanFilter *ekf){

    float f_new_data[ekf->nx];
    f_new_data[0] = ekf->x_n.pData[0] + ekf->x_n.pData[1]*ekf->time_step + 0.5*(ekf->time_step * ekf->time_step)*ekf->accelerometer[0];
    f_new_data[1] = ekf->x_n.pData[1] + ekf->accelerometer[0]*ekf->time_step;

    f_new_data[2] = ekf->x_n.pData[2] + ekf->x_n.pData[3]*ekf->time_step + 0.5*(ekf->time_step * ekf->time_step)*ekf->accelerometer[1];
    f_new_data[3] = ekf->x_n.pData[3] + ekf->accelerometer[1]*ekf->time_step;

    f_new_data[4] = ekf->x_n.pData[4] + ekf->x_n.pData[5]*ekf->time_step + 0.5*(ekf->time_step * ekf->time_step)*ekf->accelerometer[2];
    f_new_data[5] = ekf->x_n.pData[5] + ekf->accelerometer[2]*ekf->time_step;

    arm_matrix_instance_f32 f_new = {ekf->nx, 1, f_new_data};
    ekf->f = f_new;
}

void state_transition_jacobian(ExtKalmanFilter *ekf){

    float32_t dfdx_new[ekf->nx][ekf->nx];
    memset(dfdx_new, 0.0, sizeof dfdx_new);

    //x position, velocity
    dfdx_new[0][0] = 1.0;
    dfdx_new[0][1] = ekf->time_step;
    dfdx_new[1][0] = 0.0;
    dfdx_new[1][1] = 1.0;

    //y position, velocity
    dfdx_new[2][2] = 1.0;
    dfdx_new[2][3] = ekf->time_step;
    dfdx_new[3][2] = 0.0;
    dfdx_new[3][3] = 1.0;

    //z position, velocity
    dfdx_new[4][4] = 1.0;
    dfdx_new[4][5] = ekf->time_step;
    dfdx_new[5][4] = 0.0;
    dfdx_new[5][5] = 1.0;

    arm_matrix_instance_f32 F;
    arm_mat_init_f32(&F, ekf->nx, ekf->nx, dfdx_new);

    ekf->dfdx = F;

}

void predict_state(ExtKalmanFilter *ekf){

    ekf->x_next = ekf->f;
    //This is technically f + Gu, but we are letting u = [0], so this is just to say x_next = f;

}

void predict_covariance(ExtKalmanFilter *ekf){

    //P_next = FPFt + Q

    arm_status result = ARM_MATH_SUCCESS;

    arm_matrix_instance_f32 P = ekf->P_n;
    arm_matrix_instance_f32 F = ekf->dfdx;
    arm_matrix_instance_f32 Q_mat = ekf->Q;

    float32_t Ft_f32[ekf->nx * ekf->nx];
    arm_matrix_instance_f32 Ft;
    arm_mat_init_f32(&Ft, ekf->nx, ekf->nx, Ft_f32);

    float32_t FP_f32[ekf->nx * ekf->nx];
    arm_matrix_instance_f32 FP;
    arm_mat_init_f32(&FP, ekf->nx, ekf->nx, FP_f32);

    float32_t FPFt_f32[ekf->nx * ekf->nx];
    arm_matrix_instance_f32 FPFt;
    arm_mat_init_f32(&FPFt, ekf->nx, ekf->nx, FPFt_f32);

    float32_t P_future_f32[ekf->nx * ekf->nx];
    arm_matrix_instance_f32 P_future;
    arm_mat_init_f32(&P_future, ekf->nx, ekf->nx, P_future_f32);

    //Compute FP
    result |= arm_mat_mult_f32(&F, &P, &FP);

    //Compute Ft
    result = arm_mat_trans_f32(&F, &Ft);

    //Compute FPFt
    result |= arm_mat_mult_f32(&FP, &Ft, &FPFt);

    //Compute P_next = FPFt + Q
    result |= arm_mat_add_f32(&FPFt, &Q_mat, &P_future);

    //Update the value of P_next in the ekf struct
    ekf->P_next = P_future;
}

void make_measurement(ExtKalmanFilter *ekf){

    float32_t z_new_f32[ekf->nz];
    z_new_f32[0] = ekf->gps[0];
    z_new_f32[1] = ekf->gps[1];
    z_new_f32[2] = ekf->gps[2];

    arm_matrix_instance_f32 z_new;
    arm_mat_init_f32(&z_new, ekf->nz, 1, z_new_f32);
    ekf->z = z_new;

}

void acknowledge_time_passed(ExtKalmanFilter *ekf){

    ekf->x_prev = ekf->x_next;
    ekf->P_prev = ekf->P_next;

}
float *run_ekf(ExtKalmanFilter *ekf, float *GPS_sensor, float *IMU_sensor){

    update_time_step(ekf);

    ekf->gps[0] = GPS_sensor[0];
    ekf->gps[1] = GPS_sensor[1];
    ekf->gps[2] = GPS_sensor[2];

    ekf->accelerometer[0] = IMU_sensor[0];
    ekf->accelerometer[1] = IMU_sensor[1];
    ekf->accelerometer[2] = IMU_sensor[2];

    //Measurement function
    make_measurement(ekf);

    //Update
    observation_function(ekf);
    observation_jacobian(ekf);
    kalman_gain(ekf);
    update_state(ekf);
    update_covariance(ekf);

    //Predict
    state_transition_function(ekf);
    state_transition_jacobian(ekf);
    predict_state(ekf);
    predict_covariance(ekf);

    acknowledge_time_passed(ekf);

    arm_matrix_instance_f32 curr_state = ekf->x_n;
    return curr_state.pData;
}