#ifndef __EKF_CONSTANTS_H_
#define __EKF_CONSTANTS_H_

#include "arm_math.h"


float32_t dfdx_f32[6*6] = {0.0};  

float32_t dhdx_f32[6*3] = {
    1.0, 0,   0,   0,   0,   0,    
    0,   0,   1.0, 0,   0,   0,    
    0,   0,   0,   0,   1.0, 0    
};



float32_t Q_f32[6*6] = {
    0.01,  0,     0,     0,     0,     0,    // x position variance
    0,     0.1,   0,     0,     0,     0,    // x velocity variance
    0,     0,     0.01,  0,     0,     0,    // y position variance
    0,     0,     0,     0.1,   0,     0,    // y velocity variance
    0,     0,     0,     0,     0.01,  0,    // z position variance
    0,     0,     0,     0,     0,     0.1   // z velocity variance
};

float32_t K_f32[6*3] = {0.0};  // Initialize all elements to 0


float32_t R_f32[3*3] = {
    2.25,  0.0,   0.0, 
    0.0,   2.25,  0.0,   
    0.0,   0.0,   6.25   
};



float32_t x_init[6] = {0.0};

float32_t P_init[6*6] = {
    1.0, 0,   0,   0,   0,   0,
    0,   1.0, 0,   0,   0,   0,
    0,   0,   1.0, 0,   0,   0,
    0,   0,   0,   1.0, 0,   0,
    0,   0,   0,   0,   1.0, 0,
    0,   0,   0,   0,   0,   1.0
};


float32_t f_f32[6] = {0.0};
float32_t h_f32[3] = {0.0};
float32_t z_f32[3] = {0.0};


float32_t HPHtR_f32[3 * 3];
float32_t HPHtRi_f32[3 * 3];

float32_t PHt_f32[6 * 3];
float32_t HP_f32[3 * 6];
float32_t Ht_f32[6 * 3];

float32_t HPHt_f32[3 * 3];


arm_matrix_instance_f32 HP;
arm_matrix_instance_f32 Ht;
arm_matrix_instance_f32 HPHt;
arm_matrix_instance_f32 HPHtR;
arm_matrix_instance_f32 HPHtRi;
arm_matrix_instance_f32 PHt;
arm_matrix_instance_f32 K_new;

float32_t Ft_f32[6 * 6];
arm_matrix_instance_f32 Ft;

float32_t FP_f32[6 * 6];
arm_matrix_instance_f32 FP;

float32_t FPFt_f32[6 * 6];
arm_matrix_instance_f32 FPFt;

float32_t P_future_f32[6 * 6];
arm_matrix_instance_f32 P_future;

arm_status result = ARM_MATH_SUCCESS;

#endif