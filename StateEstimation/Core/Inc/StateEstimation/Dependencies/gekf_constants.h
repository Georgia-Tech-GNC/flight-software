#ifndef __GEKF_CONSTANTS_H_
#define __GEKF_CONSTANTS_H_

#include "arm_math.h"



float32_t dfdx_f32_ground[6*6] =  {
        1.0, 0,   0,   0,   0,   0,
        0,   1.0, 0,   0,   0,   0,
        0,   0,   1.0, 0,   0,   0,
        0,   0,   0,   1.0, 0,   0,
        0,   0,   0,   0,   1.0, 0,
        0,   0,   0,   0,   0,   1.0
};

float32_t dhdx_f32_ground[6 * 6] = {
        1.0, 0,   0,   0,   0,   0,
        0,   1.0, 0,   0,   0,   0,
        0,   0,   1.0, 0,   0,   0,
        0,   0,   0,   1.0, 0,   0,
        0,   0,   0,   0,   1.0, 0,
        0,   0,   0,   0,   0,   1.0
    };


float32_t K_f32_ground[6*6] = {0.0};

float32_t R_f32_ground[6 * 6] = {
        1.0, 0,   0,   0,   0,   0,
        0,   1.0, 0,   0,   0,   0,
        0,   0,   1.0, 0,   0,   0,
        0,   0,   0,   1.0, 0,   0,
        0,   0,   0,   0,   1.0, 0,
        0,   0,   0,   0,   0,   1.0
    };



float32_t P_init_ground[6*6] = {
    1.0, 0,   0,   0,   0,   0,
    0,   1.0, 0,   0,   0,   0,
    0,   0,   1.0, 0,   0,   0,
    0,   0,   0,   1.0, 0,   0,
    0,   0,   0,   0,   1.0, 0,
    0,   0,   0,   0,   0,   1.0
};
float32_t x_init_ground[6] = {0.0};
float32_t f_f32_ground[6] = {0.0};
float32_t h_f32_ground[3] = {0.0};

float32_t Q_f32_ground[6 * 6] = {
    0.01, 0,   0,   0,   0,   0,
    0,   0.01, 0,   0,   0,   0,
    0,   0,   0.01, 0,   0,   0,
    0,   0,   0,   0.01, 0,   0,
    0,   0,   0,   0,   0.01, 0,
    0,   0,   0,   0,   0,   0.01
};

float32_t x_p[6] = {0.0};
float32_t x_f[6] = {0.0};
float32_t P_f[6*6] = {
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0
};
float32_t G_f32[3*3] = {0.0};   

float32_t HPHtR_f32_ground[6 * 6];
float32_t HPHtRi_f32_ground[6 * 6];

float32_t PHt_f32_ground[6 * 6];
float32_t HP_f32_ground[6 * 6];
float32_t Ht_f32_ground[6 * 6];

float32_t HPHt_f32_ground[6 * 6];

arm_matrix_instance_f32 HP_ground;
arm_matrix_instance_f32 Ht_ground;
arm_matrix_instance_f32 HPHt_ground;
arm_matrix_instance_f32 HPHtR_ground;
arm_matrix_instance_f32 HPHtRi_ground;
arm_matrix_instance_f32 PHt_ground;
arm_matrix_instance_f32 K_new_ground;

#endif