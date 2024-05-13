/**
 * @file state_est_helpers.c
 * @author Patrick Barry 
 * @brief Source file for helper functions for the state estimation MCU
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/
#include <stdio.h>
#include "state_est_helpers.h"
#include "arm_math.h"

float *com_to_imu(float seconds_since_launch, int launch_has_occurred){

    float x_dist;

    if (launch_has_occurred == 1){
        if (seconds_since_launch >= burn_time){
            x_dist = com_dist_end;
        }
        else{
            x_dist = (seconds_since_launch/burn_time)*(com_dist_start - com_dist_end); //linear interpolation
        }
    }
    else{
        x_dist = com_dist_start;
    }

    float imu_distances[3] = {x_dist, com_to_imu_y, com_to_imu_z};
    return imu_distances;

}

float *multiply_matrices(float A_0_0, int rowsA, int colsA, float B_0_0, int rowsB, int colsB){
    arm_matrix_instance_f32 matA_inst;
    arm_matrix_instance_f32 matB_inst;
    arm_matrix_instance_f32 matAB_inst;

    float matAB[rowsA][colsB];

    arm_mat_init_f32(&matA_inst, rowsA, colsA, &A_0_0);
    arm_mat_init_f32(&matB_inst, rowsB, colsB, &B_0_0);
    arm_mat_init_f32(&matAB_inst, rowsA, colsB, &matAB[0][0]);
    arm_status_temp = arm_mat_mult_f32(&matA_inst, &matB_inst, &matAB_inst);
}

float *transpose_matrix(float *matA){
    arm_matrix_instance_f32 matA_inst;
    arm_matrix_instance_f32 matB_inst; //B is A'
    
    int rowsA = sizeof(matA) / sizeof(matA[0]);
    int colsA = sizeof(matA[0]);

    float matB[colsA][rowsA];

    float matB[colsA][rowsA]; //Transpose has dimensions c x r if original has dimensions r x c
    arm_mat_init_f32(&matA_inst, rowsA, colsA, matA);
    arm_mat_init_f32(&matB_inst, colsA, rowsA, matB);

    arm_mat_trans_f32(&matA_inst, &matB_inst);
    
    return matB; //Not sure if this is correct
}

float *inverse_matrix(float *matA){
    arm_matrix_instance_f32 matA_inst;
    arm_matrix_instance_f32 matB_inst; //B is A^-1
    
    int rowsA = sizeof(matA) / sizeof(matA[0]);
    int colsA = sizeof(matA[0]);

    float matB[rowsA][colsA];

    arm_mat_init_f32(&matA_inst, rowsA, colsA, matA);
    arm_mat_init_f32(&matB_inst, rowsA, colsA, matB);

    arm_mat_inverse_f32(&matA_inst, &matB_inst);

    return matB; //Not sure if this is correct

}

float *add_matrices(float *matA, float *matB){
    arm_matrix_instance_f32 matA_inst;
    arm_matrix_instance_f32 matB_inst;
    arm_matrix_instance_f32 matAplusB_inst;

    int rowsA = sizeof(matA) / sizeof(matA[0]);
    int colsA = sizeof(matA[0]);

    float matAplusB[rowsA][colsA];

    arm_mat_init_f32(&matA_inst, rowsA, colsA, matA);
    arm_mat_init_f32(&matB_inst, rowsA, colsA, matB);
    arm_mat_init_f32(&matAplusB_inst, rowsA, colsA, matAplusB);

    arm_mat_add_f32(&matA_inst, &matB_inst, &matAplusB_inst);
    return matAplusB; //Not sure if this is correct
}

float *subtract_matrices(float *matA, float *matB){ //Subtract matrix B from matrix A
    arm_matrix_instance_f32 matA_inst;
    arm_matrix_instance_f32 matB_inst;
    arm_matrix_instance_f32 matAminusB_inst;

    int rowsA = sizeof(matA) / sizeof(matA[0]);
    int colsA = sizeof(matA[0]);

    float matAminusB[rowsA][colsA];

    arm_mat_init_f32(&matA_inst, rowsA, colsA, matA);
    arm_mat_init_f32(&matB_inst, rowsA, colsA, matB);
    arm_mat_init_f32(&matAminusB_inst, rowsA, colsA, matAminusB);

    arm_mat_add_f32(&matA_inst, &matB_inst, &matAminusB_inst);
    return matAminusB; //Not sure if this is correct
}