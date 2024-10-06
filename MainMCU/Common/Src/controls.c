/**
 * @file controls.c
 * @author Patrick Barry
 * @brief Source file for in-flight control algorithm of the jet vanes rocket
 *  
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "controls.h"

/**
 * @brief Takes into account current state estimate and time since launch
 * @param 
 * @return
 * @note
*/
void LQR_gain_selector(controller *ctrl){

}

/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void reference_selector(controller *ctrl){

}


/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void compute_controls(controller *ctrl){

}


/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void update_moment_arm(controller *ctrl){

}


/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void moment_to_sideforce(controller *ctrl){

}


/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void sideforce_to_vane_angle(controller *ctrl){

}


/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void vane_angle_to_servo_angle(controller *ctrl){

}

/**
 * @brief 
 * @param 
 * @return
 * @note
*/
void run_controls(controller *ctrl, float *state, float elapsed_time){

}