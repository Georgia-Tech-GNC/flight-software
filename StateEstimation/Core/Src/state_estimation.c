/**
 * @file state_estimation.c
 * @author Kanav Chugh
 * @brief Implementation of EKF interrupt handlers
 */
#include "main.h"

static volatile uint32_t last_ekf_dwt = 0;
static volatile uint32_t last_attitude_dwt = 0;
static volatile bool ground_ekf_init = false;
static volatile bool flight_ekf_init = false;
static volatile bool rocket_atd_init = false;


/**
 * @brief Callback for timers for rocket attitude and state estimation
 * @param htim timer instance in interrupt
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        uint32_t current_dwt = DWT->CYCCNT;
        if (rocket_state == GROUND) {
            if (!ground_ekf_init) {
                last_ekf_dwt = current_dwt;
                ground_ekf_init = true;
                return;
            }
            uint32_t elapsed_ticks = current_dwt - last_ekf_dwt;
            gekf.time_step = DWT_TicksToSeconds(elapsed_ticks);
            last_ekf_dwt = current_dwt;
            update_ekf_ground(&gekf, &sensors);
        } else if (rocket_state > GROUND) {
            if (!flight_ekf_init) {
                last_ekf_dwt = current_dwt;
                flight_ekf_init = true;
                return;
            }
            uint32_t elapsed_ticks = current_dwt - last_ekf_dwt;
            fekf.time_step = DWT_TicksToSeconds(elapsed_ticks);
            last_ekf_dwt = current_dwt;
            update_ekf(&fekf, &rocket_atd, &sensors);
        }
    } else if (htim->Instance == TIM7) {
        if (rocket_state > GROUND) {
            uint32_t current_dwt = DWT->CYCCNT;
            if (!rocket_atd_init) {
                last_attitude_dwt = current_dwt;
                rocket_atd_init = true;
                return;
            }
            uint32_t elapsed_ticks = current_dwt - last_attitude_dwt;
            rocket_atd.time_step = DWT_TicksToSeconds(elapsed_ticks);
            last_attitude_dwt = current_dwt;
            
        }
    }
}