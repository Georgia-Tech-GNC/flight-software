#include "run_controls.h"
#include "controls.h"

#include "globals.h"

#include "port_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "utils.h"

#include "stdint.h"

#ifndef STATIC_FIRE

int grab_state(float *state, float *seconds_since_launch); 
void set_servo_positions(Controller *controller);

/**
 * @brief Task to run the controls algorithm
 * @param args Unused
 */
void run_controls_task(void *args) {
    Controller controller;
    float state[9];
    float seconds_since_launch = 0;

    initialize_controls(&controller);

    /* Wait for notification to begin */
    wait_for_notification(BEGIN_CONTROLS_NOTIFICATION_BIT);

    log_debug_message("Running controls\r\n");

    while (1) {
        /* Update state array */
        int current_rocket_state = grab_state(state, &seconds_since_launch);
        if (current_rocket_state > 3) break; // If the state is greater than 3, we are out of the control period of the flight

        run_controls(&controller, state, seconds_since_launch);

        log_debug_message(
            "Running controls timestamp %f servos %f %f %f %f\r\n", 
            seconds_since_launch, controller.servo_deflections[0], 
            controller.servo_deflections[1], controller.servo_deflections[2], 
            controller.servo_deflections[3]
        );
    
        /* Set the most recent commanded servo positions from the servo */
        set_servo_positions(&controller);

        /* Trigger ADC conversion sequence */
        HAL_ADC_Start_IT(FIRST_ADC);

        /* Maximum frequency of 20Hz */
        static TickType_t last_ticks = xTaskGetTickCount();
        TickType_t diff_ticks = xTaskGetTickCount() - last_ticks;

        for (int i = 0; i < 4; i ++) {
            update_servo_true_command_position(&servos[i], diff_ticks);

        }

        last_ticks = xTaskGetTickCount();

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Tasks are not allowed to exit, so stall here */
    vTaskDelay(portMAX_DELAY);
}


int grab_state(float *state, float *seconds_since_launch) {
    /* We read this in semaphore to avoid data tearing between tasks */
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        /* Update state vector */
        state[0] = g_current_state.state_vector.velocity_y;
        state[1] = g_current_state.state_vector.velocity_x;
        state[2] = g_current_state.state_vector.velocity_z;
        state[3] = g_current_state.sensor_data.gyro_y;
        state[4] = g_current_state.sensor_data.gyro_x;
        state[5] = g_current_state.sensor_data.gyro_z;
        state[6] = g_current_state.state_vector.attitude_y;
        state[7] = g_current_state.state_vector.attitude_x;
        state[8] = g_current_state.state_vector.attitude_z;

        /* Calculate time since launch */
        uint64_t ms_since_launch = pdTICKS_TO_MS(xTaskGetTickCount() - g_current_state.launch_timestamp);
        *seconds_since_launch = (float) ms_since_launch / 1000.0;

        xSemaphoreGive(g_state_mutex_handle);

        return g_current_state.rocket_state.rocket_state;
    }

    return -1;
}


void set_servo_positions(Controller *controller) {
    /* We read this in semaphore to avoid data tearing between tasks */
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        /* Update servo deflections */
        g_current_state.servo_deflection.servo_deflection_1 = controller->servo_deflections[0];
        g_current_state.servo_deflection.servo_deflection_2 = controller->servo_deflections[1];
        g_current_state.servo_deflection.servo_deflection_3 = controller->servo_deflections[2];
        g_current_state.servo_deflection.servo_deflection_4 = controller->servo_deflections[3];

        xSemaphoreGive(g_state_mutex_handle);
    }
}

#endif