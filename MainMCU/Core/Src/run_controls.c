#include "run_controls.h"
#include "controls.h"

#include "globals.h"

#include "port_config.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 * @brief Task to run the controls algorithm
 * @param args Unused
 */
void run_controls_task(void *args) {
    controller controller;
    float state[9];
    float seconds_since_launch = 0;

    initialize_controls(&controller);

    /* Wait for notification to begin */
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_CONTROLS_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_CONTROLS_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Running controls\r\n", 18, HAL_MAX_DELAY);

    while (1) {
        /* Always use mutex on g_current_state */
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
            seconds_since_launch = (float) ms_since_launch / 1000.0;

            xSemaphoreGive(g_state_mutex_handle);

            /* Stop running controls after (???) state */
            if (g_current_state.rocket_state.rocket_state > 3) {
                break;
            }
        }

        run_controls(&controller, state, seconds_since_launch);

        char buf[100];
        sprintf(buf, "Running controls timestamp %f servos %f %f %f %f\r\n", seconds_since_launch, controller.servo_deflections[0], controller.servo_deflections[1], controller.servo_deflections[2], controller.servo_deflections[3]);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
    
        /* Always use mutex on g_current_state */
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            /* Update servo deflections */
            g_current_state.servo_deflection.servo_deflection_1 = controller.servo_deflections[0];
            g_current_state.servo_deflection.servo_deflection_2 = controller.servo_deflections[1];
            g_current_state.servo_deflection.servo_deflection_3 = controller.servo_deflections[2];
            g_current_state.servo_deflection.servo_deflection_4 = controller.servo_deflections[3];

            xSemaphoreGive(g_state_mutex_handle);
        }

        /* Trigger ADC conversion sequence */
        HAL_ADC_Start_IT(FIRST_ADC);

        /* Maximum frequency of 20Hz */
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Tasks are not allowed to exit, so stall here */
    vTaskDelay(portMAX_DELAY);
}