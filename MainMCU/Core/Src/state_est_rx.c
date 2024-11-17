#include "state_est_rx.h"

#include "string.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"

#include "globals.h"

/**
 * Task to receive new states and put them into the g_current_state global variable
 * @param args Unused
 */
void state_est_rx_task(void *args) {
    uint8_t _state_rx_buff[STATE_ESTIMATION_BYTES];
    uint8_t *state_rx_buff = _state_rx_buff;

    while(1) {
        size_t bytes_read = 0;

        /* Loop until we've received the full next state */
        while (bytes_read < STATE_ESTIMATION_BYTES) {
            size_t read = xStreamBufferReceive(g_state_rx_sb_handle, state_rx_buff + bytes_read, STATE_ESTIMATION_BYTES - bytes_read, portMAX_DELAY);
            bytes_read += read;
        }

        TickType_t timestamp = xTaskGetTickCount();
        /* Copy state iestimation result into rocket state */
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            /* TODO: Do something here */

            g_current_state.ground_ekf.pn_matrix_d1 = (float) timestamp / 100.0;
            g_current_state.ground_ekf.pn_matrix_d2 = (float) timestamp / 200.0;
            g_current_state.ground_ekf.pn_matrix_d3 = (float) timestamp / 300.0;
            g_current_state.ground_ekf.pn_matrix_d4 = (float) timestamp / 400.0;
            g_current_state.ground_ekf.pn_matrix_d5 = (float) timestamp / 500.0;
            g_current_state.ground_ekf.pn_matrix_d6 = (float) timestamp / 600.0;
            g_current_state.ground_ekf.timestamp = timestamp;

            g_current_state.rocket_state.firing_channel_1 = timestamp % 2;
            g_current_state.rocket_state.firing_channel_2 = timestamp % 3;
            g_current_state.rocket_state.firing_channel_3 = timestamp % 4;
            g_current_state.rocket_state.rocket_state = timestamp % 5;
            g_current_state.rocket_state.timestamp = timestamp;

            g_current_state.sensor_data.accelerometer_x = (float) timestamp / 1000.0;
            g_current_state.sensor_data.accelerometer_y = (float) timestamp / 2000.0;
            g_current_state.sensor_data.accelerometer_z = (float) timestamp / 3000.0;
            g_current_state.sensor_data.gps_x = (float) timestamp / 5000.0;
            g_current_state.sensor_data.gps_y = (float) timestamp / 6000.0;
            g_current_state.sensor_data.gps_z = (float) timestamp / 7000.0;
            g_current_state.sensor_data.gyro_x = (float) timestamp / 8000.0;
            g_current_state.sensor_data.gyro_y = (float) timestamp / 9000.0;
            g_current_state.sensor_data.gyro_z = (float) timestamp / 10000.0;
            g_current_state.sensor_data.timestamp = timestamp;

            g_current_state.servo_deflection.servo_deflection_1 = (float) timestamp / 30.0 + 10;
            g_current_state.servo_deflection.servo_deflection_2 = (float) timestamp / 40.0 + 20;
            g_current_state.servo_deflection.servo_deflection_3 = (float) timestamp / 50.0 + 30;
            g_current_state.servo_deflection.servo_deflection_4 = (float) timestamp / 60.0 + 40;
            g_current_state.servo_deflection.timestamp = timestamp;

            g_current_state.state_vector.attitude_w = (float) timestamp / 70.0 + 50;
            g_current_state.state_vector.attitude_x = (float) timestamp / 80.0 + 60;
            g_current_state.state_vector.attitude_y = (float) timestamp / 90.0 + 70;
            g_current_state.state_vector.attitude_z = (float) timestamp / 100.0 + 80;
            g_current_state.state_vector.velocity_x = (float) timestamp / 110.0 + 90;
            g_current_state.state_vector.velocity_y = (float) timestamp / 120.0 + 100;
            g_current_state.state_vector.velocity_z = (float) timestamp / 130.0 + 110;
            g_current_state.state_vector.position_x = (float) timestamp / 110.0 + 90;
            g_current_state.state_vector.position_y = (float) timestamp / 120.0 + 100;
            g_current_state.state_vector.position_z = (float) timestamp / 130.0 + 110;
            g_current_state.state_vector.world_x = timestamp % 2;
            g_current_state.state_vector.world_y = timestamp % 3;
            g_current_state.state_vector.world_z = timestamp % 4;
            g_current_state.state_vector.timestamp = timestamp;

            char buf[100];
            for (int i = 0; i < STATE_ESTIMATION_BYTES; i++) {
                sprintf(buf, "%c", state_rx_buff[i]);
                HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
            }
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Received state\r\n", 16, HAL_MAX_DELAY);
            xSemaphoreGive(g_state_mutex_handle);
        }
    }
}