#include "state_est_rx.h"

#include "string.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"

#include "state_flash.h"
#include "globals.h"

#include "run_controls.h"

/**
 * Task to receive new states and put them into the g_current_state global variable
 * @param args Unused
 */
void state_est_rx_task(void *args) {
    uint8_t _state_rx_buff[STATE_ESTIMATION_BYTES];
    uint8_t *state_rx_buff = _state_rx_buff;

    uint8_t counter = 0;

    uint8_t launched = 0;
    uint8_t landed = 0;

    uint8_t drogue_parachute_deploy = 0;
    uint8_t main_parachute_deploy = 0;

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

            uint8_t *serial_buffer = state_rx_buff + 37;
            uint8_t *sensors_buffer = state_rx_buff;

            int offset = 1;
            memcpy(&g_current_state.sensor_data.accelerometer_x, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.accelerometer_y, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.accelerometer_z, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.gyro_x, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.gyro_y, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.gyro_z, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.gps_x, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.gps_y, sensors_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.sensor_data.gps_z, sensors_buffer + offset, 4);
            
            offset = 0;

            memcpy(&g_current_state.rocket_state.rocket_state, serial_buffer + offset, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            memcpy(&g_current_state.state_vector.position_x, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.position_y, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.position_z, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.velocity_x, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.velocity_y, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.velocity_z, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.attitude_w, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.attitude_x, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.attitude_y, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.attitude_z, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.world_x, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.world_y, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.state_vector.world_z, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.ground_ekf.pn_matrix_d1, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.ground_ekf.pn_matrix_d2, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.ground_ekf.pn_matrix_d3, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.ground_ekf.pn_matrix_d4, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.ground_ekf.pn_matrix_d5, serial_buffer + offset, 4);
            offset += 4;
            memcpy(&g_current_state.ground_ekf.pn_matrix_d6, serial_buffer + offset, 4);
            offset += 4;

            g_current_state.analog_feedback_data.timestamp = xTaskGetTickCount();
            g_current_state.ground_ekf.timestamp = xTaskGetTickCount();
            g_current_state.state_vector.timestamp = xTaskGetTickCount();
            g_current_state.sensor_data.timestamp = xTaskGetTickCount();
            g_current_state.rocket_state.timestamp = xTaskGetTickCount();
            g_current_state.servo_deflection.timestamp = xTaskGetTickCount();

            /* Launched */
            if (launched == 0 && g_current_state.rocket_state.rocket_state >= 2) {
                launched = 1;
                g_current_state.launch_timestamp = xTaskGetTickCount();
                xTaskNotify(g_run_controls_task_handle, BEGIN_CONTROLS_NOTIFICATION_BIT, eSetBits);
            }

            if (landed == 0 && g_current_state.rocket_state.rocket_state == 6) {
                landed = 1;
                HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flashing SD card\r\n", 18, HAL_MAX_DELAY);
                
                xTaskNotifyIndexed(g_state_flash_task_handle, 1, FLASH_SD_CARD_NOTIFICATION_BIT, eSetBits);
            }

            if (drogue_parachute_deploy == 0 && g_current_state.rocket_state.rocket_state == 5) {
                drogue_parachute_deploy = 1;
                g_current_state.rocket_state.firing_channel_1 = 1;
            }

            if (main_parachute_deploy == 0 && (g_current_state.rocket_state.rocket_state == 5 && (g_current_state.state_vector.position_z < 250 || g_current_state.state_vector.position_z > -250))) {
                main_parachute_deploy = 1;
                g_current_state.rocket_state.firing_channel_2 = 1;
            }

            if (counter++ == 10) {
                char buf[200];
                sprintf(buf, "State: %d, Accel: %f %f %f\r\nGyro: %f %f %f\r\nGPS: %f %f %f\r\n", g_current_state.rocket_state.rocket_state, g_current_state.sensor_data.accelerometer_x, g_current_state.sensor_data.accelerometer_y, g_current_state.sensor_data.accelerometer_z, g_current_state.sensor_data.gyro_x, g_current_state.sensor_data.gyro_y, g_current_state.sensor_data.gyro_z, g_current_state.sensor_data.gps_x, g_current_state.sensor_data.gps_y, g_current_state.sensor_data.gps_z);
                HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                counter = 0;
            }

            xSemaphoreGive(g_state_mutex_handle);
        }

        vTaskDelay(100);
    }
}