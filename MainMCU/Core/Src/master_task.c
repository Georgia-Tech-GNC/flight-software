#include "globals.h"
#include "string.h"
#include "rocket_pid.h"
#include "main.h"

#define RAD2DEG(x) ((x) * 57.2958)

typedef struct StateEstimationPacket {
    quaternion_t orientation;
    double upward_accel;
    double w_x, w_y, w_z;
} state_estimation_packet_t;

state_estimation_packet_t read_state_estimation_packet(int16_t* packet) {
    state_estimation_packet_t data;
    data.orientation.w = ((double)packet[0]) / 32767.0;
    data.orientation.x = ((double)packet[1]) / 32767.0;
    data.orientation.y = ((double)packet[2]) / 32767.0;
    data.orientation.z = ((double)packet[3]) / 32767.0;

    data.upward_accel = ((double)packet[4]) / 327.67;
    data.w_x = ((double)packet[5]) / 1310.68;
    data.w_y = ((double)packet[6]) / 1310.68;
    data.w_z = ((double)packet[7]) / 1310.68;

    return data;
}

/**
 * Thresholds:
 * Acceleration: 16
 * Start controls: 1s
 * Fire chutes: 11.5
 */
const TickType_t SERVO_UPDATE_PERIOD = pdMS_TO_TICKS(200);
const TickType_t CONTROLS_START_DELAY = pdMS_TO_TICKS(1000);

const TickType_t CHUTE_DEPLOYMENT_DELAY = pdMS_TO_TICKS(100000000); // pdMS_TO_TICKS(11500);
const TickType_t SD_FLASH_DELAY = pdMS_TO_TICKS(100000000); // pdMS_TO_TICKS(30000);



void master_task_handler(void* args) {
    HAL_UART_Transmit(&debug_uart, "Started master task!\r\n", 22, HAL_MAX_DELAY);

    TickType_t last_servo_update = xTaskGetTickCount();
    
    TickType_t ascent_start_time;
    TickType_t control_start_time;

    PIDController controller;
    pid_init(&controller, 1.0, 0.0, 0.0, 0.05);

    fsm_state_t rocket_state = CONTROLLED_ASCENT;

    while (1) {
        // Wait to recieve message
        uint16_t state_rx_buffer[8];
        size_t recv_bytes = xStreamBufferReceive(g_state_stream_buffer.handle, (uint8_t*)state_rx_buffer, 16, portMAX_DELAY);
        if (recv_bytes != 16) {
            // State Estimation failed
            if (rocket_state == GROUND || rocket_state == ARMED || rocket_state == FREEFALL || rocket_state == SD_FLASH) {
                rocket_state = SD_FLASH; 
            } else {
                // state is fast_ascent, controlled_ascent, or uncontrolled_ascent
                rocket_state = UNCONTROLLED_ASCENT;
            }
        }
        state_estimation_packet_t state_data = read_state_estimation_packet(state_rx_buffer);
        TickType_t current_time = xTaskGetTickCount();

        // Handle state transitions
        switch (rocket_state) {
        case GROUND:
            if (xSemaphoreTake(g_state_lock.handle, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (g_current_state.arm_signal_recieved) {
                    rocket_state = ARMED;
                }
                xSemaphoreGive(g_state_lock.handle);
            }
            break;
        
        case ARMED:
            if (state_data.upward_accel < -16) {
                rocket_state = FAST_ASCENT;
                ascent_start_time = current_time;
            }
            break;

        case FAST_ASCENT:
            if (current_time > CONTROLS_START_DELAY + ascent_start_time) {
                rocket_state = CONTROLLED_ASCENT;
                control_start_time = current_time;
            }
            break;

        case CONTROLLED_ASCENT:
            if (current_time > CHUTE_DEPLOYMENT_DELAY + ascent_start_time) {
                HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);
                rocket_state = FREEFALL;
            }
            // TODO: figure out how to check rocket angle threshold for failure case
            break;

         case UNCONTROLLED_ASCENT:
            if (current_time > CHUTE_DEPLOYMENT_DELAY + ascent_start_time) {
                HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);
                rocket_state = FREEFALL;
            }
            break;

        case FREEFALL:
            if (current_time > SD_FLASH_DELAY + ascent_start_time) {
                HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_RESET);
                rocket_state = SD_FLASH;
            }
            break;
        }

        double error[3];
        // Set servo positions depending on state
        float servo_1_command, servo_2_command;
        if (rocket_state == CONTROLLED_ASCENT) {
            double output[3];
            double state[14] = {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                state_data.orientation.w,
                state_data.orientation.x,
                state_data.orientation.y,
                state_data.orientation.z,
                state_data.w_x,
                state_data.w_y,
                state_data.w_z,
            };
            getControl(&controller, state, current_time - control_start_time, output, error);

            // TODO: adjust servos
            servo_1_command = output[1];
            servo_2_command = output[2];
        } else {
            servo_1_command = 0;
            servo_2_command = 0;
        }
    
        // Update global state
        if (xSemaphoreTake(g_state_lock.handle, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state.orientation = state_data.orientation;
            g_current_state.w_x = state_data.w_x;
            g_current_state.w_y = state_data.w_y;
            g_current_state.w_z = state_data.w_z;
            g_current_state.timestamp = current_time;
            g_current_state.state = rocket_state;
            g_current_state.servo_cmd_1 = servo_1_command;
            g_current_state.servo_cmd_2 = servo_2_command;
            xSemaphoreGive(g_state_lock.handle);
        }
        
        if (current_time > last_servo_update + SERVO_UPDATE_PERIOD) {
            last_servo_update = current_time;

            char msg_buffer[256];
            size_t msg_size = sprintf(msg_buffer, "Error: %.3f\t%.3f\t%.3f\tState: %.3f\t%.3f\t%.3f\t%.3f\r\n", 
                error[0], error[1], error[2],
                state_data.orientation.w, state_data.orientation.x, state_data.orientation.y, state_data.orientation.z
            );
            HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);

            /*
            if (b > 60) { b = 60; }
            if (b < -60) { b = -60; }

            uint16_t servo_pos = 1500 + b * 10;
            servo_set_pos(&servo_1, servo_pos);

            char msg_buffer[256];
            size_t msg_size = sprintf(msg_buffer, "%.4f %.4f %.4f %.4f\r\n", a, b, c, d);
            HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);
            */
        }
    }
}