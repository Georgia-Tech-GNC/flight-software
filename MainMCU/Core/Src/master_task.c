#include "globals.h"
#include "string.h"
#include "rocket_pid.h"
#include "main.h"
#include "protocol.h"

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

fsm_state_t run_state_machine(fsm_state_t current_state, uint8_t command, const state_estimation_packet_t* sensors);
TickType_t get_active_control_duration();

/**
 * Thresholds:
 * Acceleration: 16
 * Start controls: 1s
 * Fire chutes: 11.5
 */
const TickType_t SERVO_UPDATE_PERIOD = pdMS_TO_TICKS(20); // pdMS_TO_TICKS(20);
const TickType_t PYRO_TIME = pdMS_TO_TICKS(1000);
const TickType_t CONTROLS_START_DELAY = pdMS_TO_TICKS(1000);

const TickType_t CHUTE_DEPLOYMENT_DELAY = pdMS_TO_TICKS(11500);
const TickType_t SD_FLASH_DELAY = pdMS_TO_TICKS(13000);

void master_task_handler(void* args) {
    HAL_UART_Transmit(&debug_uart, "Started master task!\r\n", 22, HAL_MAX_DELAY);

    vTaskDelay(200);

    TickType_t last_servo_update = xTaskGetTickCount();

    PIDController controller;
    // 0.55, 0, 0.22
    pid_init(&controller, 0.24215, 0.0, 0.11415, 0.015);

    fsm_state_t rocket_state = GROUND;

    while (1) {
        // Wait to recieve message
        uint16_t state_rx_buffer[8];
        size_t recv_bytes = xMessageBufferReceive(g_state_message_buffer.handle, (uint8_t*)state_rx_buffer, 16, 20);
        if (recv_bytes < 16) {
            HAL_UART_Transmit(&debug_uart, "recv0\r\n", 7, HAL_MAX_DELAY);
            continue;
        }
        
        state_estimation_packet_t state_data = read_state_estimation_packet(state_rx_buffer);
        TickType_t current_time = xTaskGetTickCount();

        // Check for commands
        uint8_t cmd = 0;
        xStreamBufferReceive(g_telemetry_command_stream_buffer.handle, &cmd, 1, 0);
        if (cmd > 0) {
            char buf[40];
            size_t buf_len = sprintf(buf, "Recv: %d\r\n", cmd);
            HAL_UART_Transmit(&debug_uart, buf, buf_len, HAL_MAX_DELAY);
        }
        
        rocket_state = run_state_machine(rocket_state, cmd, &state_data);

        

        double error[3];
        // Set servo positions depending on state
        float servo_1_command, servo_2_command;
        if (rocket_state == CONTROLLED_ASCENT || rocket_state == GROUND) {
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
            //char buffer[128];
            //size_t sz = sprintf(buffer, "%f %f %f\r\n", state_data.w_x,
            //    state_data.w_y,
            //    state_data.w_z);
            //HAL_UART_Transmit(&debug_uart, buffer, sz, HAL_MAX_DELAY);
            double angle = getControl(&controller, state, current_time - get_active_control_duration(), output, error);
            if (fabs(angle) > 0.349066) {
                servo_1_command = 0;
                servo_2_command = 0;
                rocket_state = (rocket_state == CONTROLLED_ASCENT) ? UNCONTROLLED_ASCENT : GROUND;
            } else {
                // TODO: adjust servos
                servo_1_command = output[1];
                servo_2_command = -output[2];
            }
        } else {
            servo_1_command = 0;
            servo_2_command = 0;
        }

        servo_1_command = RAD2DEG(servo_1_command);
        servo_2_command = RAD2DEG(servo_2_command);

    
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
        } else {
            HAL_UART_Transmit(&debug_uart, "mfail\r\n", 7, HAL_MAX_DELAY);
        }
        
        if (current_time > last_servo_update + SERVO_UPDATE_PERIOD) {
            last_servo_update = current_time;

            /*
            char msg_buffer[256];
            size_t msg_size = sprintf(msg_buffer, "%.8f,\t%.8f,\t%.8f\r\n", 
                state_data.w_x, state_data.w_y, state_data.w_z
            );
            HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);
            //HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);
            //char msg_buffer[256];
            //size_t msg_size = sprintf(msg_buffer, "Error: %.3f\t%.3f\t%.3f\tState: %.3f\t%.3f\t%.3f\t%.3f\r\n", 
            //    error[0], error[1], error[2],
            //    state_data.orientation.w, state_data.orientation.x, state_data.orientation.y, state_data.orientation.z
            //);
            //HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);

            
            if (b > 60) { b = 60; }
            if (b < -60) { b = -60; }
            */

            float servo_tick_1 = 1500 + (8.38 * servo_1_command);
            if (servo_tick_1 > 1583) { servo_tick_1 = 1583; }
            if (servo_tick_1 < 1416) { servo_tick_1 = 1416; }

            servo_set_pos(&servo_1, (uint32_t)servo_tick_1);            

            float servo_tick_2 = 1500 + (8.38 * servo_2_command);
            if (servo_tick_2 > 1583) { servo_tick_2 = 1583; }
            if (servo_tick_2 < 1416) { servo_tick_2 = 1416; }

            servo_set_pos(&servo_2, (uint32_t)servo_tick_2);       
        }
    }
}


TickType_t launch_start_time; 
TickType_t pyro_deploy_time;

TickType_t get_active_control_duration() {
    TickType_t current_time = xTaskGetTickCount();

    if (current_time <= launch_start_time + CONTROLS_START_DELAY) {
        return 0;
    } else {
        return current_time - launch_start_time - CONTROLS_START_DELAY;
    }
}

fsm_state_t run_state_machine(fsm_state_t current_state, uint8_t command, const state_estimation_packet_t* sensors) {
    TickType_t current_time = xTaskGetTickCount();

    if (command == ESTOP_COMMAND_ID) {
        return STASIS;
    } else if (command  == PYRO_DEPLOY) {
        HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);
        pyro_deploy_time = current_time;
        return PYRO_DEPLOY;
    }

    switch (current_state) {
        case GROUND:
            if (command == ARM_COMMAND_ID) {
                return ARMED;
            } else {
                return GROUND;
            }

        case ARMED:
            if (sensors->upward_accel > 16) {
                launch_start_time = current_time;
                return FAST_ASCENT;
            } else {
                return ARMED;
            }

         case FAST_ASCENT:
            if (current_time > CONTROLS_START_DELAY + launch_start_time) {
                return CONTROLLED_ASCENT;
            } else {
                return FAST_ASCENT;
            }

        case CONTROLLED_ASCENT:
            if (current_time > CHUTE_DEPLOYMENT_DELAY + launch_start_time) {
                HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);
                pyro_deploy_time = current_time;
                return PYRO_DEPLOY;
            } else {
                return CONTROLLED_ASCENT;
            }
            // TODO: figure out how to check rocket angle threshold for failure case

        case UNCONTROLLED_ASCENT: 
            if (current_time > CHUTE_DEPLOYMENT_DELAY + launch_start_time) {
                HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);
                pyro_deploy_time = current_time;
                return PYRO_DEPLOY;
            } else {
                return UNCONTROLLED_ASCENT;
            }

        case PYRO_DEPLOY:
            if (current_time > PYRO_TIME + pyro_deploy_time) {
                HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_RESET);
                return STASIS;
            } else {
                return PYRO_DEPLOY;
            }

        case STASIS: 
            if (command == FLASH_SD_COMMAND_ID) {
                return SD_FLASH;
            } else {
                return STASIS;
            }

        case SD_FLASH:
            return SD_FLASH;
    }

    return current_state;
}
