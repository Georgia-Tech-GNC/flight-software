#include "globals.h"
#include "string.h"
#include "rocket_pid.h"

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

void master_task_handler(void* args) {
    HAL_UART_Transmit(&debug_uart, "Started master task!\r\n", 22, HAL_MAX_DELAY);

    TickType_t last_servo_update = xTaskGetTickCount();
    TickType_t start_time = last_servo_update;
    TickType_t servo_update_period = pdMS_TO_TICKS(20);

    PIDController controller;
    RefTrajectory trajectory = {.count = 1, .time = 0, .quat = (quaternion_t){.w = 1, .x = 0, .y = 0, .z = 0}};
    pid_init(&controller, 1.0, 0.0, 0.0, 0.05, &trajectory);

    while (1) {
        uint16_t state_rx_buffer[8];
        xStreamBufferReceive(g_state_stream_buffer.handle, (uint8_t*)state_rx_buffer, 16, portMAX_DELAY);
        state_estimation_packet_t state_data = read_state_estimation_packet(state_rx_buffer);
        
        TickType_t current_time = xTaskGetTickCount();

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
        getControl(&controller, state, current_time - start_time, output);
        
        if (current_time > last_servo_update + servo_update_period) {
            last_servo_update = current_time;

            char buffer[256];
            size_t sz = sprintf(buffer, "Orientation: %.4f %.4f %.4f %.4f Output: %.4f %.4f %.4f", state_data.orientation.w,
                state_data.orientation.x,
                state_data.orientation.y,
                state_data.orientation.z,
                RAD2DEG(output[0]),
                RAD2DEG(output[1]),
                RAD2DEG(output[2])
            );
            HAL_UART_Transmit(&debug_uart, buffer, sz, HAL_MAX_DELAY);

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