#include "protocol.h"
#include "globals.h"
#include "packet_encode.h"



void telemetry_task_handler(void* args) {
    while (1) {
        RocketState state;
        // Update global state
        if (xSemaphoreTake(g_state_lock.handle, pdMS_TO_TICKS(200)) == pdTRUE) {
            state = g_current_state;
            xSemaphoreGive(g_state_lock.handle);
        }

        
        struct RocketStatePacket packet_data = {
            .orientation_w = state.orientation.w,
            .orientation_x = state.orientation.x,
            .orientation_y = state.orientation.y,
            .orientation_z = state.orientation.z,
            .rocket_state = (uint8_t)state.state,
            .servo_cmd_1 = state.servo_cmd_1,
            .servo_cmd_2 = state.servo_cmd_2,
            .timestamp = state.timestamp
        };
        uint8_t payload[ROCKETSTATEPACKET_SIZE];
        RocketStatePacket_encode(&packet_data, payload);

        uint8_t packet[256];
        size_t packet_size = generate_packet(payload, ROCKETSTATEPACKET_SIZE, packet, ROCKETSTATEPACKET_MSG_ID);
        HAL_UART_Transmit(&telemetry_uart, packet, packet_size, HAL_MAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}