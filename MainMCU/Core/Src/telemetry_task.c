
#include "globals.h"



void telemetry_task_handler(void* args) {
    while (1) {
        RocketState state;
        // Update global state
        if (xSemaphoreTake(g_state_lock.handle, pdMS_TO_TICKS(200)) == pdTRUE) {
            state = g_current_state;
            xSemaphoreGive(g_state_lock.handle);
        }

        
        char msg_buffer[256];
        size_t msg_size = sprintf(msg_buffer, "%.3f \t%.3f\r\n", 
            state.servo_cmd_1, state.servo_cmd_2
        );
        HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}