#include "globals.h"
#include "string.h"


void master_task_handler(void* args) {
    HAL_UART_Transmit(&debug_uart, "Started master task!\r\n", 22, HAL_MAX_DELAY);

    while (1) {
        uint8_t state_rx_buffer[16];
        xStreamBufferReceive(g_state_stream_buffer.handle, state_rx_buffer, 16, portMAX_DELAY);

        float a, b, c, d;
        memcpy(&a, state_rx_buffer, sizeof(float));
        memcpy(&b, state_rx_buffer + 4, sizeof(float));
        memcpy(&c, state_rx_buffer + 8, sizeof(float));
        memcpy(&d, state_rx_buffer + 12, sizeof(float));

        char msg_buffer[256];
        size_t msg_size = sprintf(msg_buffer, "%.4f %.4f %.4f %.4f\r\n", a, b, c, d);
        HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);
    }
}