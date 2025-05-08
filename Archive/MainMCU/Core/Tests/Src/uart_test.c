#include "uart_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "globals.h"

void uart_test_task(void *args) {
    char *msg = "Hello World!\r\n";

    while (1) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) msg, 14, HAL_MAX_DELAY);

        vTaskDelay(1000);
    }
}