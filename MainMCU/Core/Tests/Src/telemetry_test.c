#include "telemetry_test.h"
#include "telemetry.h"
#include "stdint.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "globals.h"
#include "port_config.h"

extern uint16_t uart_rx_count;

void telemetry_test_task(void *args) {

    uint8_t rx[2];

    while (1) {
        char s[19];
        sprintf(s, "%d\r\n", uart_rx_count);
        HAL_UART_Transmit(&telemetry_uart, s, strlen(s), HAL_MAX_DELAY);

        vTaskDelay(1000);
    }
}