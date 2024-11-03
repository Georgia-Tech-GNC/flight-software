#include "telemetry_test.h"
#include "telemetry.h"
#include "stdint.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "globals.h"
#include "port_config.h"

void telemetry_test_task(void *args) {

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t message_id = 0x03;

    while (1) {
        HAL_UART_Transmit(&debug_uart, "Sending message\n", 16, HAL_MAX_DELAY);
        send_message(data, 4, message_id);

        vTaskDelay(1000);
    }
}