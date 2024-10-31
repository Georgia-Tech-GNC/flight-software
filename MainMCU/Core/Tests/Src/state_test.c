#include "state_test.h"

#include "globals.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

void state_test_task(void *args) {
    char tx_message[14];

    char *test_msg = "Hello World!\r\n";

    while (1) {
        vTaskDelay(100);
        
        /*
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            memcpy(g_current_state, tx_message, strlen(tx_message));
            xSemaphoreGive(g_state_mutex_handle);
        }
        */

        HAL_UART_Transmit(&huart2, (uint8_t *) test_msg, strlen(test_msg), HAL_MAX_DELAY);
    }

}