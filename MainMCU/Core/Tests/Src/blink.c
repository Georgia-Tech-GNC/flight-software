#include "blink.h"

#include "FreeRTOS.h"
#include "task.h"

#include "port_config.h"

void blink_task(void *args) {
    while (1) {
        HAL_GPIO_TogglePin(LD3_GPIO_PORT, LD3_PIN);
        vTaskDelay(1000);
    }
}