#include "blink.h"

#include "FreeRTOS.h"
#include "task.h"

#include "port_config.h"

void blink_task(void *args) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        vTaskDelay(1000);
    }
}