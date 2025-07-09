#include "jet_vanes.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "log.h"
#include "rtos_globals.h"
#include "state_flash.h"

JetVanesRocketState g_current_state = {0};

void jet_vanes_task(void *args) {
    uint16_t iters = 0;

    uint16_t seconds = 0;

    while (1) {
        if (iters % 100 == 0) {
            xTaskNotifyIndexed(g_state_flash_task_handle, FLASH_NOTIFICATION_INDEX, FLASH_STATE_NOTIFICATION_BIT, eSetBits);
        }

        if ((iters++) == 1000) {
            log_printf(LOG_INFO, "Running!");
            iters = 0;
            seconds ++;
        }

        usb_host_process();

        vTaskDelay(pdMS_TO_TICKS(1));

        if (seconds == 10) {
            log_printf(LOG_INFO, "Writing disk");
            xTaskNotifyIndexed(g_state_flash_task_handle, FLASH_NOTIFICATION_INDEX, FLASH_SD_CARD_NOTIFICATION_BIT, eSetBits);
            break;
        }
    }

    while (1);
}   