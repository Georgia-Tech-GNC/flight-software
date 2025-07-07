#include "util.h"

#include <string.h>
#include "lib.h"
#include "log.h"
#include "rocket.h"
#include "rtos_globals.h"

uint32_t await_notification_indexed(uint32_t index, uint32_t mask, TickType_t timeout) {
    uint32_t notification_value = 0;
    while ((notification_value & mask) == 0) {
        xTaskNotifyWaitIndexed(index, 0, mask, &notification_value, timeout);
    }

    return notification_value;
}

uint32_t await_notification(uint32_t mask, TickType_t timeout) {
    uint32_t notification_value = 0;
    while ((notification_value & mask) == 0) {
        xTaskNotifyWait(0, mask, &notification_value, timeout);
    }

    return notification_value;
}

uint8_t memcpy_state_bytes(uint8_t *rocket_state, size_t buffer_size, size_t *bytes_copied) {
    if (!rocket_assert(sizeof(RocketStateStruct) <= buffer_size, "Rocket state size less than buffer size")) {
        return 0;
    }

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        memcpy(rocket_state, &g_current_state, sizeof(RocketStateStruct));
        xSemaphoreGive(g_state_mutex_handle);

        return 1;
    }

    return 0;
}

uint8_t memcpy_state(RocketStateStruct *rocket_state) {
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        memcpy(rocket_state, &g_current_state, sizeof(RocketStateStruct));
        xSemaphoreGive(g_state_mutex_handle);

        return 1;
    }

    return 0;
}

uint8_t rocket_assert(uint8_t val, const char *assert_name) {
    if (!val) {
        log_printf(LOG_ERROR, "Assert failed: '%s'", assert_name);
    }

    return val;
}