#include "util.h"

#include <string.h>
#include "lib.h"
#include "log.h"
#include "rocket.h"
#include "rtos_globals.h"

const TickType_t SPINLOCK_TIMEOUT = portMAX_DELAY;

/** 
 * @brief pauses task execution until a notification is recieved at the specified index with a bit in the provided mask set.
 * @param index the notification index to use
 * @param mask the mask of bits to wait for
 * 
 * @note This method will resume task execution when _any_, not all bits in the mask are set.
 */
uint32_t await_notification_indexed(uint32_t index, uint32_t mask) {
    uint32_t notification_value = 0;
    while ((notification_value & mask) == 0) {
        xTaskNotifyWaitIndexed(index, 0, mask, &notification_value, SPINLOCK_TIMEOUT);
    }

    return notification_value;
}

/** 
 * @brief pauses task execution until a notification is recieved at index 0 with a bit in the provided mask set.
 * @param mask the mask of bits to wait for
 * 
 * @note This method will resume task execution when _any_, not all bits in the mask are set.
 */
uint32_t await_notification(uint32_t mask) {
    uint32_t notification_value = 0;
    while ((notification_value & mask) == 0) {
        xTaskNotifyWait(0, mask, &notification_value, SPINLOCK_TIMEOUT);
    }

    return notification_value;
}

/** 
 * @brief Acquires the global state mutex and copies the current rocket state into the provided buffer 
 * @param rocket_state_buffer the buffer where the current rocket state should be copied to.
 * @return returns RET_SUCCESS if the state was successfully copied, and RET_FAILURE if the mutex timed out before being acquired.
*/
uint8_t memcpy_state(const RocketStateStruct *rocket_state) {
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        memcpy(rocket_state, &g_current_state, sizeof(RocketStateStruct));
        xSemaphoreGive(g_state_mutex_handle);

        return RET_SUCCESS;
    }

    return RET_FAILURE;
}

/** 
 * @brief A "soft" assertion that prints the provided message via log_printf if the assertion value is false.
 * @param val the value to test
 * @param assert_name the message to print if the assertion fails.
 * @return the value passed in via the parameter val
*/
uint8_t rocket_assert(uint8_t val, const char *assert_name) {
    if (!val) {
        log_printf(LOG_ERROR, "Assert failed: '%s'", assert_name);
    }

    return val;
}