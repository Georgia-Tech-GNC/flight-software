#include "state_est_rx.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"

#include "globals.h"

/**
 * Task to receive new states and put them into the g_current_state global variable
 * @param args Unused
 */
void state_est_rx_task(void *args) {
    uint8_t _state_rx_buff[STATE_ESTIMATION_BYTES];
    uint8_t *state_rx_buff = _state_rx_buff;

    while(1) {
        size_t bytes_read = 0;

        /* Loop until we've received the full next state */
        while (bytes_read < STATE_ESTIMATION_BYTES) {
            size_t read = xStreamBufferReceive(g_state_rx_sb_handle, state_rx_buff + bytes_read, STATE_ESTIMATION_BYTES - bytes_read, portMAX_DELAY);
            bytes_read += read;
        }

        /* Copy state iestimation result into rocket state */
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            /* TODO: Do something here */
            xSemaphoreGive(g_state_mutex_handle);
        }
    }
}