#include "state_rx.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"

#include "globals.h"

uint8_t _g_current_state[STATE_STRUCT_SIZE];
uint8_t *g_current_state = _g_current_state;

/**
 * Task to receive new states and put them into the g_current_state global variable
 * @param args Unused
 */
void state_rx_task(void *args) {
    uint8_t _state_rx_buff[STATE_STRUCT_SIZE];
    uint8_t *state_rx_buff = _state_rx_buff;

    while(1) {
        size_t total_read = 0;

        /* Loop until we've received the full next state */
        while (total_read < STATE_STRUCT_SIZE) {
            size_t read = xStreamBufferReceive(g_state_rx_sb_handle, state_rx_buff + total_read, STATE_STRUCT_SIZE - total_read, portMAX_DELAY);
            total_read += read;
        }

        /* Swap buffers */
        if (xSemaphoreTake(g_state_mutex_handle, STATE_SEMAPHORE_TIMEOUT) == pdTRUE) {
            uint8_t *temp = g_current_state;
            g_current_state = state_rx_buff;
            state_rx_buff = temp;

            xSemaphoreGive(g_state_mutex_handle);
        }
    }
}