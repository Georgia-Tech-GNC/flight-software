#include "state_tx.h"

#include "FreeRTOS.h"

#include "telemetry.h"

#include "stdint.h"
#include "stddef.h"
#include "lib.h"
#include "log.h"
#include "util.h"
#include "packet_encode.h"

/* Private defines */
#define TX_FREQ_HZ 5

/* Private function definitions */
uint8_t telemetry_send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id);

/**
 * @brief Task to handle transmitting state over telemetry
 * @param args Unused
 */
void state_tx_task(void *args) {
    UNUSED(args);

    uint8_t payload_buf[TELEMETRY_MAX_PAYLOAD_SIZE];

    RocketStateStruct local_state;

    /* Wait for start notification */
    await_notification(BEGIN_STATE_TX_NOTIFICATION_BIT);

    uint8_t start_msg_id = get_start_msg_id();
    uint8_t end_msg_id = get_end_msg_id();

    while (1) {
        await_notification(SEND_STATE_NOTIFICATION_BIT);

        memcpy_state(&local_state);

        for (uint8_t msg_id = start_msg_id; msg_id <= end_msg_id; msg_id ++) {           
            size_t payload_size;
            if (!packet_encode(msg_id, &local_state, payload_buf, TELEMETRY_MAX_PAYLOAD_SIZE, &payload_size)) {
                log_printf(LOG_ERROR, "Failed to encode telemetry packet id %d", msg_id);
            }
            
            if (!telemetry_send_message(payload_buf, payload_size, msg_id)) {
                log_printf(LOG_ERROR, "Failed to send telemetry packet id %d", msg_id);
            }
        }
    }
}