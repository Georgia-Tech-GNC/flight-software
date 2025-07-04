#include "state_tx.h"

#include "stdint.h"
#include "stddef.h"
#include "lib.h"
#include "log.h"
#include "hal_modules.h"

/* Private defines */
#define TX_FREQ_HZ 5
#define TELEMETRY_MAX_PAYLOAD_SIZE 64
#define TELEMETRY_MAX_PACKET_SIZE (TELEMETRY_MAX_PAYLOAD_SIZE + 5)

/* Private function definitions */
uint8_t telemetry_send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id);

/**
 * @brief Task to handle transmitting state over telemetry
 * @param args Unused
 */
void state_tx_task(void *args) {
    uint8_t payload_buf[TELEMETRY_MAX_PAYLOAD_SIZE];

    RocketState local_state;

    /* Wait for start notification */
    await_notification(BEGIN_STATE_TX_NOTIFICATION_BIT);

    while (1) {
        await_notification(SEND_STATE_NOTIFICATION_BIT);

        memcpy_state(&local_state);

        for (uint8_t i = 0; i < N_TELEMETRY_MESSAGE_PACKETS; i ++) {
            uint32_t timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
            uint8_t packet_id = telemetry_msg_ids[i];
            
            size_t payload_size;
            if (!lib_packet_encode(packet_id, &local_state, payload_buf, TELEMETRY_MAX_PAYLOAD_SIZE, &payload_size)) {
                log_printf(LOG_ERROR, "Failed to encode telemetry packet id %d", packet_id);
            }
            
            if (!telemetry_send_message(payload_buf, payload_size, packet_id)) {
                log_printf(LOG_ERROR, "Failed to send telemetry packet id %d", packet_id);
            }
        }
    }
}

/**
 * Transmit a telemetry packet over uart
 * @param payload The payload of the message
 * @param payload_size The size of the payload
 * @param message_id The message id of the message
 * @return 1 if the message was sent successfully, 0 otherwise
 */
uint8_t telemetry_send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id) {
    uint8_t packet_buf[TELEMETRY_MAX_PACKET_SIZE];
    int packet_size = generate_packet(payload, payload_size, packet_buf, message_id);    
 
    if (HAL_UART_Transmit(&telemetry_uart, packet_buf, packet_size, HAL_MAX_DELAY) != HAL_OK) {
        return 0;
    }

    return 1;
}
