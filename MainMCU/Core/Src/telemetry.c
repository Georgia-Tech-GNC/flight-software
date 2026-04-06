#include <string.h>

#include "telemetry.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

#include "packet_encode.h"
#include "protocol.h"

#include "globals.h"

uint8_t packet_buffer[TELEMETRY_MAX_PACKET_SIZE];

/**
 * Transmit a telemetry packet over uart
 * @param payload The payload of the message
 * @param payload_size The size of the payload
 * @param message_id The message id of the message
 * @return 1 if the message was sent successfully, 0 otherwise
 */
int telemetry_send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id) {
    uint8_t packet_buf[TELEMETRY_MAX_PACKET_SIZE];
    int packet_size = generate_packet(payload, payload_size, packet_buf, message_id);    
 
    if (HAL_UART_Transmit(&telemetry_uart, packet_buf, packet_size, HAL_MAX_DELAY) != HAL_OK) {
        return 0;
    }

    return 1;
}
