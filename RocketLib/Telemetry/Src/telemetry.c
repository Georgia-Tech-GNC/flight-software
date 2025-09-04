#include "telemetry.h"
#include "halal.h"
#include "radio.h"
#include "util.h"
#include "packet_encode.h"

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
 
    if (!HALAL_radio_transmit(packet_buf, packet_size, HAL_MAX_DELAY)) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}
