#ifndef TELEMETRY_H
#define v

#include "stdint.h"

#define TELEMETRY_MAX_PAYLOAD_SIZE 64
#define TELEMETRY_MAX_PACKET_SIZE (TELEMETRY_MAX_PAYLOAD_SIZE + 5)

uint8_t telemetry_send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id);

#endif