#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"
#include "state_flash.h"
#include "state_tx.h"
#include "adc_convert.h"

#define TELEMETRY_SEND_TIMEOUT 100
#define TX_PACKET_BUFFER_SIZE 258
#define MAX_PACKET_SIZE_TELEMETRY 64
#define MAX_PACKET_SIZE_STATE 256
#define MAX_PACKETS_QUEUED 4
#define TX_MESSAGE_BUFFER_SIZE (TX_PACKET_BUFFER_SIZE * MAX_PACKETS_QUEUED)

typedef struct {
    uint8_t message_id;
    uint8_t payload_size;
    uint8_t *payload;
} Message;

void telemetry_rx_task(void *args);
void telemetry_tx_task(void *args);
int send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id);

#endif