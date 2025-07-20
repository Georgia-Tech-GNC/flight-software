#include "telemetry_rx.h"

#include "stdint.h"
#include "stddef.h"
#include "telemetry.h"
#include "log.h"
#include "protocol.h"
#include "packet_encode.h"
#include "rtos_globals.h"
#include "commands.h"
#include "halal.h"

/* Private defines */
#define N_COMMAND_UUIDS 256

/* Private function definitions */
void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *extracted_buffer, uint8_t *packet_buffer_size, uint8_t *recieved_uuids);

/**
 * Task to handle incoming telemetry data
 * @param args Unused
 */
void telemetry_rx_task(void *args) {
    /* Buffer to hold received bytes before they are processed */
    uint8_t bytes_to_process[TELEMETRY_RX_MAX_PROCESS_SIZE];

    /* Buffer to hold the incoming packet before it is extracted */
    uint8_t packet_buffer[TELEMETRY_MAX_PACKET_SIZE];

    /* Buffer to hold the extracted packet */
    uint8_t extracted_packet_buffer[TELEMETRY_MAX_PACKET_SIZE];

    /* Buffer to hold the UUIDs of recieved commands */
    uint8_t recieved_uuids[N_COMMAND_UUIDS] = {0};

    /* Current size of incoming packet buffer */
    uint8_t packet_buffer_size = 0;

    if (HALAL_radio_start()) {
        log_printf(LOG_INFO, "Started radio RX");
    } else {
        log_printf(LOG_ERROR, "Error starting radio RX");
    }

    while (1) {
        /* Wait for new bytes to read */
        size_t bytes_read = xStreamBufferReceive(g_telemetry_rx_sb_handle, bytes_to_process, TELEMETRY_RX_MAX_PROCESS_SIZE, portMAX_DELAY);

        /* Process them */
        for (size_t i = 0; i < bytes_read; i ++) {
            rx_process_byte(bytes_to_process[i], packet_buffer, extracted_packet_buffer, &packet_buffer_size, recieved_uuids);
        }
    }
}

/**
 * Process a single byte of incoming data
 * @param byte The byte to process
 * @param packet_buffer The buffer to hold the incoming packet
 * @param packet_buffer_size The current size of the incoming packet buffer
 * @param extracted_buffer The buffer to hold the extracted packet
 */
void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *extracted_buffer, uint8_t *packet_buffer_size, uint8_t *recieved_uuids) {
    /* TODO: This function needs some commenting and/or refactoring */
    int next_packet_buffer_size = process_incoming_byte(byte, packet_buffer, *packet_buffer_size);
    log_printf(LOG_INFO, "Recieved byte: %02x", byte);

    if (next_packet_buffer_size < 0) {
        *packet_buffer_size = 0;
        next_packet_buffer_size *= -1;

        if (verify_packet(packet_buffer, next_packet_buffer_size)) {
            uint8_t message_id = extract_packet(packet_buffer, next_packet_buffer_size, extracted_buffer);

            if (message_id != COMMAND_MSG_ID) {
                return;
            }

            int command_id = extracted_buffer[0];
            int command_uuid = extracted_buffer[1];

            if (recieved_uuids[command_uuid] == 1) {
                return;
            }

            recieved_uuids[command_uuid] = 1;

            log_printf(LOG_INFO, "Command ID: %d, Command UUID: %d", command_id, command_uuid);

            uint8_t payload[2];
            payload[0] = command_id; // command id
            payload[1] = command_uuid; // command uuid

            telemetry_send_message(payload, 2, COMMAND_ACK_MSG_ID);

            process_command(command_id);
        }
    } else if (next_packet_buffer_size < TELEMETRY_MAX_PACKET_SIZE) {
        *packet_buffer_size = next_packet_buffer_size;
    } else {
        *packet_buffer_size = 0;
    }
}
