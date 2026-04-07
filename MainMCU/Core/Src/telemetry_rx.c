#include "telemetry_rx.h"

#ifdef DO_NOT_RUN
void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *extracted_buffer, uint8_t *packet_buffer_size, uint8_t *recieved_uuids);

void process_command(int command_id);

void command_idle_to_ground();
void command_fire_pyro();
void command_flash_sd_card();
void command_ignite();
void command_zero_servos();

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
    uint8_t recieved_uuids[256] = {0};

    /* Current size of incoming packet buffer */
    uint8_t packet_buffer_size = 0;

    while (1) {
        /* Wait for new bytes to read */
        int bytes_read = xStreamBufferReceive(g_telemetry_rx_sb_handle, bytes_to_process, TELEMETRY_RX_MAX_PROCESS_SIZE, portMAX_DELAY);

        /* Process them */
        for (int i = 0; i < bytes_read; i ++) {
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
    int next_packet_buffer_size = process_incoming_byte(byte, packet_buffer, *packet_buffer_size);
    char buf[100];
    sprintf(buf, "Recieved byte: %02x\r\n", byte);
    HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);

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

            char buf2[100];
            sprintf(buf2, "Command ID: %d, Command UUID: %d\r\n", command_id, command_uuid);
            HAL_UART_Transmit(&debug_uart, (uint8_t *) buf2, strlen(buf2), HAL_MAX_DELAY);

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

/**
 * Process ground station command
 * @param command_id The id of the command to be processed
 */
void process_command(int command_id) {
    switch (command_id) {
#ifndef STATIC_FIRE
        case ROCKET_IDLE_TO_GROUND_COMMAND_ID:
            command_idle_to_ground();
            break;
        case ROCKET_FIRE_PYRO_COMMAND_ID:
            command_fire_pyro();
            break;
        case ROCKET_FLASH_SD_CARD_COMMAND_ID:
            command_flash_sd_card();
            break;
#else
        case IGNITE_COMMAND_ID:
            command_ignite();
            break;
        case ZERO_SERVOS_COMMAND_ID:
            command_zero_servos();
            break;
#endif
    }
}

/**
 * @brief Perform the idle to ground command
 */
void command_idle_to_ground() {
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Idle to ground\r\n", 17, HAL_MAX_DELAY);

    HAL_UART_Transmit(&state_uart, (uint8_t *) "GO", 2, HAL_MAX_DELAY);

    xTaskNotify(g_state_tx_task_handle, BEGIN_STATE_TX_NOTIFICATION_BIT, eSetBits);
}

/**
 * @brief Perform the fire pyro command
 */
void command_fire_pyro() {
    /* Unimplemented */
}

/**
 * @brief Perform the flash SD card command
 */
void command_flash_sd_card() {
    xTaskNotify(g_state_flash_task_handle, FLASH_SD_CARD_NOTIFICATION_BIT, eSetBits);
}

/**
 * @brief Perform the vane activation test command
 */
void command_ignite() {
    xTaskNotify(g_state_tx_task_handle, BEGIN_STATE_TX_NOTIFICATION_BIT, eSetBits);
    xTaskNotify(g_state_flash_task_handle, BEGIN_STATE_FLASH_NOTIFICATION_BIT, eSetBits);
    xTaskNotify(g_static_fire_task_handle, BEGIN_STATIC_FIRE_NOTIFICATION_BIT, eSetBits);
}

/**
 * @brief Perform the zero servos command
 */
void command_zero_servos() {
    xTaskNotify(g_static_fire_task_handle, ZERO_SERVOS_NOTIFICATION_BIT, eSetBits);
}
#endif