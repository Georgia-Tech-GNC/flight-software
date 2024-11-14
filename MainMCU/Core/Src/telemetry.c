#include <string.h>

#include "telemetry.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

#include "packet_encode.h"
#include "protocol.h"

#include "globals.h"

void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *packet_buffer_size);
int uart_transmit_message(Message *message, uint8_t *packet_buf);

void process_command(int command_id);

void command_idle_to_ground();
void command_fire_pyro();
void command_flash_sd_card();
void command_run_vane_activation_test();

void telemetry_tx_task(void *args) {
    /* Buffer to store the generated packet */
    uint8_t tx_packet_buffer[TX_PACKET_BUFFER_SIZE];
    /* Temporary buffer to store what is received from the tx queue message buffer */
    uint8_t tx_temp_message_buffer[TX_PACKET_BUFFER_SIZE];
    
    while (1) {
        xMessageBufferReceive(g_telemetry_tx_mb_handle, tx_temp_message_buffer, TX_PACKET_BUFFER_SIZE, portMAX_DELAY);

        Message *message = (Message *) tx_temp_message_buffer;
        uart_transmit_message(message, tx_packet_buffer);
    }
}

/**
 * Queue a message to be sent
 * @param payload The payload of the message
 * @param payload_size The size of the payload
 * @param message_id The message id
 * @return 1 if the message was queued successfully, 0 otherwise
 */
int send_message(uint8_t *payload, uint8_t payload_size, uint8_t message_id) { 
    Message message = {
        .message_id = message_id,
        .payload_size = payload_size,
        .payload = payload
    };

    size_t bytes_sent = xMessageBufferSend(g_telemetry_tx_mb_handle, &message, sizeof(Message), portMAX_DELAY);
    
    if (bytes_sent != sizeof(Message)) {
        return 0;
    }

    return 1;
}

/**
 * Actually transmit a telemetry packet over uart
 * @param header The header of the message containing the message id and payload size
 * @param payload The payload of the message
 * @param packet_buf The buffer to hold the packet
 * @return 1 if the message was sent successfully, 0 otherwise
 */
int uart_transmit_message(Message *message, uint8_t *packet_buf) {
    int packet_size = generate_packet(message->payload, message->payload_size, packet_buf, message->message_id);    

    if (HAL_UART_Transmit_IT(&telemetry_uart, packet_buf, packet_size) != HAL_OK) {
        return 0;
    }

    return 1;
}

/**
 * Task to handle incoming telemetry data
 * @param args Unused
 */
void telemetry_rx_task(void *args) {
    /* Buffer to hold received bytes before they are processed */
    uint8_t bytes_to_process[16];

    /* Buffer to hold the incoming packet before it is extracted */
    uint8_t packet_buffer[9];

    /* Current size of incoming packet buffer */
    uint8_t packet_buffer_size = 0;

    while (1) {
        /* Wait for new bytes to read */
        int bytes_read = xStreamBufferReceive(g_telemetry_rx_sb_handle, bytes_to_process, 16, portMAX_DELAY);
        
        /* Process them */
        for (int i = 0; i < bytes_read; i ++) {
            rx_process_byte(bytes_to_process[i], packet_buffer, &packet_buffer_size);
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
void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *packet_buffer_size) {
    int next_packet_buffer_size = process_incoming_byte(byte, packet_buffer, *packet_buffer_size);

    if (next_packet_buffer_size < 0) {
        *packet_buffer_size = 0;
        next_packet_buffer_size *= -1;

        if (verify_packet(packet_buffer, next_packet_buffer_size)) {
            int command_id = is_command_packet(packet_buffer, next_packet_buffer_size);
            if (command_id == -1) {
                return;
            }

            uint8_t payload[2];
            payload[0] = command_id; // command id
            payload[1] = get_command_uuid(packet_buffer); // command uuid
            send_message(payload, 2, COMMAND_ACK_MSG_ID);

            process_command(command_id);
        }
    } else if (next_packet_buffer_size < MAX_PACKET_SIZE_TELEMETRY) {
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
        case ROCKET_IDLE_TO_GROUND_COMMAND_ID:
            command_idle_to_ground();
            break;
        case ROCKET_FIRE_PYRO_COMMAND_ID:
            command_fire_pyro();
            break;
        case ROCKET_FLASH_SD_CARD_COMMAND_ID:
            command_flash_sd_card();
            break;
        case ROCKET_RUN_VANE_ACTIVATION_TEST_COMMAND_ID:
            command_run_vane_activation_test();
            break;
    }
}

void command_idle_to_ground() {
    HAL_UART_Transmit_IT(&state_uart, (uint8_t *) "GO", 2);
}

void command_fire_pyro() {
    /* Unimplemented */
}

void command_flash_sd_card() {
    /* Unimplemented */
}

void command_run_vane_activation_test() {
    /* Unimplemented */
}