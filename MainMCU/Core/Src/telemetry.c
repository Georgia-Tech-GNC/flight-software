#include "telemetry.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

#include "packet_encode.h"
#include "protocol.h"

#include "globals.h"

void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *packet_buffer_size, uint8_t *extracted_buffer);
int uart_transmit_message(Message *message, uint8_t *packet_buf);

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

    HAL_UART_Transmit(&debug_uart, "Going insane\r\n", 14, HAL_MAX_DELAY);

    /* 0x61 0x62 0x64 0x65 0x66 0x67 0x0D 0x0A */

    uint8_t foo_broken[] = {0x61U, 0x62U, 0x63U, 0x64U, 0x65U, 0x66U, 0x67U, 0x0DU, 0x0AU, 0x00U};

    /*
    if (strcmp(foo_broken, foo_working) == 0) {
        HAL_UART_Transmit(&debug_uart, "Strings are equal\r\n", 19, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, "Strings are not equal\r\n", 23, HAL_MAX_DELAY);
    }

    for (int i = 0; i < packet_size; i++) {
        char buf[100];

        sprintf(buf, "%02X ", foo_send[i]);
        HAL_UART_Transmit(&debug_uart, buf, strlen(buf), HAL_MAX_DELAY);
    }
    */

    if (HAL_UART_Transmit(&telemetry_uart, "kms\r\n", 5, HAL_MAX_DELAY) != HAL_OK) {
        HAL_UART_Transmit(&debug_uart, "bad", 3, HAL_MAX_DELAY);
        return 0;
    } else {
        HAL_UART_Transmit(&debug_uart, "good", 4, HAL_MAX_DELAY);
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

    /* Buffer to hold the extracted packet */
    uint8_t extracted_buffer[9];

    /* Buffer to hold the incoming packet before it is extracted */
    uint8_t packet_buffer[9];

    /* Current size of incoming packet buffer */
    uint8_t packet_buffer_size = 0;

    while (1) {
        /* Wait for new bytes to read */
        int bytes_read = xStreamBufferReceive(g_telemetry_rx_sb_handle, bytes_to_process, 128, portMAX_DELAY);
        
        /* Process them */
        for (int i = 0; i < bytes_read; i ++) {
            rx_process_byte(bytes_to_process[i], packet_buffer, &packet_buffer_size, extracted_buffer);
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
void rx_process_byte(uint8_t byte, uint8_t *packet_buffer, uint8_t *packet_buffer_size, uint8_t *extracted_buffer) {
    int next_packet_buffer_size = process_incoming_byte(byte, packet_buffer, *packet_buffer_size);

    if (next_packet_buffer_size < 0) {
        next_packet_buffer_size = -next_packet_buffer_size;

        if (verify_packet(packet_buffer, next_packet_buffer_size)) {
            int extracted_buffer_size = next_packet_buffer_size - 5;
            int message_id = extract_packet(packet_buffer, next_packet_buffer_size, extracted_buffer);

            if (message_id != 1) return;
        }
    } else {
        *packet_buffer_size = next_packet_buffer_size;
    }
}