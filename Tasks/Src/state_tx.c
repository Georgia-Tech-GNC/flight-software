#include "state_tx.h"

void send_state_vector(RocketState *rocket_state, uint8_t *payload_buf);
void send_servo_deflection(RocketState *rocket_state, uint8_t *payload_buf);
void send_state(RocketState *rocket_state, uint8_t *payload_buf);
void send_ground_ekf(RocketState *rocket_state, uint8_t *payload_buf);
void send_sensor_data(RocketState *rocket_state, uint8_t *payload_buf);
void send_analog_feedback_data(RocketState *rocket_state, uint8_t *payload_buf);

#define MULT 1

/**
 * @brief Task to handle transmitting state over telemetry
 * @param args Unused
 */
void state_tx_task(void *args) {
    uint8_t payload_buf[TELEMETRY_MAX_PAYLOAD_SIZE];

    RocketState rocket_state;

    /* Wait for start notification */
    await_notification(BEGIN_STATE_TX_NOTIFICATION_BIT)
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_STATE_TX_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATE_TX_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    while (1) {
        uint32_t notification_value = 0;
        while ((notification_value & SEND_STATE_NOTIFICATION_BIT) == 0) {
            xTaskNotifyWait(0, SEND_STATE_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
        }

        memcpy_state(&rocket_state);

        for (uint8_t i = 0; i < N_TELEMETRY_STATE_PACKETS; i ++) {
            uint32_t timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
            lib_packet_encode(telemetry_state_packets[i], &rocket_state, payload_buf);
        }
    }
}