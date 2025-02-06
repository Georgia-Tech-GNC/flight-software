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
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_STATE_TX_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATE_TX_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    while (1) {
        uint32_t notification_value = 0;
        while ((notification_value & SEND_STATE_NOTIFICATION_BIT) == 0) {
            xTaskNotifyWait(0, SEND_STATE_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
        }

        /* Always use mutex with g_current_state */
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            /* Memcpy data out of global rocket state so we an give back mutex quickly */
            memcpy(&rocket_state, &g_current_state, sizeof(RocketState));
            xSemaphoreGive(g_state_mutex_handle);
        }

#ifndef STATIC_FIRE
        send_state_vector(&rocket_state, payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSTATEVECTOR_SIZE + 5) * MULT));
        send_servo_deflection(&rocket_state, payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSERVODEFLECTION_SIZE + 5) * MULT));
#endif
        send_state(&rocket_state, payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSTATE_SIZE + 5) * MULT));

#ifndef STATIC_FIRE
        /* Only send ground ekf at state (?) */
        if (rocket_state.rocket_state.rocket_state == 1) {
            send_ground_ekf(&rocket_state, payload_buf);
            vTaskDelay(pdMS_TO_TICKS((ROCKETGROUNDEKF_SIZE + 5) * MULT));
        }

        send_sensor_data(&rocket_state, payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSENSORDATA_SIZE + 5) * MULT));
        send_analog_feedback_data(&rocket_state, payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETANALOGFEEDBACKDATA_SIZE + 5) * MULT));
#endif
    }
}

/**
 * @brief Send the state vector over telemetry
 * @param rocket_state RocketState to send
 * @param payload_buf Buffer to write to
 */
#ifndef STATIC_FIRE
void send_state_vector(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketStateVector *state_vector = &rocket_state->state_vector;
    state_vector->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketStateVector_encode(state_vector, payload_buf);
    telemetry_send_message(payload_buf, ROCKETSTATEVECTOR_SIZE, ROCKETSTATEVECTOR_MSG_ID);
}
#endif

/**
 * @brief Send the servo deflection over telemetry
 * @param rocket_state RocketState to send
 * @param payload_buf Buffer to write to
 */
#ifndef STATIC_FIRE
void send_servo_deflection(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketServoDeflection *servo_deflection = &rocket_state->servo_deflection;
    servo_deflection->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketServoDeflection_encode(servo_deflection, payload_buf);
    telemetry_send_message(payload_buf, ROCKETSERVODEFLECTION_SIZE, ROCKETSERVODEFLECTION_MSG_ID);
}
#endif

/**
 * @brief Send the rocket state over telemetry
 * @param rocket_state RocketState to send
 * @param payload_buf Buffer to write to
 */
void send_state(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketState *_rocket_state = &rocket_state->rocket_state;
    _rocket_state->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketState_encode(_rocket_state, payload_buf);
    telemetry_send_message(payload_buf, ROCKETSTATE_SIZE, ROCKETSTATE_MSG_ID);
}

/**
 * @brief Send the ground ekf over telemetry
 * @param rocket_state RocketState to send
 * @param payload_buf Buffer to write to
 */
#ifndef STATIC_FIRE
void send_ground_ekf(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketGroundEKF *ground_ekf = &rocket_state->ground_ekf;
    ground_ekf->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketGroundEKF_encode(ground_ekf, payload_buf);
    telemetry_send_message(payload_buf, ROCKETGROUNDEKF_SIZE, ROCKETGROUNDEKF_MSG_ID);
}
#endif

/**
 * @brief Send the sensor data over telemetry
 * @param rocket_state RocketState to send
 * @param payload_buf Buffer to write to
 */
#ifndef STATIC_FIRE
void send_sensor_data(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketSensorData *sensor_data = &rocket_state->sensor_data;
    sensor_data->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketSensorData_encode(sensor_data, payload_buf);
    telemetry_send_message(payload_buf, ROCKETSENSORDATA_SIZE, ROCKETSENSORDATA_MSG_ID);
}
#endif

/**
 * @brief Send the analog feedback data over telemetry
 * @param rocket_state RocketState to send
 * @param payload_buf Buffer to write to
 */

#ifndef STATIC_FIRE
void send_analog_feedback_data(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketAnalogFeedbackData *analog_feedback_data = &rocket_state->analog_feedback_data;
    analog_feedback_data->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketAnalogFeedbackData_encode(analog_feedback_data, payload_buf);
    telemetry_send_message(payload_buf, ROCKETANALOGFEEDBACKDATA_SIZE, ROCKETANALOGFEEDBACKDATA_MSG_ID);
}
#endif
