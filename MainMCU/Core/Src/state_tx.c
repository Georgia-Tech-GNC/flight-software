#include "state_tx.h"

void send_state_vector(RocketState *rocket_state, uint8_t *payload_buf);
void send_servo_deflection(RocketState *rocket_state, uint8_t *payload_buf);
void send_state(RocketState *rocket_state, uint8_t *payload_buf);
void send_ground_ekf(RocketState *rocket_state, uint8_t *payload_buf);
void send_sensor_data(RocketState *rocket_state, uint8_t *payload_buf);
void send_analog_feedback_data(RocketState *rocket_state, uint8_t *payload_buf);

#define MULT 1

void state_tx_task(void *args) {
    uint8_t state_vector_payload_buf[ROCKETSTATEVECTOR_SIZE];
    uint8_t servo_deflection_payload_buf[ROCKETSERVODEFLECTION_SIZE];
    uint8_t state_payload_buf[ROCKETSTATE_SIZE];
    uint8_t ground_ekf_payload_buf[ROCKETGROUNDEKF_SIZE];
    uint8_t sensor_data_payload_buf[ROCKETSENSORDATA_SIZE];
    uint8_t analog_feedback_data_payload_buf[ROCKETANALOGFEEDBACKDATA_SIZE];

    RocketState rocket_state;

    /*
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_STATE_TX_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATE_TX_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }
    */

    vTaskDelay(pdMS_TO_TICKS(1000));

    HAL_UART_Transmit(&debug_uart, (uint8_t *)"Beginning telemetry TX\n", 24, 1000);

    while (1) {
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            memcpy(&rocket_state, &g_current_state, sizeof(RocketState));
            xSemaphoreGive(g_state_mutex_handle);
        }

        send_state_vector(&rocket_state, state_vector_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSTATEVECTOR_SIZE + 5) * MULT));
        send_servo_deflection(&rocket_state, servo_deflection_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSERVODEFLECTION_SIZE + 5) * MULT));
        send_state(&rocket_state, state_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSTATE_SIZE + 5) * MULT));

        if (rocket_state.rocket_state.rocket_state == 1) {
            send_ground_ekf(&rocket_state, ground_ekf_payload_buf);
            vTaskDelay(pdMS_TO_TICKS((ROCKETGROUNDEKF_SIZE + 5) * MULT));
        }

        send_sensor_data(&rocket_state, sensor_data_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSENSORDATA_SIZE + 5) * MULT));
        send_analog_feedback_data(&rocket_state, analog_feedback_data_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETANALOGFEEDBACKDATA_SIZE + 5) * MULT));

        vTaskDelay(200);
    }
}

void send_state_vector(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketStateVector *state_vector = &rocket_state->state_vector;
    state_vector->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketStateVector_encode(state_vector, payload_buf);
    send_message(payload_buf, ROCKETSTATEVECTOR_SIZE, ROCKETSTATEVECTOR_MSG_ID);
}

void send_servo_deflection(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketServoDeflection *servo_deflection = &rocket_state->servo_deflection;
    servo_deflection->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketServoDeflection_encode(servo_deflection, payload_buf);
    send_message(payload_buf, ROCKETSERVODEFLECTION_SIZE, ROCKETSERVODEFLECTION_MSG_ID);
}

void send_state(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketState *_rocket_state = &rocket_state->rocket_state;
    _rocket_state->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketState_encode(_rocket_state, payload_buf);
    send_message(payload_buf, ROCKETSTATE_SIZE, ROCKETSTATE_MSG_ID);
}

void send_ground_ekf(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketGroundEKF *ground_ekf = &rocket_state->ground_ekf;
    ground_ekf->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketGroundEKF_encode(ground_ekf, payload_buf);
    send_message(payload_buf, ROCKETGROUNDEKF_SIZE, ROCKETGROUNDEKF_MSG_ID);
}

void send_sensor_data(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketSensorData *sensor_data = &rocket_state->sensor_data;
    sensor_data->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketSensorData_encode(sensor_data, payload_buf);
    send_message(payload_buf, ROCKETSENSORDATA_SIZE, ROCKETSENSORDATA_MSG_ID);
}

void send_analog_feedback_data(RocketState *rocket_state, uint8_t *payload_buf) {
    struct RocketAnalogFeedbackData *analog_feedback_data = &rocket_state->analog_feedback_data;
    analog_feedback_data->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

    RocketAnalogFeedbackData_encode(analog_feedback_data, payload_buf);
    send_message(payload_buf, ROCKETANALOGFEEDBACKDATA_SIZE, ROCKETANALOGFEEDBACKDATA_MSG_ID);
}