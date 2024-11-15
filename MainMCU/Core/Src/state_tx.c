#include "state_tx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "protocol.h"
#include "globals.h"

#include "telemetry.h"

void send_state_vector(uint8_t *payload_buf);
void send_servo_deflection(uint8_t *payload_buf);
void send_state(uint8_t *payload_buf);
void send_ground_ekf(uint8_t *payload_buf);
void send_sensor_data(uint8_t *payload_buf);
void send_analog_feedback_data(uint8_t *payload_buf);

#define MULT 1

void state_tx_task(void *args) {
    uint8_t state_vector_payload_buf[ROCKETSTATEVECTOR_SIZE];
    uint8_t servo_deflection_payload_buf[ROCKETSERVODEFLECTION_SIZE];
    uint8_t state_payload_buf[ROCKETSTATE_SIZE] = {0};
    uint8_t ground_ekf_payload_buf[ROCKETGROUNDEKF_SIZE];
    uint8_t sensor_data_payload_buf[ROCKETSENSORDATA_SIZE];
    uint8_t analog_feedback_data_payload_buf[ROCKETANALOGFEEDBACKDATA_SIZE];

    while (1) {
        uint64_t timestamp = xTaskGetTickCount() / 100;
        
        struct RocketAnalogFeedbackData dummy_analog = {
            .voltage_fb_33 = timestamp * 2,
            .current_fb_33 = timestamp % 20,
            .pyro_0_cont = timestamp * 3,
            .pyro_1_cont = timestamp - 5,
            .pyro_2_cont = 0,
            .pyro_channel_deploy = 0,
            .timestamp = 0
        };

        g_current_state.analog_feedback_data = dummy_analog;

        struct RocketState dummy_state = {
            .rocket_state = timestamp % 64,
            .firing_channel_1 = timestamp,
            .firing_channel_2 = timestamp + 10,
            .firing_channel_3 = timestamp * 2,
            .timestamp = 0
        };

        g_current_state.rocket_state = dummy_state;
        

        send_state_vector(state_vector_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSTATEVECTOR_SIZE + 5) * MULT));
        send_servo_deflection(servo_deflection_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSERVODEFLECTION_SIZE + 5) * MULT));
        send_state(state_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSTATE_SIZE + 5) * MULT));
        send_ground_ekf(ground_ekf_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETGROUNDEKF_SIZE + 5) * MULT));
        send_sensor_data(sensor_data_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETSENSORDATA_SIZE + 5) * MULT));
        send_analog_feedback_data(analog_feedback_data_payload_buf);
        vTaskDelay(pdMS_TO_TICKS((ROCKETANALOGFEEDBACKDATA_SIZE + 5) * MULT));
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "State TX\n", 9, HAL_MAX_DELAY);
    }
}

void send_state_vector(uint8_t *payload_buf) {
    struct RocketStateVector *state_vector = &g_current_state.state_vector;
    state_vector->timestamp = xTaskGetTickCount();

    RocketStateVector_encode(state_vector, payload_buf);
    send_message(payload_buf, ROCKETSTATEVECTOR_SIZE, ROCKETSTATEVECTOR_MSG_ID);
}

void send_servo_deflection(uint8_t *payload_buf) {
    struct RocketServoDeflection *servo_deflection = &g_current_state.servo_deflection;
    servo_deflection->timestamp = xTaskGetTickCount();

    RocketServoDeflection_encode(servo_deflection, payload_buf);
    send_message(payload_buf, ROCKETSERVODEFLECTION_SIZE, ROCKETSERVODEFLECTION_MSG_ID);
}

void send_state(uint8_t *payload_buf) {
    struct RocketState *rocket_state = &g_current_state.rocket_state;
    rocket_state->timestamp = xTaskGetTickCount();

    RocketState_encode(rocket_state, payload_buf);
    send_message(payload_buf, ROCKETSTATE_SIZE, ROCKETSTATE_MSG_ID);
}

void send_ground_ekf(uint8_t *payload_buf) {
    struct RocketGroundEKF *ground_ekf = &g_current_state.ground_ekf;
    ground_ekf->timestamp = xTaskGetTickCount();

    RocketGroundEKF_encode(ground_ekf, payload_buf);
    send_message(payload_buf, ROCKETGROUNDEKF_SIZE, ROCKETGROUNDEKF_MSG_ID);
}

void send_sensor_data(uint8_t *payload_buf) {
    struct RocketSensorData *sensor_data = &g_current_state.sensor_data;
    sensor_data->timestamp = xTaskGetTickCount();

    RocketSensorData_encode(sensor_data, payload_buf);
    send_message(payload_buf, ROCKETSENSORDATA_SIZE, ROCKETSENSORDATA_MSG_ID);
}

void send_analog_feedback_data(uint8_t *payload_buf) {
    struct RocketAnalogFeedbackData *analog_feedback_data = &g_current_state.analog_feedback_data;
    analog_feedback_data->timestamp = xTaskGetTickCount();

    RocketAnalogFeedbackData_encode(analog_feedback_data, payload_buf);
    send_message(payload_buf, ROCKETANALOGFEEDBACKDATA_SIZE, ROCKETANALOGFEEDBACKDATA_MSG_ID);
}