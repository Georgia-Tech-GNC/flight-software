#include "state_tx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "protocol.h"
#include "globals.h"

#include "telemetry.h"

void send_state_vector();
void send_servo_deflection();
void send_state();
void send_ground_ekf();
void send_sensor_data();
void send_analog_feedback_data();

void state_tx_task(void *args) {
    while (1) {
        send_state_vector();
        vTaskDelay(pdMS_TO_TICKS(1000 / TX_FREQ_HZ));
        send_servo_deflection();
        vTaskDelay(pdMS_TO_TICKS(1000 / TX_FREQ_HZ));
        send_state();
        vTaskDelay(pdMS_TO_TICKS(1000 / TX_FREQ_HZ));
        send_ground_ekf();
        vTaskDelay(pdMS_TO_TICKS(1000 / TX_FREQ_HZ));
        send_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(1000 / TX_FREQ_HZ));
        send_analog_feedback_data();
        vTaskDelay(pdMS_TO_TICKS(1000 / TX_FREQ_HZ));
    }
}

void send_state_vector() {
    struct RocketStateVector *state_vector = &g_current_state.state_vector;
    state_vector->timestamp = xTaskGetTickCount();

    uint8_t payload[ROCKETSTATEVECTOR_SIZE];
    RocketStateVector_encode(state_vector, payload);

    send_message(payload, ROCKETSTATEVECTOR_SIZE, ROCKETSTATEVECTOR_MSG_ID);
}

void send_servo_deflection() {
    struct RocketServoDeflection *servo_deflection = &g_current_state.servo_deflection;
    servo_deflection->timestamp = xTaskGetTickCount();

    uint8_t payload[ROCKETSERVODEFLECTION_SIZE];
    RocketServoDeflection_encode(servo_deflection, payload);

    send_message(payload, ROCKETSERVODEFLECTION_SIZE, ROCKETSERVODEFLECTION_MSG_ID);
}

void send_state() {
    struct RocketState *rocket_state = &g_current_state.rocket_state;
    rocket_state->timestamp = xTaskGetTickCount();

    uint8_t payload[ROCKETSTATE_SIZE];
    RocketState_encode(rocket_state, payload);

    send_message(payload, ROCKETSTATE_SIZE, ROCKETSTATE_MSG_ID);
}

void send_ground_ekf() {
    struct RocketGroundEKF *ground_ekf = &g_current_state.ground_ekf;
    ground_ekf->timestamp = xTaskGetTickCount();

    uint8_t payload[ROCKETGROUNDEKF_SIZE];
    RocketGroundEKF_encode(ground_ekf, payload);

    send_message(payload, ROCKETGROUNDEKF_SIZE, ROCKETGROUNDEKF_MSG_ID);
}

void send_sensor_data() {
    struct RocketSensorData *sensor_data = &g_current_state.sensor_data;
    sensor_data->timestamp = xTaskGetTickCount();

    uint8_t payload[ROCKETSENSORDATA_SIZE];
    RocketSensorData_encode(sensor_data, payload);

    send_message(payload, ROCKETSENSORDATA_SIZE, ROCKETSENSORDATA_MSG_ID);
}

void send_analog_feedback_data() {
    struct RocketAnalogFeedbackData *analog_feedback_data = &g_current_state.analog_feedback_data;
    analog_feedback_data->timestamp = xTaskGetTickCount();

    uint8_t payload[ROCKETANALOGFEEDBACKDATA_SIZE];
    RocketAnalogFeedbackData_encode(analog_feedback_data, payload);

    send_message(payload, ROCKETANALOGFEEDBACKDATA_SIZE, ROCKETANALOGFEEDBACKDATA_MSG_ID);
}