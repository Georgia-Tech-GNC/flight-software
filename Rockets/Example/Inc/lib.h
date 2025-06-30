#ifndef LIB_H
#define LIB_H


/* 
 * TODO: where did the definitions for RocketStateVector, RocketServoDeflection, etc. go?? 
 * When found, they should go in this file
 */
typedef struct {
    struct RocketStateVector state_vector;
    struct RocketServoDeflection servo_deflection;
    struct ServoDeflections servo_deflections;
    struct RocketState rocket_state;
    struct RocketGroundEKF ground_ekf;
    struct RocketSensorData sensor_data;
    struct RocketAnalogFeedbackData analog_feedback_data;
    uint64_t launch_timestamp;
} RocketState;


uint8_t lib_packet_encode(uint8_t packet_id, RocketState rocket_state, uint8_t *payload_buf, size_t payload_buf_size);

uint8_t lib_csv_encode(RocketState *rocket_state, uint8_t *csv_line, size_t csv_line_size, size_t *bytes_written);

#endif