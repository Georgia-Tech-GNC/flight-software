#ifndef STATE_H
#define STATE_H

#include "stdint.h"
#include "protocol.h"

typedef struct {
    struct RocketStateVector state_vector;
    struct RocketServoDeflection servo_deflection;
    struct RocketState rocket_state;
    struct RocketGroundEKF ground_ekf;
    struct RocketSensorData sensor_data;
    struct RocketAnalogFeedbackData analog_feedback_data;
    uint64_t launch_timestamp;
} RocketState;


#endif