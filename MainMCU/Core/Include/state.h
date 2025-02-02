#ifndef STATE_H
#define STATE_H

#include "stdint.h"
#include "protocol.h"

typedef struct {
#ifndef STATIC_FIRE
    struct RocketStateVector state_vector;
    struct RocketServoDeflection servo_deflection;
#endif
    struct ServoDeflections servo_deflections;
    struct RocketState rocket_state;
#ifndef STATIC_FIRE
    struct RocketGroundEKF ground_ekf;
    struct RocketSensorData sensor_data;
    struct RocketAnalogFeedbackData analog_feedback_data;
#endif
    uint64_t launch_timestamp;
} RocketState;


#endif