#ifndef SHARED_MAIN_H
#define SHARED_MAIN_H

#include "FreeRTOS.h"
#include "message_buffer.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GROUND,
    ARMED,
    FAST_ASCENT,
    CONTROLLED,
    UNCONTROLLED,
    PYRO,
    EMERGENCY_STOP,
} fsm_state_t;

typedef struct TelemetryPacketSend {
    fsm_state_t state;
    uint64_t time_since_startup;
    float imu_velocity[3];
    float imu_acceleration[3];
    float orientation[4];
    float velocity[3];
    float position[3];
    float pyro;
    float servo1;
    float servo2;
} telemetry_packet_send_t;

typedef struct ImuPacketReceive {
    float velocity[3];
    float acceleration[3];
    float angular_velocity[3];
    uint64_t timestep;
    float barometer_altitude;
    bool barometer_valid;
} imu_packet_receive_t;

extern MessageBufferHandle_t imu_buffer_handle;

[[noreturn]] void shared_main();

#endif
