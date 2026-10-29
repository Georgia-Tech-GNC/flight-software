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
} FSM_State;

typedef struct {
    FSM_State state;
    uint64_t time_since_startup;
    float imu_velocity[3];
    float imu_acceleration[3];
    float orientation[4];
    float velocity[3];
    float position[3];
    float pyro;
    float servo1;
    float servo2;
} Telemetry_Packet_Send;

typedef struct {
    float velocity[3];
    float acceleration[3];
    float angular_velocity[3];
    float dt_seconds;
    float barometer_altitude;
    bool barometer_valid;
} Imu_Packet_Receive;

extern MessageBufferHandle_t imu_buffer_handle;

[[noreturn]] void shared_main();

#endif
