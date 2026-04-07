#ifndef STATE_H
#define STATE_H

#include "stdint.h"
#include "protocol.h"

typedef struct {
    float attitude_x;
    float attitude_y;
    float attitude_z;
    float velocity_y;
    float servo_cmd_1;
    float servo_cmd_2;
    uint64_t launch_timestamp;
} RocketState;


#endif