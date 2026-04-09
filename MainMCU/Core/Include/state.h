#ifndef STATE_H
#define STATE_H

#include "stdint.h"
#include "stdbool.h"
#include "protocol.h"
#include "rocket_pid.h"

typedef enum FSMState {
    GROUND, ARMED, FAST_ASCENT, CONTROLLED_ASCENT, UNCONTROLLED_ASCENT, FREEFALL, SD_FLASH
} fsm_state_t;

typedef struct {
    bool arm_signal_recieved;
    fsm_state_t state;
    quaternion_t orientation;
    float w_x, w_y, w_z;
    float servo_cmd_1, servo_cmd_2;
    uint32_t timestamp;
} RocketState;

#define a sizeof(RocketState)


#endif