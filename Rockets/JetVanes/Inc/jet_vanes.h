#ifndef JET_VANES_H
#define JET_VANES_H

#include "FreeRTOS.h"

struct RocketState {
	uint8_t rocket_state;
	int64_t timestamp;
};
#define ROCKETSTATE_MSG_ID 10
#define ROCKETSTATE_SIZE 9
#define ROCKETSTATE_NUM_VALUES 2

struct ServoDeflections {
	uint16_t servo_1_desired;
	uint16_t servo_1_actual;
	uint16_t servo_2_desired;
	uint16_t servo_2_actual;
	uint16_t servo_3_desired;
	uint16_t servo_3_actual;
	uint16_t servo_4_desired;
	uint16_t servo_4_actual;
	int64_t timestamp;
};

typedef struct {
    struct RocketState rocket_state;
    struct ServoDeflections servo_deflections;
    uint32_t startup_timestamp; /* Not when the rocket powers on, but something like idle to ground, for example */
    uint32_t launch_timestamp; /* When the motor actually ignites */
} JetVanesRocketState;

#define SERVODEFLECTIONS_MSG_ID 11
#define SERVODEFLECTIONS_SIZE 24
#define SERVODEFLECTIONS_NUM_VALUES 9

#define N_TELEMETRY_MESSAGE_PACKETS 2

extern uint8_t telemetry_msg_ids[2];

#define IGNITE_COMMAND_ID 1
#define ZERO_SERVOS_COMMAND_ID 2
#define PING_ROCKET_COMMAND_ID 3

#define NUM_COMMAND_TYPES 3

extern StaticTask_t g_jet_vanes_task;

void jet_vanes_task(void *args);

#endif