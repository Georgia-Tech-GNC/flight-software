#ifndef JET_VANES_H
#define JET_VANES_H

#include "FreeRTOS.h"

#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

#define ROCKET_IDLE_TO_GROUND_COMMAND_ID 1
#define ROCKET_FIRE_PYRO_COMMAND_ID 2
#define ROCKET_FLASH_SD_CARD_COMMAND_ID 3
#define PING_ROCKET_COMMAND_ID 4

#define NUM_COMMAND_TYPES 4

struct RocketStateVector {
	float velocity_x;
	float velocity_y;
	float velocity_z;
	float attitude_w;
	float attitude_x;
	float attitude_y;
	float attitude_z;
	float position_x;
	float position_y;
	float position_z;
	float world_x;
	float world_y;
	float world_z;
	int64_t timestamp;
};
#define ROCKETSTATEVECTOR_MSG_ID 10
#define ROCKETSTATEVECTOR_SIZE 60
#define ROCKETSTATEVECTOR_NUM_VALUES 14

struct RocketServoDeflection {
	float servo_deflection_1;
	float servo_deflection_2;
	float servo_deflection_3;
	float servo_deflection_4;
	int64_t timestamp;
};
#define ROCKETSERVODEFLECTION_MSG_ID 11
#define ROCKETSERVODEFLECTION_SIZE 24
#define ROCKETSERVODEFLECTION_NUM_VALUES 5

struct RocketState {
	uint8_t rocket_state;
	uint8_t firing_channel_1;
	uint8_t firing_channel_2;
	uint8_t firing_channel_3;
	int64_t timestamp;
};
#define ROCKETSTATE_MSG_ID 12
#define ROCKETSTATE_SIZE 12
#define ROCKETSTATE_NUM_VALUES 5

struct RocketGroundEKF {
	float pn_matrix_d1;
	float pn_matrix_d2;
	float pn_matrix_d3;
	float pn_matrix_d4;
	float pn_matrix_d5;
	float pn_matrix_d6;
	int64_t timestamp;
};
#define ROCKETGROUNDEKF_MSG_ID 13
#define ROCKETGROUNDEKF_SIZE 32
#define ROCKETGROUNDEKF_NUM_VALUES 7

struct RocketSensorData {
	float accelerometer_x;
	float accelerometer_y;
	float accelerometer_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float gps_x;
	float gps_y;
	float gps_z;
	int64_t timestamp;
};
#define ROCKETSENSORDATA_MSG_ID 14
#define ROCKETSENSORDATA_SIZE 44
#define ROCKETSENSORDATA_NUM_VALUES 10

struct RocketAnalogFeedbackData {
	uint16_t current_fb_33;
	uint16_t pyro_0_cont;
	uint16_t pyro_1_cont;
	uint16_t pyro_2_cont;
	uint8_t pyro_channel_deploy;
	int64_t timestamp;
};
#define ROCKETANALOGFEEDBACKDATA_MSG_ID 15
#define ROCKETANALOGFEEDBACKDATA_SIZE 17
#define ROCKETANALOGFEEDBACKDATA_NUM_VALUES 6

typedef struct {
    struct RocketStateVector state_vector;
    struct RocketServoDeflection servo_deflection;
    struct RocketState rocket_state;
    struct RocketGroundEKF ground_ekf;
    struct RocketSensorData sensor_data;
    struct RocketAnalogFeedbackData analog_feedback_data;
   	uint32_t startup_timestamp; /* Not when the rocket powers on, but something like idle to ground, for example */
    uint32_t launch_timestamp; /* When the motor actually ignites */
} JetVanesRocketState;

extern StaticTask_t g_jet_vanes_task;

void jet_vanes_task(void *args);

#endif