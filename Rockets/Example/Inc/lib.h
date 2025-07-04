#ifndef LIB_H
#define LIB_H

typedef struct {
    struct RocketState rocket_state;
    struct ServoDeflections servo_deflections;
    uint32_t startup_timestamp; /* Not when the rocket powers on, but something like idle to ground, for example */
    uint32_t launch_timestamp; /* When the motor actually ignites */
} RocketState;

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
#define SERVODEFLECTIONS_MSG_ID 11
#define SERVODEFLECTIONS_SIZE 24
#define SERVODEFLECTIONS_NUM_VALUES 9

#define N_TELEMETRY_MESSAGE_PACKETS 2
uint8_t telemetry_msg_ids[2] = {ROCKETSTATE_MSG_ID, SERVODEFLECTIONS_MSG_ID};

uint8_t lib_packet_encode(uint8_t message_id, RocketState *rocket_state, uint8_t *payload_buf, size_t payload_buf_size, size_t *bytes_written);

uint8_t lib_csv_encode(RocketState *rocket_state, uint8_t *csv_line, size_t csv_line_size, size_t *bytes_written);

#endif