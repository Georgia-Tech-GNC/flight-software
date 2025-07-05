#include "lib.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "util.h"
#include "protocol.h"

#define MSG_MIN_ID ROCKETSTATEVECTOR_MSG_ID
#define MSG_MAX_ID ROCKETANALOGFEEDBACKDATA_MSG_ID
#define MSG_NUM_IDS (MSG_MAX_ID - MSG_MIN_ID + 1)

void RocketStateVector_encode(struct RocketStateVector *input, uint8_t *output);
void RocketServoDeflection_encode(struct RocketServoDeflection *input, uint8_t *output);
void RocketState_encode(struct RocketState *input, uint8_t *output);
void RocketGroundEKF_encode(struct RocketGroundEKF *input, uint8_t *output);
void RocketSensorData_encode(struct RocketSensorData *input, uint8_t *output);
void RocketAnalogFeedbackData_encode(struct RocketAnalogFeedbackData *input, uint8_t *output);

size_t get_msg_size(uint8_t msg_id) {
    switch (msg_id) {
    	case ROCKETSTATEVECTOR_MSG_ID:
			return ROCKETSTATEVECTOR_SIZE;
    	case ROCKETSERVODEFLECTION_MSG_ID:
			return ROCKETSERVODEFLECTION_SIZE;
    	case ROCKETSTATE_MSG_ID:
			return ROCKETSTATE_SIZE;
    	case ROCKETGROUNDEKF_MSG_ID:
			return ROCKETGROUNDEKF_SIZE;
    	case ROCKETSENSORDATA_MSG_ID:
			return ROCKETSENSORDATA_SIZE;
    	case ROCKETANALOGFEEDBACKDATA_MSG_ID:
			return ROCKETANALOGFEEDBACKDATA_SIZE;
        default:
            return 0;
	}
}

uint8_t get_start_msg_id(void) {
    return MSG_MIN_ID;
}

uint8_t get_end_msg_id(void) {
    return MSG_MAX_ID;
}

uint8_t packet_encode(uint8_t message_id, JetVanesRocketState *rocket_state, uint8_t *payload_buf, size_t payload_buf_size, size_t *bytes_written) {
    int msg_size = get_msg_size(message_id);

    rocket_assert(msg_size > 0, "Encode valid packet");
    rocket_assert(msg_size <= payload_buf_size, "Encode packet size less than buffer size");
    
    switch(message_id) {
        case ROCKETSTATEVECTOR_MSG_ID:
            RocketStateVector_encode(&rocket_state->state_vector, payload_buf);
            break;
        case ROCKETSERVODEFLECTION_MSG_ID:
            RocketServoDeflection_encode(&rocket_state->servo_deflection, payload_buf);
            break;
        case ROCKETSTATE_MSG_ID:
            RocketState_encode(&rocket_state->rocket_state, payload_buf);
            break;
        case ROCKETGROUNDEKF_MSG_ID:
            RocketGroundEKF_encode(&rocket_state->ground_ekf, payload_buf);
            break;
        case ROCKETSENSORDATA_MSG_ID:
            RocketSensorData_encode(&rocket_state->sensor_data, payload_buf);
            break;
        case ROCKETANALOGFEEDBACKDATA_MSG_ID:
            RocketAnalogFeedbackData_encode(&rocket_state->analog_feedback_data, payload_buf);
            break;
        default:
            rocket_assert(FALSE, "Encode valid packet");
            break;
    }

    return SUCCESS;
}

uint8_t csv_encode(JetVanesRocketState *rocket_state, char *csv_line, size_t csv_line_size, size_t *bytes_written) {
    const char *line = "Hello, World!\n"; /* Obviously temporary */

    size_t bytes_copied = strlen(strncpy(csv_line, line, csv_line_size));

    if (!rocket_assert(bytes_copied <= csv_line_size, "CSV line size less than buffer size")) {
        return FAILURE;
    }

    *bytes_written = bytes_copied;

    return SUCCESS;
}

void RocketStateVector_encode(struct RocketStateVector *input, uint8_t *output) {
		memcpy(output + 0, &input->velocity_x, 4);
		memcpy(output + 4, &input->velocity_y, 4);
		memcpy(output + 8, &input->velocity_z, 4);
		memcpy(output + 12, &input->attitude_w, 4);
		memcpy(output + 16, &input->attitude_x, 4);
		memcpy(output + 20, &input->attitude_y, 4);
		memcpy(output + 24, &input->attitude_z, 4);
		memcpy(output + 28, &input->position_x, 4);
		memcpy(output + 32, &input->position_y, 4);
		memcpy(output + 36, &input->position_z, 4);
		memcpy(output + 40, &input->world_x, 4);
		memcpy(output + 44, &input->world_y, 4);
		memcpy(output + 48, &input->world_z, 4);
		memcpy(output + 52, &input->timestamp, 8);
}

void RocketServoDeflection_encode(struct RocketServoDeflection *input, uint8_t *output) {
		memcpy(output + 0, &input->servo_deflection_1, 4);
		memcpy(output + 4, &input->servo_deflection_2, 4);
		memcpy(output + 8, &input->servo_deflection_3, 4);
		memcpy(output + 12, &input->servo_deflection_4, 4);
		memcpy(output + 16, &input->timestamp, 8);
}

void RocketState_encode(struct RocketState *input, uint8_t *output) {
		memcpy(output + 0, &input->rocket_state, 1);
		memcpy(output + 1, &input->firing_channel_1, 1);
		memcpy(output + 2, &input->firing_channel_2, 1);
		memcpy(output + 3, &input->firing_channel_3, 1);
		memcpy(output + 4, &input->timestamp, 8);
}

void RocketGroundEKF_encode(struct RocketGroundEKF *input, uint8_t *output) {
		memcpy(output + 0, &input->pn_matrix_d1, 4);
		memcpy(output + 4, &input->pn_matrix_d2, 4);
		memcpy(output + 8, &input->pn_matrix_d3, 4);
		memcpy(output + 12, &input->pn_matrix_d4, 4);
		memcpy(output + 16, &input->pn_matrix_d5, 4);
		memcpy(output + 20, &input->pn_matrix_d6, 4);
		memcpy(output + 24, &input->timestamp, 8);
}

void RocketSensorData_encode(struct RocketSensorData *input, uint8_t *output) {
		memcpy(output + 0, &input->accelerometer_x, 4);
		memcpy(output + 4, &input->accelerometer_y, 4);
		memcpy(output + 8, &input->accelerometer_z, 4);
		memcpy(output + 12, &input->gyro_x, 4);
		memcpy(output + 16, &input->gyro_y, 4);
		memcpy(output + 20, &input->gyro_z, 4);
		memcpy(output + 24, &input->gps_x, 4);
		memcpy(output + 28, &input->gps_y, 4);
		memcpy(output + 32, &input->gps_z, 4);
		memcpy(output + 36, &input->timestamp, 8);
}

void RocketAnalogFeedbackData_encode(struct RocketAnalogFeedbackData *input, uint8_t *output) {
		memcpy(output + 0, &input->current_fb_33, 2);
		memcpy(output + 2, &input->pyro_0_cont, 2);
		memcpy(output + 4, &input->pyro_1_cont, 2);
		memcpy(output + 6, &input->pyro_2_cont, 2);
		memcpy(output + 8, &input->pyro_channel_deploy, 1);
		memcpy(output + 9, &input->timestamp, 8);
}