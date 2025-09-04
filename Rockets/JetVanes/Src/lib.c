#include "lib.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "util.h"
#include "protocol.h"
#include "rocket.h"
#include "rtos_globals.h"

#define MSG_MIN_ID ROCKETSTATEVECTOR_MSG_ID
#define MSG_MAX_ID ROCKETANALOGFEEDBACKDATA_MSG_ID
#define MSG_NUM_IDS (MSG_MAX_ID - MSG_MIN_ID + 1)

#define CSV_CELL_MAX_LEN 32
#define CSV_N_CELLS_PER_LINE 43
#define CSV_NEWLINE "\n"
#define CSV_LINE_LEN (CSV_CELL_MAX_LEN * CSV_N_CELLS_PER_LINE + strlen(CSV_NEWLINE))

void RocketStateVector_encode(struct RocketStateVector *input, uint8_t *output);
void RocketServoDeflection_encode(struct RocketServoDeflection *input, uint8_t *output);
void RocketState_encode(struct RocketState *input, uint8_t *output);
void RocketGroundEKF_encode(struct RocketGroundEKF *input, uint8_t *output);
void RocketSensorData_encode(struct RocketSensorData *input, uint8_t *output);
void RocketAnalogFeedbackData_encode(struct RocketAnalogFeedbackData *input, uint8_t *output);
static size_t printf_fixed_float(char *buf, float f);

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

	*bytes_written = get_msg_size(message_id);
    
    switch(message_id) {
        case ROCKETSTATEVECTOR_MSG_ID:
            RocketStateVector_encode(&rocket_state->state_vector, payload_buf);
            break;
        case ROCKETSERVODEFLECTION_MSG_ID:
            RocketServoDeflection_encode(&rocket_state->servo_deflections, payload_buf);
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

    return RET_SUCCESS;
}

uint8_t csv_encode(JetVanesRocketState *rocket_state, char *csv_line, size_t csv_line_size, size_t *bytes_written) {
    size_t len = 0;

	if (!rocket_assert(csv_line_size >= CSV_LINE_LEN + 1, "CSV line buffer size")) {
		return RET_FAILURE;
	}
    
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%lu,", (uint32_t) rocket_state->launch_timestamp);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%lu,", (uint32_t) (rocket_state->state_vector.timestamp));
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.velocity_x);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.velocity_y);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.velocity_z);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.attitude_w);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.attitude_x);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.attitude_y);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.attitude_z);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.position_x);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.position_y);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.position_z);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.world_x);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.world_y);
    len += printf_fixed_float(csv_line + len, rocket_state->state_vector.world_z);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%lu,", (uint32_t) (rocket_state->rocket_state.timestamp));
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->rocket_state.rocket_state);
	len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->rocket_state.firing_channel_1);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->rocket_state.firing_channel_2);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->rocket_state.firing_channel_3);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%lu,", (uint32_t) (rocket_state->ground_ekf.timestamp));
    len += printf_fixed_float(csv_line + len, rocket_state->ground_ekf.pn_matrix_d1);
    len += printf_fixed_float(csv_line + len, rocket_state->ground_ekf.pn_matrix_d2);
    len += printf_fixed_float(csv_line + len, rocket_state->ground_ekf.pn_matrix_d3);
    len += printf_fixed_float(csv_line + len, rocket_state->ground_ekf.pn_matrix_d4);
    len += printf_fixed_float(csv_line + len, rocket_state->ground_ekf.pn_matrix_d5);
    len += printf_fixed_float(csv_line + len, rocket_state->ground_ekf.pn_matrix_d6);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%lu,", (uint32_t) (rocket_state->sensor_data.timestamp));
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.accelerometer_x);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.accelerometer_y);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.accelerometer_z);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.gyro_x);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.gyro_y);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.gyro_z);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.gps_x);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.gps_y);
    len += printf_fixed_float(csv_line + len, rocket_state->sensor_data.gps_z);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%lu,", (uint32_t) (rocket_state->analog_feedback_data.timestamp));
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->analog_feedback_data.current_fb_33);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->analog_feedback_data.pyro_0_cont);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->analog_feedback_data.pyro_1_cont);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d,", rocket_state->analog_feedback_data.pyro_2_cont);
    len += snprintf(csv_line + len, CSV_CELL_MAX_LEN, "%d", rocket_state->analog_feedback_data.pyro_channel_deploy);

    csv_line[len++] = '\n';
	csv_line[len++] = '\0';

	*bytes_written = len;

    return RET_SUCCESS;
}

void set_adc_value(JetVanesRocketState *rocket_state, JetVanesADCChannel channel, uint16_t value) {
	switch (channel) {
		case ADC_PYRO_I_0:
			rocket_state->analog_feedback_data.pyro_0_cont = value;
			break;
		case ADC_PYRO_I_1:
			rocket_state->analog_feedback_data.pyro_1_cont = value;
			break;
		case ADC_PYRO_I_2:
			rocket_state->analog_feedback_data.pyro_2_cont = value;
			break;
		case ADC_VCC_I:
			rocket_state->analog_feedback_data.current_fb_33 = value;
			break;
		case ADC_SERVO_0:
			rocket_state->servo_adcs[0] = value;
			break;
		case ADC_SERVO_1:
			rocket_state->servo_adcs[1] = value;
			break;
		case ADC_SERVO_2:
			rocket_state->servo_adcs[2] = value;
			break;
		case ADC_SERVO_3:
			rocket_state->servo_adcs[3] = value;
			break;
		case ADC_SERVO_4:
			break;
	}
}

uint8_t update_rocket_state(RocketStateStruct *rocket_state, uint8_t *state_estimation_bytes, size_t size) {
	uint8_t *serial_buffer = state_estimation_bytes + 37;
	uint8_t *sensors_buffer = state_estimation_bytes;

	uint16_t offset = 1;
	memcpy(&rocket_state->sensor_data.accelerometer_x, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.accelerometer_y, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.accelerometer_z, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.gyro_x, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.gyro_y, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.gyro_z, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.gps_x, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.gps_y, sensors_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->sensor_data.gps_z, sensors_buffer + offset, 4);
	
	offset = 0;

	memcpy(&rocket_state->rocket_state.rocket_state, serial_buffer + offset, sizeof(uint8_t));
	offset += sizeof(uint8_t);
	memcpy(&rocket_state->state_vector.position_x, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.position_y, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.position_z, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.velocity_x, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.velocity_y, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.velocity_z, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.attitude_w, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.attitude_x, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.attitude_y, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.attitude_z, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.world_x, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.world_y, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->state_vector.world_z, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->ground_ekf.pn_matrix_d1, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->ground_ekf.pn_matrix_d2, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->ground_ekf.pn_matrix_d3, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->ground_ekf.pn_matrix_d4, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->ground_ekf.pn_matrix_d5, serial_buffer + offset, 4);
	offset += 4;
	memcpy(&rocket_state->ground_ekf.pn_matrix_d6, serial_buffer + offset, 4);
	offset += 4;

	TickType_t ticks = pdTICKS_TO_MS(xTaskGetTickCount());

	g_current_state.state_vector.timestamp = ticks;
	g_current_state.servo_deflections.timestamp = ticks;
	g_current_state.rocket_state.timestamp = ticks;
	g_current_state.ground_ekf.timestamp = ticks;
	g_current_state.sensor_data.timestamp = ticks;
	g_current_state.analog_feedback_data.timestamp = ticks;

	return RET_SUCCESS;
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

/**
 * @brief Print a float to a buffer with fixed precision (3 decimal places)
 * This helps us avoid adding floating point support to printf with -u _printf_float
 * @param buf Buffer to write to
 * @param f Float to write
 * @return Number of bytes written
 */
static size_t printf_fixed_float(char *buf, float f) {
    int i = (int) f;
    int d = (int) ((f - i) * 1000);
    return snprintf(buf, CSV_CELL_MAX_LEN, "%d.%03d,", i, d);
}