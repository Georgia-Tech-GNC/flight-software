#include "lib.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "util.h"
#include "protocol.h"

void RocketState_encode(struct RocketState *input, uint8_t *output);
void RocketState_decode(uint8_t *input, struct RocketState *output);
void ServoDeflections_encode(struct ServoDeflections *input, uint8_t *output);
void ServoDeflections_decode(uint8_t *input, struct ServoDeflections *output);

uint8_t lib_packet_encode(uint8_t message_id, JetVanesRocketState *rocket_state, uint8_t *payload_buf, size_t payload_buf_size, size_t *bytes_written) {
    int msg_size = get_msg_size(message_id);

    rocket_assert(msg_size > 0, "Encode valid packet");
    rocket_assert(msg_size <= payload_buf_size, "Encode packet size less than buffer size");
    
    switch(message_id) {
        case ROCKETSTATE_MSG_ID:
            RocketState_encode(&rocket_state->rocket_state, payload_buf);
            break;
        case SERVODEFLECTIONS_MSG_ID:
            ServoDeflections_encode(&rocket_state->servo_deflections, payload_buf);
            break;
        default:
            rocket_assert(FALSE, "Encode valid packet");
            break;
    }

    return SUCCESS;
}

uint8_t lib_csv_encode(JetVanesRocketState *rocket_state, char *csv_line, size_t csv_line_size, size_t *bytes_written) {
    const char *line = "Hello, World!\n"; /* Obviously temporary */

    size_t bytes_copied = strlen(strncpy(csv_line, line, csv_line_size));

    if (!rocket_assert(bytes_copied <= csv_line_size, "CSV line size less than buffer size")) {
        return FAILURE;
    }

    *bytes_written = bytes_copied;

    return SUCCESS;
}

void RocketState_encode(struct RocketState *input, uint8_t *output) {
    memcpy(output + 0, &input->rocket_state, 1);
    memcpy(output + 1, &input->timestamp, 8);
}

void RocketState_decode(uint8_t *input, struct RocketState *output) {
    memcpy(&output->rocket_state, input + 0, 1);
    memcpy(&output->timestamp, input + 1, 8);
}

void ServoDeflections_encode(struct ServoDeflections *input, uint8_t *output) {
    memcpy(output + 0, &input->servo_1_desired, 2);
    memcpy(output + 2, &input->servo_1_actual, 2);
    memcpy(output + 4, &input->servo_2_desired, 2);
    memcpy(output + 6, &input->servo_2_actual, 2);
    memcpy(output + 8, &input->servo_3_desired, 2);
    memcpy(output + 10, &input->servo_3_actual, 2);
    memcpy(output + 12, &input->servo_4_desired, 2);
    memcpy(output + 14, &input->servo_4_actual, 2);
    memcpy(output + 16, &input->timestamp, 8);
}

void ServoDeflections_decode(uint8_t *input, struct ServoDeflections *output) {
    memcpy(&output->servo_1_desired, input + 0, 2);
    memcpy(&output->servo_1_actual, input + 2, 2);
    memcpy(&output->servo_2_desired, input + 4, 2);
    memcpy(&output->servo_2_actual, input + 6, 2);
    memcpy(&output->servo_3_desired, input + 8, 2);
    memcpy(&output->servo_3_actual, input + 10, 2);
    memcpy(&output->servo_4_desired, input + 12, 2);
    memcpy(&output->servo_4_actual, input + 14, 2);
    memcpy(&output->timestamp, input + 16, 8);
}