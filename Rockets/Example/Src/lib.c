#include "lib.h"

uint8_t lib_packet_encode(uint8_t packet_id, RocketState rocket_state, uint8_t *payload_buf, size_t payload_buf_size) {
    return 1;
}

uint8_t lib_csv_encode(RocketState *rocket_state, uint8_t *csv_line, size_t csv_line_size, size_t *bytes_written) {
    return 1;
}