#ifndef LIB_H
#define LIB_H

#include "stdint.h"
#include "stddef.h"
#include "lib.h"

#include "jet_vanes.h"

uint8_t lib_packet_encode(uint8_t message_id, RocketStateStruct *rocket_state, uint8_t *payload_buf, size_t payload_buf_size, size_t *bytes_written);

uint8_t lib_csv_encode(RocketStateStruct *rocket_state, char *csv_line, size_t csv_line_size, size_t *bytes_written);

#endif