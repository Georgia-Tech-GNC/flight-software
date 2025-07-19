#ifndef LIB_H
#define LIB_H

#include "stdint.h"
#include "stddef.h"
#include "lib.h"

#include "jet_vanes.h"

uint8_t get_start_msg_id(void);
uint8_t get_end_msg_id(void);
size_t get_msg_size(uint8_t msg_id);

uint8_t packet_encode(uint8_t message_id, RocketStateStruct *rocket_state, uint8_t *payload_buf, size_t payload_buf_size, size_t *bytes_written);

uint8_t csv_encode(RocketStateStruct *rocket_state, char *csv_line, size_t csv_line_size, size_t *bytes_written);

uint8_t update_rocket_state(RocketStateStruct *rocket_state, uint8_t *state_estimation_bytes, size_t size);

void set_adc_value(RocketStateStruct *rocket_state, JetVanesADCChannel channel, uint16_t value);

#endif