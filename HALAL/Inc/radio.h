#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>

uint8_t HALAL_radio_init(void);
uint8_t HALAL_radio_start(void);
uint8_t HALAL_radio_transmit(const uint8_t *msg, size_t len, uint32_t timeout);

#endif