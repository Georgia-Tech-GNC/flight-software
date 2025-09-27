#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>

uint8_t HALAL_barometer_init(void);
uint8_t HALAL_barometer_read(int32_t* pressure);

#endif