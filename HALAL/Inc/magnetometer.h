#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <stdint.h>

typedef enum {
    MAG_ERR_OK,
    MAG_ERR_GENERAL,
} magnetometer_err;

magnetometer_err HALAL_magnetometer_initialize();
magnetometer_err HALAL_magnetometer_write_hard_iron(double *hard_iron_offset);
magnetometer_err HALAL_magnetometer_read_hard_iron(double *hard_iron_offset);
magnetometer_err HALAL_magnetometer_read_temp(double *temp);
magnetometer_err HALAL_magnetometer_read_mag(double *mag_reading);

#endif