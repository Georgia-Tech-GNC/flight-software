#ifndef LPS25HB_H
#define LPS25HB_H

#include "main.h"

#define I2C1_CS_GPIO_Port 0
#define I2C1_CS_Pin 0

/**
 * "If the SA0 pad is connected to the voltage supply, LSb is ‘1’ (address 1011101b), 
 * otherwise if the SA0 pad is connected to ground, the LSb value is ‘0’ (address 1011100b)" (Pg 25)
 */
#define LPS25HB_ADDRESS 0b101110

typedef enum {
    LPS25HB_REGISTER_WHO_AM_I = 0x0F
} LPS25HB_Register;

#define LPS25HB_REGISTER_WHO_AM_I_SIZE 1

#define LPS25HB_REGISTER_WHO_AM_I_VALUE 0xBD

bool lps25hb_initialize(I2C_HandleTypeDef* i2c_handle);
bool lps25hb_calibrate(I2C_HandleTypeDef* i2c_handle);


float lps25hb_get_temperature();
float lps25hb_get_pressure();
float lps25hb_get_altitude();


bool lps25hb_read_register(I2C_HandleTypeDef* i2c_handle, LPS25HB_Register reg, uint8_t* data, uint16_t size);
bool lps25hb_write_register(I2C_HandleTypeDef* i2c_handle, LPS25HB_Register reg, const uint8_t* data, uint16_t size);


#endif
