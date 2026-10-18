#ifndef LPS25HB_H
#define LPS25HB_H

#include "main.h"


/**
 * "If the SA0 pad is connected to the voltage supply, LSb is ‘1’ (address 1011101b), 
 * otherwise if the SA0 pad is connected to ground, the LSb value is ‘0’ (address 1011100b)" (Pg 25)
 */
#define LPS25HB_ADDRESS 0b101110

typedef enum {
    LPS25HB_REGISTER_WHO_AM_I = 0x0F,
    LPS25HB_REGISTER_PRESS_OUT_XL = 0x28,
    LPS25HB_REGISTER_TEMP_OUT_L = 0x2B
} LPS25HB_Register;

#define LPS25HB_REGISTER_WHO_AM_I_SIZE 1
#define LPS25HB_REGISTER_PRESS_OUT_SIZE 3
#define LPS25HB_REGISTER_TEMP_OUT_SIZE 2

#define LPS25HB_REGISTER_WHO_AM_I_VALUE 0xBD

#define LPS25HB_T_sens 480.0f

typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    float reference_pressure;
    float reference_temperature;
} lps25hb_data;

bool lps25hb_initialize(lps25hb_data* lps25hb, I2C_HandleTypeDef* i2c_handle);
void lps25hb_set_reference(lps25hb_data* lps25hb);


float lps25hb_get_temperature(lps25hb_data* lps25hb);
float lps25hb_get_pressure(lps25hb_data* lps25hb);
float lps25hb_get_altitude(lps25hb_data* lps25hb);


bool lps25hb_read_register(I2C_HandleTypeDef* i2c_handle, LPS25HB_Register reg, uint8_t* data, uint16_t size);
bool lps25hb_write_register(I2C_HandleTypeDef* i2c_handle, LPS25HB_Register reg, const uint8_t* data, uint16_t size);


#endif
