#ifndef LPS25HB_H
#define LPS25HB_H

#include "main.h"

/**
 * "If the SA0 pad is connected to the voltage supply, LSb is ‘1’ (address 1011101b), 
 * otherwise if the SA0 pad is connected to ground, the LSb value is ‘0’ (address 1011100b)" (Pg 25)
 */
// #define LPS25HB_ADDRESS 0b101110

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
#define LPS25HB_P_sens 4096.0f

typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_address;
    float reference_pressure;
    float reference_temperature;
} lps25hb_data;

/**
 * @param lps25hb initialize the lps25hb device.
 * @param i2c_handle the i2c handle.
 * @param i2c_address the i2c address.
 * @return true of successful.
 */
bool lps25hb_initialize(lps25hb_data* lps25hb, I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address);

/**
 * Sets the reference level to the current altitude. 
 * Future calls to lps25hb_get_altitude will be relative to this level.
 * @note To minimize drift from changes in atmospheric pressure, this method should probably be called just before launch.
 * @param lps25hb an initialized instance of the lps25hb device.
 */
void lps25hb_set_reference(lps25hb_data* lps25hb);

/**
 * @param lps25hb an initialized instance of the device.
 * @return the measured temperature in Celsius.
 */
float lps25hb_get_temperature(lps25hb_data* lps25hb);

/**
 * Gets the pressure measured by the lps25hb device.
 * @param lps25hb an initialized instance of the lps25hb device.
 * @return the measured pressure (in hPa).
 */
float lps25hb_get_pressure(lps25hb_data* lps25hb);

/**
 * Calculates the altitude using the reference pressure, reference temperature, and current pressure.
 * See https://en.wikipedia.org/wiki/Barometric_formula for more info on the barometric formula.
 * @param lps25hb an initialized instance of the lps25hb device.
 * @return the altitude (in meters) relative to the reference altitude.
 */
float lps25hb_get_altitude(lps25hb_data* lps25hb);

/**
 * Read a register from the lps25hb device.
 * 
 * @param lps25hb an initialized lps25hb_data object.
 * @param reg the device register to read.
 * @param data the pointer to the received data.
 * @param size the size of the register being read from (<= 3).
 * @return true if successful.
 */
bool lps25hb_read_register(lps25hb_data* lps25hb, LPS25HB_Register reg, uint8_t* data, uint16_t size);

/**
 * Write a register from the lps25hb device.
 * 
 * @param lps25hb an initialized instance of lps25hb.
 * @param reg the device register to write.
 * @param data a pointer to the data to transmit.
 * @param size the size of the register to write to.
 * @return true if successful.
 */
bool lps25hb_write_register(lps25hb_data* lps25hb, LPS25HB_Register reg, const uint8_t* data, uint16_t size);

#endif
