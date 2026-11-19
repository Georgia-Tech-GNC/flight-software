#ifndef LPS25HBTR_H
#define LPS25HBTR_H

#include "main.h"

typedef enum {
    LPS25HBTR_REGISTER_WHO_AM_I = 0x0F,
    LPS25HBTR_REGISTER_CTRL_1 = 0x20,
    LPS25HBTR_REGISTER_PRESS_OUT_XL = 0x28,
    LPS25HBTR_REGISTER_TEMP_OUT_L = 0x2B,
    LPS25HBTR_REGISTER_PRESS_OFFSET = 0x39
} LPS25HBTR_Register;

#define LPS25HBTR_REGISTER_WHO_AM_I_SIZE 1
#define LPS25HBTR_REGISTER_CTRL_1_SIZE 1
#define LPS25HBTR_REGISTER_PRESS_OUT_SIZE 3
#define LPS25HBTR_REGISTER_TEMP_OUT_SIZE 2
#define LPS25HBTR_REGISTER_PRESS_OFFSET_SIZE 2


typedef struct {
    double reference_pressure;
    double reference_temperature;
} lps25hbtr_data;

/**
 * @param lps25hbtr initialize the lps25hb device.
 * @return true of successful.
 */
bool lps25hbtr_initialize(lps25hbtr_data* lps25hbtr);

/**
 * Sets the reference level to the current altitude. 
 * Future calls to lps25hb_get_altitude will be relative to this level.
 * @note To minimize drift from changes in atmospheric pressure, this method should probably be called just before launch.
 * @param lps25hbtr an initialized instance of the lps25hb device.
 */
void lps25hbtr_set_reference(lps25hbtr_data* lps25hbtr);

/**
 * @param lps25hbtr an initialized instance of the device.
 * @return the measured temperature in Celsius.
 */
double lps25hbtr_get_temperature(lps25hbtr_data* lps25hbtr);

/**
 * Gets the pressure measured by the lps25hb device.
 * @return the measured pressure (in hPa).
 */
double lps25hbtr_get_pressure(void);

/**
 * Calculates the altitude using the reference pressure, reference temperature, and current pressure.
 * See https://en.wikipedia.org/wiki/Barometric_formula for more info on the barometric formula.
 * @param lps25hbtr an initialized instance of the lps25hb device.
 * @return the altitude (in meters) relative to the reference altitude.
 */
double lps25hbtr_get_altitude(lps25hbtr_data* lps25hbtr);

/**
 * Read a register from the lps25hb device.
 * 
 * @param reg the device register to read.
 * @param data the pointer to the received data.
 * @param size the size of the register being read from (<= 3).
 * @return true if successful.
 */
bool lps25hbtr_read_register(LPS25HBTR_Register reg, uint8_t* data, uint16_t size);

/**
 * Write a register from the lps25hb device.
 * 
 * @param reg the device register to write.
 * @param data a pointer to the data to transmit.
 * @param size the size of the register to write to.
 * @return true if successful.
 */
bool lps25hbtr_write_register(LPS25HBTR_Register reg, uint8_t* data, uint16_t size);

#endif
