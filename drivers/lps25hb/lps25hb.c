#include "lps25hb.h"

#include <string.h>

#include "main.h"

// Constants used in barometric equation
// https://en.wikipedia.org/wiki/Barometric_formula

/** Default reference pressure (Pa) */
#define BARO_P_0 101325.0f
/** Default reference temperature (K) */
#define BARO_T_M_0 216.65f

/** Temperature Gradient (K/km) */
#define BARO_L_M_0 (-0.0065f)
/** Universal gas constant (J / (kmol * K)) */
#define BARO_R_star 8314.32f
/** Mean molar mass of air at sea level (kg/kmol) */
#define BARO_M_0 28.9644f
/** Gravitational acceleration (m/s^2) */
#define BARO_G_0 9.80665f

/**
 * Read a register from the lps25hb device
 * 
 * @param i2c_handle the handle to the i2c interface
 * @param reg the device register to read
 * @param data the pointer to the received data
 * @param size the size of the register being read from (<= 2)
 * @return true if successful 
 */
bool lps25hb_read_register(I2C_HandleTypeDef* i2c_handle, LPS25HB_Register reg, uint8_t* data, uint16_t size) {
    // TODO: Address in tx?
    uint8_t tx[] = { 0x80 | (uint8_t)reg, LPS25HB_ADDRESS << 1 | 1 };
    // TODO: | 0 for r/w bit?
    if (HAL_I2C_Master_Transmit(i2c_handle, LPS25HB_ADDRESS << 1 | 0, tx, sizeof(tx), HAL_MAX_DELAY) != HAL_OK)
        return false;

    uint8_t rx[3] =  { 0, 0, 0 };
    if (HAL_I2C_Master_Receive(i2c_handle, LPS25HB_ADDRESS << 1 | 1, rx, size, HAL_MAX_DELAY) != HAL_OK)
        return false;
    
    memcpy(data, &rx[0], size);
    
    return true;
}

bool lps25hb_write_register(I2C_HandleTypeDef* i2c_handle, LPS25HB_Register reg, const uint8_t* data, uint16_t size) {
    uint8_t tx[3] = { 0x80 | (uint8_t)reg, 0, 0 };
    memcpy(&tx[2], data, size);
    
    if (HAL_I2C_Master_Transmit(i2c_handle, LPS25HB_ADDRESS << 1 | 0, tx, sizeof(tx), HAL_MAX_DELAY) != HAL_OK)
        return false;
    
    return true;
}


bool lps25hb_initialize(lps25hb_data* lps25hb, I2C_HandleTypeDef* i2c_handle) {
    lps25hb->i2c_handle = i2c_handle;
    lps25hb->reference_pressure = BARO_P_0;
    lps25hb->reference_temperature = BARO_T_M_0;
    
    // TODO: HAL_Delay(T_on)?

    #warning "I2C1_CS_GPIO_Port not configured"
    #warning "I2C1_CS_Pin not configured"
    HAL_GPIO_WritePin(0, 0, GPIO_PIN_SET);
    // TODO: delay_us?
    
    // HAL_Delay(1);

    uint8_t result = 0;
    if (!lps25hb_read_register(i2c_handle, LPS25HB_REGISTER_WHO_AM_I, &result, LPS25HB_REGISTER_WHO_AM_I_SIZE))
        return false;
    
    if (result != LPS25HB_REGISTER_WHO_AM_I_VALUE)
        return false;
    
    // TODO: Configure Device
    return true;
}

/**
 * Sets the reference level to the current altitude. 
 * Future calls to lps25hb_get_altitude will be relative to this level.
 * @note To minimize drift from changes in atmospheric pressure, this method should be called just before launch.
 * @param lps25hb an initialized instance of the lps25hb device
 * @return true if the operation was successful
 */
void lps25hb_set_reference(lps25hb_data* lps25hb) {
    lps25hb->reference_pressure = lps25hb_get_pressure(lps25hb);
    lps25hb->reference_temperature = lps25hb_get_temperature(lps25hb);
}

float lps25hb_get_temperature(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_TEMP_OUT_SIZE];
    lps25hb_read_register(lps25hb->i2c_handle, LPS25HB_REGISTER_TEMP_OUT_L, data, LPS25HB_REGISTER_TEMP_OUT_SIZE);
    
    float temp = (float)(~(data[1] - 1) << 1 | ~(data[0] - 1)) * (1.0f / LPS25HB_T_sens);
    
    return temp;
}

/**
 * Gets the pressure measured by the lps25hb device
 * @param lps25hb an initialized instance of the lps25hb device
 * @return the measured pressure (in hPa)
 */
float lps25hb_get_pressure(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_PRESS_OUT_SIZE];
    lps25hb_read_register(lps25hb->i2c_handle, LPS25HB_REGISTER_PRESS_OUT_XL, data, LPS25HB_REGISTER_PRESS_OUT_SIZE);
    
    uint32_t pressure_counts = data[2] & data[1] & data[0];
    const float pressure = (float)pressure_counts / 4096.0f;
    
    return pressure;
}

/**
 * Calculates the altitude using the reference pressure, reference temperature, and current pressure.
 * See https://en.wikipedia.org/wiki/Barometric_formula for more info on the barometric formula.
 * @param lps25hb an initialized instance of the lps25hb device
 * @return the altitude (in meters) relative to the reference altitude.
 */
float lps25hb_get_altitude(lps25hb_data* lps25hb) {
    const float pressure = lps25hb_get_pressure(lps25hb);
    const float pressure_ratio = pressure / lps25hb->reference_pressure;

    constexpr float exponent = (-BARO_R_star * BARO_L_M_0) / (BARO_G_0 * BARO_M_0);
    
    const float altitude = (lps25hb->reference_temperature / BARO_L_M_0) * (powf(pressure_ratio, exponent) - 1.0f);
    
    return altitude;
}
