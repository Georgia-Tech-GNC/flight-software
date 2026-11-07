#include "lps25hb.h"

#include <string.h>

#include "main.h"

// Constants used in barometric equation
// https://en.wikipedia.org/wiki/Barometric_formula

/** Default reference pressure (hPa) */
#define BARO_P_0 (101325.0f / 100.0f)
/** Default reference temperature (K) */
#define BARO_T_M_0 288.15f

/** Temperature Gradient (K/m) */
#define BARO_L_M_0 (-6.5f / 1000.0f)
/** Universal gas constant (J / (kmol * K)) */
#define BARO_R_star 8314.32f
/** Mean molar mass of air at sea level (kg/kmol) */
#define BARO_M_0 28.9644f
/** Gravitational acceleration (m/s^2) */
#define BARO_G_0 9.80665f

bool lps25hb_read_register(lps25hb_data* lps25hb, LPS25HB_Register reg, uint8_t* data, uint16_t size) {
    uint8_t tx[] = { 0x80 | (uint8_t)reg, (uint8_t)(lps25hb->i2c_address << 1) | 1 };
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        lps25hb->i2c_handle, 
        (uint16_t)(lps25hb->i2c_address << 1), 
        tx, 
        sizeof(tx), 
        HAL_MAX_DELAY
    );
    
    if (status != HAL_OK)
        return false;

    uint8_t rx[3] =  { 0, 0, 0 };
    
    status = HAL_I2C_Master_Receive(
        lps25hb->i2c_handle, 
        (uint16_t)(lps25hb->i2c_address << 1 | 1), 
        rx, 
        size, 
        HAL_MAX_DELAY
    );
    
    if (status != HAL_OK)
        return false;
    
    memcpy(data, &rx[0], size >= sizeof(rx) ? sizeof(rx) : size);
    
    return true;
}

bool lps25hb_write_register(lps25hb_data* lps25hb, LPS25HB_Register reg, const uint8_t* data, uint16_t size) {
    // TODO: make tx larger?
    uint8_t tx[3] = { 0x80 | (uint8_t)reg, 0, 0 };
    memcpy(&tx[2], data, size > sizeof(tx) - 2 ? sizeof(tx) - 2 : size);
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        lps25hb->i2c_handle, 
        (uint16_t)(lps25hb->i2c_address << 1), 
        tx, 
        sizeof(tx), 
        HAL_MAX_DELAY
    );
    
    return status == HAL_OK;
}


bool lps25hb_initialize(lps25hb_data* lps25hb, I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address) {
    lps25hb->i2c_handle = i2c_handle;
    lps25hb->i2c_address = i2c_address;
    lps25hb->reference_pressure = BARO_P_0;
    lps25hb->reference_temperature = BARO_T_M_0;
    
    // TODO: HAL_Delay(T_on)?

    #warning "I2C1_CS_GPIO_Port not configured"
    #warning "I2C1_CS_Pin not configured"
    HAL_GPIO_WritePin(0, 0, GPIO_PIN_SET);
    // TODO: delay_us?
    
    // HAL_Delay(1);

    uint8_t result = 0;
    if (!lps25hb_read_register(lps25hb, LPS25HB_REGISTER_WHO_AM_I, &result, LPS25HB_REGISTER_WHO_AM_I_SIZE))
        return false;
    
    if (result != LPS25HB_REGISTER_WHO_AM_I_VALUE)
        return false;
    
    // TODO: Configure Device
    return true;
}

void lps25hb_set_reference(lps25hb_data* lps25hb) {
    lps25hb->reference_pressure = lps25hb_get_pressure(lps25hb);
    lps25hb->reference_temperature = lps25hb_get_temperature(lps25hb);
}

float lps25hb_get_temperature(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_TEMP_OUT_SIZE];
    lps25hb_read_register(lps25hb, LPS25HB_REGISTER_TEMP_OUT_L, data, LPS25HB_REGISTER_TEMP_OUT_SIZE);
    
    const uint8_t high = data[1];
    const uint8_t low = data[0];
    uint16_t native = (uint16_t)high << 8 | low;
    native = ~native + 1;
    
    const float temp = native / LPS25HB_T_sens;
    
    return temp;
}

float lps25hb_get_pressure(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_PRESS_OUT_SIZE];
    lps25hb_read_register(lps25hb, LPS25HB_REGISTER_PRESS_OUT_XL, data, LPS25HB_REGISTER_PRESS_OUT_SIZE);
    
    const uint8_t high = data[2];
    const uint8_t med = data[1];
    const uint8_t low = data[0];
    uint32_t native = (uint32_t)high << 16 | (uint32_t)med << 8 | low;
    native = (~native + 1) & 0x00FFFFFF;
    
    const float pressure = native / LPS25HB_P_sens;
    
    return pressure;
}

float lps25hb_get_altitude(lps25hb_data* lps25hb) {
    const float pressure = lps25hb_get_pressure(lps25hb);
    const float pressure_ratio = pressure / lps25hb->reference_pressure;
    const float reference_temperature = lps25hb->reference_temperature + 273.15f;

    constexpr float exponent = (-BARO_R_star * BARO_L_M_0) / (BARO_G_0 * BARO_M_0);
    
    const float altitude = (reference_temperature / BARO_L_M_0) * (powf(pressure_ratio, exponent) - 1.0f);
    
    return altitude;
}
