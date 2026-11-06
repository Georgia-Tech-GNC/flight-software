#include "lps25hb.h"

#include <string.h>

#include "main.h"
#include "nanoprintf.h"
#include "port.h"

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

static void uart_send_message(const char* str) {
    const uint16_t length = (uint16_t)strlen(str);
    const uint16_t size = sizeof(char) * length;
    
    HAL_UART_Transmit(&debug_uart, (const uint8_t*)str, size, HAL_MAX_DELAY);
}

/**
 * Read a register from the lps25hb device
 * 
 * @param lps25hb the handle to the i2c interface
 * @param reg the device register to read
 * @param data the pointer to the received data
 * @param size the size of the register being read from (<= 2)
 * @return true if successful 
 */
bool lps25hb_read_register(lps25hb_data* lps25hb, LPS25HB_Register reg, uint8_t* data, uint16_t size) {
    // TODO: Address in tx?
    uint8_t tx[] = { 0x80 | (uint8_t)reg, (lps25hb->i2c_address << 1) | 1 };
    // TODO: | 0 for r/w bit?
    if (HAL_I2C_Master_Transmit(lps25hb->i2c_handle, (lps25hb->i2c_address << 1) | 0, tx, sizeof(tx), HAL_MAX_DELAY) != HAL_OK)
        return false;

    uint8_t rx[4] =  { 0, 0, 0, 0 };
    if (HAL_I2C_Master_Receive(lps25hb->i2c_handle, lps25hb->i2c_address << 1 | 1, rx, size, HAL_MAX_DELAY) != HAL_OK)
        return false;
    
    memcpy(data, &rx[0], size);
    
    return true;
}

bool lps25hb_write_register(lps25hb_data* lps25hb, LPS25HB_Register reg, const uint8_t* data, uint16_t size) {
    uint8_t tx[3] = { 0x80 | (uint8_t)reg, 0, 0 };
    memcpy(&tx[2], data, size);
    
    if (HAL_I2C_Master_Transmit(lps25hb->i2c_handle, lps25hb->i2c_address << 1 | 0, tx, sizeof(tx), HAL_MAX_DELAY) != HAL_OK)
        return false;
    
    return true;
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
    lps25hb_read_register(lps25hb, LPS25HB_REGISTER_TEMP_OUT_L, data, LPS25HB_REGISTER_TEMP_OUT_SIZE);
    
    const uint8_t high = data[1];
    const uint8_t low = data[0];
    uint16_t native = (uint16_t)high << 8 | low;
    native = ~native + 1;
    
    const float temp = native / LPS25HB_T_sens;
    
    char str[100];
    npf_snprintf(str, sizeof(str), "native=%d, data[0]=0x%X, data[1]=0x%X, temp=%f\r\n", native, data[0], data[1], temp);
    uart_send_message(str);
    
    return temp;
}

/**
 * Gets the pressure measured by the lps25hb device
 * @param lps25hb an initialized instance of the lps25hb device
 * @return the measured pressure (in hPa)
 */
float lps25hb_get_pressure(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_PRESS_OUT_SIZE];
    lps25hb_read_register(lps25hb, LPS25HB_REGISTER_PRESS_OUT_XL, data, LPS25HB_REGISTER_PRESS_OUT_SIZE);
    
    const uint8_t high = data[2];
    const uint8_t med = data[1];
    const uint8_t low = data[0];
    uint32_t native = (uint32_t)high << 16 | (uint32_t)med << 8 | low;
    native = (~native + 1) & 0x00FFFFFF;
    
    const float pressure = native / LPS25HB_P_sens;
    
    char str[100];
    npf_snprintf(str, sizeof(str), "native=%d, data[0]=0x%X, data[1]=0x%X, data[2]=0x%X, pressure=%f\r\n", native, data[0], data[1], data[2], pressure);
    uart_send_message(str);
    
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
    const float reference_temperature = lps25hb->reference_temperature + 273.15f;

    constexpr float exponent = (-BARO_R_star * BARO_L_M_0) / (BARO_G_0 * BARO_M_0);
    
    const float altitude = (reference_temperature / BARO_L_M_0) * (powf(pressure_ratio, exponent) - 1.0f);
    
    char str[200];
    npf_snprintf(str, sizeof(str), "Pressure=%f, ReferencePressure=%f, PressureRatio=%f, ReferenceTemperature=%f, Exponent=%f, Altitude=%f\r\n", pressure, lps25hb->reference_pressure, pressure_ratio, reference_temperature, exponent, altitude);
    uart_send_message(str);
    
    return altitude;
}
