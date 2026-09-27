#include "lps25hb.h"

#include <string.h>

#include "main.h"

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

    uint8_t rx[2] =  { 0, 0 };
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


bool lps25hb_initialize(I2C_HandleTypeDef* i2c_handle) {
    // TODO: HAL_Delay(T_on)?

    HAL_GPIO_WritePin(I2C1_CS_GPIO_Port, I2C1_CS_Pin, GPIO_PIN_SET);
    // TODO: delay_us?
    
    HAL_Delay(1);

    uint8_t result = 0;
    if (!lps25hb_read_register(i2c_handle, LPS25HB_REGISTER_WHO_AM_I, &result, LPS25HB_REGISTER_WHO_AM_I_SIZE))
        return false;
    
    if (result != LPS25HB_REGISTER_WHO_AM_I_VALUE)
        return false;
    
    // TODO: Configure Device
    return true;
}


bool lps25hb_calibrate(I2C_HandleTypeDef* i2c_handle) {
    
}

float lps25hb_get_temperature() {
    
}

float lps25hb_get_pressure() {
    
}

float lps25hb_get_altitude() {
    
}
