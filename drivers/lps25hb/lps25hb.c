#include "lps25hb.h"

#include <string.h>

#include "main.h"
#include "nanoprintf.h"
#include "port.h"

// Constants used in barometric equation
// https://en.wikipedia.org/wiki/Barometric_formula

/** Default reference pressure (hPa) */
#define BARO_P_0 (101325.0 / 100.0)
/** Default reference temperature (K) */
#define BARO_T_M_0 288.15

/** Temperature Gradient (K/m) */
#define BARO_L_M_0 (-6.5 / 1000.0)
/** Universal gas constant (J / (kmol * K)) */
#define BARO_R_star 8314.32
/** Mean molar mass of air at sea level (kg/kmol) */
#define BARO_M_0 28.9644
/** Gravitational acceleration (m/s^2) */
#define BARO_G_0 9.80665

#define LPS25HB_T_sens 480.0
#define LPS25HB_P_sens 4096.0

/**
 * - Power-down control (PD): active
 * - Output Data Rate (ODR): 25Hz
 * - Interrupt Generation (DIFF_EN): disabled
 * - Block Data Update (BDU): enabled
 * - Reset Autozero (RESET_AZ): disabled
 * - SPI Serial Interface Mode: 4-wire
 */
#define LPS25HB_REGISTER_CTRL_1_VALUE 0b11000100

#define LPS25HB_REGISTER_WHO_AM_I_VALUE 0xBD

#define LPS25HB_REGISTER_PRESS_OFFSET_VALUE ((int16_t)(-23.5 * 16) - 80)

static void uart_send_message(const char* str) {
    const uint16_t length = (uint16_t)strlen(str);
    const uint16_t size = sizeof(char) * length;
    
    HAL_UART_Transmit(&debug_uart, (const uint8_t*)str, size, HAL_MAX_DELAY);
}

bool lps25hb_read_register(lps25hb_data* lps25hb, LPS25HB_Register reg, uint8_t* data, uint16_t size) {
    // uint8_t tx[] = { 0x80 | (uint8_t)reg, (uint8_t)(lps25hb->i2c_address << 1) | 1 };
    // 
    // uart_send_message("1.1\r\n");
    // 
    // HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
    //     lps25hb->i2c_handle, 
    //     (uint16_t)(lps25hb->i2c_address << 1), 
    //     tx, 
    //     sizeof(tx), 
    //     HAL_MAX_DELAY
    // );
    // 
    // uart_send_message("1.2\r\n");
    // switch (status) {
    // case HAL_OK:
    //     uart_send_message("HAL_OK\r\n");
    //     
    //     break;
    // case HAL_ERROR:
    //     uart_send_message("HAL_ERROR\r\n");
    //     
    //     break;
    // case HAL_BUSY:
    //     uart_send_message("HAL_BUSY\r\n");
    //     
    //     break;
    // case HAL_TIMEOUT:
    //     uart_send_message("HAL_TIMEOUT\r\n");
    //     
    //     break;
    // default:
    //     uart_send_message("default\r\n");
    //     
    //     break;
    // }
    // 
    // if (status != HAL_OK)
    //     return false;
    // 
    // uart_send_message("1.3\r\n");

    // uint8_t rx[3] =  { 0, 0, 0 };
    // 
    // status = HAL_I2C_Master_Receive(
    //     lps25hb->i2c_handle, 
    //     (uint16_t)(lps25hb->i2c_address << 1 | 1), 
    //     rx, 
    //     size, 
    //     HAL_MAX_DELAY
    // );
    // 
    // uart_send_message("1.4\r\n");
    // 
    // if (status != HAL_OK)
    //     return false;
    // 
    // uart_send_message("1.5\r\n");
    // 
    // for (int i = 0; i < 3; i++) {
    //     char str[20];
    //     npf_snprintf(str, 20, "rx[%d]: %d.\r\n", i, rx[i]);
    //     uart_send_message(str);
    // }
    // 
    // memcpy(data, &rx[0], size >= sizeof(rx) ? sizeof(rx) : size);
    // 
    // return true;
    
    uint8_t rx[3] = { 0, 0, 0 };
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        lps25hb->i2c_handle, 
        (uint16_t)(lps25hb->i2c_address << 1),
        0x80 | (uint16_t)reg, 
        I2C_MEMADD_SIZE_8BIT, 
        rx, 
        size, 
        HAL_MAX_DELAY
    );
    
    if (status != HAL_OK)
        return false;
    
    memcpy(data, rx, size > sizeof(rx) ? sizeof(rx) : size);
    
    return true;
}

bool lps25hb_write_register(lps25hb_data* lps25hb, LPS25HB_Register reg, uint8_t* data, uint16_t size) {
    // TODO: make tx larger?
    // uint8_t tx[3] = { 0x80 | (uint8_t)reg, 0, 0 };
    // memcpy(&tx[2], data, size > sizeof(tx) - 2 ? sizeof(tx) - 2 : size);
    // 
    // HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
    //     lps25hb->i2c_handle, 
    //     (uint16_t)(lps25hb->i2c_address << 1), 
    //     tx, 
    //     sizeof(tx), 
    //     HAL_MAX_DELAY
    // );
    // 
    // return status == HAL_OK;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        lps25hb->i2c_handle, 
        (uint16_t)(lps25hb->i2c_address << 1),
        0x80 | (uint16_t)reg, 
        I2C_MEMADD_SIZE_8BIT, 
        data, 
        size, 
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

    // #warning "I2C1_CS_GPIO_Port not configured"
    // #warning "I2C1_CS_Pin not configured"
    // HAL_GPIO_WritePin(0, 0, GPIO_PIN_SET);
    // TODO: delay_us?
    // HAL_Delay(1);
    

    uint8_t result = 0;
    if (!lps25hb_read_register(lps25hb, LPS25HB_REGISTER_WHO_AM_I, &result, LPS25HB_REGISTER_WHO_AM_I_SIZE))
        return false;
    
    if (result != LPS25HB_REGISTER_WHO_AM_I_VALUE)
        return false;
    
    uint8_t ctrl_1 = LPS25HB_REGISTER_CTRL_1_VALUE;
    if (!lps25hb_write_register(lps25hb, LPS25HB_REGISTER_CTRL_1, &ctrl_1, LPS25HB_REGISTER_CTRL_1_SIZE))
        return false;
    
    // int16_t press_offset_native = 0;
    // if (!lps25hb_read_register(lps25hb, LPS25HB_REGISTER_PRESS_OFFSET, (uint8_t*)(&press_offset_native), LPS25HB_REGISTER_PRESS_OFFSET_SIZE))
    //     return false;
    // 
    // char str[50];
    // npf_snprintf(str, sizeof(str), "press_offset_native: %d\r\n", press_offset_native);
    // uart_send_message(str);
    
    int16_t press_offset_native = LPS25HB_REGISTER_PRESS_OFFSET_VALUE;
    // int16_t press_offset_native = -80 + 16;
    if (!lps25hb_write_register(lps25hb, LPS25HB_REGISTER_PRESS_OFFSET, (uint8_t*)(&press_offset_native), LPS25HB_REGISTER_PRESS_OFFSET_SIZE))
        return false;
    
    return true;
}

void lps25hb_set_reference(lps25hb_data* lps25hb) {
    lps25hb->reference_pressure = lps25hb_get_pressure(lps25hb);
    lps25hb->reference_temperature = lps25hb_get_temperature(lps25hb);
}

double lps25hb_get_temperature(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_TEMP_OUT_SIZE] = { 0, 0 };
    bool result = lps25hb_read_register(lps25hb, LPS25HB_REGISTER_TEMP_OUT_L, data, LPS25HB_REGISTER_TEMP_OUT_SIZE);
    if (!result) {
        uart_send_message("lps25hb_get_temperature, BAD\r\n");
    }
    
    const uint8_t high = data[1];
    const uint8_t low = data[0];
    int16_t native = (int16_t)(((uint16_t)high << 8) | low);
    
    char str[100];
    npf_snprintf(str, sizeof(str), "low: 0x%02X, high: 0x%02X, native: 0x%04X\r\n", low, high, native);
    uart_send_message(str);
    
    const double temp = native / LPS25HB_T_sens + 42.5;
    
    return temp;
}

double lps25hb_get_pressure(lps25hb_data* lps25hb) {
    uint8_t data[LPS25HB_REGISTER_PRESS_OUT_SIZE] = { 0, 0, 0 };
    lps25hb_read_register(lps25hb, LPS25HB_REGISTER_PRESS_OUT_XL, data, LPS25HB_REGISTER_PRESS_OUT_SIZE);
    
    const uint8_t high = data[2];
    const uint8_t med = data[1];
    const uint8_t low = data[0];
    int32_t native = (int32_t)((uint32_t)high << 16 | (uint32_t)med << 8 | low);
    // native = native & 0x00FFFFFF;
    
    const double pressure = native / LPS25HB_P_sens;
    
    return pressure;
}

double lps25hb_get_altitude(lps25hb_data* lps25hb) {
    const double pressure = lps25hb_get_pressure(lps25hb);
    const double pressure_ratio = pressure / lps25hb->reference_pressure;
    const double reference_temperature = lps25hb->reference_temperature + 273.15;

    constexpr double exponent = (-BARO_R_star * BARO_L_M_0) / (BARO_G_0 * BARO_M_0);
    
    const double altitude = (reference_temperature / BARO_L_M_0) * (pow(pressure_ratio, exponent) - 1.0);
    
    return altitude;
}
