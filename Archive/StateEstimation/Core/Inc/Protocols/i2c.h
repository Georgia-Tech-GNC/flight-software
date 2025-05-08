/**
 * @file i2c.h
 * @brief I2C peripheral configuration
 */
#ifndef __I2C_H__
#define __I2C_H__

#include "stm32h7xx_hal.h"

void MX_I2C4_Init(void);

// I2C handles (used by other modules)
extern I2C_HandleTypeDef hi2c4;

#endif /* __I2C_H__ */