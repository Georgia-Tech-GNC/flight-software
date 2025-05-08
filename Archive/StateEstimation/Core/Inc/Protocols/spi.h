/**
 * @file spi.h
 * @brief SPI peripheral configuration
 */
#ifndef __SPI_H__
#define __SPI_H__

#include "stm32h7xx_hal.h"

void MX_SPI2_Init(void);
void MX_SPI4_Init(void);
void MX_SPI6_Init(void);

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi6;

#endif /* __SPI_H__ */