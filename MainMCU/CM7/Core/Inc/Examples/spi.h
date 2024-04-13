/**
 * @file spi.h
 * @author Kanav Chugh
 * @brief Bare Metal SPI Driver
*/

#ifndef __SPI_H__
#define __SPI_H__

#ifndef STM32H755
#define STM32H755
#include "stm32h745xx.h"
#endif

void initClocks(void);


void initSPI_SSM(void);
void configSPIPins_SSM(void);
void setPinMode_SSM(void);
void setAF_SSM(void);
void configSPI_SSM(void);
uint8_t transferSPI_SSM(uint8_t tx_data);

void initSPI_HSM(void);
void configSPIPins_HSM(void);
void setPinMode_HSM(void);
void setAF_HSM(void);
void configSPI_HSM(void);
uint8_t transferSPI_HSM(uint8_t tx_data);

#endif