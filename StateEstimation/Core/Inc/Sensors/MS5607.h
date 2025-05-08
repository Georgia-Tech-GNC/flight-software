/**
 ******************************************************************************
 * @file    MS5607SPI.h
 * @author  Kanav Chugh
 * @brief   MS5607 barometer header driver file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/


#ifndef _MS5607SPI_H_
#define _MS5607SPI_H_

#include "stm32h7xx_hal.h"

#define RESET_COMMAND                 0x1E
#define PROM_READ(address)            (0xA0 | ((address) << 1))         // Macro to change values for the 8 PROM addresses
#define CONVERT_D1_COMMAND            0x40
#define CONVERT_D2_COMMAND            0x50
#define READ_ADC_COMMAND              0x00

typedef enum OSRFactors {
  OSR_256,
  OSR_512=0x02,
  OSR_1024=0x04,
  OSR_2048=0x06,
  OSR_4096=0x08
} MS5607OSRFactors;


typedef enum MS5607States {
  MS5607_STATE_FAILED,
  MS5607_STATE_READY
} MS5607StateTypeDef;

typedef struct PromData {
  uint16_t reserved;
  uint16_t sens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tempsens;
  uint16_t crc;
} PromData;

typedef struct MS5607UncompensatedValues {
  uint32_t  pressure;
  uint32_t  temperature;
} MS5607UncompensatedValues;


typedef struct MS5607Readings {
  int32_t  pressure;
  int32_t  temperature;
} MS5607Readings;

MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);
void MS5607PromRead(struct PromData *prom);
void MS5607UncompensatedRead(struct MS5607UncompensatedValues*);
void MS5607Convert(struct MS5607UncompensatedValues*, struct MS5607Readings*);
void MS5607Update(void);
double MS5607GetTemperatureC(void);
int32_t MS5607GetPressurePa(void);
void enableCSB(void);
void disableCSB(void);
void MS5607SetTemperatureOSR(MS5607OSRFactors);
void MS5607SetPressureOSR(MS5607OSRFactors);

#endif 