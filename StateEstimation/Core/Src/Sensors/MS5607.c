/**
 ******************************************************************************
 * @file    MS5607SPI.c
 * @author  Kanav Chugh
 * @brief   MS5607 source file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "MS5607.h"

static SPI_HandleTypeDef *hspi;
static GPIO_TypeDef *CS_GPIO_Port;
static uint16_t CS_Pin;
static uint8_t SPITransmitData;
static uint8_t Pressure_OSR =  OSR_256;
static uint8_t Temperature_OSR =  OSR_256;

static struct PromData promData;
static struct MS5607UncompensatedValues uncompValues;
static struct MS5607Readings readings;


/**
 * @brief resets and prepares the MS5607 device for general usage
 * @param hspix: SPI Handle for channel in use
 * @param GPIOx: GPIO handle for GPIO ports
 * @param GPIO_Pin: GPIO pin to do chip select in SPI Channel
 * @returns MS5607 structure for state status
*/
MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef *hspix, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  hspi = hspix;
  CS_GPIO_Port = GPIOx;
  CS_Pin = GPIO_Pin;
  enableCSB();
  SPITransmitData = RESET_COMMAND;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  HAL_Delay(3);
  disableCSB();
  MS5607PromRead(&promData);
  return promData.reserved == 0x00 || promData.reserved == 0xff ? MS5607_STATE_FAILED : MS5607_STATE_READY;
}

/**
 * @brief Reads data on devices PROM
 * @param prom: PROM address
*/
void MS5607PromRead(struct PromData *prom) {
  uint8_t   address;
  uint16_t  *structPointer;
  uint8_t receiveBuffer[2];
  structPointer = (uint16_t *) prom;
  for (address = 0; address < 8; address++) {
    SPITransmitData = PROM_READ(address);
    enableCSB();
    HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
    HAL_SPI_Receive(hspi, receiveBuffer, 2, 10);
    disableCSB();
    structPointer++;
  }
  structPointer = (uint16_t *) prom;
  for (address = 0; address < 8; address++) {
    uint8_t   *toSwap = (uint8_t *) structPointer;
    uint8_t secondByte = toSwap[0];
    toSwap[0] = toSwap[1];
    toSwap[1] = secondByte;
    structPointer++; 
  }
}

/**
 * @brief Reads direct data on devices PROM
 * @param uncompValues: Array with raw values
*/
void MS5607UncompensatedRead(struct MS5607UncompensatedValues *uncompValues){
  uint8_t reply[3];
  enableCSB();
  SPITransmitData = CONVERT_D1_COMMAND | Pressure_OSR;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  switch (Pressure_OSR) {
    case 0x00:
        HAL_Delay(1);
        break;
    case 0x02:
        HAL_Delay(2);
        break;
    case 0x04:
        HAL_Delay(3);
        break;
    case 0x06:
        HAL_Delay(5);
        break;
    default:
        HAL_Delay(10);
        break;
  }
  disableCSB();
  enableCSB();
  SPITransmitData = READ_ADC_COMMAND;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  HAL_SPI_Receive(hspi, reply, 3, 10);
  disableCSB();
  uncompValues->pressure = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];
  enableCSB();
  SPITransmitData = CONVERT_D2_COMMAND | Temperature_OSR;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  switch (Temperature_OSR) {
    case 0x00:
        HAL_Delay(1);
        break;
    case 0x02:
        HAL_Delay(2);
        break;
    case 0x04:
        HAL_Delay(3);
        break;
    case 0x06:
        HAL_Delay(5);
        break;
    default:
        HAL_Delay(10);
        break;
  }
  disableCSB();
  enableCSB();
  SPITransmitData = READ_ADC_COMMAND;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  HAL_SPI_Receive(hspi, reply, 3, 10);
  disableCSB();
  uncompValues->temperature = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];
}

/**
 * @brief Performs data conversion according to the MS5607 datasheet
 * @param sample: MS5607 uncompensated values
 * @param value: Array of compensated values
*/
void MS5607Convert(struct MS5607UncompensatedValues *sample, struct MS5607Readings *value){
  int32_t dT;
  int32_t TEMP;
  int64_t OFF;
  int64_t SENS;
  dT = sample->temperature - ((int32_t) (promData.tref << 8));
  TEMP = 2000 + (((int64_t) dT * promData.tempsens) >> 23);
  OFF = ((int64_t) promData.off << 17) + (((int64_t) promData.tco * dT) >> 6);
  SENS = ((int64_t) promData.sens << 16) + (((int64_t) promData.tcs * dT) >> 7);
  if (TEMP < 2000) {
    int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
    int32_t TEMPM = TEMP - 2000;
    int64_t OFF2 = (61 * (int64_t) TEMPM * (int64_t) TEMPM) >> 4;
    int64_t SENS2 = 2 * (int64_t) TEMPM * (int64_t) TEMPM;
    if (TEMP < -1500) {
      int32_t TEMPP = TEMP + 1500;
      int32_t TEMPP2 = TEMPP * TEMPP;
      OFF2 = OFF2 + (int64_t) 15 * TEMPP2;
      SENS2 = SENS2 + (int64_t) 8 * TEMPP2;
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }
  value->pressure = ((((int64_t) sample->pressure * SENS) >> 21) - OFF) >> 15;
  value->temperature = TEMP;
}

/**
 * @brief 
*/
void MS5607Update(void) {
  MS5607UncompensatedRead(&uncompValues);
  MS5607Convert(&uncompValues, &readings);
}

/**
 * @brief
 * @returns
*/
double MS5607GetTemperatureC(void) {
  return (double)readings.temperature/(double)100.0;
}

/**
 * @brief
 * @returns
*/
int32_t MS5607GetPressurePa(void) {
  return readings.pressure;
}

/**
 * @brief 
*/
void enableCSB(void){
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 
*/
void disableCSB(void){
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 
 * @param
*/
void MS5607SetTemperatureOSR(MS5607OSRFactors tOSR){
  Temperature_OSR = tOSR;
}

/**
 * @brief 
 * @param
*/
void MS5607SetPressureOSR(MS5607OSRFactors pOSR){
  Pressure_OSR = pOSR;
}