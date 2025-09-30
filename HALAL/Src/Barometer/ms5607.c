#include "barometer.h"
#include "halal.h"
#include "util.h"

#define PROM_READ(address)            (0xA0 | ((address) << 1))         // Macro to change values for the 8 PROM addresses
#define RESET_COMMAND                 0x1E
#define CONVERT_D1_COMMAND            0x40
#define CONVERT_D2_COMMAND            0x50
#define READ_ADC_COMMAND              0x00

static uint8_t spi_transmit_data;
static uint8_t spi_reply[3];

static struct {
  uint16_t reserved;
  uint16_t sens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tempsens;
  uint16_t crc;
} prom_data;

// oversampling ratios, higher = more precise, but longer conversion times
static enum osr_factors {
  OSR_256=0x00,
  OSR_512=0x02,
  OSR_1024=0x04,
  OSR_2048=0x06,
  OSR_4096=0x08
};

#define PRESSURE_OSR    OSR_256
#define TEMPERATURE_OSR OSR_256

/**
 * @brief Initializes the MS5607 barometer.
 *
 * Sends reset signal and reads PROM data into private struct for later use.
 * 
 * @returns RET_SUCCESS if initialization is successful and PROM data is valid,
 * RET_FAILURE otherwise.
 */
uint8_t HALAL_barometer_init(void) {
#ifndef HALAL_BAROMETER_MODULE_ENABLED
    return RET_FAILURE;
#endif
    // reset
    spi_transmit_data = RESET_COMMAND;
    HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
    HAL_SPI_Transmit((SPI_HandleTypeDef*)HALAL_BAROMETER_SPI_HANDLE, &spi_transmit_data, 1, 10);
    HAL_Delay(3);
    HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB

    // read and set prom data
    uint16_t *curr_prom_word = (uint16_t*) &prom_data;
    for (uint8_t address = 0; address < 8; address++) {
        spi_transmit_data = PROM_READ(address);

        HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
        HAL_SPI_Transmit((SPI_HandleTypeDef*)HALAL_BAROMETER_SPI_HANDLE, &spi_transmit_data, 1, 10);
        HAL_SPI_Receive((SPI_HandleTypeDef*)HALAL_BAROMETER_SPI_HANDLE, spi_reply, 2, 10);
        HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB

        // swap bytes to correct endianness
        *curr_prom_word = (spi_reply[0] << 8) | spi_reply[1];
        curr_prom_word++;
    }

    if (prom_data.reserved == 0x00 || prom_data.reserved == 0xFF) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

/**
 * @brief Reads a compensated pressure value from the sensor.
 *
 * @param pressure: Pointer to a 32-bit signed integer where the final
 * calculated pressure will be stored in Pa.
 * @returns RET_SUCCESS upon a successful read operation.
 */
uint8_t HALAL_barometer_read(int32_t *pressure) {
    uint32_t uncomp_pressure = read_adc(CONVERT_D1_COMMAND, PRESSURE_OSR);
    uint32_t uncomp_temp = read_adc(CONVERT_D2_COMMAND, TEMPERATURE_OSR);

    int32_t dT = uncomp_temp - ((int32_t) (prom_data.tref) << 8);
    int32_t TEMP = 2000 + (((int64_t) dT * prom_data.tempsens) >> 23);
    int64_t OFF = ((int64_t) prom_data.off << 17) + (((int64_t) prom_data.tco * dT) >> 6);
    int64_t SENS = ((int64_t) prom_data.sens << 16) + (((int64_t) prom_data.tcs * dT) >> 7);
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
    *pressure = ((((int64_t) uncomp_pressure * SENS) >> 21) - OFF) >> 15;
    // temperature = TEMP

    return RET_SUCCESS;
}

/**
 * @brief Performs a single ADC conversion (pressure or temperature) at a specific OSR and reads the result.
 * 
 * @param command: The conversion command (CONVERT_D1 or CONVERT_D2).
 * @param osr:     The desired oversampling ratio, which determines the
 * trade-off between conversion time and precision.
 * @returns The 24-bit raw ADC result, assembled into a 32-bit unsigned integer.
 */
static uint32_t read_adc(uint8_t command, enum osr_factors osr) {
    spi_transmit_data = command | osr;
    HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
    HAL_SPI_Transmit((SPI_HandleTypeDef*)HALAL_BAROMETER_SPI_HANDLE, &spi_transmit_data, 1, 10);
    switch (osr) {
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
    HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB

    // get response
    spi_transmit_data = READ_ADC_COMMAND;
    HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
    HAL_SPI_Transmit((SPI_HandleTypeDef*)HALAL_BAROMETER_SPI_HANDLE, &spi_transmit_data, 1, 10);
    HAL_SPI_Receive((SPI_HandleTypeDef*)HALAL_BAROMETER_SPI_HANDLE, spi_reply, 3, 10);
    HAL_GPIO_WritePin((GPIO_TypeDef*)HALAL_BAROMETER_CS_PIN_PORT, (uint16_t)HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB
    
    return ((uint32_t) spi_reply[0] << 16) | ((uint32_t) spi_reply[1] << 8) | (uint32_t) spi_reply[2]; // return w/ corrected endianness
}