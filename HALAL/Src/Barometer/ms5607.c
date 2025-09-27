#include "barometer.h"
#include "halal.h"
#include "util.h"

#define PROM_READ(address)            (0xA0 | ((address) << 1))         // Macro to change values for the 8 PROM addresses
#define RESET_COMMAND                 0x1E
#define CONVERT_D1_COMMAND            0x40
#define CONVERT_D2_COMMAND            0x50
#define READ_ADC_COMMAND              0x00

static uint8_t spi_transmit_data;

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

// oversampling ratios
static enum osr_factors {
  OSR_256,
  OSR_512=0x02,
  OSR_1024=0x04,
  OSR_2048=0x06,
  OSR_4096=0x08
};
static enum osr_factors pressure_osr = OSR_256;
static enum osr_factors temperature_osr = OSR_256;

static uint8_t reply[3];

static SPI_HandleTypeDef barometer_spi = {0};

// move to board HALAL headers
#define HALAL_BAROMETER_SPI_INSTANCE 0
#define HALAL_BAROMETER_SPI_BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_16 // needs to ensure <= 20Mhz for ms5607
#define HALAL_BAROMETER_CS_PIN 0
#define HALAL_BAROMETER_CS_PORT 0

uint8_t HALAL_barometer_init(void) {
#ifndef HALAL_BAROMETER_MODULE_ENABLED
    return RET_FAILURE;
#endif
    // setup spi
    barometer_spi.Instance = HALAL_BAROMETER_SPI_INSTANCE;
    barometer_spi.Init.Mode = SPI_MODE_MASTER;
    barometer_spi.Init.Direction = SPI_DIRECTION_2LINES;
    barometer_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    barometer_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    barometer_spi.Init.CLKPhase = SPI_PHASE_1EDGE; 
    barometer_spi.Init.NSS = SPI_NSS_SOFT;
    barometer_spi.Init.BaudRatePrescaler = HALAL_BAROMETER_SPI_BAUDRATE_PRESCALER;
    barometer_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    barometer_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    barometer_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&barometer_spi) != HAL_OK) {
        return RET_FAILURE;
    }
    
    // setup gpio
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = HALAL_BAROMETER_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // deselect
    HAL_GPIO_Init(HALAL_BAROMETER_CS_PORT, &gpio);

    // reset
    spi_transmit_data = RESET_COMMAND;
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
    HAL_SPI_Transmit(&barometer_spi, &spi_transmit_data, 1, 10);
    HAL_Delay(3);
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB

    // read and set prom data
    uint16_t *curr_prom_word = (uint16_t*) &prom_data;
    for (uint8_t address = 0; address < 8; address++) {
        spi_transmit_data = PROM_READ(address);

        HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
        HAL_SPI_Transmit(&barometer_spi, &spi_transmit_data, 1, 10);
        HAL_SPI_Receive(&barometer_spi, reply, 2, 10);
        HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB

        // swap bytes to correct endianness
        *curr_prom_word = (reply[0] << 8) | reply[1];
        curr_prom_word++;
    }

    if (prom_data.reserved == 0x00 || prom_data.reserved == 0xFF) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

uint8_t HALAL_barometer_read(int32_t *pressure) {
    if (barometer_spi.Instance == NULL) {
        return RET_FAILURE;
    }
    uint32_t uncomp_pressure = read_adc(CONVERT_D1_COMMAND, pressure_osr);
    uint32_t uncomp_temp = read_adc(CONVERT_D2_COMMAND, temperature_osr);

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
}

static uint32_t read_adc(uint8_t command, enum osr_factors osr) {
    spi_transmit_data = command | osr;
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
    HAL_SPI_Transmit(&barometer_spi, &spi_transmit_data, 1, 10);
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
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB

    // get response
    spi_transmit_data = READ_ADC_COMMAND;
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_RESET); // enable CSB
    HAL_SPI_Transmit(&barometer_spi, &spi_transmit_data, 1, 10);
    HAL_SPI_Receive(&barometer_spi, reply, 3, 10);
    HAL_GPIO_WritePin(HALAL_BAROMETER_CS_PORT, HALAL_BAROMETER_CS_PIN, GPIO_PIN_SET); // disable CSB
    return ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];
}

