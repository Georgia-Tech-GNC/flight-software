/**
 ******************************************************************************
 * @file    ADIS16500.c
 * @author  Kanav Chugh
 * @brief   ADIS16500 driver
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "stdio.h"
#include "ADIS16500.h"

#define UNUSED __attribute__((unused))

#define WRITE_ADDR(addr) ((addr) |	0x80)
#define READ_ADDR(addr)  ((addr) & ~0x80)

typedef enum {
	SPI_SLOW = SPI_BAUDRATEPRESCALER_64,
	SPI_MEDIUM = SPI_BAUDRATEPRESCALER_8,
    SPI_FAST = SPI_BAUDRATEPRESCALER_2
} SPI_Speed;


#define BURST_EXCHANGE_LEN (sizeof(ADIS16500_Data)+ 2) //+2 for initial addr
uint8_t adis_raw_in[BURST_EXCHANGE_LEN];
static const ADIS16500_Config * CONF;
SPI_HandleTypeDef *hspi;

const ADIS16500_Config adis_imu = {
	.spi_cs = {GPIOA, GPIO_PIN_0},
	.spi_sck = {GPIOA, GPIO_PIN_5},
	.spi_miso = {GPIOA, GPIO_PIN_6},
	.spi_mosi = {GPIOB, GPIO_PIN_3},
	.spi = {SPI1},
	.nrst = {GPIOD, GPIO_PIN_8},
    .dr = {GPIOD, GPIO_PIN_8}
};

void adis_init(const ADIS16500_Config * conf) {
    hspi->Instance = conf->spi;
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi);
}

uint16_t adis_get(ADIS_RegAddr *addr) {
    uint8_t txbuf[2] = {*addr, 0};
    uint8_t rxbuf[2];
	HAL_GPIO_WritePin(adis_imu.spi_cs, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, txbuf, rxbuf, sizeof(rxbuf), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(adis_imu.spi_cs, GPIO_PIN_3, GPIO_PIN_SET);
    return (uint16_t)(rxbuf[0] << 8 | rxbuf[1]);
}

void adis_set(ADIS_RegAddr addr, uint16_t value) {
    uint8_t txbuf[4] = {
        WRITE_ADDR(addr + 1), (uint8_t)(value >> 8),
        WRITE_ADDR(addr), (uint8_t)value
    };
	HAL_GPIO_WritePin(adis_imu.spi_cs, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, txbuf, txbuf, sizeof(txbuf));
	HAL_GPIO_WritePin(adis_imu.spi_cs, GPIO_PIN_3, GPIO_PIN_SET);
}


static int16_t sign_extend(uint16_t val, int bits) {
	if ((val&(1<<(bits-1))) != 0) {
		val = val - (1<<bits);
	}
	return val;
}

static void buffer_to_burst_data(uint8_t * raw, ADIS16500_Data * data) {
	data->x_gyro_out  = sign_extend((raw[2]	<< 8 | raw[3]) & 0x3fff, 14);
	data->y_gyro_out  = sign_extend((raw[4]	<< 8 | raw[5]) & 0x3fff, 14);
	data->z_gyro_out  = sign_extend((raw[6]	<< 8 | raw[7]) & 0x3fff, 14);
	data->x_accl_out  = sign_extend((raw[8]	<< 8 | raw[9]) & 0x3fff, 14);
	data->y_accl_out  = sign_extend((raw[10] << 8 | raw[11]) & 0x3fff, 14);
	data->z_accl_out  = sign_extend((raw[12] << 8 | raw[13]) & 0x3fff, 14);
	data->x_deltang_out  = sign_extend((raw[8]	<< 8 | raw[15]) & 0x3fff, 14);
	data->y_deltang_out  = sign_extend((raw[10] << 8 | raw[17]) & 0x3fff, 14);
	data->z_deltang_out  = sign_extend((raw[12] << 8 | raw[19]) & 0x3fff, 14);
	data->x_deltvel_out  = sign_extend((raw[8]	<< 8 | raw[15]) & 0x3fff, 14);
	data->y_deltvel_out  = sign_extend((raw[10] << 8 | raw[17]) & 0x3fff, 14);
	data->z_deltvel_out  = sign_extend((raw[12] << 8 | raw[19]) & 0x3fff, 14);
	data->temp_out   = sign_extend((raw[20] << 8 | raw[21]) & 0x0fff, 12);
}

void adis_get_data(ADIS16500_Data * data) { 
	buffer_to_burst_data(adis_raw_in + 2, data);
}

uint16_t adis_self_test(void) {
	uint16_t diagstat = adis_get(ADIS_DIAG_STAT);
	uint16_t msc = adis_get(ADIS_MSC_CTRL);
	adis_set(ADIS_MSC_CTRL, msc | 1<<10);
	do {
		msc = adis_get(ADIS_MSC_CTRL);
	} while (msc & 1<< 10);
	diagstat = adis_get(ADIS_DIAG_STAT);
	return diagstat;
}

void adis_reset(void) {
	const unsigned int ADIS_RESET_MSECS = 500;
	HAL_GPIO_WritePin(CONF->nrst, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(ADIS_RESET_MSECS);
	HAL_GPIO_WritePin(CONF->nrst, GPIO_PIN_8, GPIO_PIN_SET);
}