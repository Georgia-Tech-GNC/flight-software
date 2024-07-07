/**
 ******************************************************************************
 * @file    ADIS16500.c
 * @author  Kanav Chugh
 * @brief   ADIS16500 source file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */


#include "ADIS16500.h"

/**
 * @brief initializes DWT for advanced clock control
 * 
*/
void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
        DWT->CYCCNT = 0; 
    }
}

/**
 * @brief delays system thread by time in microseconds
 * @param microseconds time in microseconds to delay
*/
void delay_us(uint32_t microseconds) {
    uint32_t startTick = DWT->CYCCNT,
            delayTicks = microseconds * (SystemCoreClock / 1000000); 
    while (DWT->CYCCNT - startTick < delayTicks); 
}

/**
 * @brief reads a register given the memory address
 * @param device instance of ADIS IMU device
 * @param addr address to read data from
 * @return data in bits, MSB first
*/
int16_t adis_read_register(struct ADIS_Device *device, uint8_t addr) {
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*)device->cs_pin_port, GPIO_PIN_RESET);
	uint8_t address[2] = {addr, 0x00};
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, address, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*)device->cs_pin_port, GPIO_PIN_SET);
	delay_us(20);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*)device->cs_pin_port, GPIO_PIN_RESET);
	uint8_t txbuf[2] = {0x00, 0x00};
	uint8_t rxbuf[2];
	HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)device->spi_handle, txbuf, rxbuf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*)device->cs_pin_port, GPIO_PIN_SET);
	return (rxbuf[0] << 8) | (rxbuf[1] & 0xFF);
}

/**
 * @brief writes a value to a register, given the value
 * @param device instance of ADIS IMU device
 * @param addr register address to write to
 * @param value value to write to register
*/
void adis_write_register(struct ADIS_Device *device, uint8_t addr, uint16_t value) {
	uint16_t address = (addr | 0x80) << 8;
	uint16_t low_word = (address | (value & 0xFF));
	uint16_t high_word = (address | 0x100) | ((value >> 8) & 0xFF);
	uint8_t upper_word[2] = {high_word >> 8, high_word & 0xFF};
	uint8_t lower_word[2] = {low_word >> 8, low_word & 0xFF};

	/* Writing words to SPI channel */
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_RESET);
	HAL_SPI_Transfer((SPI_HandleTypeDef*)device->spi_handle, high_word, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_SET);
	delay_us(5);

	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_RESET);
	HAL_SPI_Transfer((SPI_HandleTypeDef*)device->spi_handle, low_word, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_SET);
	delay_us(5);

}

/**
 * @brief burst reads a batch of output registers and returns a byte array of those registers
 * @param device instance of ADIS IMU deviced
 * @param burst_data array of bytes of size 21
 * @return burst byte data in an array of size 21
*/
uint8_t *byte_burst(struct ADIS_Device *device, uint8_t* burst_data) {
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_RESET);
	uint8_t txbuf[2] = {0x68, 0x00};
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, &txbuf, 2, HAL_MAX_DELAY);
	for (int i = 0; i < 20; i++) {
		HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)device->spi_handle, 0x00, burst_data[i], 1, HAL_MAX_DELAY);
	}
	burst_data[20] = 0x01;
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_SET);
	return burst_data;
}

/**
 * @brief burst reads a batch of output registers and returns a word array of those registers
 * @param device instance of ADIS IMU device
 * @return burst word data in an array of size 10
*/
uint16_t *word_burst(struct ADIS_Device *device, uint16_t* burst_words) {
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_RESET);
	uint8_t txbuf[2] = {0x68, 0x00};
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, &txbuf, 2, HAL_MAX_DELAY);
	for (int i = 0; i < 10; i++) {
		uint8_t txbuf[2] = {0x00, 0x00};
		uint8_t rxbuf[2];
		HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)device->spi_handle, txbuf, rxbuf, 2, HAL_MAX_DELAY);
		burst_words[i] = (rxbuf[0] << 8) | rxbuf[1];
	}
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t*) device->cs_pin_port, GPIO_PIN_SET);
	return burst_words;
}

/**
 * @brief converts an array of byte data and combines it into a word array
 * @param burstdata input array of burst byte data
 * @param word_data word data array to modify values
 * @note do this if you like fingering yourself
*/
uint16_t *word_data(uint8_t *burstdata, uint16_t* word_data) {
    word_data[0] = ((burstdata[0] << 8) | (burstdata[1] & 0xFF)); 
    word_data[1] = ((burstdata[2] << 8) | (burstdata[3] & 0xFF)); 
    word_data[2] = ((burstdata[4] << 8) | (burstdata[5] & 0xFF)); 
    word_data[3] = ((burstdata[6] << 8) | (burstdata[7] & 0xFF)); 
    word_data[4] = ((burstdata[8] << 8) | (burstdata[9] & 0xFF)); 
    word_data[5] = ((burstdata[10] << 8) | (burstdata[11] & 0xFF)); 
    word_data[6] = ((burstdata[12] << 8) | (burstdata[13] & 0xFF));
    word_data[7] = ((burstdata[14] << 8) | (burstdata[15] & 0xFF)); 
    word_data[8] = ((burstdata[16] << 8) | (burstdata[17] & 0xFF)); 
    word_data[9] = ((burstdata[18] << 8) | (burstdata[19] & 0xFF)); 
    return word_data;
}

/**
 * @brief updates input struct of register data
 * @param data input structure of ADIS IMU data
 * @param word_data returned words from IMU output
*/

void update_data(ADIS16500_Data* data, uint16_t* word_data) {
	data->diag_stat = word_data[0];
	data->x_gyro_out = (float) word_data[1] * 0.1f;
	data->y_gyro_out = (float) word_data[2] * 0.1f;
	data->z_gyro_out = (float) word_data[3] * 0.1f; //in degrees per second
	data->x_accl_out = (float) word_data[4] * 0.01225f; //in meters per s^2
	data->y_accl_out = (float) word_data[5] * 0.01225f;
	data->z_accl_out = (float) word_data[6] * 0.01225f;
	data->temp_out = (float) word_data[7] * 0.1f; //in celsius
	data->data_cntr = word_data[8];
	data->checksum = word_data[9];
}
