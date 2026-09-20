#include "lsm6dso.h"

/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */

//register values
//Page 39
#define WHO_AM_I 0x0F
#define WHO_AM_I_VALUE 0x6C
//hex form of binary of 1000 0000
#define SPI_READ_BIT 0x80

//turn gyroscope on
#define REG_CTRL2_G 0x11

//reset register
#define REG_CTRL3_C 0x12
//SW reset bit
#define CTRL3_C_BIT 0x01
//BDU+INCREMENT
#define	BDU_BIT	0x40
#define IF_INC_BIT 0x04

//pitch
#define REG_OUTX_L_G 0x22
//roll
#define REG_OUTY_L_G 0x24
//yaw
#define REG_OUTZ_L_G 0x26

//Page 9 - +-250dps
#define GYRO_SENSITIVITY 0.00875f
#include <stdio.h>



bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    //reset to make sure clean state
     //spi write - first part (what register), second (what value)
    uint8_t reset_tx[2] = { REG_CTRL3_C, CTRL3_C_BIT};
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_handle, reset_tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

     //Set BDU to 1 - BDU | INC = 0x40 + 0x04 = 0x44
    uint8_t bdu_tx[2] = { REG_CTRL3_C, 0x44};
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_handle, bdu_tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    //turn gyro on
    uint8_t gyro_tx[2] = { REG_CTRL2_G, 0x40};
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_handle, gyro_tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    //avoid reading garbage data in tx[0]
    //transmit just the register to read value from there
    uint8_t tx[2] = { SPI_READ_BIT | WHO_AM_I, 0x00};
    uint8_t rx[2] = { 0 };
     //cs low (select)
    for (int i = 0; i<10; i++) {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);  
    	HAL_SPI_TransmitReceive(spi_handle, tx, rx, 2, HAL_MAX_DELAY);
    	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);    
    	char dbg[32];
    	int n = snprintf(dbg, sizeof dbg, "WHOAMI=0x%02X\r\n", rx[1]);
    	HAL_UART_Transmit(&huart3, (uint8_t*)dbg, (uint16_t)n, HAL_MAX_DELAY);
    	HAL_Delay(100);
    }
    //make sure correct value returned
    return rx[1] == WHO_AM_I_VALUE;
}

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    //Read the pitch data from lower register (2 registers)
	//set bit to state that it will read next value
	//block data
    uint8_t tx[3] = { SPI_READ_BIT | REG_OUTX_L_G, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_handle, tx, rx, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    //low byte from first register
    uint8_t initial_gyro_value_low  = rx[1];
    //high byte from second register
    uint8_t initial_gyro_value_high = rx[2];  

    //combine two registers (shift upper) with bitwise OR
    int16_t gyro_value = (int16_t)((uint16_t)initial_gyro_value_low | (uint16_t)initial_gyro_value_high << 8);

    return (float)gyro_value * GYRO_SENSITIVITY;
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    uint8_t tx[3] = { SPI_READ_BIT | REG_OUTZ_L_G, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_handle, tx, rx, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    //low byte from first register
    uint8_t initial_gyro_value_low  = rx[1];
    //high byte from second register
    uint8_t initial_gyro_value_high = rx[2];  

    //combine two registers (shift upper) with bitwise OR
    int16_t gyro_value = (int16_t)((uint16_t)initial_gyro_value_low | (uint16_t)initial_gyro_value_high << 8);

    return (float)gyro_value * GYRO_SENSITIVITY;
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    uint8_t tx[3] = { SPI_READ_BIT | REG_OUTY_L_G, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_handle, tx, rx, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    //low byte from first register
    uint8_t initial_gyro_value_low  = rx[1];
    //high byte from second register
    uint8_t initial_gyro_value_high = rx[2];  

    //combine two registers (shift upper) with bitwise OR
    int16_t gyro_value = (int16_t)((uint16_t)initial_gyro_value_low | (uint16_t)initial_gyro_value_high << 8);

    return (float)gyro_value * GYRO_SENSITIVITY;
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}
