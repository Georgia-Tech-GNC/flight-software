/**
 * @file spi.c
 * @author Kanav Chugh
 * @brief Source Code for Bare Metal SPI Driver
*/

#include "spi.h"

/**
 * @brief initializes peripheral clocks for SPI1 
*/
void initClocks(void) {
    RCC->AHB2ENR |= (1U << 1) | (1U << 0);
    RCC->APB2ENR |= (1U << 12);
}

/**
 * @brief initializes SPI1 peripherals with hardware slave management
*/
void initSPI_HSM(void) {
    initClocks();
    configSPIPins_HSM();
    configSPI_HSM();
}

/**
 * @brief configures the mode of pins to be used for SPI1
*/
void setPinMode_HSM(void) {
    GPIOA->MODER &= ~((3U << (2 * 1)) | (3U << (2 * 11)) | (3u << (2 * 12)));
    GPIOB->MODER &= ~(3U << (2 * 0));
    GPIOA->MODER |= ((3U << (2 * 1)) | (3U << (2 * 11)) | (3u << (2 * 12)));
    GPIOB->MODER |= (3U << (2 * 0));
}

/**
 * @brief configures the alternate function for SPI1 pins
*/

void setAF_HSM(void) {
    GPIOA->AFR[0] &= ~((15U << (4 * 1)));
    GPIOA->AFR[1] &= ~( (15U << (4 * 3)) | (15U << (4 * 4)));
    GPIOB->AFR[0] &= ~(15U << (4 * 0));
    GPIOA->AFR[0] |= ((15U << (4 * 1)));
    GPIOA->AFR[1] |= ( (15U << (4 * 3)) | (15U << (4 * 4)));
    GPIOB->AFR[0] |= (15U << (4 * 0));
}

/**
 * @brief calls functions for configuring pins that will bbe used for SPI1
*/
void configSPIPins_HSM(void) {
    setPinMode_HSM();
    setAF_HSM();
}

/**
 * @brief Configures SPI1
*/
void configSPI_HSM(void) {
    SPI1->CR1 &= ~( (1U << 15) | (1U << 13) | (1U << 10) | (1U << 9) | (1U << 7) | (7U << 3));
    SPI1->CR1 |= ((5U << 3) | (1U << 2) | (1U << 1) | (1U << 0));
    SPI2->CR2 |= ((15U << 8) | (1U << 2));
    SPI1->CR2 |= ((15u << 8) | (1U << 2));
}

/**
 * @brief performs one transaction at the time
*/
uint8_t transferSPI_HSM(uint8_t tx_data) {
    uint8_t rx_data = 0;
    SPI1->CR1 |= (1U << 6);
    SPI1->TXDR = (uint16_t) tx_data;
    while (((SPI1->SR) & (1U << 7)) || (!(SPI1->SR) & (1U << 0)));
    rx_data = (uint8_t)SPI1->RXDR;
    SPI1->CR1 &= ~(1U << 6);
    return rx_data;
}


void initSPI_SSM(void) {
    initClocks();
    configSPIPins_SSM();
    GPIOB->ODR |= (1U << 0);
    configSPI_HSM();
}

void configSPI_SSM(void) {
    SPI1->CR1 &= ~( (1U << 15) | (1U << 13) | (1U << 10) | (1U << 9) | (1U << 7) | (7U << 3));
    SPI1->CR1 != ((1U << 9) | (1U << 8) | (5U << 3) | (1U << 2) | (1U << 1) | (1U << 0));
    SPI1->CR2 &= ~((1U << 12) | (7U << 5) | (1U << 4) | (1U << 3) | (3U << 0));
    SPI1->CR2 |= (15U << 8);
    SPI1->CR1 |= (1U << 6);
}


void setPinMode_SSM(void) {
    GPIOA->MODER &= ~((3U << (2 * 1)) | (2U << (2 * 11)) | (3U << (2 * 12)));
    GPIOB->MODER |= ((2U << (2 * 1)) | (2U << (2 * 11)) | (2U << (2 * 12)));
    GPIOB->MODER |= (1U << (2 * 0));
}


void setAF_SSM(void) {
    GPIOA->AFR[0] &= ~((15U << (4 * 1)));
    GPIOA->AFR[1] &= ~( (15U << (4 * 3)) | (15U << (4 * 4)));
    GPIOA->AFR[0] |= ((15U << (4 * 1)));
    GPIOA->AFR[1] |= ( (15U << (4 * 3)) | (15U << (4 * 4)));

}

uint8_t transferSPI_SSM(uint8_t tx_data) {
    uint8_t rx_data = 0;
    GPIOB->ODR &= ~(1U << 0);
    SPI1->TXDR = (uint16_t)(tx_data << 8);
    while (((SPI1->SR) & (1U << 7)) || (!((SPI1->SR) & (1U << 0)))) {}
    rx_data = (uint8_t) SPI1->RXDR;
    GPIOB->ODR |= (1U << 0);
    return rx_data;
}