#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#include "stm32h7xx_hal.h"
#include "adc.h"

/* Change these to the values for your board */
#define MCU_H725ZGT6

#define SD_CS_GPIO_PORT     GPIOG
#define SD_CS_PIN           GPIO_PIN_10

#define FLASH_CS_GPIO_PORT  GPIOG
#define FLASH_CS_PIN        GPIO_PIN_6

#define LD1_GPIO_PORT       GPIOB
#define LD1_PIN             GPIO_PIN_0
#define LD2_GPIO_PORT       GPIOE
#define LD2_PIN             GPIO_PIN_1
#define LD3_GPIO_PORT       GPIOB
#define LD3_PIN             GPIO_PIN_14

#define telemetry_uart      huart4
#define state_uart          huart5
#define debug_uart          huart2

#define sd_spi              hspi1

#define flash_spi           hospi1

//#define USE_ADC1
#define USE_ADC2
#define USE_ADC3

#define ADC2_N_CHANNELS     1
#define ADC3_N_CHANNELS     3

#define FIRST_ADC           &hadc2
#define ADC2_NEXT           &hadc3
#define ADC3_NEXT           NULL

static const ADC_Channel ADC2_SEQUENCE[ADC2_N_CHANNELS] = {
    ADC_PYRO_I_2,
};

static const ADC_Channel ADC3_SEQUENCE[ADC3_N_CHANNELS] = {
    ADC_PYRO_I_0,
    ADC_PYRO_I_1,
    ADC_VCC_I,
};

#endif