#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#include "stm32h7xx_hal.h"
#include "stdint.h"
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

#define USE_ADC1
#define USE_ADC2
#define USE_ADC3

#define ADC1_N_CHANNELS     3
#define ADC2_N_CHANNELS     2
#define ADC3_N_CHANNELS     4

#define FIRST_ADC           &hadc1
#define ADC1_NEXT           NULL
#define ADC2_NEXT           NULL
#define ADC3_NEXT           NULL

static const ADC_Channel ADC1_SEQUENCE[ADC1_N_CHANNELS] = {
    ADC_SERVO_2,
    ADC_SERVO_3,
    ADC_SERVO_4
};

static const uint32_t ADC1_CHANNELS[ADC1_N_CHANNELS] = {
    ADC_CHANNEL_2,
    ADC_CHANNEL_5,
    ADC_CHANNEL_9,
};

static const uint32_t ADC2_CHANNELS[ADC2_N_CHANNELS] = {
    ADC_CHANNEL_6,
    ADC_CHANNEL_10,
};

static const uint32_t ADC3_CHANNELS[ADC3_N_CHANNELS] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_2,
    ADC_CHANNEL_6,
    ADC_CHANNEL_8,
};

static const ADC_Channel ADC2_SEQUENCE[ADC2_N_CHANNELS] = {
    ADC_SERVO_1,
    ADC_PYRO_I_2,
};

static const ADC_Channel ADC3_SEQUENCE[ADC3_N_CHANNELS] = {
    ADC_SERVO_0,
    ADC_PYRO_I_0,
    ADC_PYRO_I_1,
    ADC_VCC_I,
};

#define USE_TIM2
#define USE_TIM3
#define USE_TIM4

#define PWM0_TIMER &htim2
#define PWM0_CHANNEL TIM_CHANNEL_1

#define PWM1_TIMER &htim2
#define PWM1_CHANNEL TIM_CHANNEL_2

#define PWM2_TIMER &htim2
#define PWM2_CHANNEL TIM_CHANNEL_3

#define PWM3_TIMER &htim4
#define PWM3_CHANNEL TIM_CHANNEL_2

#define PWM4_TIMER &htim3
#define PWM4_CHANNEL TIM_CHANNEL_2

#endif