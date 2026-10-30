#ifndef PORT_H
#define PORT_H

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;

extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1, huart3, huart7;
extern SPI_HandleTypeDef hspi1, hspi2, hspi3;
extern OSPI_HandleTypeDef hospi1;
extern I2C_HandleTypeDef hi2c1, hi2c2, hi2c3;

extern TIM_HandleTypeDef htim3, htim4, htim5;

// For backwards compatibility
// TODO: delete after we update shared_main.c to not reference these.
#define LED1_GPIO_PORT STATUS_LED_R_GPIO_Port
#define LED1_PIN STATUS_LED_R_Pin
#define LED2_GPIO_PORT STATUS_LED_G_GPIO_Port
#define LED2_PIN STATUS_LED_G_Pin
#define LED3_GPIO_PORT STATUS_LED_B_GPIO_Port
#define LED3_PIN STATUS_LED_B_Pin

#define debug_uart huart3
#define rgb_led_tim htim3

#define RGB_LED_CH_R TIM_CHANNEL_1
#define RGB_LED_CH_G TIM_CHANNEL_2
#define RGB_LED_CH_B TIM_CHANNEL_3

#endif
