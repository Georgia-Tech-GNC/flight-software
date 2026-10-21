/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shared_main.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PYRO1_EN_Pin GPIO_PIN_2
#define PYRO1_EN_GPIO_Port GPIOE
#define PYRO1_STATUS_Pin GPIO_PIN_4
#define PYRO1_STATUS_GPIO_Port GPIOE
#define PYRO2_EN_Pin GPIO_PIN_5
#define PYRO2_EN_GPIO_Port GPIOE
#define SPI2_CS2_Pin GPIO_PIN_14
#define SPI2_CS2_GPIO_Port GPIOC
#define SPI2_CS1_Pin GPIO_PIN_15
#define SPI2_CS1_GPIO_Port GPIOC
#define PYRO_GLOBAL_EN_Pin GPIO_PIN_0
#define PYRO_GLOBAL_EN_GPIO_Port GPIOA
#define PIEZO_OUT_Pin GPIO_PIN_4
#define PIEZO_OUT_GPIO_Port GPIOA
#define GPIO_1_Pin GPIO_PIN_12
#define GPIO_1_GPIO_Port GPIOB
#define GPIO_2_Pin GPIO_PIN_10
#define GPIO_2_GPIO_Port GPIOD
#define PWM_1_Pin GPIO_PIN_12
#define PWM_1_GPIO_Port GPIOD
#define PWM_2_Pin GPIO_PIN_13
#define PWM_2_GPIO_Port GPIOD
#define PWM_3_Pin GPIO_PIN_14
#define PWM_3_GPIO_Port GPIOD
#define PWM_4_Pin GPIO_PIN_15
#define PWM_4_GPIO_Port GPIOD
#define STATUS_LED_R_Pin GPIO_PIN_6
#define STATUS_LED_R_GPIO_Port GPIOC
#define STATUS_LED_G_Pin GPIO_PIN_7
#define STATUS_LED_G_GPIO_Port GPIOC
#define STATUS_LED_B_Pin GPIO_PIN_8
#define STATUS_LED_B_GPIO_Port GPIOC
#define GPS_PPS_Pin GPIO_PIN_15
#define GPS_PPS_GPIO_Port GPIOA
#define SPI3_CS2_Pin GPIO_PIN_2
#define SPI3_CS2_GPIO_Port GPIOD
#define SPI3_CS1_Pin GPIO_PIN_3
#define SPI3_CS1_GPIO_Port GPIOD
#define SPI1_CS2_Pin GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOD
#define SPI1_CS1_Pin GPIO_PIN_5
#define SPI1_CS1_GPIO_Port GPIOD
#define GPIO_HPSB_Pin GPIO_PIN_7
#define GPIO_HPSB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
