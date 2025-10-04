#include "Nucleo-H723ZG/Modules/Inc/gpio.h"
#include "stm32h7xx_hal.h"
#include "util.h"

uint8_t gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = USB_POWER_SWITCH_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_POWER_SWITCH_PORT, &gpio_init);
    HAL_GPIO_WritePin(USB_POWER_SWITCH_PORT, USB_POWER_SWITCH_PIN, GPIO_PIN_SET);

    return RET_SUCCESS;
}