#include "adc.h"
#include "util.h"

ADC_HandleTypeDef g_adc2 = {0};
ADC_HandleTypeDef g_adc3 = {0};

static uint8_t adc2_init(void);
static uint8_t adc3_init(void);

static void adc2_msp_init(void);
static void adc3_msp_init(void);

uint8_t adc_init(void) {
    if (!adc2_init()) return RET_FAILURE;
    if (!adc3_init()) return RET_FAILURE;
    
    return RET_SUCCESS;
}

static void adc2_msp_init(void) {
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_ADC2_CLK_ENABLE();

    /*
        PC3 -> ADC2_IN13
    */

    gpio_init.Pin = GPIO_PIN_3;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    HAL_NVIC_SetPriority(ADC_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}



static uint8_t adc2_init(void) {
    ADC_ChannelConfTypeDef adc_channel = {0};

    g_adc2.Instance = ADC2;
    g_adc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_adc2.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc2.Init.ScanConvMode = DISABLE;
    g_adc2.Init.ContinuousConvMode = ENABLE;
    g_adc2.Init.DiscontinuousConvMode = DISABLE;
    g_adc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_adc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc2.Init.NbrOfConversion = 1;
    g_adc2.Init.DMAContinuousRequests = DISABLE;
    g_adc2.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;

    if (HAL_ADC_Init(&g_adc2) != HAL_OK) {
        return RET_FAILURE;
    }

    adc_channel.Channel = ADC_CHANNEL_13;
    adc_channel.Rank = 1;
    adc_channel.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&g_adc2, &adc_channel) != HAL_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static uint8_t adc3_init(void) {
    ADC_ChannelConfTypeDef adc_channel = {0};

    g_adc3.Instance = ADC3;
    g_adc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_adc3.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc3.Init.ScanConvMode = DISABLE;
    g_adc3.Init.ContinuousConvMode = ENABLE;
    g_adc3.Init.DiscontinuousConvMode = DISABLE;
    g_adc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_adc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc3.Init.NbrOfConversion = 1;
    g_adc3.Init.DMAContinuousRequests = DISABLE;
    g_adc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&g_adc3) != HAL_OK) {
        return RET_FAILURE;
    }

    adc_channel.Channel = ADC_CHANNEL_6;
    adc_channel.Rank = 1;
    adc_channel.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&g_adc3, &adc_channel) != HAL_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC2) adc2_msp_init();
    if (hadc->Instance == ADC3) adc3_msp_init();
}

static void adc3_msp_init(void) {
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_ADC3_CLK_ENABLE();

    /*
        PF3 -> ADC3_IN9
        PF8 -> ADC3_IN6
        PC0 -> ADC3_IN10
    */

    gpio_init.Pin = GPIO_PIN_3|GPIO_PIN_8;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio_init);

    gpio_init.Pin = GPIO_PIN_0;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    HAL_NVIC_SetPriority(ADC_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}