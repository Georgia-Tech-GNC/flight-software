/**
 * @file adc_internal.c
 * @brief Helper functions to make it easier to initialize HAL ADC modules.
 * 
 * Note that this is NOT a HALAL module itself, rather it is provided to make creating ADC HALAL modules easier.
 * This file also handles adc interrupts and sends the data to HALAL_adc_convert_callback 
 */

#include "adc_internal.h"
#include "adc_isr.h"
#include "util.h"

uint8_t n_handles = 0;
HALAL_ADCInternal adcs[HALAL_ADC_INTERNAL_MAX_N_MODULES];

uint8_t adc_internal_init(HALAL_ADCInternalInit *init, HALAL_ADCInternalHandle *handle) {
    if (n_handles >= HALAL_ADC_INTERNAL_MAX_N_MODULES) {
        return RET_FAILURE;
    }

    if (init->n_channels >= HALAL_ADC_INTERNAL_MAX_N_MODULES) {
        return RET_FAILURE;
    }

    if (init->instance == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        *handle = HALAL_ADC_INTERNAL_1;
    } else if (init->instance == ADC2) {
        __HAL_RCC_ADC2_CLK_ENABLE();
        *handle = HALAL_ADC_INTERNAL_2;
    } else if (init->instance == ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
        *handle = HALAL_ADC_INTERNAL_3;
    } else {
        return RET_FAILURE;
    }

    HALAL_ADCInternal *adc = &adcs[*handle];

    for (uint8_t i = 0; i < init->n_channels; i ++) {
        adc->channel_uuids[i] = init->channels[i].uuid;
    }

    adc->n_channels = init->n_channels;
    
    ADC_HandleTypeDef *hal_adc = &adc->hal_adc;
    ADC_ChannelConfTypeDef adc_channel = {0};

    hal_adc->Instance = init->instance;
    hal_adc->Init.ClockPrescaler = init->clock_prescaler;
    hal_adc->Init.Resolution = init->resolution;
    hal_adc->Init.ScanConvMode = ENABLE;
    hal_adc->Init.ContinuousConvMode = DISABLE;
    hal_adc->Init.DiscontinuousConvMode = DISABLE;
    hal_adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hal_adc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hal_adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hal_adc->Init.NbrOfConversion = init->n_channels;
    hal_adc->Init.DMAContinuousRequests = DISABLE;
    hal_adc->Init.EOCSelection = ADC_EOC_SEQ_CONV;

    if (HAL_ADC_Init(hal_adc) != HAL_OK) {
        return RET_FAILURE;
    }

    for (uint8_t i = 0; i < init->n_channels; i ++) {
        HALAL_ADCInternalChannelInit *channel = &init->channels[i];

        adc_channel.Channel = channel->channel;
        adc_channel.SamplingTime = channel->sampling_time;
        adc_channel.Rank = i + 1;

        GPIO_InitTypeDef gpio_init = {0};

        gpio_init.Pin = channel->gpio_pin;
        gpio_init.Mode = GPIO_MODE_ANALOG;
        gpio_init.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(channel->gpio_port, &gpio_init);

        if (HAL_ADC_ConfigChannel(hal_adc, &adc_channel) != HAL_OK) {
            return RET_FAILURE;
        }
    }

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    DMA_HandleTypeDef *hal_dma = &adc->hal_dma;

    hal_dma->Instance = init->dma_instance;
    hal_dma->Init.Channel = init->dma_channel;
    hal_dma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hal_dma->Init.PeriphInc = DMA_PINC_DISABLE;
    hal_dma->Init.MemInc = DMA_MINC_ENABLE;
    hal_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hal_dma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hal_dma->Init.Mode = DMA_NORMAL;
    hal_dma->Init.Priority = DMA_PRIORITY_LOW;
    hal_dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(hal_dma) != HAL_OK) {
        return RET_FAILURE;
    }

    __HAL_LINKDMA(hal_adc, DMA_Handle, *hal_dma);

    HAL_NVIC_SetPriority(init->dma_irq, 5, 0);
    HAL_NVIC_EnableIRQ(init->dma_irq);

    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    return RET_SUCCESS;
}

#ifdef HALAL_ADC1_HAL_DMA_ISR
void HALAL_ADC1_HAL_DMA_ISR(void) {
    HAL_DMA_IRQHandler(&adcs[HALAL_ADC_INTERNAL_1].hal_dma);
}
#endif

#ifdef HALAL_ADC2_HAL_DMA_ISR
void HALAL_ADC2_HAL_DMA_ISR(void) {
    HAL_DMA_IRQHandler(&adcs[HALAL_ADC_INTERNAL_2].hal_dma);
}
#endif

#ifdef HALAL_ADC3_HAL_DMA_ISR
void HALAL_ADC3_HAL_DMA_ISR(void) {
    HAL_DMA_IRQHandler(&adcs[HALAL_ADC_INTERNAL_3].hal_dma);
}
#endif

void ADC_IRQHandler(void) {
    for (uint8_t i = 0; i < n_handles; i ++) {
        HAL_ADC_IRQHandler(&adcs[i].hal_adc);
    }
}

uint8_t adc_internal_convert(HALAL_ADCInternalHandle handle) {
    HALAL_ADCInternal *adc = &adcs[handle];

    if (HAL_ADC_Start_DMA(&adc->hal_adc, adc->channel_values, adc->n_channels) != HAL_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

void adc_internal_conv_complete(ADC_HandleTypeDef *hal_adc, BaseType_t *xHigherPriorityTaskWoken) {
    HALAL_ADCInternal *adc;

    if (hal_adc->Instance == ADC1) {
        adc = &adcs[HALAL_ADC_INTERNAL_1];
    } else if (hal_adc->Instance == ADC2) {
        adc = &adcs[HALAL_ADC_INTERNAL_2];
    } else if (hal_adc->Instance == ADC3) {
        adc = &adcs[HALAL_ADC_INTERNAL_3];
    }
    
    for (uint8_t j = 0; j < adc->n_channels; j ++) {
        HALAL_adc_convert_callback(adc->channel_uuids[j], adc->channel_values[j], xHigherPriorityTaskWoken);
    }
}