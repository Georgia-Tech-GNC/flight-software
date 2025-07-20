#ifndef ADC_HAL_H
#define ADC_HAL_H

#include "halal.h"

#define HALAL_ADC_MAX_N_MODULES 3
#define HALAL_ADC_MAX_N_CHANNELS 16

typedef struct {
    uint16_t uuid;
    uint32_t channel;
    uint32_t sampling_time;
    uint32_t gpio_pin;
    uint32_t gpio_port;
} HALAL_ADCInternalChannelInit;

typedef struct {
    ADC_TypeDef *instance;
    DMA_TypeDef *dma_instance;
    IRQn_Type dma_irq;
    uint32_t dma_channel;
    uint32_t resolution;
    uint32_t clock_prescaler;
    uint8_t n_channels;
    HALAL_ADCInternalChannelInit channels[HALAL_ADC_MAX_N_CHANNELS];
} HALAL_ADCInternalInit;

typedef struct {
    ADC_HandleTypeDef hal_adc;
    DMA_HandleTypeDef hal_dma;
    uint16_t channel_uuids[HALAL_ADC_MAX_N_CHANNELS];
    uint16_t channel_values[HALAL_ADC_MAX_N_CHANNELS];
    uint8_t n_channels;
} HALAL_ADCInternal;

typedef enum {
    HALAL_ADC_INTERNAL_1 = 0,
    HALAL_ADC_INTERNAL_2 = 1,
    HALAL_ADC_INTERNAL_3 = 2
} HALAL_ADCInternalHandle;


uint8_t adc_internal_init(HALAL_ADCInternalInit *init, HALAL_ADCInternalHandle *handle);
uint8_t adc_internal_convert(HALAL_ADCInternalHandle handle);

#endif