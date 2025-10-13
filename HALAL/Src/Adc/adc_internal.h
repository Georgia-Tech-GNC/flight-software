///@file adc_internal.h

#ifndef ADC_INTERNAL_H
#define ADC_INTERNAL_H

#include "halal.h"

#define HALAL_ADC_INTERNAL_MAX_N_MODULES 3      ///< Maximum number of internal ADC modules supported by the HALAL.
#define HALAL_ADC_INTERNAL_MAX_N_CHANNELS 16    ///< Maximum number of channels per internal ADC supported by the HALAL

///Initialization struct for a HALAL internal ADC channel.
typedef struct {
    uint16_t uuid;          ///< Global unique identifier (across all ADC channels in the system) to be passed to HALAL_adc_convert_callback
    uint32_t channel;       ///< ADC channel ID as defined by the HAL
    uint32_t sampling_time; ///< Sampling time as defined by the HAL
    uint32_t gpio_pin;      ///< GPIO pin as defined by the HAL
    GPIO_TypeDef *gpio_port;     ///< GPIO port as defined by the HAL
} HALAL_ADCInternalChannelInit;

///Initialization struct of a HALAL internal ADC module.
typedef struct {
    ADC_TypeDef *instance;                                                      ///< Base address of the ADC module as defined by the HAL
    DMA_Stream_TypeDef *dma_instance;                                                  ///< Base address of the DMA module as defined by the HAL
    IRQn_Type dma_irq;                                                          ///< IRQ number as defined by the HAL
    uint32_t dma_channel;                                                       ///< DMA channel ID as defined by the HAL
    uint32_t resolution;                                                        ///< ADC resolution as defined by the HAL
    uint32_t clock_prescaler;                                                   ///< ADC clock prescaler as defined by the HAL
    uint8_t n_channels;                                                         ///< Number of channels in the module
    HALAL_ADCInternalChannelInit channels[HALAL_ADC_INTERNAL_MAX_N_CHANNELS];   ///< Array of channel init structs
} HALAL_ADCInternalInit;

/** Represents a HALAL internal ADC module. Note that this is not the handle that the user will use, rather it is only internal to the HALAL */
typedef struct {
    ADC_HandleTypeDef hal_adc;                                    ///< HAL ADC handle
    DMA_HandleTypeDef hal_dma;                                    ///< HAL DMA handle
    uint16_t channel_uuids[HALAL_ADC_INTERNAL_MAX_N_CHANNELS];    ///< Array of channel UUIDs
    uint32_t channel_values[HALAL_ADC_INTERNAL_MAX_N_CHANNELS];   ///< Array of channel values
    uint8_t n_channels;                                           ///< Number of channels in the module
} HALAL_ADCInternal;

/** HALAL internal ADC handle */
typedef enum {
    HALAL_ADC_INTERNAL_1 = 0,   ///< HAL ADC1
    HALAL_ADC_INTERNAL_2 = 1,   ///< HAL ADC2
    HALAL_ADC_INTERNAL_3 = 2    ///< HAL ADC3
} HALAL_ADCInternalHandle;

uint8_t adc_internal_init(HALAL_ADCInternalInit *init, HALAL_ADCInternalHandle *handle);
uint8_t adc_internal_convert(HALAL_ADCInternalHandle handle);

#endif