#include "adc.h"
#include "adc_internal.h"
#include "halal.h"
#include "util.h"

#include "jet_vanes.h"

/* 
 * YES, I AM AWARE THAT THIS CODE IS ATROCIOUS. 
 * We need to use some kind of template generation along with a real JSON config file for these ADCs 
 * instead of just using flat defines, but I just don't have the time to do that right now.
 * 
 * - Sam
 */
uint8_t HALAL_adc_init(void) {
    HALAL_ADCInternalInit adc_inits[HALAL_ADC_INTERNAL_MAX_N_MODULES] = { 0 };
    HALAL_ADCInternalInit *adc_init;

#ifdef HALAL_ADC_JET_VANES_USE_ADC1
    adc_init = &adc_inits[HALAL_ADC_INTERNAL_1];

    adc_init->clock_prescaler = HALAL_ADC_JET_VANES_ADC1_CLOCK_PRESCALER;
    adc_init->instance = ADC1;
    adc_init->resolution = HALAL_ADC_JET_VANES_ADC1_RESOLUTION;
    adc_init->dma_instance = HALAL_ADC_JET_VANES_ADC1_DMA_STREAM;
    adc_init->dma_channel = HALAL_ADC_JET_VANES_ADC1_DMA_CHANNEL;
    adc_init->dma_irq = HALAL_ADC_JET_VANES_ADC1_DMA_IRQ;
#endif

#ifdef HALAL_ADC_JET_VANES_USE_ADC2
    adc_init = &adc_inits[HALAL_ADC_INTERNAL_2];

    adc_init->clock_prescaler = HALAL_ADC_JET_VANES_ADC2_CLOCK_PRESCALER;
    adc_init->instance = ADC2;
    adc_init->resolution = HALAL_ADC_JET_VANES_ADC2_RESOLUTION;
    adc_init->dma_instance = HALAL_ADC_JET_VANES_ADC2_DMA_STREAM;
    adc_init->dma_channel = HALAL_ADC_JET_VANES_ADC2_DMA_CHANNEL;
    adc_init->dma_irq = HALAL_ADC_JET_VANES_ADC2_DMA_IRQ;
#endif

#ifdef HALAL_ADC_JET_VANES_USE_ADC3
    adc_init = &adc_inits[HALAL_ADC_INTERNAL_3];

    adc_init->clock_prescaler = HALAL_ADC_JET_VANES_ADC3_CLOCK_PRESCALER;
    adc_init->instance = ADC3;
    adc_init->resolution = HALAL_ADC_JET_VANES_ADC3_RESOLUTION;
    adc_init->dma_instance = HALAL_ADC_JET_VANES_ADC3_DMA_STREAM;
    adc_init->dma_channel = HALAL_ADC_JET_VANES_ADC3_DMA_CHANNEL;
    adc_init->dma_irq = HALAL_ADC_JET_VANES_ADC3_DMA_IRQ;
#endif

    HALAL_ADCInternalChannelInit *channel_init;
    
    adc_init = &adc_inits[HALAL_ADC_JET_VANES_PYRO_I_0_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_PYRO_I_0_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_PYRO_I_0_SAMPLE_TIME;
    channel_init->uuid = ADC_PYRO_I_0;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_PYRO_I_0_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_PYRO_I_0_GPIO_PORT;

    adc_init->n_channels ++;
    
    adc_init = &adc_inits[HALAL_ADC_JET_VANES_PYRO_I_1_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_PYRO_I_1_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_PYRO_I_1_SAMPLE_TIME;
    channel_init->uuid = ADC_PYRO_I_1;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_PYRO_I_1_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_PYRO_I_1_GPIO_PORT;
  
    adc_init->n_channels ++;
    
    adc_init = &adc_inits[HALAL_ADC_JET_VANES_PYRO_I_2_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_PYRO_I_2_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_PYRO_I_2_SAMPLE_TIME;
    channel_init->uuid = ADC_PYRO_I_2;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_PYRO_I_2_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_PYRO_I_2_GPIO_PORT;
  
    adc_init->n_channels ++;

    adc_init = &adc_inits[HALAL_ADC_JET_VANES_VCC_I_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_VCC_I_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_VCC_I_SAMPLE_TIME;
    channel_init->uuid = ADC_VCC_I;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_VCC_I_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_VCC_I_GPIO_PORT;
  
    adc_init->n_channels ++;

    adc_init = &adc_inits[HALAL_ADC_JET_VANES_SERVO_0_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_SERVO_0_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_SERVO_0_SAMPLE_TIME;
    channel_init->uuid = ADC_SERVO_0;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_SERVO_0_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_SERVO_0_GPIO_PORT;
  
    adc_init->n_channels ++;

    adc_init = &adc_inits[HALAL_ADC_JET_VANES_SERVO_1_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_SERVO_1_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_SERVO_1_SAMPLE_TIME;
    channel_init->uuid = ADC_SERVO_1;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_SERVO_1_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_SERVO_1_GPIO_PORT;
  
    adc_init->n_channels ++;

    adc_init = &adc_inits[HALAL_ADC_JET_VANES_SERVO_2_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_SERVO_2_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_SERVO_2_SAMPLE_TIME;
    channel_init->uuid = ADC_SERVO_2;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_SERVO_2_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_SERVO_2_GPIO_PORT;
  
    adc_init->n_channels ++;

    adc_init = &adc_inits[HALAL_ADC_JET_VANES_SERVO_3_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_SERVO_3_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_SERVO_3_SAMPLE_TIME;
    channel_init->uuid = ADC_SERVO_3;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_SERVO_3_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_SERVO_3_GPIO_PORT;
  
    adc_init->n_channels ++;

    adc_init = &adc_inits[HALAL_ADC_JET_VANES_SERVO_4_MODULE];
    channel_init = &adc_init->channels[adc_init->n_channels];

    channel_init->channel = HALAL_ADC_JET_VANES_SERVO_4_CHANNEL;
    channel_init->sampling_time = HALAL_ADC_JET_VANES_SERVO_4_SAMPLE_TIME;
    channel_init->uuid = ADC_SERVO_4;
    channel_init->gpio_pin = HALAL_ADC_JET_VANES_SERVO_4_GPIO_PIN;
    channel_init->gpio_port = HALAL_ADC_JET_VANES_SERVO_4_GPIO_PORT;

    adc_init->n_channels ++;

    HALAL_ADCInternalHandle dummy;

#ifdef HALAL_ADC_JET_VANES_USE_ADC1
    if (!adc_internal_init(&adc_inits[HALAL_ADC_INTERNAL_1], &dummy)) {
        return RET_FAILURE;
    }
#endif

#ifdef HALAL_ADC_JET_VANES_USE_ADC2
    if (!adc_internal_init(&adc_inits[HALAL_ADC_INTERNAL_2], &dummy)) {
        return RET_FAILURE;
    }
#endif

#ifdef HALAL_ADC_JET_VANES_USE_ADC3
    if (!adc_internal_init(&adc_inits[HALAL_ADC_INTERNAL_3], &dummy)) {
        return RET_FAILURE;
    }
#endif

    return RET_SUCCESS;
}

uint8_t HALAL_adc_convert(void) {
#ifdef HALAL_ADC_JET_VANES_USE_ADC1
    if (!adc_internal_convert(HALAL_ADC_INTERNAL_1)) {
        return RET_FAILURE;
    }
#endif

#ifdef HALAL_ADC_JET_VANES_USE_ADC2
    if (!adc_internal_convert(HALAL_ADC_INTERNAL_2)) {
        return RET_FAILURE;
    }
#endif

#ifdef HALAL_ADC_JET_VANES_USE_ADC3
    if (!adc_internal_convert(HALAL_ADC_INTERNAL_3)) {
        return RET_FAILURE;
    }
#endif

    return RET_SUCCESS;
}