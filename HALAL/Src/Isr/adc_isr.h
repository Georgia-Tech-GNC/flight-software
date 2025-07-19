#ifndef ADC_ISR_H
#define ADC_ISR_H

#ifdef HALAL_ADC_HAL_ENABLED
void adc_hal_conv_complete(HAL_ADCTypeDef *hadc);
#endif

#endif