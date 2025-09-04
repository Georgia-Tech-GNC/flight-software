#ifndef ADC_ISR_H
#define ADC_ISR_H

#include "FreeRTOS.h"
#include "halal.h"

#ifdef HALAL_ADC_MODULE_ENABLED
void adc_internal_conv_complete(ADC_HandleTypeDef *hal_adc, BaseType_t *xHigherPriorityTaskWoken);
#endif

#endif