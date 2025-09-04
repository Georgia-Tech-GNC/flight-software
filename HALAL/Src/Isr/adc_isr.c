#include "adc_isr.h"
#include "adc_internal.h"
#include "halal.h"
#include "log.h"

#include "FreeRTOS.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#ifdef HALAL_ADC_MODULE_ENABLED
    adc_internal_conv_complete(hadc, &xHigherPriorityTaskWoken);
#endif

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}