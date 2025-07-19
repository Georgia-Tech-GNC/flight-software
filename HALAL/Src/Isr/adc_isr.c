#include "adc_isr.h"
#include "adc_internal.h"
#include "halal.h"
#include "log.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
#ifdef HALAL_ADC_MODULE_ENABLED
    adc_internal_conv_complete(hadc);
#endif
}