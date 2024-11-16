#ifndef ADC_CONVERT_H
#define ADC_CONVERT_H

#include "FreeRTOS.h"
#include "task.h"

#include "globals.h"

#define ADC_CONVERT_FREQ_HZ 100
#define BEGIN_ADC_NOTIFICATION_BIT 0x01

void adc_convert_task(void *args);

#endif