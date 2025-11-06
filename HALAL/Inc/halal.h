#ifndef HALAL_H
#define HALAL_H

#include "FreeRTOS.h"

#ifdef TARGET_NUCLEO_F429ZI
    #include "stm32f4xx_hal.h"
    #include "nucleo_f429zi_halal.h"
#endif

#ifdef TARGET_NUCLEO_H723ZG
    #include "stm32h7xx_hal.h"
    #include "nucleo_h723zg_halal.h"
#endif

#ifdef HALAL_ADC_MODULE_ENABLED
    #include "adc.h"
#endif

#ifdef HALAL_DEBUG_MODULE_ENABLED
    #include "debug.h"
#endif

#ifdef HALAL_RADIO_MODULE_ENABLED
    #include "radio.h"
#endif

#ifdef HALAL_STORAGE_MODULE_ENABLED
    #include "storage.h"
#endif

#ifdef HALAL_FLASH_CHIP_MODULE_ENABLED
    #include "flash_chip.h"
#endif

#ifdef HALAL_MAGNETOMETER_MODULE_ENABLED
    #include "magnetometer.h"
#endif

uint8_t HALAL_init(void);

/* HALAL Weak function definitions */
void HALAL_adc_convert_callback(uint32_t channel_uuid, uint32_t adc_value, BaseType_t *xHigherPriorityTaskWoken);
void HALAL_state_estimation_callback(uint8_t *state_estimation_bytes, size_t size, BaseType_t *xHigherPriorityTaskWoken);
void HALAL_radio_callback(uint8_t *radio_bytes, size_t size, BaseType_t *xHigherPriorityTaskWoken);

#endif