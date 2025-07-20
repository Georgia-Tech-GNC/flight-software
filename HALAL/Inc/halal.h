#ifndef HALAL_H
#define HALAL_H

#ifdef TARGET_NUCLEO_F429ZI
    #include "stm32f4xx_hal.h"
    #include "nucleo_f429zi_halal.h"
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

uint8_t HALAL_init(void);

#endif