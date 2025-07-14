#ifndef HALAL_H
#define HALAL_H

#ifdef TARGET_NUCLEO_F429ZI
    #include "stm32f4xx_hal.h"
    #include "nucleo_f429zi_halal.h"
#endif

uint8_t HALAL_init(void);

#endif