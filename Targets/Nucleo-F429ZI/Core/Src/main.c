#include "main.h"
#include "log.h"

#include "rcc.h"
#include "gpio.h"
#include "adc_internal.h"

#include "rocket.h"
#include "target.h"
#include "halal.h"

static void error_handler(void);

int main(void) {
  HAL_Init();

  if (!rcc_init()) {
    error_handler();
  }

  gpio_init();
  
  if (HALAL_init()) {
    log_printf(LOG_INFO, "HALAL initialized");
  } else {
    log_printf(LOG_ERROR, "HALAL not initialized");
  }
  
  rocket_init();
  rocket_start();
}

void HALAL_adc_convert_callback(uint32_t channel_uuid, uint16_t adc_value) {
  log_printf(LOG_INFO, "ADC %d, %d", channel_uuid, adc_value);
}

void error_handler(void) {
  log_printf(LOG_ERROR, "HAL Error Handler entered");
  __disable_irq();
  while (1);
}