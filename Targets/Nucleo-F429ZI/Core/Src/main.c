#include "main.h"
#include "log.h"

#include "rcc.h"
#include "gpio.h"
#include "adc.h"

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

  if (adc_init()) {
    log_printf(LOG_INFO, "ADC initialized");
  } else {
    log_printf(LOG_ERROR, "ADC not initialized");
    error_handler();
  }

  rocket_init();
  rocket_start();
}

void error_handler(void) {
  log_printf(LOG_ERROR, "HAL Error Handler entered");
  __disable_irq();
  while (1);
}