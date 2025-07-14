#include "main.h"
#include "fatfs.h"
#include "log.h"
#include "usb_host.h"

#include "rcc.h"
#include "gpio.h"
#include "tim.h"
#include "adc.h"
#include "usb_host.h"

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

  if (tim_init()) {
    log_printf(LOG_INFO, "TIM initialized");
  } else {
    log_printf(LOG_ERROR, "TIM not initialized");
    error_handler();
  }
  
  if (adc_init()) {
    log_printf(LOG_INFO, "ADC initialized");
  } else {
    log_printf(LOG_ERROR, "ADC not initialized");
    error_handler();
  }

  if (FATFS_LinkDriver(&usbh_diskio_driver, "") == 0) {
    log_printf(LOG_INFO, "FATFS driver linked");
  } else {
    log_printf(LOG_ERROR, "FATFS driver not linked");
    error_handler();
  }

  if (usb_host_init()) {
    log_printf(LOG_INFO, "USB host initialized");
  } else {
    log_printf(LOG_ERROR, "USB host not initialized");
    error_handler();
  }

  rocket_init();

  if (HALAL_radio_start()) {
    log_printf(LOG_INFO, "Radio started");
  } else {
    log_printf(LOG_ERROR, "Radio not started");
  }

  if (HAL_TIM_Base_Start_IT(&g_tim2) == HAL_OK) {
    log_printf(LOG_INFO, "USB update timer started");
  } else {
    log_printf(LOG_ERROR, "USB update timer not started");
    error_handler();
  }

  rocket_start();
}

void error_handler(void) {
  log_printf(LOG_ERROR, "HAL Error Handler entered");
  __disable_irq();
  while (1);
}