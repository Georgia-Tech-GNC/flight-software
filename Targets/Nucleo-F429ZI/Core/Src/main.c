#include "main.h"
#include "fatfs.h"
#include "log.h"
#include "usb_host.h"

#include "rcc.h"
#include "uart.h"
#include "gpio.h"
#include "usb_host.h"

#include "rocket.h"
#include "target.h"

static void error_handler(void);

int main(void) {
  HAL_Init();

  if (!rcc_init()) {
    error_handler();
  }

  gpio_init();
  
  if (uart_init()) {
    log_printf(LOG_INFO, "UART Initialized");
  } else {
    error_handler();
  }
  
  FATFS_LinkDriver(&usbh_diskio_driver, "");

  usb_host_init();

  for (int i = 0; i < 5; i ++) {
    log_printf(LOG_INFO, "Testing HAL_DELAY (1 second): %ld", HAL_GetTick());
    HAL_Delay(1000);
  }

  rocket_init();
  rocket_start();
}

void error_handler(void) {
  log_printf(LOG_ERROR, "HAL Error Handler entered");
  __disable_irq();
  while (1);
}