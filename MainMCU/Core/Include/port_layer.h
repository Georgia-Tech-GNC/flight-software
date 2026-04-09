#ifndef PORT_LAYER_H
#define PORT_LAYER_H

#ifndef USE_HW_FLASH_CHIP
//#define USE_HW_FLASH_CHIP
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "semphr.h"
#include "task.h"

#include "port_config.h"
#include "globals.h"
#include "periph_io.h"
#include "master_task.h"
#include "telemetry_task.h"
#include "state_flash.h"
#include "tests.h"

int port_init(void);
void port_start(void);


#endif