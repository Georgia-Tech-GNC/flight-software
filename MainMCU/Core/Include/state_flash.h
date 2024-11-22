#ifndef STATE_FLASH_H
#define STATE_FLASH_H

#include "state.h"
#include "globals.h"
#include "periph_io.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "string.h"

#define FLASH_FREQ_HZ 10

#define BEGIN_STATE_FLASH_NOTIFICATION_BIT 0x01

#define FLASH_SD_CARD_NOTIFICATION_BIT 0x02
#define SD_WRITE_COMPLETE_NOTIFICATION_BIT 0x04
#define SD_READ_COMPLETE_NOTIFICATION_BIT 0x20
#define FLASH_WRITE_COMPLETE_NOTIFICATION_BIT 0x08
#define FLASH_READ_COMPLETE_NOTIFICATION_BIT 0x10
#define FLASH_RESET_COMPLETE_NOTIFICATION_BIT 0x40
#define SD_RESET_COMPLETE_NOTIFICATION_BIT 0x80

#define FLASH_WRITE_CHANNEL_ID 0
#define FLASH_READ_CHANNEL_ID 1
#define SD_WRITE_CHANNEL_ID 2
#define SD_READ_CHANNEL_ID 3

void state_flash_task(void *args);

#endif