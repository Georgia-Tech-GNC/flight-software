#ifndef STATE_FLASH_H
#define STATE_FLASH_H

#include "periph_io.h"
#include "globals.h"

#define BEGIN_STATE_FLASH_NOTIFICATION_BIT 0x01

#define FLASH_STATE_NOTIFICATION_BIT 0x01
#define FLASH_SD_CARD_NOTIFICATION_BIT 0x02

#define STATE_FLASH_START_SECTOR 0
#define STATE_FLASH_N_SECTORS 16

#define FLASH_TEST_SIZE EXT_FLASH_SECTOR_SIZE
#define SD_TEST_SIZE 2048

void state_flash_task(void *args);

#endif
