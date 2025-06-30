#ifndef STATE_FLASH_H
#define STATE_FLASH_H

#define BEGIN_STATE_FLASH_NOTIFICATION_BIT 0x01

#define FLASH_STATE_NOTIFICATION_BIT 0x01
#define FLASH_SD_CARD_NOTIFICATION_BIT 0x02

void state_flash_task(void *args);

#endif
