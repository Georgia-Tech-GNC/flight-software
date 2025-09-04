#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include "ff_gen_drv.h"

uint8_t HALAL_storage_init(void);
uint8_t HALAL_storage_start(void);

DSTATUS HALAL_storage_initialize(BYTE);
DSTATUS HALAL_storage_status(BYTE);
DRESULT HALAL_storage_read(BYTE, BYTE*, DWORD, UINT);
DRESULT HALAL_storage_write(BYTE, const BYTE*, DWORD, UINT);
DRESULT HALAL_storage_ioctl(BYTE, BYTE, void*);

#endif