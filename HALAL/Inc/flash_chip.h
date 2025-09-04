#ifndef FLASH_CHIP_H
#define FLASH_CHIP_H

#include <stdint.h>
#include <stddef.h>
#include "halal.h"

#ifdef HALAL_FAKE_FLASH_CHIP_ENABLED
    #define HALAL_FLASH_PAGE_SIZE HALAL_FAKE_FLASH_CHIP_PAGE_SIZE
#endif

uint8_t HALAL_flash_init(void);
uint8_t HALAL_flash_write_page(size_t page, uint8_t *data, size_t size);
uint8_t HALAL_flash_read_page(size_t page, uint8_t *data, size_t n_pages);
uint8_t HALAL_flash_erase_page(size_t page, size_t n_pages);

#endif