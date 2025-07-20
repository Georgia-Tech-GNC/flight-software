#include "flash_chip.h"
#include "halal.h"
#include "util.h"
#include <string.h>

#define HALAL_FAKE_FLASH_CHIP_SIZE (HALAL_FAKE_FLASH_CHIP_PAGE_SIZE * HALAL_FAKE_FLASH_CHIP_N_PAGES)

uint8_t fake_flash_chip[HALAL_FAKE_FLASH_CHIP_SIZE];
uint8_t flash_init = 0;

uint8_t HALAL_flash_init(void) {
    flash_init = 1;

    return RET_SUCCESS;
}

uint8_t HALAL_flash_write_page(size_t page, uint8_t *data, size_t n_pages) {
    if (page >= HALAL_FAKE_FLASH_CHIP_N_PAGES || page + n_pages >= HALAL_FAKE_FLASH_CHIP_N_PAGES) {
        return RET_FAILURE;
    }

    memcpy(fake_flash_chip + page * HALAL_FAKE_FLASH_CHIP_PAGE_SIZE, data, n_pages * HALAL_FAKE_FLASH_CHIP_PAGE_SIZE);

    return RET_SUCCESS;
}


uint8_t HALAL_flash_read_page(size_t page, uint8_t *data, size_t n_pages) {
    if (page >= HALAL_FAKE_FLASH_CHIP_N_PAGES || page + n_pages >= HALAL_FAKE_FLASH_CHIP_N_PAGES) {
        return RET_FAILURE;
    }

    memcpy(data, fake_flash_chip + page * HALAL_FAKE_FLASH_CHIP_PAGE_SIZE, n_pages * HALAL_FAKE_FLASH_CHIP_PAGE_SIZE);

    return RET_SUCCESS;
}

uint8_t HALAL_flash_erase_page(size_t page, size_t n_pages) {
    if (page >= HALAL_FAKE_FLASH_CHIP_N_PAGES || page + n_pages >= HALAL_FAKE_FLASH_CHIP_N_PAGES) {
        return RET_FAILURE;
    }

    memset(fake_flash_chip + page * HALAL_FAKE_FLASH_CHIP_PAGE_SIZE, 0, n_pages * HALAL_FAKE_FLASH_CHIP_PAGE_SIZE);
}
