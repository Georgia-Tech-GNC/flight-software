#include "periph_io.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "ff.h"

#include "w25q.h"
#include "target.h"

#include "stdio.h"
#include "log.h"

/* Size of flash chip in software emulation */
#define SW_FLASH_CHIP_SIZE (8 * EXT_FLASH_SECTOR_SIZE)

/* Buffer for software emulation of flash chip */
#ifndef USE_HW_FLASH_CHIP
uint8_t _fake_flash_chip[SW_FLASH_CHIP_SIZE];
uint8_t *fake_flash_chip = _fake_flash_chip;
#endif

uint8_t sd_mounted = 0;
uint8_t flash_init = 0;

struct w25q_device flash_chip;
FATFS fs;

BYTE mkfs_work[4096];

int io_init(void) {
    log_printf(LOG_INFO, "Initializing IO devices...");
    f_mkfs("/", FM_FAT32, 0, mkfs_work, sizeof(mkfs_work));
    int sd_status = f_mount(&fs, "/", 1);
    if (sd_status == FR_OK) {
        log_printf(LOG_INFO, "Mounted SD card");
        sd_mounted = 1;
    } else {
        log_printf(LOG_ERROR, "Failed to mount SD card: %d", sd_status);
        sd_mounted = 0;
    }

#ifdef MCU_H725ZGT6
    if (w25q_init(&flash_chip) == W25Q_ERR_OK) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Initialized flash chip\r\n", 24, HAL_MAX_DELAY);
        flash_init = 1;
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to initialize flash chip\r\n", 32, HAL_MAX_DELAY);
        flash_init = 0;
    }
#else
    flash_init = 1;
#endif

    return sd_mounted && flash_init;
}

int sd_init_file(SDFile *file, const char *file_path) {
    if (strlen(file_path) >= SD_MAX_PATH_LEN) {
        return 0;
    }

    strcpy(file->file_path, file_path);

    file->current_mode = 0;
    file->is_open = 0;
    file->is_deleted = 0;

    return 1;
}

int flash_init_block(FlashBlock *block, uint16_t start_sector, uint16_t n_sectors) {
    if (block == NULL) {
        return 0;
    }

    block->start_sector = start_sector;
    block->n_sectors = n_sectors;

    return 1;
}

int sd_open_file(SDFile *file, uint8_t mode) {
    if (file == NULL || !sd_mounted) {
        return 0;
    }

    if (f_open(&file->fil, file->file_path, mode) != FR_OK) {
        return 0;
    }

    file->is_open = 1;
    file->is_deleted = 0;
    file->current_mode = mode;

    return 1;
}

int sd_write_file(SDFile *file, uint16_t start_addr, uint8_t *data, size_t len) {
    if (file == NULL || file->is_deleted || (file->current_mode & FA_WRITE) != FA_WRITE || data == NULL || !sd_mounted) {
        return 0;
    }
    
    if (f_lseek(&file->fil, start_addr) != FR_OK) {
        return 0;
    }

    for (size_t offset = 0; offset < len; offset += SD_MAX_READ_WRITE_SIZE) {
        size_t remaining = len - offset;
        size_t write_size = (remaining > SD_MAX_READ_WRITE_SIZE) ? SD_MAX_READ_WRITE_SIZE : remaining;

        UINT bytes_written;
        if (f_write(&file->fil, data + offset, write_size, &bytes_written) != FR_OK) {
            return 0;
        }
    }

    return 1;
}

int sd_read_file(SDFile *file, uint16_t start_addr, uint8_t *data, size_t len) {
    if (file == NULL || file->is_deleted || (file->current_mode & FA_READ) != FA_READ || data == NULL || !sd_mounted) {
        return 0;
    }

    if (f_lseek(&file->fil, start_addr) != FR_OK) {
        return 0;
    }

    for (size_t offset = 0; offset < len; offset += SD_MAX_READ_WRITE_SIZE) {
        size_t remaining = len - offset;
        size_t read_size = (remaining > SD_MAX_READ_WRITE_SIZE) ? SD_MAX_READ_WRITE_SIZE : remaining;

        UINT bytes_read;
        if (f_read(&file->fil, data + offset, read_size, &bytes_read) != FR_OK) {
            return 0;
        }
    }

    return 1;
}

int sd_close_file(SDFile *file) {
    if (file == NULL || file->is_deleted || !file->is_open || !sd_mounted) {
        return 0;
    }

    if (f_close(&file->fil) != FR_OK) {
        return 0;
    }

    file->is_open = 0;

    return 1;
}

int sd_delete_file(SDFile *file) {
    if (file == NULL || file->is_deleted || !sd_mounted) {
        return 0;
    }

    if (f_unlink(file->file_path) != FR_OK) {
        return 0;
    }

    file->is_deleted = 1;

    return 1;
}

int flash_erase_block(FlashBlock *block) {
    if (!flash_init) {
        return 0;
    }

    #ifdef USE_HW_FLASH_CHIP
        for (size_t i = 0; i < block->n_sectors; i++) {
            if (w25q_erase_sector(&flash_chip, block->start_sector + i) != W25Q_ERR_OK) {
                return 0;
            }
        }
    #else
        memset(fake_flash_chip + block->start_sector * EXT_FLASH_SECTOR_SIZE, 0, block->n_sectors * EXT_FLASH_SECTOR_SIZE);
    #endif

    return 1;
}

int flash_write_block(FlashBlock *block, uint16_t start_addr, uint8_t *data, size_t len) {
    if (block == NULL || !flash_init || start_addr % EXT_FLASH_PAGE_SIZE != 0) {
        return 0;
    }

    size_t addr = block->start_sector * EXT_FLASH_SECTOR_SIZE + start_addr;

    for (size_t offset = 0; offset < len; offset += EXT_FLASH_PAGE_SIZE) {
        size_t remaining = len - offset;
        size_t write_size = (remaining > EXT_FLASH_PAGE_SIZE) ? EXT_FLASH_PAGE_SIZE : remaining;

#ifdef USE_HW_FLASH_CHIP
        if (w25q_write_raw(&flash_chip, data + offset, write_size, addr + offset) != W25Q_ERR_OK) {
            return 0;
        }
#else
        memcpy(fake_flash_chip + addr + offset, data + offset, write_size);
#endif
    }

    return 1;
}

int flash_read_block(FlashBlock *block, uint16_t start_addr, uint8_t *data, size_t len) {
    if (block == NULL || !flash_init || start_addr % EXT_FLASH_PAGE_SIZE != 0) {
        return 0;
    }

    size_t addr = block->start_sector * EXT_FLASH_SECTOR_SIZE + start_addr;

    for (size_t offset = 0; offset < len; offset += EXT_FLASH_PAGE_SIZE) {
        size_t remaining = len - offset;
        size_t read_size = (remaining > EXT_FLASH_PAGE_SIZE) ? EXT_FLASH_PAGE_SIZE : remaining;

#ifdef USE_HW_FLASH_CHIP
        if (w25q_read_raw(&flash_chip, data + offset, read_size, addr + offset) != W25Q_ERR_OK) {
            return 0;
        }
#else
        memcpy(data + offset, fake_flash_chip + addr + offset, read_size);
#endif
    }

    return 1;
}
