/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/
/* !!!!!!!TO BE DEPRECATED!!!!!!! Will be replaced by appropriate HALAL drivers*/

#ifndef PERIPH_IO_H
#define PERIPH_IO_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "string.h"
#include "ff.h"
#include "w25q.h"

#define SD_MAX_READ_WRITE_SIZE 512
#define EXT_FLASH_PAGE_SIZE W25Q_MEM_PAGE_SIZE
#define EXT_FLASH_SECTOR_SIZE (W25Q_MEM_SECTOR_SIZE * 1024)

#define SD_MAX_PATH_LEN 64

typedef struct {
    uint16_t start_sector;
    uint16_t n_sectors;
} FlashBlock;

typedef struct {
    char file_path[SD_MAX_PATH_LEN];
    uint8_t is_open;
    uint8_t is_deleted;
    uint8_t current_mode;
    FIL fil;
} SDFile;

int io_init(void);

int sd_init_file(SDFile *file, const char *file_path);
int flash_init_block(FlashBlock *block, uint16_t start_sector, uint16_t n_sectors);

int sd_open_file(SDFile *file, uint8_t mode);
int sd_write_file(SDFile *file, uint16_t start_addr, uint8_t *data, size_t len);
int sd_read_file(SDFile *file, uint16_t start_addr, uint8_t *data, size_t len);
int sd_close_file(SDFile *file);
int sd_delete_file(SDFile *file);

int flash_erase_block(FlashBlock *block);
int flash_write_block(FlashBlock *block, uint16_t start_addr, uint8_t *data, size_t len);
int flash_read_block(FlashBlock *block, uint16_t start_addr, uint8_t *data, size_t len);

#endif