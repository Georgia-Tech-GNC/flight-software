#ifndef FS_WRAPPER_H
#define FS_WRAPPER_H

#define FS_MAX_READ_WRITE_SIZE 512
#define FS_MAX_PATH_LEN 64

#include <stdint.h>
#include <stddef.h>
#include "ff.h"

typedef struct {
    char file_path[FS_MAX_PATH_LEN];
    uint8_t is_open;
    uint8_t is_deleted;
    uint8_t current_mode;
    FIL fil;
} FSFile;

uint8_t fs_mount(void);
uint8_t fs_init_file(FSFile *file, const char *file_path);
uint8_t fs_open_file(FSFile *file, uint8_t mode);
uint8_t fs_write_file(FSFile *file, uint16_t start_addr, uint8_t *data, size_t len);
uint8_t fs_read_file(FSFile *file, uint16_t start_addr, uint8_t *data, size_t len);
uint8_t fs_close_file(FSFile *file);
uint8_t fs_delete_file(FSFile *file);
#endif