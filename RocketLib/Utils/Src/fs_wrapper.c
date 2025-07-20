#include "fs_wrapper.h"
#include "ff.h"

#include "target.h"
#include "log.h"
#include "util.h"
#include <string.h>

uint8_t fs_mounted = 0;
FATFS fs;

uint8_t fs_mount(void) {
    log_printf(LOG_INFO, "Initializing IO devices...");
    uint8_t fs_status = f_mount(&fs, "/", 1);
    if (fs_status == FR_OK) {
        log_printf(LOG_INFO, "Mounted file system");
        fs_mounted = RET_SUCCESS;
    } else {
        log_printf(LOG_ERROR, "Failed to mount file system: %d", fs_status);
        fs_mounted = RET_FAILURE;
    }

    return fs_mounted;
}


uint8_t fs_init_file(FSFile *file, const char *file_path) {
    if (strlen(file_path) >= FS_MAX_PATH_LEN) {
        return 0;
    }

    strcpy(file->file_path, file_path);

    file->current_mode = 0;
    file->is_open = 0;
    file->is_deleted = 0;

    return 1;
}

uint8_t fs_open_file(FSFile *file, uint8_t mode) {
    if (file == NULL || !fs_mounted) {
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

uint8_t fs_close_file(FSFile *file) {
    if (file == NULL || file->is_deleted || !file->is_open || !fs_mounted) {
        return 0;
    }

    if (f_close(&file->fil) != FR_OK) {
        return 0;
    }

    file->is_open = 0;

    return 1;
}

uint8_t fs_write_file(FSFile *file, uint16_t start_addr, uint8_t *data, size_t len) {
    if (file == NULL || file->is_deleted || (file->current_mode & FA_WRITE) != FA_WRITE || data == NULL || !fs_mounted) {
        return 0;
    }
    
    if (f_lseek(&file->fil, start_addr) != FR_OK) {
        return 0;
    }

    for (size_t offset = 0; offset < len; offset += FS_MAX_READ_WRITE_SIZE) {
        size_t remaining = len - offset;
        size_t write_size = (remaining > FS_MAX_READ_WRITE_SIZE) ? FS_MAX_READ_WRITE_SIZE : remaining;

        UINT bytes_written;
        if (f_write(&file->fil, data + offset, write_size, &bytes_written) != FR_OK) {
            return 0;
        }
    }

    return 1;
}

uint8_t fs_read_file(FSFile *file, uint16_t start_addr, uint8_t *data, size_t len) {
    if (file == NULL || file->is_deleted || (file->current_mode & FA_READ) != FA_READ || data == NULL || !fs_mounted) {
        return 0;
    }

    if (f_lseek(&file->fil, start_addr) != FR_OK) {
        return 0;
    }

    for (size_t offset = 0; offset < len; offset += FS_MAX_READ_WRITE_SIZE) {
        size_t remaining = len - offset;
        size_t read_size = (remaining > FS_MAX_READ_WRITE_SIZE) ? FS_MAX_READ_WRITE_SIZE : remaining;

        UINT bytes_read;
        if (f_read(&file->fil, data + offset, read_size, &bytes_read) != FR_OK) {
            return 0;
        }
    }

    return 1;
}

uint8_t fs_delete_file(FSFile *file) {
    if (file == NULL || file->is_deleted || !fs_mounted) {
        return 0;
    }

    if (f_unlink(file->file_path) != FR_OK) {
        return 0;
    }

    file->is_deleted = 1;

    return 1;
}