#include "fs_wrapper.h"
#include "ff.h"

#include "log.h"
#include "util.h"
#include <string.h>

uint8_t fs_mounted = 0;
FATFS fs;

/** 
 * @brief Mounts a file system with FatFs. This method must be called prior to accessing/modifying any files.
 * @return RET_SUCCESS if the file system was mounted, and RET_FAILURE otherwise
 */
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

/**
 * @brief initializes a file struct with the default starting values and the desired file path
 * @param file the file object to initialize
 * @param file_path the path that this file object should use.
 * @return RET_FAILURE if the file path was longer than the max allowed file path length, and RET_SUCCESS otherwise.
 * 
 * Note that this method merely initializes the struct's values. It does not interact with the mounted
 * file system in any way.
 */
uint8_t fs_init_file(FSFile *file, const char *file_path) {
    if (strlen(file_path) >= FS_MAX_PATH_LEN) {
        return RET_FAILURE;
    }

    strcpy(file->file_path, file_path);

    file->current_mode = 0;
    file->is_open = 0;
    file->is_deleted = 0;

    return RET_SUCCESS;
}

/** 
 * @brief opens the specified file in the mounted file system.
 * @param file the file to open
 * @param the Bitmask representing mode flags that this file should use. See note for all flags.
 * @return RET_SUCCESS if the file was opened, and RET_FAILUER if the operation failed.
 * 
 * @note Mode flags are defined in ff.h. The following flags are available:
 *
 *  - FA_READ: allow read access to file
 *  - FA_WRITE: allow write access to file
 *  - FA_OPEN_EXISTING: only open the file if the file does exists, and fail if it doesn't exist.
 *  - FA_CREATE_ALWAYS: always create a new file, and erase the old one if it exists.
 *  - FA_CREATE_NEW: only create a new file, and fail if the file already exists. 
 *  - FA_OPEN_ALWAYS: create the file if it doesnt exist and open it if it does. The write pointer will be at the start of the file.
 *  - FA_OPEN_APPEND: performs the same action as FA_OPEN_ALWAYS, except it sets the write pointer to the end of the file.
*/
uint8_t fs_open_file(FSFile *file, uint8_t mode) {
    if (file == NULL || !fs_mounted) {
        return RET_FAILURE;
    }

    if (f_open(&file->fil, file->file_path, mode) != FR_OK) {
        return RET_FAILURE;
    }

    file->is_open = 1;
    file->is_deleted = 0;
    file->current_mode = mode;

    return RET_SUCCESS;
}

/** 
 * @brief Closes the specified file in the file system
 * @param file the file to close
 * @return RET_SUCCESS if the file was successfully closed, and RET_FAILURE if the operation failed.
 * 
 * @note If the file has been modified, closing it will cause any cached changes to be written to the volume.
 * Depending on the magnitude of changes, this method may incur a minor runtime delay.
 */
uint8_t fs_close_file(FSFile *file) {
    if (file == NULL || file->is_deleted || !file->is_open || !fs_mounted) {
        return RET_FAILURE;
    }

    if (f_close(&file->fil) != FR_OK) {
        return RET_FAILURE;
    }

    file->is_open = 0;

    return RET_SUCCESS;
}

/**
 * @brief Writes the provided data to the opened file.
 * @param file. The file to write to. Ensure that the file has been opened with write permissions enabled prior to writing.
 * @param start_addr The location from which to begin writing. See the note for more details.
 * @param data The data to write to the file
 * @param len The length of the data to write.
 * @return RET_SUCCESS if the data was written to the file, and RET_FAILURE otherwise.
 * 
 * @note the start_addr is the number of bytes from the start of the file where the write pointer should be placed.
 * If the offset is larger than the number of bytes in the file, than the file will be expanded until the size matches the offset.
 * The padding data will be added to the end of the file and will be uninitialized. Note that this behavior will only occur 
 * in write mode. If the file is readonly, start_addr will be truncated to the length of the file.
 * 
 * @warning due to the implementation of this method, a failed write may still modify the provided file.
 * These modifications are limited to moving the write/read pointer and writing a partial portion of the provided data.
 * No changes will occur to portions of the file that would not have been otherwise modified in a successful operation.
 */
uint8_t fs_write_file(FSFile *file, uint16_t start_addr, const uint8_t *data, size_t len) {
    if (file == NULL || file->is_deleted || (file->current_mode & FA_WRITE) != FA_WRITE || data == NULL || !fs_mounted) {
        return RET_FAILURE;
    }
    
    if (f_lseek(&file->fil, start_addr) != FR_OK) {
        return RET_FAILURE;
    }

    for (size_t offset = 0; offset < len; offset += FS_MAX_READ_WRITE_SIZE) {
        size_t remaining = len - offset;
        size_t write_size = (remaining > FS_MAX_READ_WRITE_SIZE) ? FS_MAX_READ_WRITE_SIZE : remaining;

        UINT bytes_written;
        FRESULT status = f_write(&file->fil, data + offset, write_size, &bytes_written);

        if (status != FR_OK) {
            log_printf(LOG_ERROR, "Failed to write to file: %d\n", status);
            return RET_FAILURE;
        }
    }

    return RET_SUCCESS;
}

/** 
 * @brief Reads the specified number of bytes from the provided file.
 * @param file. The file to read from. Ensure that the file has been opened with read permissions enabled prior to reading.
 * @param start_addr The location from which to begin reading. See the note for more details.
 * @param data The data read from the file will be stored here.
 * @param len The length of the data to read in bytes.
 * @return RET_SUCCESS if the data was read from the file, and RET_FAILURE otherwise.
 * 
 * @note the start_addr is the number of bytes from the start of the file where the read pointer should be placed.
 * Offsets larger than the length of the file will usually be truncated to the length of the file.
 * The exception is if the file has been opened with write permissions as well. In this scenario, the end
 * of the file will be padded with uninitialized data to match the size of the offset.
 * 
 * @warning This method may move the read/write pointer in the file, even in the case of a failed operation.
 * If the operation does fail, the contents of data are undefined, and may be overwritten.
 */
uint8_t fs_read_file(FSFile *file, uint16_t start_addr, uint8_t *data, size_t len) {
    if (file == NULL || file->is_deleted || (file->current_mode & FA_READ) != FA_READ || data == NULL || !fs_mounted) {
        return RET_FAILURE;
    }

    if (f_lseek(&file->fil, start_addr) != FR_OK) {
        return RET_FAILURE;
    }

    for (size_t offset = 0; offset < len; offset += FS_MAX_READ_WRITE_SIZE) {
        size_t remaining = len - offset;
        size_t read_size = (remaining > FS_MAX_READ_WRITE_SIZE) ? FS_MAX_READ_WRITE_SIZE : remaining;

        UINT bytes_read;
        if (f_read(&file->fil, data + offset, read_size, &bytes_read) != FR_OK) {
            return RET_FAILURE;
        }
    }

    return RET_SUCCESS;
}

/** 
 * @brief Deletes the specified file from the mounted file system.
 * @param file the file to delete
 * @return RET_SUCCESS if the file was successfully deleted, and RET_FAILURE otherwise.
 */
uint8_t fs_delete_file(FSFile *file) {
    if (file == NULL || file->is_deleted || !fs_mounted) {
        return RET_FAILURE;
    }

    if (f_unlink(file->file_path) != FR_OK) {
        return RET_FAILURE;
    }

    file->is_deleted = 1;

    return RET_SUCCESS;
}