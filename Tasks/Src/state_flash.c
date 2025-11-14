#include "state_flash.h"
#include "storage.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stdint.h"
#include "stddef.h"
#include "lib.h"
#include "log.h"
#include "util.h"
#include "fs_wrapper.h"
#include "halal.h"
#include <string.h>

/* Private defines */
#define CSV_LINE_SIZE 2048

#define STATE_FLASH_START_SECTOR 0
#define STATE_FLASH_N_SECTORS 8

#define FLASH_TEST_SIZE 4096
#define FS_TEST_SIZE 4096

/* Private function definitions */
uint8_t flash_test(void);
uint8_t fs_test(void);

void write_to_flash(RocketStateStruct *rocket_state, size_t addr);
void flash_fs(FSFile *file, size_t n_states);

/**
 * @brief Task to handle writing state to flash chip and file system
 * @param args Unused
 */
void state_flash_task(void *args) {
    UNUSED(args);

    vTaskDelay(100);

#ifdef HALAL_STORAGE_USB_MODULE_ENABLED
    log_printf(LOG_INFO, "Awaiting filesystem");

    await_notification(FS_READY_NOTIFICATION_BIT);
#endif

    if (fs_mount()) {
        log_printf(LOG_INFO, "File system mounted");
    } else {
        log_printf(LOG_INFO, "Error mounting file system");
    }

    /* Run tests */
    if (flash_test()) {
        log_printf(LOG_INFO, "Flash test PASS");
    } else {
        log_printf(LOG_ERROR, "Flash test FAIL");
    }

    if (fs_test()) {
        log_printf(LOG_INFO, "File system test PASS");
    } else {
        log_printf(LOG_ERROR, "File system test FAIL");
    }

    FSFile file;
    fs_init_file(&file, "/data.csv");

    size_t n_states = 0;

    while (1) {
        /* Wait for next notification */
        uint32_t notification_value = await_notification(FLASH_STATE_NOTIFICATION_BIT | FLASH_FS_NOTIFICATION_BIT);

        /* Flash state to file system */
        if (notification_value & FLASH_FS_NOTIFICATION_BIT) {
            log_printf(LOG_INFO, "Flashing file system");
            flash_fs(&file, n_states);

            /* Stop all other tasks */
            log_printf(LOG_INFO, "Suspending all tasks");
            vTaskSuspendAll();

            /* Tasks aren't allowed to exit, so stall here */
            vTaskDelay(portMAX_DELAY);
        }
        /* Write state to flash chip */
        if (notification_value & FLASH_STATE_NOTIFICATION_BIT) {
            RocketStateStruct current_state;

            if (memcpy_state(&current_state) == RET_SUCCESS) {
                HALAL_flash_write_page(n_states, (uint8_t *) &current_state, sizeof(RocketStateStruct));
                n_states ++;
            } else {
                log_printf(LOG_ERROR, "Failed to copy state bytes");
            }
        }
    }
}

/**
 * @brief Do a simple test of the flash chip.
 * Writes a test pattern to the flash chip, reads it back, and erases the block.
 * @return 1 if the test passes, 0 otherwise
 */
uint8_t flash_test(void) {
    uint8_t test_bytes[FLASH_TEST_SIZE];
    const uint8_t offset = xTaskGetTickCount() % 256;

    size_t n_pages = FLASH_TEST_SIZE / HALAL_FLASH_PAGE_SIZE;

    /* Create test pattern */
    for (size_t i = 0; i < FLASH_TEST_SIZE; i ++) {
        test_bytes[i] = (i + offset) % 256;
    }

    /* Write pattern to flash chip */
    if (!HALAL_flash_write_page(0, test_bytes, n_pages)) {
        log_printf(LOG_ERROR, "Failed to write to flash");
        return 0;
    }
    

    memset(test_bytes, 0, FLASH_TEST_SIZE);

    /* Read it back */
    if (!HALAL_flash_read_page(0, test_bytes, n_pages)) {
        log_printf(LOG_ERROR, "Failed to read from flash");
        return 0;
    }

    /* Check if the pattern matches */
    for (size_t i = 0; i < FLASH_TEST_SIZE; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            return 0;
        }
    }

    /* Erase what we just wrote */
    if (!HALAL_flash_erase_page(0, n_pages)) {
        log_printf(LOG_ERROR, "Failed to erase flash");
        return 0;
    }

    return 1;
}

/**
 * @brief Do a simple test of the file system.
 * Writes a test pattern to the file system, reads it back, and deletes the file.
 * @return 1 if the test passes, 0 otherwise
 */
uint8_t fs_test(void) {
    FSFile test_file;

    if (!fs_init_file(&test_file, "/test.txt")) {
        log_printf(LOG_ERROR, "Failed to initialize file");
        return 0;
    }

    if (!fs_open_file(&test_file, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)) {
        log_printf(LOG_ERROR, "Failed to open file");
        return 0;
    }

    uint8_t test_bytes[FS_TEST_SIZE + 1];
    const uint8_t offset = xTaskGetTickCount() % 256;

    /* Create test pattern */
    for (size_t i = 0; i < FS_TEST_SIZE; i ++) {
        test_bytes[i] = (i + offset) % 256;
    }

    /* Write pattern to file system */
    if (!fs_write_file(&test_file, 0, test_bytes, FS_TEST_SIZE)) {
        log_printf(LOG_ERROR, "Failed to write to file system");
        return 0;
    }

    memset(test_bytes, 0, FS_TEST_SIZE);

    /* Read it back */
    if (!fs_read_file(&test_file, 0, test_bytes, FS_TEST_SIZE)) {
        log_printf(LOG_ERROR, "Failed to read from file system");
        return 0;
    }

    /* Check if the pattern matches */
    for (size_t i = 0; i < FS_TEST_SIZE; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            log_printf(LOG_ERROR, "File system read error");
            return 0;
        }
    }

    if (!fs_close_file(&test_file)) {
        log_printf(LOG_ERROR, "Failed to close file");
        return 0;
    }

    /* Erase what we just wrote */
    if (!fs_delete_file(&test_file)) {
        log_printf(LOG_ERROR, "Failed to delete file");
        return 0;
    }

    return 1;
}

/**
 * @brief Copy the state from the flash chip to the file system
 * @param flash_block Flash block to read from
 * @param file File to write to
 * @param n_states Number of states to copy
 */
void flash_fs(FSFile *file, size_t n_states) {
    if (fs_open_file(file, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)) {
        log_printf(LOG_INFO, "Successfully opened file");
    }

    RocketStateStruct rocket_state;
    uint8_t data_buffer[HALAL_FLASH_PAGE_SIZE];

    char line_buf[CSV_LINE_SIZE];
    log_printf(LOG_INFO, "Writing to file system...");

    size_t fs_bytes_written = 0;

    for (size_t i = 0; i < n_states; i ++) {
        log_printf(LOG_INFO, "Writing to file system: %d/%d", i + 1, n_states);

        HALAL_flash_read_page(i, data_buffer, 1);

        memcpy(&rocket_state, data_buffer, sizeof(RocketStateStruct));

        size_t bytes_written;
        
        if (!csv_encode(&rocket_state, line_buf, CSV_LINE_SIZE, &bytes_written)) {
            log_printf(LOG_ERROR, "Error encoding rocket CSV line #%zu", bytes_written);
            continue;
        }

        if (fs_write_file(file, fs_bytes_written, (uint8_t *) line_buf, bytes_written)) {
            log_printf(LOG_INFO, "Wrote CSV line #%d to file system", i);
            log_printf(LOG_INFO, "CSV Data: %s", line_buf);
        } else {
            log_printf(LOG_ERROR, "Error writing rocket CSV line #%zu", bytes_written);
        }

        fs_bytes_written += bytes_written;
    }

    if (fs_close_file(file)) {
        log_printf(LOG_INFO, "Closed file");
    }

    log_printf(LOG_INFO, "Wrote to file system");
}