#include "periph_io.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "main.h"
#include "ff.h"

#include "globals.h"
#include "w25q.h"

#define W25Q_WRITE_START 0
#define N_SECTORS 256

void io_save_complete(IOChannel *channel, int status, size_t bytes_saved);
void io_load_complete(IOChannel *channel, int status, size_t bytes_loaded);
void io_reset_complete(IOChannel *channel, int status);

int sd_load_operation(IOOperation *operation, uint8_t *data_buffer, size_t *bytes_loaded, uint8_t sd_mounted);
int sd_save_operation(IOOperation *operation, uint8_t *data_buffer, size_t *bytes_saved, uint8_t sd_mounted);
int sd_reset_operation(IOOperation *operation, uint8_t sd_mounted);
int flash_load_operation(struct w25q_device w25q, IOOperation *operation, uint8_t *data_buffer, size_t *bytes_loaded, uint8_t w25q_initialized);
int flash_save_operation(struct w25q_device w25q, IOOperation *operation, uint8_t *data_buffer, size_t *bytes_saved, size_t *w25q_write_ptr, uint8_t w25q_initialized);
int flash_reset_operation(struct w25q_device w25q, IOOperation *operation, size_t *w25q_write_ptr, uint8_t w25q_initialized);

#ifndef MCU_H725ZGT6
uint8_t _fake_flash_chip[15000];
uint8_t *fake_flash_chip = _fake_flash_chip;
#endif

/**
 * Initializes an IO channel
 * @param channel The channel to initialize
 * @param file_path The path of the file to read/write to
 * @param mode The mode of the channel (IO_MODE_READ or IO_MODE_WRITE)
 * @param id The ID of the channel. Must be unique.
 * @param sb_buff The static stream buffer to use internally
 * @param sb_storage_area The storage area for the internal stream buffer (should be at least sb_size + 1 bytes)
 * @param sb_size The size of the internal stream buffer.
 * @return 1 if successful, 0 otherwise
 */
int sd_channel_init(IOChannel *channel, const char *file_path, uint8_t mode, uint8_t id, StaticStreamBuffer_t *sb_buff, uint8_t *sb_storage_area, size_t sb_size) {
    if (channel == NULL || sb_storage_area == NULL || file_path == NULL) {
        return 0;
    }

    channel->id = id;
    channel->type = IO_CHANNEL_TYPE_SD;
    channel->file_path = file_path;
    channel->mode = mode;

    StreamBufferHandle_t sb_handle = xStreamBufferCreateStatic(sb_size + 1, 1, sb_storage_area, sb_buff);

    if (sb_handle == NULL) {
        return 0;
    }

    channel->sb_handle = sb_handle;

    return 1;
}

int flash_channel_init(IOChannel *channel, uint8_t mode, uint8_t id, StaticStreamBuffer_t *sb_buff, uint8_t *sb_storage_area, size_t sb_size) {
    if (channel == NULL || sb_storage_area == NULL) {
        return 0;
    }

    channel->id = id;
    channel->type = IO_CHANNEL_TYPE_FLASH;
    channel->file_path = NULL;
    channel->mode = mode;

    StreamBufferHandle_t sb_handle = xStreamBufferCreateStatic(sb_size + 1, 1, sb_storage_area, sb_buff);

    if (sb_handle == NULL) {
        return 0;
    }

    channel->sb_handle = sb_handle;

    return 1;
}

/**
 * Get the number of bytes available to write to or read into in a channel
 * @param channel The channel to check
 * @return The number of bytes available
 */
size_t io_channel_get_free(IOChannel *channel) {
    if (channel == NULL) {
        return 0;
    }

    return xStreamBufferSpacesAvailable(channel->sb_handle);
}

/**
 * Get the number of bytes that have been written to or read into in a channel
 * @param channel The channel to check
 * @return The number of bytes written or read
 */
size_t io_channel_get_full(IOChannel *channel) {
    if (channel == NULL) {
        return 0;
    }

    return xStreamBufferBytesAvailable(channel->sb_handle);
}

/**
 * Write data to the internal buffer of a channel. 
 * This does NOT save the data to the SD card. 
 * The channel must be in write mode.
 * @param channel The channel to write to
 * @param data The data to write
 * @param len The length of the data. Must not exceed IO_MAX_READ_WRITE_SIZE
 * @return 1 if successful, 0 otherwise
 */
int io_write_channel(IOChannel *channel, uint8_t *data, size_t len) {
    if (channel == NULL || data == NULL || channel->mode != IO_MODE_WRITE) {
        return 0;
    }

    if ((channel->type == IO_CHANNEL_TYPE_SD && len > SD_MAX_READ_WRITE_SIZE) ||
        (channel->type == IO_CHANNEL_TYPE_FLASH && len > FLASH_MAX_READ_WRITE_SIZE)) {
        return 0;
    }

    if (xStreamBufferSpacesAvailable(channel->sb_handle) < len) {
        return 0;
    }

    xStreamBufferSend(channel->sb_handle, data, len, 0);

    return 1;
}

/**
 * Save the data in a channel to the SD card. The channel must be in write mode.
 * @param channel The channel to save
 * @return 1 if successful, 0 otherwise
 */
int io_save_channel(IOChannel *channel) {
    if (channel == NULL || channel->mode != IO_MODE_WRITE) {
        return 0;
    }

    IOOperation operation = {
        .type = IO_OPERATION_SAVE,
        .channel = channel,
        .offset = 0, /* Unused */
        .n_bytes = 0, /* Unused */
    };

    size_t bytes_written = xMessageBufferSend(g_periph_io_mb_handle, &operation, sizeof(IOOperation), 0);

    if (bytes_written != sizeof(IOOperation)) {
        return 0;
    }

    return 1;
}

/**
 * Load data from the SD card into a channel. The channel must be in read mode.
 * @param channel The channel to load from
 * @param offset The byte offset to start reading from
 * @param bytes_to_read The number of bytes to read
 * @return 1 if successful, 0 otherwise
 */
int io_load_channel(IOChannel *channel, size_t offset, size_t bytes_to_read) {
    /* Validate channel */
    if (channel == NULL || channel->mode != IO_MODE_READ) {
        return 0;
    }

    IOOperation operation = {
        .type = IO_OPERATION_LOAD,
        .channel = channel,
        .offset = offset,
        .n_bytes = bytes_to_read,
    };

    size_t bytes_written = xMessageBufferSend(g_periph_io_mb_handle, &operation, sizeof(IOOperation), 0);

    if (bytes_written != sizeof(IOOperation)) {
        return 0;
    }

    return 1;
}

int io_reset_channel(IOChannel *channel) {
    if (channel == NULL) {
        return 0;
    }

    IOOperation operation = {
        .type = IO_OPERATION_RESET,
        .channel = channel,
        .offset = 0,
        .n_bytes = 0,
    };

    size_t bytes_written = xMessageBufferSend(g_periph_io_mb_handle, &operation, sizeof(IOOperation), 0);

    if (bytes_written != sizeof(IOOperation)) {
        return 0;
    }

    return 1;
}

/**
 * Read data from a channel. The channel must be in read mode.
 * @param channel The channel to read from
 * @param data The buffer to read into
 * @param len The target number of bytes to read
 * @return The number of bytes read
 */
int io_read_channel(IOChannel *channel, uint8_t *data, size_t len) {
    if (channel == NULL || data == NULL || channel->mode != IO_MODE_READ) {
        return 0;
    }

    return xStreamBufferReceive(channel->sb_handle, data, len, 0);
}

/**
 * Task to handle SD card operations
 * @param args Unused
 */
void periph_io_task(void *args) {
    uint8_t sd_mounted = 0;
    FATFS fs;

    uint8_t w25q_initialized = 0;
    size_t w25q_write_ptr = W25Q_WRITE_START;
    struct w25q_device w25q;

    /* Buffer to mediate between fatfs and freertos stream buffer */
    uint8_t data_buffer[SD_MAX_READ_WRITE_SIZE + 1]; // +1 because theres some weird buffer overflow in f_read

    /* Buffer to store operation messages */
    uint8_t operation_buffer[sizeof(IOOperation)];

    /* Initial attempt to mount SD card */
    if (f_mount(&fs, "/", 1) == FR_OK) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD card mounted\r\n", 17, HAL_MAX_DELAY);
        sd_mounted = 1;
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD card not mounted\r\n", 21, HAL_MAX_DELAY);
    }

#ifdef MCU_H725ZGT6
    if (w25q_init(&w25q) == W25Q_ERR_OK && w25q_erase_sector(&w25q, 0) == W25Q_ERR_OK) {
        w25q_initialized = 1;
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "W25Q initialized\r\n", 18, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Erasing W25Q sectors\r\n", 22, HAL_MAX_DELAY);

    for (size_t i = 0; i < N_SECTORS; i++) {
        char erase_progress[100];
        sprintf(erase_progress, "Erasing W25Q sector %d/%d\r\n", i+1, N_SECTORS);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) erase_progress, strlen(erase_progress), HAL_MAX_DELAY);
        if (w25q_erase_sector(&w25q, i) != W25Q_ERR_OK) {
            w25q_initialized = 0;
        }
    }

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Erased W25Q sectors\r\n", 21, HAL_MAX_DELAY);
#else
    w25q_initialized = 1;
#endif

    for (;;) {
        /* Wait for an operation to be sent to us */
        xMessageBufferReceive(g_periph_io_mb_handle, operation_buffer, sizeof(IOOperation), portMAX_DELAY);

        /* Re-attempt to mount SD card if not already */
        if (!sd_mounted && f_mount(&fs, "/", 1) == FR_OK) {
            sd_mounted = 1;
        }

#ifdef MCU_H725ZGT6
        if (!w25q_initialized && w25q_init(&w25q) == W25Q_ERR_OK) {
            w25q_initialized = 1;
        }
#endif
        
        IOOperation *operation = (IOOperation *) operation_buffer;

        /* Perform a save/load operation */
        if (operation->type == IO_OPERATION_SAVE) {
            size_t bytes_saved = 0;
            int status = 0;

            if (operation->channel->type == IO_CHANNEL_TYPE_SD) {
                status = sd_save_operation(operation, data_buffer, &bytes_saved, sd_mounted);
            } else if (operation->channel->type == IO_CHANNEL_TYPE_FLASH) {
                status = flash_save_operation(w25q, operation, data_buffer, &bytes_saved, &w25q_write_ptr, w25q_initialized);
            }

            io_save_complete(operation->channel, status, bytes_saved);
        } else if (operation->type == IO_OPERATION_LOAD) {
            size_t bytes_loaded = 0;
            int status = 0;

            if (operation->channel->type == IO_CHANNEL_TYPE_SD) {
                status = sd_load_operation(operation, data_buffer, &bytes_loaded, sd_mounted);
            } else if (operation->channel->type == IO_CHANNEL_TYPE_FLASH) {
                status = flash_load_operation(w25q, operation, data_buffer, &bytes_loaded, w25q_initialized);
            }

            io_load_complete(operation->channel, status, bytes_loaded);
        } else if (operation->type == IO_OPERATION_RESET) {
            int status = 0;

            if (operation->channel->type == IO_CHANNEL_TYPE_SD) {
                status = sd_reset_operation(operation, sd_mounted);
            } else if (operation->channel->type == IO_CHANNEL_TYPE_FLASH) {
                status = flash_reset_operation(w25q, operation, &w25q_write_ptr, w25q_initialized);
            }

            io_reset_complete(operation->channel, status);
        }
    }
}

int flash_load_operation(struct w25q_device w25q, IOOperation *operation, uint8_t *data_buffer, size_t *bytes_loaded, uint8_t w25q_initialized) {
    *bytes_loaded = 0;

    if (!w25q_initialized) {
        return 0;
    }

    IOChannel *channel = operation->channel;

#ifdef MCU_H725ZGT6
    if (w25q_read_raw(&w25q, data_buffer, operation->n_bytes, operation->offset + W25Q_WRITE_START) != W25Q_ERR_OK) {
        return 0;
    }
#else
    memcpy(data_buffer, fake_flash_chip + operation->offset, operation->n_bytes);
#endif

    size_t available = xStreamBufferSpacesAvailable(channel->sb_handle);

    if (available < operation->n_bytes) {
        return 0;
    }

    *bytes_loaded = operation->n_bytes;

    xStreamBufferSend(channel->sb_handle, data_buffer, operation->n_bytes, 0);

    return 1;
}

int flash_save_operation(struct w25q_device w25q, IOOperation *operation, uint8_t *data_buffer, size_t *bytes_saved, size_t *w25q_write_ptr, uint8_t w25q_initialized) {
    *bytes_saved = 0;

    if (!w25q_initialized) {
        return 0;
    }

    IOChannel *channel = operation->channel;

    size_t available = xStreamBufferBytesAvailable(channel->sb_handle);

    xStreamBufferReceive(channel->sb_handle, data_buffer, available, 0);

#ifdef MCU_H725ZGT6
    if (w25q_write_raw(&w25q, data_buffer, available, *w25q_write_ptr) != W25Q_ERR_OK) {
        return 0;
    }
#else
    memcpy(fake_flash_chip + *w25q_write_ptr, data_buffer, available);
#endif

    *w25q_write_ptr += available;
    *bytes_saved = available;

    return 1;
}

int flash_reset_operation(struct w25q_device w25q, IOOperation *operation, size_t *w25q_write_ptr, uint8_t w25q_initialized) {
    if (!w25q_initialized) {
        return 0;
    }

    *w25q_write_ptr = W25Q_WRITE_START;

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Erasing W25Q sectors\r\n", 22, HAL_MAX_DELAY);

    for (size_t i = 0; i < N_SECTORS; i++) {
        char erase_progress[100];
        sprintf(erase_progress, "Erasing W25Q sector %d/%d\r\n", i+1, N_SECTORS);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) erase_progress, strlen(erase_progress), HAL_MAX_DELAY);
        if (w25q_erase_sector(&w25q, i) != W25Q_ERR_OK) {
            return 0;
        }
    }

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Erased W25Q sectors\r\n", 21, HAL_MAX_DELAY);

    return 1;
}

/**
 * Load data from the SD card into a channel.
 * @param operation The operation to perform
 * @param data_buffer The buffer to use as temporary storage
 * @param sd_mounted Whether the SD card is mounted
 * @return 1 if successful, 0 otherwise
 */
int sd_load_operation(IOOperation *operation, uint8_t *data_buffer, size_t *bytes_loaded, uint8_t sd_mounted) {
    *bytes_loaded = 0;

    if (!sd_mounted) {
        return 0;
    }

    IOChannel *channel = operation->channel;
    FIL fil;

    if (f_open(&fil, channel->file_path, FA_READ) != FR_OK) {
        return 0;
    }

    /* Set file pointer to an offset */
    if (f_lseek(&fil, operation->offset) != FR_OK) {
        return 0;
    }
    
    UINT bytes_read; 
    if (f_read(&fil, data_buffer, operation->n_bytes, &bytes_read) != FR_OK) {
        return 0;
    }

    size_t available = xStreamBufferSpacesAvailable(channel->sb_handle);

    if (available < bytes_read) {
        return 0;
    }

    *bytes_loaded = bytes_read;

    xStreamBufferSend(channel->sb_handle, data_buffer, bytes_read, 0);

    if (f_close(&fil) != FR_OK) {
        return 0;
    }

    return 1;
}

/**
 * Save data to the SD card from a channel.
 * @param operation The operation to perform
 * @param data_buffer The buffer to use as temporary storage
 * @param sd_mounted Whether the SD card is mounted
 * @return 1 if successful, 0 otherwise
 */
int sd_save_operation(IOOperation *operation, uint8_t *data_buffer, size_t *bytes_saved, uint8_t sd_mounted) {
    *bytes_saved = 0;

    if (!sd_mounted) {
        return 0;
    }

    IOChannel *channel = operation->channel;
    FIL fil;
    
    if (f_open(&fil, channel->file_path, FA_WRITE | FA_OPEN_APPEND) != FR_OK) {
        return 0;
    } 

    size_t available = xStreamBufferBytesAvailable(channel->sb_handle);
    *bytes_saved = available;

    xStreamBufferReceive(channel->sb_handle, data_buffer, available, 0);
    
    UINT written;
    if (f_write(&fil, data_buffer, available, &written) != FR_OK) {
        return 0;
    }
    
    if (f_close(&fil) != FR_OK) {
        return 0;
    }

    return 1;
}

int sd_reset_operation(IOOperation *operation, uint8_t sd_mounted) {
    if (!sd_mounted) {
        return 0;
    }

    IOChannel *channel = operation->channel;

    if (f_unlink(channel->file_path) != FR_OK) {
        return 0;
    }

    return 1;
}

#include "sd_test.h"

/**
 * Callback for when a save operation is complete
 * @param channel The channel that was saved
 * @param bytes_saved The number of bytes saved
 * @param status The status of the save operation. 1 if successful, 0 otherwise
 */
void io_save_complete(IOChannel *channel, int status, size_t bytes_saved) {
#ifdef USE_TESTS
    if (channel->id == 0) {
        xTaskNotify(g_test_task_handle, SD_TASK_WRITE_COMPLETE_BIT, eSetBits);
    }
#endif
    if (status == 0) {
        return;
    }

    switch (channel->id) {
        case FLASH_WRITE_CHANNEL_ID:
            xTaskNotify(g_state_flash_task_handle, FLASH_WRITE_COMPLETE_NOTIFICATION_BIT, eSetBits);
            break;
        case SD_WRITE_CHANNEL_ID:
            xTaskNotify(g_state_flash_task_handle, SD_WRITE_COMPLETE_NOTIFICATION_BIT, eSetBits);
            break;
    }
}

/**
 * Callback for when a load operation is complete
 * @param channel The channel that was loaded
 * @param bytes_loaded The number of bytes loaded
 * @param status The status of the load operation. 1 if successful, 0 otherwise
 */
void io_load_complete(IOChannel *channel, int status, size_t bytes_loaded) {
#ifdef USE_TESTS
    if (channel->id == 1) {
        xTaskNotify(g_test_task_handle, SD_TASK_READ_COMPLETE_BIT, eSetBits);
    }
#endif
    if (status == 0) {
        return;
    }

    switch (channel->id) {
        case FLASH_READ_CHANNEL_ID:
            xTaskNotify(g_state_flash_task_handle, FLASH_READ_COMPLETE_NOTIFICATION_BIT, eSetBits);
            break;
        case SD_READ_CHANNEL_ID:
            xTaskNotify(g_state_flash_task_handle, SD_READ_COMPLETE_NOTIFICATION_BIT, eSetBits);
            break;
    }
}

void io_reset_complete(IOChannel *channel, int status) {
    if (status == 0) {
        return;
    }

    switch (channel->id) {
        case FLASH_WRITE_CHANNEL_ID:
            xTaskNotify(g_state_flash_task_handle, FLASH_RESET_COMPLETE_NOTIFICATION_BIT, eSetBits);
            break;
        case SD_WRITE_CHANNEL_ID:
            xTaskNotify(g_state_flash_task_handle, SD_RESET_COMPLETE_NOTIFICATION_BIT, eSetBits);
            break;
    }
}
