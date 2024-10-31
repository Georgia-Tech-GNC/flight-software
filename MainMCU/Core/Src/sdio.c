#include "sdio.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "main.h"
#include "ff.h"

#include "globals.h"

void sd_save_complete(SDChannel *channel, int status, size_t bytes_saved);
void sd_load_complete(SDChannel *channel, int status, size_t bytes_loaded);
int load_operation(FATFS *fs, SDOperation *operation, uint8_t *data_buffer, size_t *bytes_loaded, uint8_t sd_mounted);
int save_operation(SDOperation *operation, uint8_t *data_buffer, size_t *bytes_saved, uint8_t sd_mounted);

/**
 * Initializes an SD channel
 * @param channel The channel to initialize
 * @param file_path The path of the file to read/write to
 * @param mode The mode of the channel (SD_MODE_READ or SD_MODE_WRITE)
 * @param id The ID of the channel. Must be unique.
 * @param sb_buff The static stream buffer to use internally
 * @param sb_storage_area The storage area for the internal stream buffer (should be at least sb_size + 1 bytes)
 * @param sb_size The size of the internal stream buffer.
 * @return 1 if successful, 0 otherwise
 */
int sd_channel_init(SDChannel *channel, const char *file_path, uint8_t mode, uint8_t id, StaticStreamBuffer_t *sb_buff, uint8_t *sb_storage_area, size_t sb_size) {
    if (channel == NULL || sb_storage_area == NULL || file_path == NULL) {
        return 0;
    }

    channel->id = id;
    channel->file_path = file_path;
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
size_t sd_channel_get_free(SDChannel *channel) {
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
size_t sd_channel_get_full(SDChannel *channel) {
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
 * @param len The length of the data. Must not exceed SD_MAX_READ_WRITE_SIZE
 * @return 1 if successful, 0 otherwise
 */
int sd_write_channel(SDChannel *channel, uint8_t *data, size_t len) {
    if (channel == NULL || data == NULL || channel->mode != SD_MODE_WRITE || len > SD_MAX_READ_WRITE_SIZE) {
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
int sd_save_channel(SDChannel *channel) {
    /* Validate channel */
    if (channel == NULL || channel->mode != SD_MODE_WRITE) {
        return 0;
    }

    SDOperation operation = {
        .type = SD_OPERATION_SAVE,
        .channel = channel,
        .offset = 0, /* Unused */
        .n_bytes = 0, /* Unused */
    };

    size_t bytes_written = xMessageBufferSend(g_sdio_mb_handle, &operation, sizeof(SDOperation), 0);

    if (bytes_written != sizeof(SDOperation)) {
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
int sd_load_channel(SDChannel *channel, size_t offset, size_t bytes_to_read) {
    /* Validate channel */
    if (channel == NULL || channel->mode != SD_MODE_READ) {
        return 0;
    }

    SDOperation operation = {
        .type = SD_OPERATION_LOAD,
        .channel = channel,
        .offset = offset,
        .n_bytes = bytes_to_read,
    };

    size_t bytes_written = xMessageBufferSend(g_sdio_mb_handle, &operation, sizeof(SDOperation), 0);

    if (bytes_written != sizeof(SDOperation)) {
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
int sd_read_channel(SDChannel *channel, uint8_t *data, size_t len) {
    if (channel == NULL || data == NULL || channel->mode != SD_MODE_READ) {
        return 0;
    }

    return xStreamBufferReceive(channel->sb_handle, data, len, 0);
}

/**
 * Task to handle SD card operations
 * @param args Unused
 */
void sdio_task(void *args) {
    uint8_t sd_mounted = 0;
    FATFS fs;

    /* Buffer to mediate between fatfs and freertos stream buffer */
    uint8_t data_buffer[SD_MAX_READ_WRITE_SIZE + 1]; // +1 because theres some weird buffer overflow in f_read

    /* Buffer to store operation messages */
    uint8_t operation_buffer[sizeof(SDOperation)];

    /* Initial attempt to mount SD card */
    if (f_mount(&fs, "/", 1) == FR_OK) {
        sd_mounted = 1;
    }

    for (;;) {
        /* Wait for an operation to be sent to us */
        xMessageBufferReceive(g_sdio_mb_handle, operation_buffer, sizeof(SDOperation), portMAX_DELAY);

        /* Re-attempt to mount SD card if not already */
        if (!sd_mounted && f_mount(&fs, "/", 1) == FR_OK) {
            sd_mounted = 1;
        }
        
        SDOperation *operation = (SDOperation *) operation_buffer;

        /* Perform a save/load operation */
        if (operation->type == SD_OPERATION_SAVE) {
            size_t bytes_saved = 0;
            int status = save_operation(operation, data_buffer, &bytes_saved, sd_mounted);
            sd_save_complete(operation->channel, status, bytes_saved);
        } else if (operation->type == SD_OPERATION_LOAD) {
            size_t bytes_loaded = 0;
            int status = load_operation(&fs, operation, data_buffer, &bytes_loaded, sd_mounted);
            sd_load_complete(operation->channel, status, bytes_loaded);
        }
    }
}

/**
 * Load data from the SD card into a channel.
 * @param operation The operation to perform
 * @param data_buffer The buffer to use as temporary storage
 * @param sd_mounted Whether the SD card is mounted
 * @return 1 if successful, 0 otherwise
 */
int load_operation(FATFS *fs, SDOperation *operation, uint8_t *data_buffer, size_t *bytes_loaded, uint8_t sd_mounted) {
    if (!sd_mounted) {
        return 0;
    }

    SDChannel *channel = operation->channel;
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
int save_operation(SDOperation *operation, uint8_t *data_buffer, size_t *bytes_saved, uint8_t sd_mounted) {
    if (!sd_mounted) {
        return 0;
    }

    SDChannel *channel = operation->channel;
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
#include "sd_test.h"

/**
 * Callback for when a save operation is complete
 * @param channel The channel that was saved
 * @param bytes_saved The number of bytes saved
 * @param status The status of the save operation. 1 if successful, 0 otherwise
 */
void sd_save_complete(SDChannel *channel, int status, size_t bytes_saved) {
    if (channel->id == 0) {
        xTaskNotify(g_test_task_handle, SD_TASK_WRITE_COMPLETE_BIT, eSetBits);
    }
}

/**
 * Callback for when a load operation is complete
 * @param channel The channel that was loaded
 * @param bytes_loaded The number of bytes loaded
 * @param status The status of the load operation. 1 if successful, 0 otherwise
 */
void sd_load_complete(SDChannel *channel, int status, size_t bytes_loaded) {
    if (channel->id == 1) {
        xTaskNotify(g_test_task_handle, SD_TASK_READ_COMPLETE_BIT, eSetBits);
    }
}
