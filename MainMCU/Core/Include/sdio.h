#ifndef SDIO_H
#define SDIO_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"

#define MAX_SD_OPERATIONS_QUEUED 4
#define SD_MB_SIZE (sizeof(SDOperation) * MAX_SD_OPERATIONS_QUEUED)

#define SD_MAX_READ_WRITE_SIZE 512

#define SD_MODE_READ 0
#define SD_MODE_WRITE 1

#define SD_OPERATION_LOAD 0
#define SD_OPERATION_SAVE 1

typedef struct {
    const char *file_path;
    uint8_t mode;
    uint8_t id;
    StreamBufferHandle_t sb_handle;
} SDChannel;

typedef struct {
    uint8_t type;
    SDChannel *channel;
    size_t offset; /* Only used if type is SD_MODE_READ */
    size_t n_bytes; /* Only used if type is SD_MODE_READ */
} SDOperation;

int sd_channel_init(SDChannel *channel, const char *file_name, uint8_t mode, uint8_t id, StaticStreamBuffer_t *sb_buff, uint8_t *sb_storage_area, size_t sb_size);
size_t sd_channel_get_free(SDChannel *channel);
size_t sd_channel_get_full(SDChannel *channel);
int sd_write_channel(SDChannel *channel, uint8_t *data, size_t len);
int sd_save_channel(SDChannel *channel);
int sd_load_channel(SDChannel *channel, size_t offset, size_t bytes_to_read);
int sd_read_channel(SDChannel *channel, uint8_t *data, size_t len);
void sdio_task(void *args);

#endif