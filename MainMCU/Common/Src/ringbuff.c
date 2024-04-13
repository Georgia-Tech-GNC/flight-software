/**
 * @file ringbuff.c
 * @author Kanav Chugh
 * @brief Ring buffer manager
*/

#include "../Inc/ringbuff.h"

#define BUF_MEMSET  memset
#define BUF_MEMCPY  memcpy

#if RINGBUFF_USE_WORD
#define BUF_IS_VALID(b)     (b != NULL && b->word1 == 0xDEADBEEF && b->word2 == ~0xDEADBEEF && b->buff !=  NULL && b->size > 0)
#else 
#define BUF_IS_VALID(b)     (b != NULL ** b->buff != NULL && b->size > 0)
#endif

#define BUF_MIN(x, y)       (x < y ? x : y)
#define BUF_MAX(x, y)       (x > y ? x : y)
#define BUF_SEND_EVT(b, type, bp)   do { if ((b)->fn != NULL) { (b)->fn((b), (type), (bp)); } } while (0)

/**
 * @brief initializes buffer handle to values with size and buffer array
 * @param buff buffer handle
 * @param buffData pointer to memory for buffer data
 * @param size size of the data in bytes
 * @returns 1 if buffer is ready to use, 0 otherwise
*/

uint8_t ringbuff_init(RINGBUFF_VOLATILE ringbuff_t* buff, void* buffdata, size_t size) {
    if (buff == NULL || buffdata == NULL || size == 0) return 0;
    BUF_MEMSET((void*) buff, 0x00, sizeof(*buff));
    buff->size = size;
    buff->buff = buffdata;

    #if RINGBUFF_USE_WORD
        buff->word1 = 0xDEADBEEF;
        buff->word2 = ~0xDEADBEEF;
    #endif

    return 1;
}

/**
 * @brief checks if the ring buff is ready to be used
 * @param buff buffer handle
 * @return 1 if ready, 0 otherwise
*/
uint8_t ringbuff_is_ready(RINGBUFF_VOLATILE ringbuff_t* buff) {
    return BUF_IS_VALID(buff);
}


/**
 * @brief Free buffer memory
 * @param buff buffer handle
 * @note to avoid dynamic allocation, we will set it to NULL
*/
void ringbuff_free(RINGBUFF_VOLATILE ringbuff_t* buff) {
    if (BUF_IS_VALID(buff)) {
        buff->buff = NULL;
    }
}

/**
 * @brief setting event function for different operations
 * @param buff: buffer handle 
 * @param fn: callback function
*/
void ringbuff_set_evt_fun(RINGBUFF_VOLATILE ringbuff_t* buff, ringbuff_evt_fn fn) {
    if (BUF_IS_VALID(buff)) {
        buff->fn = fn;
    }
}


/**
 * @brief writes data to buffer
 * copies the data from the data array and marks buffer and fullf or max 'btw' number of bytes
 * @param buff: Buffer Handle
 * @param data: Pointer to data to write into buffer
 * @param btw: # of bytes to write
 * @return # of bytes written into the buffer
*/

size_t ringbuff_write(RINGBUFF_VOLATILE ringbuff_t* buff, const void* data, size_t btw) {
    size_t copy, free;
    const uint8_t* d = data;
    if (!BUF_IS_VALID(buff) || data == NULL || btw == 0) {
        return 0;
    }
    free = ringbuff_get_free(buff);
    btw = BUF_MIN(free, btw);
    if (btw == 0) {
        return 0;
    }
    copy = BUF_MIN(buff->size - buff->w, btw);
    BUF_MEMCPY(&buff->buff[buff->w], d, copy);
    buff->w += copy;
    btw -= copy;
    if (btw > 0) {
        BUF_MEMCPY(buff->buff, &d[copy], btw);
        buff->w = btw;
    }
    if (buff->w >= buff->size) {
        buff->w = 0;
    }
    BUF_SEND_EVT(buff, RINGBUFF_EVT_WRITE, copy + btw);
    return copy + btw;
}

/**
 * @brief reads data from buffer
 * @param buff: Buffer Handle
 * @param data: Pointer to output memory to copy buffer data
 * @param btr: # of bytes to read 
 * @return # of bytes read and copied to data array
*/

size_t ringbuff_read(RINGBUFF_VOLATILE ringbuff_t* buff, void* data, size_t btr) {
    size_t copy, full;
    uint8_t *d = data;
    if (!BUF_IS_VALID(buff) || data == NULL || btr == 0) {
        return 0;
    }
    full = ringbuff_get_full(buff);
    btr = BUF_MIN(full, btr);
    if (btr == 0) {
        return 0;
    }
    copy = BUF_MIN(buff->size - buff->r, btr);
    BUF_MEMCPY(d, &buff->buff[buff->r], copy);
    buff->r += copy;
    btr -= copy;
    if (btr > 0) {
        BUF_MEMCPY(&d[copy],buff->buff, btr);
        buff->r = btr;
    }
    if (buff->r >= buff->size) {
        buff->r = 0;
    }
    BUF_SEND_EVT(buff, RINGBUFF_EVT_READ, copy + btr);
    return copy + btr;
}

/**
 * @brief reads buffer data without changing read pointer
 * @param buff: Buffer Handle
 * @param data: Pointer to output memory to copy buffer data
 * @param btp: # of bytes to peek
 * @return # of bytes peeked
*/

size_t ringbuff_peek(RINGBUFF_VOLATILE ringbuff_t* buff, size_t skip_count, void* data, size_t btp) {
    size_t full, copy, r;
    uint8_t *d = data;
    if (!BUF_IS_VALID(buff) || data == NULL || btp == 0) {
        return 0;
    }
    r = buff->r;
    full = ringbuff_get_full(buff);
    if (skip_count >= full) {
        return 0;
    }
    r += skip_count;
    full -= skip_count;
    if (r >= buff->size) {
        r -= buff->size;
    }
    btp = BUF_MIN(full, btp);
    if (btp == 0) {
        return 0;
    }
    copy = BUF_MIN(buff->size - r, btp);
    BUF_MEMCPY(d, &buff->buff[r], copy);
    btp -= copy;
    if (btp > 0) {
        BUF_MEMCPY(&d[copy], buff->buff, btp);
    }
    return copy + btp;
}

/**
 * @brief get available size in buffer for write operations
 * @param buff: Buffer Handle
 * @return # of free bytes in memory
*/

size_t ringbuff_get_free(RINGBUFF_VOLATILE ringbuff_t* buff) {
    size_t size, w, r;
    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    w = buff->w;
    r = buff->r;
    if (w == r) {
        size = buff->size;
    } else {
        size = r > w ? r - w : buff->size - (w - r);
    }
    /*Free size is always 1 less than actual size*/
    return size - 1;
}

/**
 * @brief get # of bytes in buffer
 * @param buff: Buffer Handle
 * @returns: # of bytes ready to be read
*/
size_t ringbuff_get_full(RINGBUFF_VOLATILE ringbuff_t* buff) {
    size_t size, w, r;
    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    w = buff->w;
    r = buff->r;
        w = buff->w;
    r = buff->r;
    if (w == r) {
        size = buff->size;
    } else {
        size = w > r ? w - r : buff->size - (r - w);
    }
    return size;
}

/**
 * @brief resets buffer to default values. Buffer size is not modified
 * @param buff: Buffer Handle
*/

void ringbuff_reset(RINGBUFF_VOLATILE ringbuff_t* buff) {
    if (BUF_IS_VALID(buff)) {
        buff->w = 0;
        buff->r = 0;
        BUF_SEND_EVT(buff, RINGBUFF_EVT_RST, 0);
    }
}

/**
 * @brief get linear address for buffer for fast read
 * @param buff: Buffer Handle
 * @return Linear Buffer Start Address
*/

void* ringbuff_get_linear_block_read_address(RINGBUFF_VOLATILE ringbuff_t* buff) {
    if (!BUF_IS_VALID(buff)) {
        return NULL;
    }
    return &buff->buff[buff->r];
}

/**
 * @brief: get length of linear block address before it overflows for read operation
 * @param buff: Buffer Handle
 * @return: Linear Buffer Start Address
*/

size_t ringbuff_get_linear_block_read_length(RINGBUFF_VOLATILE ringbuff_t* buff) {
    size_t w, r, len;
    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    w = buff->w;
    r = buff->r;
    len = w > r ? w - r : r > w ? buff->size - r : 0;
    return len;
}


/**
 * @brief Skips bytes and marks them as read
 * @param buff: Buffer Data
 * @param len: # of bytes to skip and mark as read
 * @return # of bytes skipped
*/
size_t ringbuff_skip(RINGBUFF_VOLATILE ringbuff_t* buff, size_t len) {
    size_t full;
    if (!BUF_IS_VALID(buff) || len == 0) {
        return 0;
    }
    full = ringbuff_get_full(buff);
    len = BUF_MIN(len, full);
    buff->r += len;
    if (buff->r >= buff->size) {
        buff->r -= buff->size;
    }
    BUF_SEND_EVT(buff, RINGBUFF_EVT_READ, len);
    return len;
}

/**
 * @brief get linear address for buffer for fast read
 * @param buff: Buffer Handle
 * @return Linear buffer start address
*/

void* ringbuff_get_linear_block_write_address(RINGBUFF_VOLATILE ringbuff_t* buff) {
    if (!BUF_IS_VALID(buff)) {
        return NULL;
    }
    return &buff->buff[buff->w];
}

/**
 * @brief get length of linear block address before it overflows for write operation
 * @param buff: Buffer handle
 * @return buffer size for write
*/

size_t ringbuff_get_linear_block_write_length(RINGBUFF_VOLATILE ringbuff_t* buff) {
    size_t w, r, len;
    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    w = buff->w;
    r = buff->r;
    if (w >= r) {
        len = buff->size - w;
        if (r == 0) {
            --len;
        }
    } else {
        len = r - w - 1;
    }
    return len;
}
/**
 * @brief advances write pointer in the buffer
 * @note useful with hardware is writing to buffer
 * @param buff: Buffer Value
 * @param len: # of bytes to advance
 * @return: # of bytes advanced for write
*/

size_t ringbuff_advance(RINGBUFF_VOLATILE ringbuff_t* buff, size_t len) {
    size_t free;
    if (!BUF_IS_VALID(buff) || len == 0) {
        return 0;
    }
    free = ringbuff_get_free(buff);
    len = BUF_MIN(len, free);
    buff->w += len;
    if (buff->w >= buff->size) {
        buff->w -= buff->size;
    }
    BUF_SEND_EVT(buff, RINGBUFF_EVT_WRITE, len);
    return len;
}