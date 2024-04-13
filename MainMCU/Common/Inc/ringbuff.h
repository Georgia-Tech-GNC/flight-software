/**
 * @file ringbuff.h
 * @author Kanav Chugh
 * @brief ring buffer for dual processor communication
*/
#ifndef __RINGBUFF_H__
#define __RINGBUFF_H__

#include <string.h>
#include <stdint.h>

#define RINGBUFF_VOLATILE   volatile
#define RINGBUFF_USE_WORD   1

typedef enum {
    RINGBUFF_EVT_READ,
    RINGBUFF_EVT_WRITE,
    RINGBUFF_EVT_RST
} ringbuff_evt_type;

struct ringbuff;

typedef void (*ringbuff_evt_fn) (RINGBUFF_VOLATILE struct ringbuff* buff, ringbuff_evt_type event, size_t size);

typedef struct ringbuff {
    #if RINGBUFF_USE_WORD
        uint32_t word1;
    #endif
    uint8_t* buff;
    size_t size;
    size_t r;
    size_t w;
    ringbuff_evt_fn fn;
    #if RINGBUFF_USE_WORD
        uint32_t word2;
    #endif
} ringbuff_t;

uint8_t ringbuff_init(RINGBUFF_VOLATILE ringbuff_t* buff, void* buffdata, size_t size);
uint8_t ringbuff_is_ready(RINGBUFF_VOLATILE ringbuff_t* buff);
void ringbuff_free(RINGBUFF_VOLATILE ringbuff_t* buff);
void ringbuff_reset(RINGBUFF_VOLATILE ringbuff_t* buff);
void ringbuff_set_evt_fun(RINGBUFF_VOLATILE ringbuff_t* buff, ringbuff_evt_fn fn);

/*R/W functions*/

size_t ringbuff_write(RINGBUFF_VOLATILE ringbuff_t* buff, const void* data, size_t btw);
size_t ringbuff_read(RINGBUFF_VOLATILE ringbuff_t* buff, void* data, size_t btr);
size_t ringbuff_peek(RINGBUFF_VOLATILE ringbuff_t* buff, size_t skip_count, void* data, size_t btp);

/*Size info*/

void* ringbuff_get_free(RINGBUFF_VOLATILE ringbuff_t* buff);
void* ringbuff_get_full(RINGBUFF_VOLATILE ringbuff_t* buff);

/*Read data block management*/

void* ringbuff_get_linear_block_read_address(RINGBUFF_VOLATILE ringbuff_t* buff);
void* ringbuff_get_linear_block_read_length(RINGBUFF_VOLATILE ringbuff_t* buff);
size_t ringbuff_skip(RINGBUFF_VOLATILE ringbuff_t* buff, size_t len);

/*Write data block management*/

void* ringbuff_get_linear_block_write_address(RINGBUFF_VOLATILE ringbuff_t* buff);
void* ringbuff_get_linear_block_write_length(RINGBUFF_VOLATILE ringbuff_t* buff);
size_t ringbuff_advance(RINGBUFF_VOLATILE ringbuff_t* buff, size_t len);


#endif