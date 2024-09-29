#ifndef IPC_H
#define IPC_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <FreeRTOS.h>
#include <message_buffer.h>

#ifdef CORE_CM7
#define HSEM_IRQn HSEM1_IRQn
#define HSEM_TX_ID 8
#define HSEM_RX_ID 9
#define HSEM_TX_WAKEUP_ID 10
#define HSEM_RX_WAKEUP_ID 11
#elif CORE_CM4
#define HSEM_IRQn HSEM2_IRQn
#define HSEM_TX_ID 9
#define HSEM_RX_ID 8
#define HSEM_TX_WAKEUP_ID 11
#define HSEM_RX_WAKEUP_ID 10
#endif

#ifdef CORE_CM7
#define RX_HANDLE(x) ((x).cm4_to_cm7_handle)
#define TX_HANDLE(x) ((x).cm7_to_cm4_handle)
#elif CORE_CM4
#define RX_HANDLE(x) ((x).cm7_to_cm4_handle)
#define TX_HANDLE(x) ((x).cm4_to_cm7_handle)
#endif

#define IPC_BUFFER_SIZE 256
#define HSEM_PROCESS_ID 0

typedef struct {
    uint8_t cm7_to_cm4_buffer[IPC_BUFFER_SIZE];
    uint8_t cm4_to_cm7_buffer[IPC_BUFFER_SIZE];
    MessageBufferHandle_t cm7_to_cm4_handle;
    MessageBufferHandle_t cm4_to_cm7_handle;
    StaticMessageBuffer_t cm7_to_cm4_msg;
    StaticMessageBuffer_t cm4_to_cm7_msg;
} ipc_channel_t;

int ipc_init(void);
uint32_t ipc_send(uint8_t *data, uint16_t size, TickType_t timeout);
uint32_t ipc_receive(uint8_t *data, uint16_t size, TickType_t timeout);


#endif