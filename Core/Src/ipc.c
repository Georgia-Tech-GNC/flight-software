#include "ipc.h"

static volatile ipc_channel_t channel __attribute__((section(".shared_ram"))); 

static void generate_remote_interrupt(MessageBufferHandle_t xMessageBuffer, BaseType_t xIsInsideISR, BaseType_t *const pxHigherPriorityTaskWoken);
static void msg_received_isr(void);
static void msg_sent_isr(void);

void HAL_HSEM_FreeCallback(uint32_t sem_mask) {
    if ((sem_mask & __HAL_HSEM_SEMID_TO_MASK(HSEM_RX_ID)) != 0) {
        //We have received a message
        HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_RX_ID));
        msg_received_isr();
    } else if ((sem_mask & __HAL_HSEM_SEMID_TO_MASK(HSEM_TX_WAKEUP_ID)) != 0) {
        //We have sent a message
        HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_TX_WAKEUP_ID));
        msg_sent_isr();
    }
}

int ipc_init(void) {
    __HAL_RCC_HSEM_CLK_ENABLE();

#ifdef CORE_CM7
    channel.cm7_to_cm4_handle = xMessageBufferCreateStaticWithCallback(IPC_BUFFER_SIZE, (uint8_t *) channel.cm7_to_cm4_buffer, (StaticMessageBuffer_t *) &channel.cm7_to_cm4_msg, generate_remote_interrupt, generate_remote_interrupt); 
    channel.cm4_to_cm7_handle = xMessageBufferCreateStaticWithCallback(IPC_BUFFER_SIZE, (uint8_t *) channel.cm4_to_cm7_buffer, (StaticMessageBuffer_t *) &channel.cm4_to_cm7_msg, generate_remote_interrupt, generate_remote_interrupt);
#endif

    HAL_NVIC_SetPriority(HSEM_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(HSEM_IRQn);

    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_RX_ID));
    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_TX_WAKEUP_ID));

    return 0;
}

static void msg_received_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!RX_HANDLE(channel)) {
        return;
    }

    xMessageBufferSendCompletedFromISR(RX_HANDLE(channel), &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void msg_sent_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!TX_HANDLE(channel)) {
        return;
    }

    xMessageBufferReceiveCompletedFromISR(TX_HANDLE(channel), &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void generate_remote_interrupt(MessageBufferHandle_t xMessageBuffer, BaseType_t xIsInsideISR, BaseType_t *const pxHigherPriorityTaskWoken) {
    __DSB();

    if (xMessageBuffer == TX_HANDLE(channel)) {
        // Notify the other core that a message has been sent
        if (HAL_HSEM_Take(HSEM_TX_ID, HSEM_PROCESS_ID) == HAL_OK) {
            HAL_HSEM_Release(HSEM_TX_ID, HSEM_PROCESS_ID);
        }
    } else if (xMessageBuffer == RX_HANDLE(channel)) {
        // Notify the other core that its message has been received
        if (HAL_HSEM_Take(HSEM_RX_WAKEUP_ID, HSEM_PROCESS_ID) == HAL_OK) {
            HAL_HSEM_Release(HSEM_RX_WAKEUP_ID, HSEM_PROCESS_ID);
        }
    }
}

uint32_t ipc_send(uint8_t *data, uint16_t size, TickType_t timeout) {
    if (!TX_HANDLE(channel)) {
        return 0;
    }

    return xMessageBufferSend(TX_HANDLE(channel), data, size, timeout);
}

uint32_t ipc_receive(uint8_t *data, uint16_t size, TickType_t timeout) {
    if (!RX_HANDLE(channel)) {
        return 0;
    }

    return xMessageBufferReceive(RX_HANDLE(channel), data, size, timeout);
}