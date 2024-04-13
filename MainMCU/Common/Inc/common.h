/**
 * @file common.h
 * @author Kanav Chugh
 * @brief Common Configuration for both cores
*/

/*
*/

#ifndef COMMON_HDR_H
#define COMMON_HDR_H

#include "stm32h745xx.h"
#include "../Inc/ringbuff.h"

#define MEM_ALIGN(x)                ((x + 0x00000003) & ~0x00000003)
#define SHD_RAM_START_ADDR          0x38000000
#define SHD_RAM_LEN                 0x0000FFFF

#define BUFF_CM4_TO_CM7_ADDR        MEM_ALIGN(SHD_RAM_START_ADDR)
#define BUFF_CM4_TO_CM7_LEN         MEM_ALIGN(sizeof(ringbuff_t))
#define BUFFDATA_CM4_TO_CM7_ADDR    MEM_ALIGN(BUFF_CM4_TO_CM7_ADDR + BUFF_CM4_TO_CM7_LEN)
#define BUFFDATA_CM4_TO_CM7_LEN     MEM_ALIGN(0x00000400)

#define BUFF_CM7_TO_CM4_ADDR        MEM_ALIGN(BUFFDATA_CM4_TO_CM7_ADDR + BUFFDATA_CM4_TO_CM7_LEN)
#define BUFF_CM7_TO_CM4_LEN         MEM_ALIGN(sizeof(ringbuff_t))
#define BUFFDATA_CM7_TO_CM4_ADDR    MEM_ALIGN(BUFF_CM7_TO_CM4_ADDR + BUFF_CM7_TO_CM4_LEN)
#define BUFFDATA_CM7_TO_CM4_LEN     MEM_ALIGN(0x00000400)

#define HSEM_TAKE_RELEASE(_id_)     do {HAL_HSEM_FastTake((_id_)); HAL_HSEM_Release((_id_), 0);} while (0)
#define HSEM_WAKEUP_CPU2            0
#define HSEM_WAKEUP_CPU2_MASK       __HAL_HSEM_SEMID_TO_MASK(HSEM_WAKEUP_CPU2)
#define HSEM_CM4_TO_CM7             1
#define HSEM_CM4_TO_CM7_MASK        __HAL_HSEM_SEMID_TO_MASK(HSEM_CM4_TO_CM7)
#define HSEM_CM7_TO_CM4             2
#define HSEM_CM7_TO_CM4_MASK        __HAL_HSEM_SEMID_TO_MASK(HSEM_CM7_TO_CM4)

#define WAIT_COND_WITH_TIMEOUT(c, t) do {
    int32_t timeout = (int32_t) t;
    while (c && timeout-- > 0) {}
    if (timeout < 0) {
        Error_Handler();
    }
} while (0)

#define LD1_GPIO_CLK_EN             __HAL_RCC_GPIOB_CLK_ENABLE
#define LD1_GPIO_PORT               GPIOB
#define LD1_GPIO_PIN                GPIO_PIN_0
#define LD2_GPIO_CLK_EN             __HAL_RCC_GPIOE_CLK_ENABLE
#define LD2_GPIO_PORT               GPIOE
#define LD2_GPIO_PIN                GPIO_PIN_1
#define LD3_GPIO_CLK_EN             __HAL_RCC_GPIOB_CLK_ENABLE
#define LD3_GPIO_PORT               GPIOB
#define LD3_GPIO_PIN                GPIO_PIN_14


#endif