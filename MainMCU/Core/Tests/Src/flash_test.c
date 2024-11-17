#include "flash_test.h"
#include "globals.h"
#include "w25q.h"
#include "string.h"

void flash_test_task(void *args) {
    vTaskDelay(100);

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flash test task\r\n", 16, HAL_MAX_DELAY);

    struct w25q_device device = {
        .hospi = (void *) &flash_spi,
        .status = 0,
    };

    if (w25q_init(&device) != W25Q_ERR_OK) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Init failed\n", 12, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Init success\n", 13, HAL_MAX_DELAY);
    }

    vTaskDelay(100);

    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    
    vTaskDelay(100);

    
    if (w25q_write_raw(&device, test_data, 4, 2048 + 256) != W25Q_ERR_OK) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Write failed\n", 13, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Write success\n", 14, HAL_MAX_DELAY);
    }
    
    
    vTaskDelay(100);

    uint8_t read_data[32];

    if (w25q_read_raw(&device, read_data, 4, 2048 + 256) != W25Q_ERR_OK) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Read failed\n", 12, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Read success\n", 13, HAL_MAX_DELAY);
    }

    int same_data = 1;

    for (int i = 0; i < 4; i++) {
        if (test_data[i] != read_data[i]) {
            same_data = 0;
            break;
        }
    }

    char msg[100];
    
    for (int i = 0; i < 4; i ++) {
        sprintf(msg, "Data: %02X\n", read_data[i]);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
    }

    if (same_data) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Data is the same\n", 17, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Data is different\n", 19, HAL_MAX_DELAY);
    }
    
    while (1) {
        vTaskDelay(1000);
    }
    
}
