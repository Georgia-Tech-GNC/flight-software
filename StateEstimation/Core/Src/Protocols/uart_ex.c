#include "uart_ex.h"

/**
 * @brief Callback function for UART receive event with idle line detection
 * @param huart Pointer to UART handle structure
 * @param Size Number of bytes received
 * @details Processes received UART data, handles UBX messages, and manages ring buffer
 * @note Called automatically by HAL when UART receive is complete or idle line detected
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    char debug[128];
    int len;
    
    if (huart->Instance == UART4) {
        len = sprintf(debug, "UART Interrupt: Size=%d, Data: ", Size);
        
        for(int i = 0; i < Size && i < 8; i++) {
            len = sprintf(debug, "%02X ", uart4_rx_dma_buffer[i]);
        }
        
        if (Size >= 4 && uart4_rx_dma_buffer[0] == 0xB5 && uart4_rx_dma_buffer[1] == 0x62) {
            len = sprintf(debug, "\r\nUBX Message - Class: 0x%02X, ID: 0x%02X\r\n", 
                         uart4_rx_dma_buffer[2], uart4_rx_dma_buffer[3]);
        } else {
            len = sprintf(debug, "\r\nNot a UBX message\r\n");
        }
        
        ring_buffer_write(&uart4_rx_rb, uart4_rx_dma_buffer, Size);
        memset(uart4_rx_dma_buffer, 0, sizeof(uart4_rx_dma_buffer));
        
        if(HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart4_rx_dma_buffer, sizeof(uart4_rx_dma_buffer)) != HAL_OK) {
            len = sprintf(debug, "Failed to restart UART\r\n");
            HAL_UART_Transmit(&huart3, (uint8_t*)debug, len, HAL_MAX_DELAY);
            HAL_UART_DeInit(&huart4);
            HAL_UART_Init(&huart4);
            HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart4_rx_dma_buffer, sizeof(uart4_rx_dma_buffer));
        }
    }
}
/**
 * @brief Callback function for completed UART receive operation
 * @param huart Pointer to UART handle structure
 * @details Handles "GO" command to start EKF and transitions state machine
 * @note Called automatically by HAL when UART receive is complete
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        char buffer[6];
        sprintf(buffer, "%02X\r\n", signal_received[0]);
        HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
        if (strncmp(signal_received, "GO", 2) == 0) {
            HAL_UART_Transmit(&huart3, "\r\nStarting EKF...\r\n", sizeof("\r\nStarting EKF...\r\n") - 1, HAL_MAX_DELAY);
            rocket_state = GROUND;
            ready_message_printed = 0; 
        }
    }
}

/**
 * @brief Error callback function for UART operations
 * @param huart Pointer to UART handle structure
 * @details Handles UART errors by printing debug info and restarting receive operation
 * @note Called automatically by HAL when UART error occurs
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    char debug[128];
    int len;
    if (huart->Instance == UART4) {
        len = sprintf(debug, "UART4 Error 0x%lX\r\n", huart->ErrorCode);
        HAL_UART_Transmit(&huart3, (uint8_t*)debug, len, HAL_MAX_DELAY);
        HAL_UART_AbortReceive(&huart4);
        HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart4_rx_dma_buffer, sizeof(uart4_rx_dma_buffer));
    }
}

/**
 * @brief Prints rocket attitude information to UART
 * @param rocket_atd Pointer to rocket attitude structure
 * @param huart Pointer to UART handle structure
 * @details Outputs quaternion, Euler angles, and gyroscope readings in human-readable format
 */
void print_rocket_attitude(RocketAttitude *rocket_atd, UART_HandleTypeDef *huart) {
    char buf[200];
    // Print quaternion
    snprintf(buf, sizeof(buf), "Quaternion (s,x,y,z): %.3f, %.3f, %.3f, %.3f\r\n", 
             rocket_atd->q_current_s,
             rocket_atd->q_current_x,
             rocket_atd->q_current_y,
             rocket_atd->q_current_z);
    HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    
    snprintf(buf, sizeof(buf), "Euler (phi,theta,psi): %.2f, %.2f, %.2f\r\n",
             rocket_atd->phi, 
             rocket_atd->theta,
             rocket_atd->psi);
    HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    
    snprintf(buf, sizeof(buf), "Gyro (x,y,z): %.2f, %.2f, %.2f\r\n",
             rocket_atd->gyro_x,
             rocket_atd->gyro_y,
             rocket_atd->gyro_z);
    HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}