  /******************************************************************************
  * @file    XbeeInit.h
  * @author  Kanav Chugh
  * @brief   Source file for XBee Methods
  *         
  ******************************************************************************
  * @attention
  *
  * 
  * 
  *
  * 
  * 
  * 
  *  
  *
  ******************************************************************************
  */

#include "../Inc/XbeeInit.h"
#include "stdint.h"

void changeUartBaud(uint32_t baudRate) {
    huart4.Instance = UART4;
    huart4.Init.BaudRate = baudRate;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
}

bool waitResponse() {
    memset(recBuffer, '\0', 10);
    HAL_UART_Receive(&huart4, (uint8_t*) recBuffer, 10, 100);
    for (int i = 0; i < 9; i++) { 
        //seeing if the buffers says OK anywhere
        if (recBuffer[i] == 'O' && recBuffer[i + 1] == 'K') {
            HAL_Delay(100);
            return 1;
        }
    }
    return 0;
}

bool atModeActive() {
    sprintf(Buffer, "+++");
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 3, 400);
    memset(Buffer, '\0', 20);
    return waitResponse();
}

bool setId(char *ID) {
    sprintf(Buffer, "ATID%s\r", ID);
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 9, 100);
    memset(Buffer, '\0', 20);
    return waitResponse();
}

bool setDestinationAddress(char *address) {
    sprintf(Buffer, "ATDT%s\r", address);
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 9, 100);
    memset(Buffer, '\0', 20);
    return waitResponse();
}

bool setSourceAddress(char *address) {
    sprintf(Buffer, "ATMY%s\r", address);
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 9, 100);
    memset(Buffer, '\0', 20);
    return waitResponse();
}

bool setRFMode(char *mode) {
    sprintf(Buffer,"ATMY%s\r",mode);
	HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 6, 100);
	memset(Buffer,'\0',20);
	return waitResponse();
}

bool setStreamingLimit(char *limit) {
	sprintf(Buffer,"ATTT%s\r", limit);
	HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 7, 100);
	memset(Buffer,'\0',20);
	return waitResponse();
}

bool setBaudRate(char *baudRate) {
    sprintf(Buffer, "ATBD%s\r",baudRate);
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 6, 100);
    memset(Buffer,'\0',20);
    if (waitResponse()) {
        uint32_t baud = 0;
        switch(baudRate[0]) {
            case '0':
                baud = 1200;
                break;
            case '1':
                baud = 2400;
                break;
            case '2':
                baud = 4800;
                break;
            case '3':
                baud = 9600;
                break;
            case '4':
                baud = 19200;
                break;
            case '5':
                baud = 38400;
                break;
            case '6':
                baud = 57600;
                break;
            case '7':
                baud = 115200;
                break;
            default:
                baud = 38400;
        }
        changeUartBaud(baud);
        return 1;
    }
    return 0;
}

bool setPacketSize(char *packetSize) {
    sprintf(Buffer, "ATPK%s\r",packetSize);
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 7, 100);
    memset(Buffer,'\0',20);
    return waitResponse();
}

bool setGuardTime(char *guardTime) {
    sprintf(Buffer, "ATBT%s\r", guardTime);
    HAL_UART_Transmit(&huart4, (uint8_t *) Buffer, 9, 100);
    memset(Buffer,'\0',20);
    return waitResponse();
}

bool setAtTimeout(char *timeout) {
    sprintf(Buffer,"ATCT%s\r", timeout);
    HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 9, 100);
    memset(Buffer,'\0',20);
    return waitResponse();
}

bool endAtMode() {
    sprintf(Buffer,"ATWR\r");
    HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 5, 100);
    memset(Buffer,'\0',20);
    sprintf(Buffer,"ATAC\r");
    HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 5, 100);
    memset(Buffer,'\0',20);
    HAL_Delay(1000);
    sprintf(Buffer,"ATCN\r");
    HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 5, 100);
    memset(Buffer,'\0',20);
    return waitResponse();
}

bool XBeeInit(char *ID, char *sourceAddress, char *destinationAddress, char *baudRate, char *maxPacketSize) {
    HAL_Delay(100);
    if (!atModeActive() ||
        !setID(ID) ||
        !setDestinationAddress(destinationAddress) ||
        !setSourceAddress(sourceAddress) ||
        !setBaudRate(baudRate) ||
        !setPacketSize(maxPacketSize) ||
        !endAtMode()) {
        return 0;
    }
    HAL_Delay(100);
    return 1;
}