/**
  ******************************************************************************
  * @file    XbeeInit.h
  * @author  Kanav Chugh
  * @brief   This is the file for UART communication for the XBee Pro S3B
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

#ifndef __XBEEINIT_H__
#define __XBEEINIT_H__


#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h7xx_hal_uart.h"
#include "stm32h745xx.h"

extern UART_HandleTypeDef huart4; //subject to change

char Buffer[20];
char recBuffer[10];

bool waitResponse();
bool atModeActive();
bool setID(char *ID);
bool setDestinationAddress(char *address);
bool setSourceAddress(char *address);
bool setRFMode(char *mode);
bool setStreamingLimit(char *limit);
bool setBaudRate(char *baudRate);
bool setPacketSize(char *packetSize);
bool setGuardTime(char *guardTIme);
bool setAtTimeout(char *timeout);
bool endAtMode();
bool XbeeInit(char *ID, char *sourceAddress, char *destinationAddress, char *baudRate, char *maxPacketSize);

#endif