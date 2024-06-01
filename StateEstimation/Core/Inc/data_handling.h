/**
 * @file data_handling.h
 * @author Albert Zheng
 * @brief Header file for logging, transmitting, and receiving serial data
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef DATA_HANDLING_H
  #define DATA_HANDLING_H

// Data packet to send to Controls MCU
typedef struct SerialData {
  int state; // current state machine state

  float posx; // ecef frame
  float posy;
  float posz;
  float velx; // ecef frame
  float vely;
  float velz;
  float q0; // ned to body
  float q1;
  float q2;
  float q3;
  float wx; // body frame
  float wy;
  float wz;

  float t; // time

} SerialData;

typedef struct SerialDataPacket {
    SerialData data;
    uint8_t bytes[sizeof(SerialData)];
} SerialDataPacket;

typedef struct LoggedData {
  
  int state; // current state machine state

  float posx; // ecef frame
  float posy;
  float posz;
  float velx; // ecef frame
  float vely;
  float velz;
  float q0; // ned to body
  float q1;
  float q2;
  float q3;
  float wx; // body frame
  float wy;
  float wz;

  float accelerometerx;
  float accelerometery;
  float accelerometerz;
  float gyrox;
  float gyroy;
  float gyroz;
  float magnetox;
  float magnetoy;
  float magnetoz;
  float gpsx;
  float gpsy;
  float gpsz;
  float baro;

} LoggedData;

typedef struct LoggedDataPacket {
    LoggedData data;
    uint8_t bytes[sizeof(LoggedData)];
} LoggedDataPacket;

void send_serial_data(SerialData* serial_data);

void log_data(LoggedData* log_data);

#endif