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

  float pos_x; // ecef frame
  float pos_y;
  float pos_z;
  float vel_x; // ecef frame
  float vel_y;
  float vel_z;
  float q0; // world to body
  float q1;
  float q2;
  float q3;
  float wx; // body frame
  float wy;
  float wz;

  float t; // time

} SerialData;

// Sensor readings
typedef struct Sensors {
  float accelerometer_x;
  float accelerometer_y;
  float accelerometer_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float magneto_x;
  float magneto_y;
  float magneto_z;
  float gps_x;
  float gps_y;
  float gps_z;
  float baro;
  float accel_bias_x;
  float accel_bias_y;
  float accel_bias_z;
  float gyro_bias_x;
  float gyro_bias_y;
  float gyro_bias_z;
  float gps_offset_x; // subtract this offset to get GPS posn in World frame
  float gps_offset_y;
  float gps_offset_z;
  float baro_offset; // subtract this offset to get baro reading relative to World
} Sensors;

typedef struct SerialDataPacket {
    SerialData data;
    uint8_t bytes[sizeof(SerialData)];
} SerialDataPacket;


void send_serial_data(SerialData* serial_data);

void log_data(SerialData* serial_data, Sensors* sensors);

void read_sensors(Sensors* sensors);


#endif