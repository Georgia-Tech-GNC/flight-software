/**
 ******************************************************************************
 * @file    gnss.h
 * @author  Kanav Chugh
 * @brief   Zed-F9P GNSS header driver file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __GNSS_H__
#define __GNSS_H__

#include "../Inc/ringbuffer.h"

#define NUM_FIELDS_MAX      32
#define FIELD_LENGTH_MAX    32

typedef struct {
    char        message_id[6];
    double      latitude;
    char        north_south_indicator;
    double      longitude;
    char        east_west_indicator;
    double      time;
    char        status;
    char        positioning_mode;
    uint32_t    checksum;
} GPS_GLL_t;

typedef struct {
    char        message_id[6];
    double      time;
    double      latitude;
    char        north_south_indicator;
    double      longitutde;
    char        east_west_indicator;
    uint8_t     quality;
    uint8_t     number_of_satelites;
    double      horizontal_dilution_of_precision;
    double      altitude;
    char        altitude_units;
    double      geoid_separation;
    char        geoid_separation_units;
    double      differential_corrections_age;
    double      differential_corrections_station_id;
    uint32_t    checksum;
} GPS_GGA_t;

typedef struct {
    double      time;
    double      latitude;
    char        north_south_indicator;
    double      longitude;
    char        east_west_indicator;
    char        status;
    char        positioning_mode;
    double      altitude_meters;
} GPS_t;


uint8_t DeserializeMessage(uint8_t *message_id, void *message, uint8_t fields[NUM_FIELDS_MAX][FIELD_LENGTH_MAX], uint8_t fields_count);
uint8_t ParsePacket(RingBuffer_t *data, void *message, uint8_t *message_id);
void GPS_ProcessData(GPS_t *gps, RingBuffer_t *data);
double  GPS_GetTime(GPS_t *gps);
double  GPS_GetLatitude(GPS_t *gps);
char    GPS_GetNorthSouthIndicator(GPS_t *gps);
double  GPS_GetLongitude(GPS_t *gps);
char    GPS_GetEastWestIndicator(GPS_t *gps);
char    GPS_GetStatus(GPS_t *gps);
char    GPS_GetPositioningMode(GPS_t *gps);
double  GPS_GetAltitudeMeters(GPS_t *gps);


#endif