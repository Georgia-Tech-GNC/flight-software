/**
 ******************************************************************************
 * @file    gnss.c
 * @author  Kanav Chugh
 * @brief   Zed-F9P GNSS source file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "gnss.h"

uint8_t DeserializeMessage(uint8_t *message_id, void *message, uint8_t fields[NUM_FIELDS_MAX][FIELD_LENGTH_MAX], uint8_t fields_count) {
    if (strcmp((char *) message_id, "GLL") == 0) {
        switch (fields_count) {
            case 0:
                strcpy(((GPS_GLL_t *) message)->message_id, (char *) fields[fields_count]);
                break;
            case 1:
                ((GPS_GLL_t *) message)->latitude               = (double) atof((char *) fields[fields_count]);
                break;
            case 2:
                ((GPS_GLL_t *) message)->north_south_indicator  = (char) fields[fields_count][0];
                break;
            case 3:
                ((GPS_GLL_t *) message)->longitude              = (double) atof((char *) fields[fields_count]);
                break;
            case 4:
                ((GPS_GLL_t *) message)->east_west_indicator    = (char) fields[fields_count][0];
                break;
            case 5:
                ((GPS_GLL_t *) message)->time                   = (double) atof((char *) fields[fields_count]);
                break;
            case 6:
                ((GPS_GLL_t *) message)->status                 = (char) fields[fields_count][0];
                break;
            case 7:
                ((GPS_GLL_t *) message)->positioning_mode       = (char) fields[fields_count][0];
                break;
            case 8:
                ((GPS_GLL_t *) message)->checksum = 0;
                break;
            default:
                return 1;
        }
    }

    return 0;
}

uint8_t ParsePacket(RingBuffer_t *data, void *message, uint8_t *message_id) {
    // Check if there is enough data left to parse a complete NMEA string
    if (data->count < 82) {
        return 1;
    }

    uint8_t current_byte[2] = "\0\0";
    RingBuffer_Get(data, current_byte, 1);
    uint8_t fields[NUM_FIELDS_MAX][FIELD_LENGTH_MAX] = {[0 ... (NUM_FIELDS_MAX - 1)] = {[0 ... (FIELD_LENGTH_MAX - 1)] = 0}};
    uint8_t fields_count = 0;

    while (current_byte[0] != '\r') {
        RingBuffer_Get(data, current_byte, 1);
        if (current_byte[0] == ',' || current_byte[0] == '*' || current_byte[0] == '\r') {
            DeserializeMessage(message_id, message, fields, fields_count);
            fields_count++;
            continue;
        }

        strcat((char *) fields[fields_count], (char *) current_byte);
    }
    RingBuffer_Get(data, current_byte, 1);
    return 0;
}

void GPS_ProcessData(GPS_t *gps, RingBuffer_t *data) {
    uint8_t trash[1];

    while (1) {
        uint8_t message_id[4];
        if (data->count < 82) {
            break;
        }

        message_id[0] = data->buff[data->tail + 3];
        message_id[1] = data->buff[data->tail + 4];
        message_id[2] = data->buff[data->tail + 5];
        message_id[3] = '\0';

        if (strcmp((char *) message_id, "GLL") == 0) {
            GPS_GLL_t _gll;
            GPS_GLL_t *gll = &_gll;
            ParsePacket(data, (void *) gll, (uint8_t *) "GLL");
            gps->time                   = gll->time;
            gps->latitude               = gll->latitude;
            gps->north_south_indicator  = gll->north_south_indicator;
            gps->longitude              = gll->longitude;
            gps->east_west_indicator    = gll->east_west_indicator;
            gps->status                 = gll->status;
            gps->positioning_mode       = gll->positioning_mode;
        } else {
            RingBuffer_Get(data, trash, 1);
        }
    }
}

double  GPS_GetTime(GPS_t *gps) {
    return gps->time;
}

double  GPS_GetLatitude(GPS_t *gps) {
    return gps->latitude;
}

char GPS_GetNorthSouthIndicator(GPS_t *gps) {
    return gps->north_south_indicator;
}

double  GPS_GetLongitude(GPS_t *gps) {
    return gps->longitude;
}

char GPS_GetEastWestIndicator(GPS_t *gps) {
    return gps->east_west_indicator;
}

char    GPS_GetStatus(GPS_t *gps) {
    return gps->status;
}

char    GPS_GetPositioningMode(GPS_t *gps) {
    return gps->positioning_mode;
}

double  GPS_GetAltitudeMeters(GPS_t *gps) {
    return gps->altitude_meters;
}