/**
  ******************************************************************************
  * @file    gnss.h
  * @author  Kanav Chugh
  * @brief   GNSS Header file
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

#ifndef __GNSS_H__
#define __GNSS_H__

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define UBX_SYN_CHAR1 0xB5
#define UBX_SYN_CHAR2 0x62

#define SM_UBX_BEFORE   0
#define SM_UBX_SYN2     1
#define SM_UBX_CLASS    2
#define SM_UBX_ID       3
#define SM_UBX_PAYLEN1  4
#define SM_UBX_PAYLEN2  5
#define SM_UBX_PAYLOAD  6
#define SM_UBX_CHK1     7
#define SM_UBX_CHK2     8
#define SM_UBX_ERR      9
#define SM_UBX_END      10

#define MAX_GNSS        256
#define BUF_SIZE        256

enum  {
    UBX_NAV             = 0x01, 
    UBX_MON             = 0x0A, 
    UBX_NAV_PVT         = 0x07, 
    UBX_NAV_RELPOSNED   = 0x3C, 
    UBX_MON_MSGPP       = 0x06
};

typedef float float32_t;
typedef double float64_t;
typedef unsigned char byte;

typedef struct {
    float lat;
    float lon;
    float alt;
} Pos;

typedef struct {
    float N;
    float E;
    float D;
} Pos_Rel;

typedef struct {
    int fix_type;
    float h_acc;
    float v_acc;
    float i_tow;
    int UTC_year;
    int UTC_month;
    int UTC_day;
    int UTC_hour;
    int UTC_minute;
    int UTC_second;
    int msgs;
    Pos *pos;
    Pos_Rel *rel_pos;
} CGNSS;

extern uint8_t rx_buffer_gnss;

typedef struct {
    byte buf[MAX_GNSS];
    int state;
    int ctr;
} t_gnss_rx;

typedef struct {
    uint8_t b[4];
    int32_t i;
} m_long;

typedef struct {
    int8_t b[2];
    int16_t i;
} m_short;


typedef enum {
    OK = 0,
    TOO_LEFT = 1,
    TOO_RIGHT = 2
} position_state;

bool parse_ubx(CGNSS* cgnss, byte *buf, int cnt);
void init_hardware(void);

void event_sims(CGNSS *cgnss);
void handle_gnss(CGNSS *cgnss);
void init_gnss_rx(void);
int add_ubx_pkt_byte(byte ch, t_gnss_rx *pr);
int read_ubx_pkt(byte *ret_buf);
int check_ubx(byte *buf, int cnt);
void crc_ubx(byte *buf, int cnt, byte *pcha, byte *pchb);
void events_comm_gnss(CGNSS *cgnss, uint8_t *msg_buf, int32_t cnt);
int32_t bytes_to_long(uint8_t *b);
int16_t bytes_to_short(uint8_t *b);

Pos* get_pos(CGNSS *gnss);
Pos_Rel* get_rel_pos(CGNSS *gnss);
int get_fix_type(CGNSS *gnss);
float get_h_acc(CGNSS *gnss);
float get_v_acc(CGNSS *gnss);
float get_i_tow(CGNSS *gnss);
int get_UTC_year(CGNSS *gnss);
int get_UTC_month(CGNSS *gnss);
int get_UTC_day(CGNSS *gnss);
int get_UTC_hour(CGNSS *gnss);
int get_UTC_minute(CGNSS *gnss);
int get_UTC_second(CGNSS *gnss);
int get_msgs(CGNSS *gnss);


#endif