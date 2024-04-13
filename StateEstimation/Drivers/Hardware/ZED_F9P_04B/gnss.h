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


#include "basic_types.h"
#include "stm32h723xx.h"

#define UBX_SYN_CHAR1    0xB5
#define UBX_SYN_CHAR2    0x62

#define  SM_UBX_BEFORE    0
#define  SM_UBX_SYN2    	1
#define  SM_UBX_CLASS     2
#define  SM_UBX_ID    		3
#define  SM_UBX_PAYLEN1  	4
#define  SM_UBX_PAYLEN2   5
#define  SM_UBX_PAYLOAD   6
#define  SM_UBX_CHK1      7
#define  SM_UBX_CHK2      8
#define  SM_UBX_ERR       9
#define  SM_UBX_END       10

typedef enum {
  UBX_NAV     	 	    = 0x01,   //Navigation Results Messages
  UBX_MON     	 	    = 0x0A,   //Monitoring Messages
  UBX_NAV_PVT         = 0x07,   //NAV-PVT: Navigation Position Velocity Time Solution
  UBX_NAV_RELPOSNED   = 0x3C,   //NAV-RELPOSNED: Relative Positioning Information in NED frame
  UBX_MON_MSGPP       = 0x06,   //MON-MSGPP: Message Parse and Process Status
};

typedef struct {
	float lat;
	float lon;
	float alt;
} Pos;

typedef struct {
	float N;
	float E;
	float D;
} PosRel;
  
typedef struct {
    Pos pos;
    PosRel relPos;
    int fixType;
    float hAcc;
    float vAcc;
    float iTOW;
    int UTCyear;
    int UTCmonth;
    int UTCday;
    int UTChour;
    int UTCminute;
    int UTCsecond;
    int msgs;
} Cgnss;

#define MAX_GNSS          256
#define UART_BUF_SIZE     256
extern uint8_t rxBufferGNSS;

typedef struct {
  int state;
  int ctr;
  byte buf[MAX_GNSS];
} tGNSSrx;

typedef union {
  uint8_t b[4];
  int32_t i;
} mlong;

typedef union {
  int8_t b[2];
  int16_t i;
} mshort;

typedef struct{
    Cgnss *cgnss;
} CSensors;

extern CSensors sensors;

int parseUBX(byte *b, int cnt, Cgnss *cgnss);
void handleGNSS(void);
void initGNSSrx(void);
int addUBXpktByte(byte ch, tGNSSrx *pr);
int readUBXpkt(byte *retbuf);
int checkUBX(byte *buf, int cnt);
void crcUBX(byte *buf, int cnt, byte *pcha, byte *pchb);
void EventsCommGNSS(uint8_t *msgbuf, int32_t cnt);
int32_t bytesToLong(uint8_t *b);
int16_t bytesToShort(uint8_t *b);

#endif