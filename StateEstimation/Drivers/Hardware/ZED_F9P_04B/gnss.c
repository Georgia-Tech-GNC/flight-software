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

#include <math.h>
#include "tasks.h"
#include "gnss.h"

uint8_t rxString[MAX_GNSS];
int rxindex = 0;
uint8_t rxBufferGNSS = 0;
tGNSSrx GNSSrx;


/**
 * @brief Handles the reading and processing of GNSS data packets.
 */
void handleGNSS(void) {
    uint8_t msgbuf[MAX_GNSS];
    int32_t msgcnt = readUBXpkt(msgbuf);
    if(msgcnt>0) {
        EventsCommGNSS(msgbuf,msgcnt);
    } else if(msgcnt == -1) {
        initGNSSrx();
    }
}

/**
 * @brief Processes communication events for GNSS data.
 * @param msgbuf The buffer containing the message.
 * @param cnt The count of bytes in the buffer.
 */
void EventsCommGNSS(uint8_t *msgbuf, int32_t cnt) {
    parseUBX(msgbuf,cnt, sensors.cgnss);
}

/**
 * @brief Reads a UBX packet from the GNSS data stream.
 * @param retbuf A buffer to store the read packet.
 * @return The number of bytes read, or 0 if no complete packet was read.
 */
int readUBXpkt(byte *retbuf) {
    int i=0;
    if(GNSSrx.ctr<MAX_GNSS) {
        if(addUBXpktByte(rxBufferGNSS,&(GNSSrx))>0) {
            GNSSrx.state=SM_UBX_BEFORE;
            for(i=0;i<GNSSrx.ctr;i++) {
                retbuf[i]=GNSSrx.buf[i];
            }
            GNSSrx.ctr=0;
            if(checkUBX(retbuf,i)==0) {
                return(i-2);
            } else {
                return(0);
            }
        }
    }
    if(GNSSrx.ctr>=MAX_GNSS) {
        GNSSrx.ctr=0;
        GNSSrx.state=SM_UBX_BEFORE;
    }
    return 0;
}

/**
 * @brief Initializes the GNSS receiver state.
 */
void initGNSSrx(void) {
    GNSSrx.state = SM_UBX_BEFORE;
    GNSSrx.ctr = 0;
}

/**
 * @brief Adds a byte to the UBX packet being constructed.
 * @param ch The byte to add.
 * @param pr The GNSS receiver state.
 * @return The current count of bytes in the buffer or -1 if an error occurred.
 */
int addUBXpktByte(byte ch, tGNSSrx *pr) {
    switch(pr->state) {
        case SM_UBX_BEFORE:
            if(ch==UBX_SYN_CHAR1) pr->state=SM_UBX_SYN2;
            break;
        case SM_UBX_SYN2:
            if(ch==UBX_SYN_CHAR2) pr->state=SM_UBX_CLASS;
            else pr->state=SM_UBX_BEFORE;
            break;
        case SM_UBX_CLASS:
            pr->buf[pr->ctr++]=ch;
            pr->state=SM_UBX_ID;
            break;
        case SM_UBX_ID:
            pr->buf[pr->ctr++]=ch;
            pr->state=SM_UBX_PAYLEN1;
            break;
        case SM_UBX_PAYLEN1:
            pr->buf[pr->ctr++]=ch;
            pr->state=SM_UBX_PAYLEN2;    
            break;
        case SM_UBX_PAYLEN2:
            pr->buf[pr->ctr++]=ch;
            pr->state=SM_UBX_PAYLOAD;
            break;
        case SM_UBX_PAYLOAD:
            pr->buf[pr->ctr++]=ch;
            if(pr->ctr >= (bytesToShort((byte *)&(pr->buf[2])) + 4)) pr->state=SM_UBX_CHK1;
            else if(pr->ctr >= (UART_BUF_SIZE-10)) pr->state=SM_UBX_ERR;
            break;
        case SM_UBX_CHK1:
            pr->buf[pr->ctr++]=ch;
            pr->state=SM_UBX_CHK2;
            break;
        case SM_UBX_CHK2:
            pr->buf[pr->ctr++]=ch;
            pr->state=SM_UBX_END;
            break;
        case SM_UBX_ERR:
            pr->state=SM_UBX_BEFORE;
            break;
        default:
            pr->state=SM_UBX_ERR;
            break;
    }
    if(pr->state==SM_UBX_ERR || pr->state==SM_UBX_BEFORE) {
        return(-1);
    }
    else if(pr->state==SM_UBX_END) {
        pr->state=SM_UBX_BEFORE;
        return(pr->ctr);
    }
    else return(0);
}

/**
 * @brief Checks the UBX packet checksum.
 * @param buf The packet buffer.
 * @param cnt The number of bytes in the buffer.
 * @return 0 if the checksum is correct, -1 otherwise.
 */
int checkUBX(byte *buf, int cnt) {
    byte cha=0, chb=0;
    crcUBX(buf,cnt-2,&cha,&chb);
    if((cha == buf[cnt-2]) && (chb == buf[cnt-1])) return 0;
    return(-1);
}

/**
 * @brief Computes the checksum for a UBX packet.
 * @param buf The buffer containing the packet to checksum.
 * @param cnt The number of bytes in the buffer.
 * @param pcha Pointer to store the first part of the checksum.
 * @param pchb Pointer to store the second part of the checksum.
 */
void crcUBX(byte *buf, int cnt, byte *pcha, byte *pchb) {
    int i=0;
    *pcha=0;
    *pchb=0;
    for(i=0 ; i<cnt ; i++) {
        (*pcha) = (byte)((*pcha) + buf[i]);
        (*pchb) = (byte)((*pchb) + (*pcha));
    }
}

/**
 * @brief Parses a UBX packet and updates GNSS data structure accordingly.
 * @param buf The buffer containing the packet.
 * @param cnt The number of bytes in the buffer.
 * @param cgnss Pointer to the GNSS data structure to update.
 * @return Status of parsing operation.
 */
int parseUBX(byte *buf, int cnt, Cgnss *cgnss) {
    int ok = 0;
    if (buf[0]==UBX_NAV) {
        if (buf[1]==UBX_NAV_PVT && cnt>=92) {
            cgnss->iTOW = bytesToLong(&(buf[4]));
            cgnss->UTCyear = bytesToShort(&(buf[8]));
            cgnss->UTCmonth = (int)buf[10];
            cgnss->UTCday = (int)buf[11];
            cgnss->UTChour = (int)buf[12];
            cgnss->UTCminute = (int)buf[13];
            cgnss->UTCsecond = (int)buf[14];
            cgnss->fixType = (int)buf[24];
            cgnss->hAcc = bytesToLong(&(buf[44]));
            cgnss->vAcc = bytesToLong(&(buf[48]));
            cgnss->pos.lon = bytesToLong(&(buf[28]))*1.0e-7;
            cgnss->pos.lat = bytesToLong(&(buf[32]))*1.0e-7;
            cgnss->pos.alt = bytesToLong(&(buf[36]))*1.0e-7;
        } else if (buf[1]==UBX_NAV_RELPOSNED && cnt>=40) {
            cgnss->relPos.N = bytesToLong(&(buf[12]))+0.01f*(float)buf[24];
            cgnss->relPos.E = bytesToLong(&(buf[16]))+0.01f*(float)buf[25];
            cgnss->relPos.D = bytesToLong(&(buf[20]))+0.01f*(float)buf[26];
        }   
    } else if (buf[0]==UBX_MON) {
        if (buf[1]==UBX_MON_MSGPP && cnt>=120) {
            cgnss->msgs = bytesToShort(&(buf[46]));
        }
    }
    return ok;
}

/**
 * @brief Converts a byte array to a 32-bit integer.
 * @param b Pointer to the byte array.
 * @return The converted 32-bit integer.
 */
int32_t bytesToLong(uint8_t *b) {
    mlong x;
    for (int8_t i=0; i<4 ; i++) {
        x.b[i] = b[i];
    }   
    return(x.i);
}

/**
 * @brief Converts a byte array to a 16-bit short integer.
 * @param b Pointer to the byte array.
 * @return The converted 16-bit short integer.
 */
int16_t bytesToShort(uint8_t *b) {
    mshort x;
    x.b[1] = b[1];
    x.b[0] = b[0];
    return(x.i);
}
