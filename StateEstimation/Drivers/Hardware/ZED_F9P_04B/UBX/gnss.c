
/**
 ******************************************************************************
* @file    gnss.c
* @author  Kanav Chugh
* @brief   ZED-F9P source file
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/

#include "gnss.h"

uint8_t rx_string[MAX_GNSS];
int rx_index = 0;
uint8_t rx_buffer_gnss = 0;
t_gnss_rx *GNSSrx;

void handle_gnss(CGNSS *cgnss) {
    uint8_t msg_buf[MAX_GNSS];
    int32_t msg_cnt = read_ubx_pkt(msg_buf);
    if (msg_cnt > 0) {
        events_comm_gnss(cgnss, msg_buf, msg_cnt);
    } else if (msg_cnt == -1) {
        init_gnss_rx();
    }
}

void events_comm_gnss(CGNSS *cgnss, uint8_t *msg_buf, int32_t cnt) {
    parse_ubx(cgnss, msg_buf, cnt);
}

int read_ubx_pkt(byte *ret_buf) {
    int i = 0;
    if (GNSSrx->ctr < MAX_GNSS) {
        if (add_ubx_pkt_byte(rx_buffer_gnss, GNSSrx) > 0) {
            GNSSrx->state = SM_UBX_BEFORE;
            for (i = 0; i < GNSSrx->ctr; i++) {
                ret_buf[i] = GNSSrx->buf[i];
            }
            GNSSrx->ctr = 0;
            if (check_ubx(ret_buf, i) == 0) {
                return i - 2;
            } else {
                return 0;
            }
        }
    }
    if (GNSSrx->ctr >= MAX_GNSS) {
        GNSSrx->ctr = 0;
        GNSSrx->state = SM_UBX_BEFORE;
    }
    return 0;
}

void init_gnss_rx(void) {
    GNSSrx->state = SM_UBX_BEFORE;
    GNSSrx->ctr = 0;
}

int add_ubx_pkt_byte(byte ch, t_gnss_rx *pr) {
	switch (pr->state) {
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
            else if(pr->ctr >= (256-10)) pr->state=SM_UBX_ERR;
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
    if (pr->state == SM_UBX_ERR || pr->state == SM_UBX_BEFORE) {
        return -1;
    } else if (pr->state == SM_UBX_END) {
        pr->state = SM_UBX_BEFORE;
        return pr->ctr;
    } else {
        return 0;
    }
}

int check_ubx(byte *buf, int cnt) {
    byte cha = 0, chb = 0;
    crc_ubx(buf, cnt - 2, &cha, &chb);
    if ((cha == buf[cnt - 2]) && (chb == buf[cnt - 1])) {
        return 0;
    }
    return -1;
}

bool parse_ubx(CGNSS* cgnss, byte *buf, int cnt) {
    bool ok = false;
    if (buf[0] == UBX_NAV) {
        if (buf[1] == UBX_NAV_PVT && cnt >= 92) {
            cgnss->i_tow = bytes_to_long(&buf[4]);
            cgnss->UTC_year = bytes_to_short(&buf[8]);
            cgnss->UTC_month = (int) buf[10];
            cgnss->UTC_day = (int) buf[11];
            cgnss->UTC_hour = (int) buf[12];
            cgnss->UTC_minute = (int) buf[13];
            cgnss->UTC_second = (int) buf[14];
            cgnss->fix_type = (int) buf[24];
            cgnss->h_acc = bytes_to_long(&buf[44]);
            cgnss->v_acc = bytes_to_long(&buf[48]);
            cgnss->pos->lon = bytes_to_long(&buf[28]) * 1.0e-7;
            cgnss->pos->lat = bytes_to_long(&buf[32]) * 1.0e-7;
            cgnss->pos->alt = bytes_to_long(&buf[36]) * 1.0e-7;
        } else if (buf[1] == UBX_NAV_RELPOSNED && cnt >= 40) {
            cgnss->rel_pos->N = bytes_to_long(&buf[12]) + 0.01f * (float) buf[24];
            cgnss->rel_pos->E = bytes_to_long(&buf[16]) + 0.01f * (float) buf[25];
            cgnss->rel_pos->D = bytes_to_long(&buf[20]) + 0.01f * (float) buf[26];
        } else if (buf[0] == UBX_MON) {
            if (buf[1] == UBX_MON_MSGPP && cnt >= 120) {
                cgnss->msgs = bytes_to_short(&buf[46]);
            }
        }
        return ok;
    }
}

int32_t bytes_to_long(uint8_t *b) {
    int8_t i;
    m_long x;
    for (i = 0; i < 4; i++) {
        x.b[i] = b[i];
    }
    return x.i;
}

int16_t bytes_to_short(uint8_t *b) {
    m_short x;
    x.b[1] = b[1];
    x.b[0] = b[0];
    return x.i;
}

void event_sims(CGNSS *cgnss) {
    static GPIO_PinState state_led = GPIO_PIN_RESET;
    static float counter_led = 0.0f;
    static position_state pos_state = OK;
    float MAX_ERROR = 10.0f;
    float track_error_cm = cgnss->rel_pos->E;
    counter_led++;
    switch (state_led) {
        case 0:
            if (counter_led > 100) {
                state_led = GPIO_PIN_SET;
            }
            break;
        case 1:
            if (counter_led > fabs(track_error_cm)) {
                state_led = GPIO_PIN_RESET;
                counter_led = 0;
            }
    }
    if (fabs(track_error_cm) <= MAX_ERROR) {
        pos_state = OK;
    } else if (track_error_cm > MAX_ERROR) {
        pos_state = TOO_RIGHT;
    } else {
        pos_state = TOO_LEFT;
    }
    /*
    only for eval board
    switch (pos_state) {
        case OK:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            break;
        case TOO_LEFT:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, state_led);
            break;
        case TOO_RIGHT:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state_led);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            break;
    }
    */

}


Pos* get_pos(CGNSS *gnss) {
    return gnss->pos;
}

Pos_Rel* get_rel_pos(CGNSS *gnss) {
    return gnss->rel_pos;
}

int get_fix_type(CGNSS *gnss) {
    return gnss->fix_type;
}

float get_h_acc(CGNSS *gnss) {
    return gnss->h_acc;
}

float get_v_acc(CGNSS *gnss) {
    return gnss->v_acc;
}

float get_i_tow(CGNSS *gnss) {
    return gnss->i_tow;
}

int get_UTC_year(CGNSS *gnss) {
    return gnss->UTC_year;
}

int get_UTC_month(CGNSS *gnss) {
    return gnss->UTC_month;
}

int get_UTC_day(CGNSS *gnss) {
    return gnss->UTC_day;
}

int get_UTC_hour(CGNSS *gnss) {
    return gnss->UTC_hour;
}

int get_UTC_minute(CGNSS *gnss) {
    return gnss->UTC_minute;
}

int get_UTC_second(CGNSS *gnss) {
    return gnss->UTC_second;
}

int get_msgs(CGNSS *gnss) {
    return gnss->msgs;
}
