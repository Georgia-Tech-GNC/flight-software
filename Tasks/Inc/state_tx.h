#ifndef STATE_TX_H
#define STATE_TX_H

#define BEGIN_STATE_TX_NOTIFICATION_BIT 0x01
#define SEND_STATE_NOTIFICATION_BIT 0x01

void state_tx_task(void *args);

#endif