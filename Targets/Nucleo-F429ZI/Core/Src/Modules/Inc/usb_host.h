#ifndef USB_HOST_H
#define USB_HOST_H

#include "stdint.h"

#include "usbh_def.h"

extern USBH_HandleTypeDef g_usb_host;

uint8_t usb_host_init(void);
void usb_host_process(void);

void usb_host_drive_vbus(uint8_t state);

#endif