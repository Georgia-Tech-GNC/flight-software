#include "usbh_diskio.h"
#include "usb_host.h"

#include "usbh_core.h"
#include "usbh_msc.h"

#define USB_DEFAULT_BLOCK_SIZE 512

DSTATUS USBH_initialize(BYTE);
DSTATUS USBH_status(BYTE);
DRESULT USBH_read(BYTE, BYTE*, DWORD, UINT);
DRESULT USBH_write(BYTE, const BYTE*, DWORD, UINT);
DRESULT USBH_ioctl(BYTE, BYTE, void*);

const Diskio_drvTypeDef usbh_diskio_driver = {
    USBH_initialize,
    USBH_status,
    USBH_read,
    USBH_write,
    USBH_ioctl
};

DSTATUS USBH_initialize(BYTE lun) {
    return RES_OK;
}

DSTATUS USBH_status(BYTE lun) {
    DRESULT res = RES_ERROR;

    if (USBH_MSC_UnitIsReady(&g_usb_host, lun)) {
        res = RES_OK;
    } else {
        res = RES_ERROR;
    }

    return res;
}

DRESULT USBH_read(BYTE lun, BYTE *buff, DWORD sector, UINT count) {
    DRESULT res = RES_ERROR;
    MSC_LUNTypeDef info;

    if (USBH_MSC_Read(&g_usb_host, lun, sector, buff, count) == USBH_OK) {
        res = RES_OK;
    } else {
        USBH_MSC_GetLUNInfo(&g_usb_host, lun, &info);
        
        switch (info.sense.asc) {
            case SCSI_ASC_LOGICAL_UNIT_NOT_READY:
            case SCSI_ASC_MEDIUM_NOT_PRESENT:
            case SCSI_ASC_NOT_READY_TO_READY_CHANGE:
                res = RES_NOTRDY;
                break; 
            default:
                res = RES_ERROR;
                break;
        }
    }

    return res;
}

DRESULT USBH_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count) {
    DRESULT res = RES_ERROR;
    MSC_LUNTypeDef info;

    if(USBH_MSC_Write(&g_usb_host, lun, sector, (BYTE *)buff, count) == USBH_OK) {
        res = RES_OK;
    } else {
        USBH_MSC_GetLUNInfo(&g_usb_host, lun, &info);

        switch (info.sense.asc) {
            case SCSI_ASC_WRITE_PROTECTED:
                res = RES_WRPRT;
                break;
            case SCSI_ASC_LOGICAL_UNIT_NOT_READY:
            case SCSI_ASC_MEDIUM_NOT_PRESENT:
            case SCSI_ASC_NOT_READY_TO_READY_CHANGE:
                res = RES_NOTRDY;
                break;
            default:
                res = RES_ERROR;
                break;
        }
    }

    return res;
}

DRESULT USBH_ioctl(BYTE lun, BYTE cmd, void *buff) {
    DRESULT res = RES_ERROR;
    MSC_LUNTypeDef info;

    switch (cmd) {
        case CTRL_SYNC:
            res = RES_OK;
            break;
        case GET_SECTOR_COUNT:
            if(USBH_MSC_GetLUNInfo(&g_usb_host, lun, &info) == USBH_OK) {
                *(DWORD*)buff = info.capacity.block_nbr;
                res = RES_OK;
            } else {
                res = RES_ERROR;
            }

            break;
        case GET_SECTOR_SIZE:
            if(USBH_MSC_GetLUNInfo(&g_usb_host, lun, &info) == USBH_OK) {
                *(DWORD*)buff = info.capacity.block_size;
                res = RES_OK;
            } else {
                res = RES_ERROR;
            }

            break;
        case GET_BLOCK_SIZE:
            if(USBH_MSC_GetLUNInfo(&g_usb_host, lun, &info) == USBH_OK) {
                *(DWORD*)buff = info.capacity.block_size / USB_DEFAULT_BLOCK_SIZE;
                res = RES_OK;
            } else {
                res = RES_ERROR;
            }

            break;
        default:
            res = RES_PARERR;
    }

    return res;
}

