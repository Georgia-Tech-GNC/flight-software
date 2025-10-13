#include "storage.h"

#include "ff_gen_drv.h"

#include "usbh_core.h"
#include "usbh_msc.h"

#define USB_DEFAULT_BLOCK_SIZE 512

extern USBH_HandleTypeDef usb_host;

DSTATUS HALAL_storage_initialize(BYTE lun) {
    UNUSED(lun);

    return RES_OK;
}

DSTATUS HALAL_storage_status(BYTE lun) {
    DRESULT res = RES_ERROR;

    if (USBH_MSC_UnitIsReady(&usb_host, lun)) {
        res = RES_OK;
    } else {
        res = RES_ERROR;
    }

    return res;
}

DRESULT HALAL_storage_read(BYTE lun, BYTE *buff, DWORD sector, UINT count) {
    DRESULT res = RES_ERROR;
    MSC_LUNTypeDef info;

    if (USBH_MSC_Read(&usb_host, lun, sector, buff, count) == USBH_OK) {
        res = RES_OK;
    } else {
        USBH_MSC_GetLUNInfo(&usb_host, lun, &info);
        
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

DRESULT HALAL_storage_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count) {
    DRESULT res = RES_ERROR;
    MSC_LUNTypeDef info;

    if(USBH_MSC_Write(&usb_host, lun, sector, (BYTE *)buff, count) == USBH_OK) {
        res = RES_OK;
    } else {
        USBH_MSC_GetLUNInfo(&usb_host, lun, &info);

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

DRESULT HALAL_storage_ioctl(BYTE lun, BYTE cmd, void *buff) {
    DRESULT res = RES_ERROR;
    MSC_LUNTypeDef info;

    switch (cmd) {
        case CTRL_SYNC:
            res = RES_OK;
            break;
        case GET_SECTOR_COUNT:
            if(USBH_MSC_GetLUNInfo(&usb_host, lun, &info) == USBH_OK) {
                *(DWORD*)buff = info.capacity.block_nbr;
                res = RES_OK;
            } else {
                res = RES_ERROR;
            }

            break;
        case GET_SECTOR_SIZE:
            if(USBH_MSC_GetLUNInfo(&usb_host, lun, &info) == USBH_OK) {
                *(DWORD*)buff = info.capacity.block_size;
                res = RES_OK;
            } else {
                res = RES_ERROR;
            }

            break;
        case GET_BLOCK_SIZE:
            if(USBH_MSC_GetLUNInfo(&usb_host, lun, &info) == USBH_OK) {
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

