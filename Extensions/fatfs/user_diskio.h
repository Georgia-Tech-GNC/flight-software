/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DISKIO_H
#define __USER_DISKIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* USER CODE BEGIN 0 */

extern Diskio_drvTypeDef  USER_Driver;

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (BYTE pdrv);

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (BYTE pdrv);

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
    /**
     * @brief  Writes Sector(s)
     * @param  pdrv: Physical drive number (0..)
     * @param  *buff: Data to be written
     * @param  sector: Sector address (LBA)
     * @param  count: Number of sectors to write (1..128)
     * @retval DRESULT: Operation result
     */
    DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
    /**
     * @brief  I/O control operation
     * @param  pdrv: Physical drive number (0..)
     * @param  cmd: Control code
     * @param  *buff: Buffer to send/receive control data
     * @retval DRESULT: Operation result
     */
    DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */


/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISKIO_H */
