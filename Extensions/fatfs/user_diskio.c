/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
   
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function weak definitions prototypes -----------------------------------------------*/

DSTATUS __attribute__((weak)) USER_initialize (BYTE pdrv) {
    UNUSED(pdrv);
    return 1;
}

DSTATUS __attribute__((weak)) USER_status (BYTE pdrv) {
    UNUSED(pdrv);
    return 1;
}

DRESULT __attribute__((weak)) USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    UNUSED(pdrv);
    UNUSED(buff);
    UNUSED(sector);
    UNUSED(count);
    return 1;
}

#if _USE_WRITE == 1
    DRESULT __attribute__((weak)) USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
        UNUSED(pdrv);
        UNUSED(buff);
        UNUSED(sector);
        UNUSED(count);
        return 1;
    }
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
    DRESULT __attribute__((weak)) USER_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
        UNUSED(pdrv);
        UNUSED(cmd);
        UNUSED(buff);
        return 1;
    }
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};
