#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

#include "util.h"
#include "rtos_globals.h"
#include "state_flash.h"

#include "log.h"

HCD_HandleTypeDef g_hcd_usb_host = {0};
USBH_HandleTypeDef g_usb_host = {0};

static void usb_host_user_process(USBH_HandleTypeDef *phost, uint8_t id);

uint8_t usb_host_init(void) {
    if (USBH_Init(&g_usb_host, usb_host_user_process, HOST_FS) != USBH_OK) {
        return RET_FAILURE;
    }

    if (USBH_RegisterClass(&g_usb_host, USBH_MSC_CLASS) != USBH_OK) {
        return RET_FAILURE;
    }

    if (USBH_Start(&g_usb_host) != USBH_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

void usb_host_process(void) {
    USBH_Process(&g_usb_host);
}

static void usb_host_user_process(USBH_HandleTypeDef *host, uint8_t id) {
    switch(id) {
        case HOST_USER_CLASS_ACTIVE:
            xTaskNotifyIndexed(g_state_flash_task_handle, READY_NOTIFICATION_INDEX, SDCARD_READY_NOTIFICATION_BIT, eSetBits);
            break;
    }
}

void HAL_HCD_MspInit(HCD_HandleTypeDef* hcdHandle) {
    GPIO_InitTypeDef gpio_init = {0};

    if (hcdHandle->Instance == USB_OTG_FS) {
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /*    
            PA9 -> USB_OTG_FS_VBUS
            PA11 -> USB_OTG_FS_DM
            PA12 -> USB_OTG_FS_DP 
        */

        gpio_init.Pin = GPIO_PIN_9;
        gpio_init.Mode = GPIO_MODE_INPUT;
        gpio_init.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio_init);

        gpio_init.Pin = GPIO_PIN_11|GPIO_PIN_12;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &gpio_init);

        HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
}

void OTG_FS_IRQHandler(void) {
    HAL_HCD_IRQHandler(&g_hcd_usb_host);
}

USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef *phost) {
    if (phost->id == HOST_FS) {
        g_hcd_usb_host.pData = phost;
        phost->pData = &g_hcd_usb_host;
        
        g_hcd_usb_host.Instance = USB_OTG_FS;
        g_hcd_usb_host.Init.Host_channels = 8;
        g_hcd_usb_host.Init.speed = HCD_SPEED_FULL;
        g_hcd_usb_host.Init.dma_enable = DISABLE;
        g_hcd_usb_host.Init.phy_itface = HCD_PHY_EMBEDDED;
        g_hcd_usb_host.Init.Sof_enable = DISABLE;

        if (HAL_HCD_Init(&g_hcd_usb_host) != HAL_OK) {
            /* STM's usb library doesnt check the return status so we need to loop here if we fail*/
            log_printf(LOG_ERROR, "Failed to initialize USB HCD");
            __disable_irq();
            while(1);
        }
 
        USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&g_hcd_usb_host));
    }
    
    return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost, uint8_t state) {
    if (phost->id == HOST_FS) {
        GPIO_PinState pin_state = (state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
   
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, pin_state);
    }
 
    HAL_Delay(200);

    return USBH_OK;
}
