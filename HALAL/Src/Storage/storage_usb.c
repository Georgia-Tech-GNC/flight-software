#include "storage.h"
#include "util.h"
#include "halal.h"
#include "usbh_def.h"
#include "usbh_core.h"
#include "usbh_msc.h"
#include "rtos_globals.h"
#include "state_flash.h"
#include "log.h"

HCD_HandleTypeDef hcd_usb_host = {0};
USBH_HandleTypeDef usb_host = {0};

TIM_HandleTypeDef usb_process_tim = {0};

static void usb_host_user_process(USBH_HandleTypeDef *phost, uint8_t id);
static void usb_host_process_task(void *args);

TaskHandle_t usb_process_task_handle;
StackType_t usb_process_task_stack[1024];
StaticTask_t usb_process_task_buff;

uint8_t HALAL_storage_init(void) {
    if (USBH_Init(&usb_host, usb_host_user_process, HOST_FS) != USBH_OK) {
        return RET_FAILURE;
    }

    if (USBH_RegisterClass(&usb_host, USBH_MSC_CLASS) != USBH_OK) {
        return RET_FAILURE;
    }

    if (USBH_Start(&usb_host) != USBH_OK) {
        return RET_FAILURE;
    }
    log_printf(LOG_INFO, "Whee");
    usb_process_task_handle = xTaskCreateStatic(usb_host_process_task, "usb_process", 1024, NULL, tskIDLE_PRIORITY, usb_process_task_stack, &usb_process_task_buff);
    if (usb_process_task_handle == NULL) return RET_FAILURE;
    log_printf(LOG_INFO, "Whee2");

    return RET_SUCCESS;
}

static uint8_t usb_process_tim_init(void) {
    HALAL_STORAGE_USB_TIM_RCC_ENABLE();

    uint32_t tim_clock = HALAL_STORAGE_USB_TIM_CLK_FREQ;

    /* Compute the prescaler value to have TIM2 counter clock equal to 1MHz */
    uint32_t prescaler = (uint32_t) ((tim_clock / 1000000U) - 1U);

    /* Initialize TIM2 */
    usb_process_tim.Instance = HALAL_STORAGE_USB_TIM_INSTANCE;
    usb_process_tim.Init.Period = (1000000U / 1000U) - 1U; /* Period 1ms */
    usb_process_tim.Init.Prescaler = prescaler;
    usb_process_tim.Init.ClockDivision = 0;
    usb_process_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    usb_process_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_NVIC_EnableIRQ(HALAL_STORAGE_USB_TIM_IRQn);
    HAL_NVIC_SetPriority(HALAL_STORAGE_USB_TIM_IRQn, 5, 0);

    if (HAL_TIM_Base_Init(&usb_process_tim) != HAL_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

void usb_host_process_task(void *args) {
    while (1) {
        USBH_Process(&usb_host);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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

        gpio_init.Pin = HALAL_STORAGE_USB_VBUS_PIN;
        gpio_init.Mode = GPIO_MODE_INPUT;
        gpio_init.Pull = GPIO_NOPULL;

        HAL_GPIO_Init(HALAL_STORAGE_USB_VBUS_PORT, &gpio_init);

        gpio_init.Pin = HALAL_STORAGE_USB_DM_PIN;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init.Alternate = GPIO_AF10_OTG_FS;

        HAL_GPIO_Init(HALAL_STORAGE_USB_DM_PORT, &gpio_init);

        gpio_init.Pin = HALAL_STORAGE_USB_DP_PIN;
        
        HAL_GPIO_Init(HALAL_STORAGE_USB_DP_PORT, &gpio_init);

        gpio_init.Pin = HALAL_STORAGE_USB_DM_PIN;
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

        HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
}

void OTG_FS_IRQHandler(void) {
    HAL_HCD_IRQHandler(&hcd_usb_host);
}

USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef *phost) {
    if (phost->id == HOST_FS) {
        hcd_usb_host.pData = phost;
        phost->pData = &hcd_usb_host;
        
        hcd_usb_host.Instance = USB_OTG_FS;
        hcd_usb_host.Init.Host_channels = 8;
        hcd_usb_host.Init.speed = HCD_SPEED_FULL;
        hcd_usb_host.Init.dma_enable = DISABLE;
        hcd_usb_host.Init.phy_itface = HCD_PHY_EMBEDDED;
        hcd_usb_host.Init.Sof_enable = DISABLE;

        if (HAL_HCD_Init(&hcd_usb_host) != HAL_OK) {
            /* STM's usb library doesnt check the return value so we need to loop here if we fail*/
            log_printf(LOG_ERROR, "Failed to initialize USB HCD");
            __disable_irq();
            while(1);
        }
 
        USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&hcd_usb_host));
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

void HALAL_STORAGE_USB_TIM_ISR() {
    HAL_TIM_IRQHandler(&usb_process_tim);
}