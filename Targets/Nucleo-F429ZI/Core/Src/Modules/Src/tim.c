#include "tim.h"

TIM_HandleTypeDef g_tim1;

void TIM1_UP_TIM10_IRQHandler(void) {
    HAL_TIM_IRQHandler(&g_tim1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
}


HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    uint32_t tim_clock = 2*HAL_RCC_GetPCLK2Freq();

    /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
    uint32_t prescaler = (uint32_t) ((tim_clock / 1000000U) - 1U);

    /* Initialize TIM1 */
    g_tim1.Instance = TIM1;
    g_tim1.Init.Period = (1000000U / 1000U) - 1U; /* Period 1ms */
    g_tim1.Init.Prescaler = prescaler;
    g_tim1.Init.ClockDivision = 0;
    g_tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&g_tim1) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, TickPriority, 0U);

    if (HAL_TIM_Base_Start_IT(&g_tim1) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HAL_SuspendTick(void)
{
  /* Disable TIM1 update Interrupt */
  __HAL_TIM_DISABLE_IT(&g_tim1, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  /* Enable TIM1 Update interrupt */
  __HAL_TIM_ENABLE_IT(&g_tim1, TIM_IT_UPDATE);
}

