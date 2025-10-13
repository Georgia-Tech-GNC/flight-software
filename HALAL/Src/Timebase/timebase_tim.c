#include "timebase.h"
#include "halal.h"
#include "util.h"
#include "tim_isr.h"

static TIM_HandleTypeDef tim = {0};

uint8_t HALAL_timebase_init(uint32_t tick_priority) {  
    /*Configure the TIM IRQ priority */
    if (tick_priority < (1UL << __NVIC_PRIO_BITS)) {
        HAL_NVIC_SetPriority(HALAL_TIMEBASE_TIM_IRQn, tick_priority ,0U);
  
        /* Enable the TIM global Interrupt */
        HAL_NVIC_EnableIRQ(HALAL_TIMEBASE_TIM_IRQn);
        uwTickPrio = tick_priority;
    } else {
        return RET_FAILURE;
    }
  
    /* Enable TIM clock */
    HALAL_TIMEBASE_TIM_RCC_EN();
  
    RCC_ClkInitTypeDef clk_init;
    uint32_t dummy;

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clk_init, &dummy);
  
    /* Compute TIM clock */
    uint32_t tim_clock = 2*HAL_RCC_GetPCLK2Freq();
  
    /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
    uint32_t prescaler = (uint32_t) ((tim_clock / 1000000U) - 1U);
  
    /* Initialize TIM */
    tim.Instance = HALAL_TIMEBASE_TIM_INSTANCE;
  
    /* Initialize TIMx peripheral as follow:
  
    + Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
    + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
    + ClockDivision = 0
    + Counter direction = Up
    */
    tim.Init.Period = (1000000U / 1000U) - 1U;
    tim.Init.Prescaler = prescaler;
    tim.Init.ClockDivision = 0;
    tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  
    if(HAL_TIM_Base_Init(&tim) == HAL_OK) {
        /* Start the TIM time Base generation in interrupt mode */
        return HAL_TIM_Base_Start_IT(&tim) == HAL_OK ? RET_SUCCESS : RET_FAILURE;
    }
  
    /* Return function status */
    return RET_FAILURE;
}

void timebase_tim_period_elapsed(void) {
    HAL_IncTick();
}

uint8_t HALAL_timebase_suspend(void) {
    __HAL_TIM_DISABLE_IT(&tim, TIM_IT_UPDATE);
    return RET_SUCCESS;
}

uint8_t HALAL_timebase_resume(void) {
    __HAL_TIM_ENABLE_IT(&tim, TIM_IT_UPDATE);
    return RET_SUCCESS;
}

void HALAL_TIMEBASE_TIM_ISR(void) {
    HAL_TIM_IRQHandler(&tim);
}
