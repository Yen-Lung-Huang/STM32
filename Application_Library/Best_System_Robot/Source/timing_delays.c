#include "timing_delays.h"

/* Non-blocking delay functions -----------------------------------------------*/
// Start the non-blocking delay
void NonBlockingDelay_Start(NonBlockingDelay_TypeDef* self, uint32_t duration)
{
    self->start_time = HAL_GetTick();
    self->delay = duration;
}

// Check if the non-blocking delay has expired
bool NonBlockingDelay_IsExpired(NonBlockingDelay_TypeDef* self)
{
    return (HAL_GetTick() - self->start_time) >= self->delay;
}

// Create a new non-blocking delay object
NonBlockingDelay_TypeDef CreateNonBlockingDelay()
{
    NonBlockingDelay_TypeDef nbd_obj;
    nbd_obj.start_time = 0;
    nbd_obj.delay = 0;
    nbd_obj.Start = NonBlockingDelay_Start;
    nbd_obj.IsExpired = NonBlockingDelay_IsExpired;
    return nbd_obj;
}

/* Microseconds delay functions -----------------------------------------------*/
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < us);
}

// Utility function to check if SysTick counter flag is active
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

// Function to get current time in microseconds
uint32_t getCurrentMicros(void)
{
    // Ensure COUNTFLAG is reset by reading SysTick control and status register
    LL_SYSTICK_IsActiveCounterFlag();
    uint32_t m = HAL_GetTick();
    const uint32_t tms = SysTick->LOAD + 1;
    __IO uint32_t u = tms - SysTick->VAL;
    if (LL_SYSTICK_IsActiveCounterFlag()) {
        m = HAL_GetTick();
        u = tms - SysTick->VAL;
    }
    return (m * 1000 + (u * 1000) / tms);
}
