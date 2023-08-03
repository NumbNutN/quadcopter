#include <stdint.h>
#include "os.h"
#include "stm32f4xx.h"

void delay_s(uint32_t delay)
{
    uint32_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC , told = SysTick->VAL, tnow, tcnt = 0;
    while(tcnt < ticks)
    {
        tnow = SysTick->VAL;
        tcnt += (tnow < told ? told-tnow:SysTick->LOAD + told - tnow);
        told = SysTick->VAL;
    }
}

void delay_ms(uint32_t delay)
{
    uint32_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC / 1000, told = SysTick->VAL, tnow, tcnt = 0;
    while(tcnt < ticks)
    {
        tnow = SysTick->VAL;
        tcnt += (tnow < told ? told-tnow:SysTick->LOAD + told - tnow);
        told = SysTick->VAL;
    }
}

void delay_us(uint32_t delay)
{
    uint32_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC / 1000000, told = SysTick->VAL, tnow, tcnt = 0;
    while(tcnt < ticks)
    {
        tnow = SysTick->VAL;
        tcnt += (tnow < told ? told-tnow:SysTick->LOAD + told - tnow);
        told = SysTick->VAL;
    }
}