#include "delay.h"

#include <stdint.h>

/* 
SystickInterupt计数器
在SystickIrq 陷入100次/s时，计数器可计数497天
*/
uint32_t Get_Systick_Cnt()
{
    return _count_systick;
}

void delay_s(uint32_t delay)
{
    uint64_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC, tcnt = 0;
    uint64_t told = Get_TimeStamp();
    while(tcnt < ticks)
        tcnt = Get_TimeStamp() - told;
}

void delay_ms(uint32_t delay)
{
    uint64_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC / 1e3, tcnt = 0;
    uint64_t told = Get_TimeStamp();
    while(tcnt < ticks)
        tcnt = Get_TimeStamp() - told;
}

void delay_ns(uint32_t delay) {
    static float k = SysTick->LOAD*OS_TICKS_PER_SEC / 1e9;
    uint64_t ticks = delay*k, tcnt = 0;
    uint64_t told = Get_TimeStamp();
    while(tcnt < ticks)
        tcnt = Get_TimeStamp() - told;
}