#include "delay.h"

#include <stdint.h>
#include "os.h"
#include "stm32f4xx.h"

/* 
SystickInterupt计数器
在SystickIrq 陷入100次/s时，计数器可计数497天
*/
uint32_t Get_Systick_Cnt()
{
    return _count_systick;
}

/*
记录Systick总计数值
在84M下，时间戳可记录2541714天
*/
uint64_t Get_TimeStamp()
{
    return (_count_systick+1) * SysTick->LOAD - SysTick->VAL;
}

void delay_init()
{
    SysTick->LOAD = HAL_RCC_GetSysClockFreq() / OS_TICKS_PER_SEC;
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
    uint64_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC / 1000, tcnt = 0;
    uint64_t told = Get_TimeStamp();
    while(tcnt < ticks)
        tcnt = Get_TimeStamp() - told;
}

void delay_us(uint32_t delay)
{
    uint64_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC / 1000000, tcnt = 0;
    uint64_t told = Get_TimeStamp();
    while(tcnt < ticks)
        tcnt = Get_TimeStamp() - told;
}