#ifndef _DELAY_H
#define _DELAY_H

#include <stdint.h>
#include "os.h"
#include "stm32f4xx.h"

#define SYSTICK_CTRL_ENABLE (1u << 0)
#define SYSTICK_CTRL_COUNTFLAG (1u << 16)

/*
记录Systick总计数值
在84M下，时间戳可记录2541714天
*/
#define Get_TimeStamp() ((_count_systick+1) * SysTick->LOAD - SysTick->VAL)

void delay_init();

void delay_s(uint32_t delay);

void delay_ms(uint32_t delay);

#define delay_us(delay) do{ \
    uint64_t ticks = delay*SysTick->LOAD*OS_TICKS_PER_SEC / 1e6, tcnt = 0; \
    uint64_t told = Get_TimeStamp(); \
    while(tcnt < ticks) \
        tcnt = Get_TimeStamp() - told; \
}while(0)

void delay_ns(uint32_t delay);

#endif