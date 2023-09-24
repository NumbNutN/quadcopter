#ifndef _DELAY_H
#define _DELAY_H

#include <stdint.h>

#define SYSTICK_CTRL_ENABLE (1u << 0)
#define SYSTICK_CTRL_COUNTFLAG (1u << 16)

uint64_t Get_TimeStamp();

void delay_init();

void delay_s(uint32_t delay);

void delay_ms(uint32_t delay);

void delay_us(uint32_t delay);

#endif