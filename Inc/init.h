#ifndef _APP_H
#define _APP_H

#include "os.h"

void Task_Init(void);
void Test_Task_Init(void);

/**
 * @brief Initial the peripherals hardware 
*/
void PeripheralsInit(void);

/**
 * @brief Initial the system clock frequency to fit in ucOS ticks
*/
void SysClkInit(void);

void Test_Bf_OS(void);

#endif