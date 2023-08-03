#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>
#include "config.h"

#define PASTE2(x,y) x##y
#define PASTE(x,y) PASTE2(x,y)
#define PREFIX(preStr) PASTE(preStr,_)
#define FUNC(pre,func) PASTE(pre,func)

#define TEST_PREFIX PREFIX(TEST)


#ifdef I2C_USE_MX
    #define I2C_HAL_INTERFACE MX
#elif defined I2C_USE_SOFT
    #define I2C_HAL_INTERFACE SOFT
#elif defined I2C_USE_HARD
    #define I2C_HAL_INTERFACE HARD
#else
    #error "you should define I2C interface"
#endif

#define I2C_PREFIX PREFIX(I2C_HAL_INTERFACE)

#define I2C_Init FUNC(I2C_PREFIX,I2C_Init)
#define I2C_Write FUNC(I2C_PREFIX,I2C_Write)
#define I2C_Read FUNC(I2C_PREFIX,I2C_Read)
#define I2C_Transfer FUNC(I2C_PREFIX,I2C_Transfer)


#endif