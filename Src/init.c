#include <stdint.h>

#include "os.h"
#include "stm32f4xx.h"
#include "test_tasks.h"

#include "i2c.h"
#include "mpu6050.h"
#include "ssd1306_i2c.h"



void Task_Init(void){

    //TASK_BEFORE_OS_INIT

}

/**
 * @brief Initial the peripherals hardware 
*/
extern "C" void PeripheralsInit(void){

    I2C_Init((uint32_t)I2C1);
}

/**
 * @brief Initial the system clock frequency to fit in ucOS ticks
*/
extern "C" void SysClkInit()
{
    SysTick->LOAD = HAL_RCC_GetSysClockFreq() / OS_TICKS_PER_SEC;
}

void Test_Task_Init(void){
    Test_PID_Init();
    Test_Motor_Init();
    Test_Madgwick_Init();
    Test_Mahony_Init();
    Test_Anotc_Conn_Init();
    Test_Print_Init();
}

void Test_Bf_OS(void){
#if TEST_SHELL_EN > 0u
    TEST_shell();
#endif
    PeripheralsInit();
    SSD1306_Init();
    OLED_Clean();
    TEST_Fs();
}
