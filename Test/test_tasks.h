#include "os.h"
#include "common.h"
#include "config.h"

#if TEST_I2C_MPU6050_EN > 0u
    #define Test_Task_MPU6050_Get_Data_Init FUNC(TEST_PREFIX,Task_MPU6050_Get_Data_Init)

    #define TASK_MPU6050_GET_DATA_STACK_SIZE 256
    #define TASK_MPU6050_GET_DATA_PRIO 10u

    extern OS_STK Stk_MPU6050_Get_Data[TASK_MPU6050_GET_DATA_STACK_SIZE];
#else
    #define Test_Task_MPU6050_Get_Data_Init do{}while(0)
#endif

/**
* @brief gy-86数据采集
*/
extern void Test_Task_MPU6050_Get_Data_Init(void);

extern void TEST_SSD1306_PutChar(void);