#include "os.h"
#include "common.h"
#include "config.h"

#if TEST_I2C_MPU6050_EN > 0u
    #define Test_Task_MPU6050_Get_Data_Init FUNC(TEST_PREFIX,Task_MPU6050_Get_Data_Init)

    #define TASK_MPU6050_GET_DATA_STACK_SIZE 1024

    #define TASK_MPU6050_GET_DATA_PRIO 10u
    #define TASK_COUNT_EULERANGLE_PRIO 11u
    #define TASK_PRINT_EULERANGLE_PRIO 9u

    extern OS_STK Stk_MPU6050_Get_Data[TASK_MPU6050_GET_DATA_STACK_SIZE];
#else
    #define Test_Task_MPU6050_Get_Data_Init do{}while(0)
#endif

#if TEST_CPP_EN > 0u
    #define test_cpp TEST_cpp
#else
    #define test_cpp() do{}while(0)
#endif

/**
* @brief gy-86数据采集
*/
extern void TEST_Task_MPU6050_Get_Data_Init(void);

extern void TEST_SSD1306_PutChar(void);

extern void TEST_cpp(void);