#include <stdint.h>

#include "os.h"
#include "mpu6050.h"
#include "test_tasks.h"

OS_STK Stk_MPU6050_Get_Data[TASK_MPU6050_GET_DATA_STACK_SIZE];

uint8_t mpu6050_pos[6] = {0x3B,0x3C,0x3D,0x3E,0x3F,0x40};
uint8_t mpu6050_buf[6];

/**
* @brief gy-86数据采集
* @brith: 2023-6-30
*/
void TEST_Task_MPU6050_Get_Data(void* arg)
{
    float x,y,z;
    for(;;)
    {
        MPU6050_Get_Gyroscope(&x,&y,&z);
        OSTimeDlyHMSM(0, 0,0,100);
    }
}


void TEST_Task_MPU6050_Get_Data_Init()
{
    MPU6050_Init();
    OSTaskCreate(TEST_Task_MPU6050_Get_Data, (void*)0, &Stk_MPU6050_Get_Data[TASK_MPU6050_GET_DATA_STACK_SIZE - 1], TASK_MPU6050_GET_DATA_PRIO);
}