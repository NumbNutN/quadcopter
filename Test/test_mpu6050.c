#include "test_tasks.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "os.h"
#include "mpu6050.h"
#include "quat_math.h"
#include "delay.h"


OS_STK Stk_MPU6050_Get_Data[TASK_MPU6050_GET_DATA_STACK_SIZE];
OS_STK Stk_MPU6050_Print_Data[TASK_MPU6050_GET_DATA_STACK_SIZE];
OS_STK Stk_Count_EulerAngle[TASK_MPU6050_GET_DATA_STACK_SIZE];
OS_STK Stk_Print_EulerAngle[TASK_MPU6050_GET_DATA_STACK_SIZE];

uint8_t mpu6050_pos[6] = {0x3B,0x3C,0x3D,0x3E,0x3F,0x40};
double mpu6050_gyro[3];

/**
* @brief gy-86数据采集
* @brith: 2023-6-30
*/
void TEST_Task_MPU6050_Get_Data(void* arg)
{
    for(;;)
    {
        MPU6050_Get_Gyroscope(&mpu6050_gyro[0],&mpu6050_gyro[1],&mpu6050_gyro[2]);
        OSTimeDlyHMSM(0, 0,0,100);
    }
}

void TEST_Task_MPU6050_Print_Data(void* arg)
{
    for(;;)
    {
        printf("pitch:%.2f\n",mpu6050_gyro[0]);
        printf("yaw:%.2f\n",mpu6050_gyro[1]);
        printf("roll:%.2f\n\n",mpu6050_gyro[2]);
        OSTimeDlyHMSM(0, 0,2,0);
    }
}

void TEST_Task_Count_EulerAngle(void* arg)
{
    double newquat[4];
    uint64_t told = Get_TimeStamp();
    for(;;)
    {
        //使用一阶龙塔图求积分
        Runge_Kutta_1st(newquat,cur_quat,mpu6050_gyro,((double)Get_TimeStamp()-_euler_angle_told) / HAL_RCC_GetSysClockFreq());
        //更新新的四元数
        memcpy(cur_quat, newquat,sizeof(double)*4);
        OSTimeDlyHMSM(0, 0,0,500);
    }
}

void TEST_Task_Print_Euler_Angle(void* arg)
{
    double x,y,z;
    for(;;)
    {
        quat2eulerAngle_zyx(cur_quat,&x,&y,&z);
        printf("pitch:%.2f\n",x);
        printf("yaw:%.2f\n",y);
        printf("roll:%.2f\n\n",z);
        OSTimeDlyHMSM(0, 0,2,0);
    }
}


void TEST_Task_MPU6050_Get_Data_Init()
{
    MPU6050_Init();
    OSTaskCreate(TEST_Task_MPU6050_Get_Data, (void*)0, &Stk_MPU6050_Get_Data[TASK_MPU6050_GET_DATA_STACK_SIZE - 1], TASK_MPU6050_GET_DATA_PRIO);
    //OSTaskCreate(TEST_Task_MPU6050_Print_Data, (void*)0, &Stk_MPU6050_Print_Data[TASK_MPU6050_GET_DATA_STACK_SIZE - 1], TASK_MPU6050_GET_PRINT_PRIO);
    OSTaskCreate(TEST_Task_Count_EulerAngle, (void*)0, &Stk_Count_EulerAngle[TASK_MPU6050_GET_DATA_STACK_SIZE - 1], TASK_COUNT_EULERANGLE_PRIO);
    OSTaskCreate(TEST_Task_Print_Euler_Angle, (void*)0, &Stk_Print_EulerAngle[TASK_MPU6050_GET_DATA_STACK_SIZE - 1], TASK_PRINT_EULERANGLE_PRIO);
}