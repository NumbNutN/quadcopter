#include "config.h"

#if TEST_PRINT_ACCEL_EN > 0u

#define TASK_PRINT_ACCEL_PRIO 9u

#include "usart.h"
#include "os.h"

#include "mpu6050.hpp"

#include <iostream>

using namespace std;

OS_STK Stk_Print_Accel[1024];

void TEST_Task_Print_Accel(void* arg){
    mpu6050 mpu6050_dev;
    for(;;)
    {
        mpu6050_dev.update();
        quaternion accel = mpu6050_dev.get_accelero();
        // printf("%lf %lf\n",accel[0],accel[1]);
        cout << accel.normalization() << endl;
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

void TEST_Print_Accel_Init()
{
    OSTaskCreate(TEST_Task_Print_Accel, (void*)0, &Stk_Print_Accel[1023], TASK_PRINT_ACCEL_PRIO);
}

#endif