#include "quaternion.hpp"
#include "mpu6050.hpp"

#include "os.h"
#include "delay.h"
#include "myMath.hpp"

#include <math.h>


#if TEST_RK4_EN > 0u

#define TASK_RK4_PRIO 9u
#define TASK_RK4_PRINT_PRIO 10u

uint64_t __euler_angle_told = 0;

quaternion last_gyro;
quaternion cur_gyro;
quaternion last_attitude{1,0,0,0};
double gyro_sampling_duty;

OS_STK Stk_RK4[1024];
OS_STK Stk_Rk4_Print[1024];

quaternion get_m_omega(double theta)
{
    return (1 -theta)* last_gyro + theta* cur_gyro;
}

/* quaternion 导数是一个关于t和q的函数 f(t,q)*/
quaternion quaternion_derivative(double t,quaternion q)
{
    return 0.5*q*get_m_omega(t/gyro_sampling_duty);
}

void print_quaternion2EulerAngle(quaternion& q)
{
    printf("X:%lf\nY:%lf\nZ:%lf\n\n",-asin(2*q[1]*q[3]-2*q[0]*q[2]), \
        atan((2*q[0]*q[1] + 2*q[2]*q[3]) / (1-2*q[1]*q[1] - 2*q[2]*q[2])), \
        atan((2*q[1]*q[2] + 2*q[0]*q[3]) / (1-2*q[2]*q[2] - 2*q[3]*q[3])) );
}

void TEST_Task_RK1_Attitude_Calculate(void* arg){

    mpu6050 mpu6050_dev;
    for(;;){
        mpu6050_dev.update();
        if(__euler_angle_told == 0)
        {
            __euler_angle_told = Get_TimeStamp();
            last_gyro = mpu6050_dev.get_gyro();
            OSTimeDlyHMSM(0, 0, 0, 100);
        }

        cur_gyro = mpu6050_dev.get_gyro();
        gyro_sampling_duty = ((double)Get_TimeStamp()-__euler_angle_told) / HAL_RCC_GetSysClockFreq();
        last_attitude = RK1(0,
                            last_attitude, 
                            gyro_sampling_duty,
                            quaternion_derivative);
        last_gyro = cur_gyro;
        __euler_angle_told = Get_TimeStamp();
        last_attitude.normalization();
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void TEST_Task_Attitude(void* arg){
    for(;;)
    {
        cout << last_attitude << endl;
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

void TEST_RK4_Init()
{
    OSTaskCreate(TEST_Task_RK1_Attitude_Calculate, (void*)0, &Stk_RK4[1023], TASK_RK4_PRIO);
    OSTaskCreate(TEST_Task_Attitude, NULL, &Stk_Rk4_Print[1023], TASK_RK4_PRINT_PRIO);
}

#endif      