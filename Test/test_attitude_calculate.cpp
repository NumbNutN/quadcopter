#include <iostream>

#include "quaternion.hpp"
#include "mpu6050.hpp"
#include "myMath.hpp"
#include "gradient_decent.hpp"

#include <math.h>

#include "os.h"
#include "delay.h"


using namespace std;


#if TEST_RK4_EN > 0u

#define TASK_RK4_PRIO 9u
#define TASK_RK4_PRINT_PRIO 10u

uint64_t __euler_angle_told = 0;

quaternion last_gyro;
quaternion cur_gyro;
quaternion cur_accel;
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

/* 梯度下降 */
quaternion error_function_gradient(const quaternion& q){

    return quaternion{
        8*q[2]*q[2]*q[0] + 8*q[1]*q[1]*q[0] + 4*cur_accel[1]*q[2] - 4*cur_accel[2]*q[1],
        8*q[3]*q[3]*q[1] + 8*q[0]*q[0]*q[1] + 16*q[2]*q[2]*q[1] + 16*q[3]*q[3]*q[3] - 4*cur_accel[1]*q[3] - 4*cur_accel[2]*q[0] + 8*cur_accel[3]*q[1] -8*q[1],
        8*q[0]*q[0]*q[2] + 4*cur_accel[1]*q[0] + 8*q[3]*q[3]*q[2] - 4*cur_accel[2]*q[3] - 8*q[2] + 16*q[1]*q[1]*q[2] + 16*q[2]*q[2]*q[2] + 8*cur_accel[3]*q[2],
        8*q[1]*q[1]*q[3] + 8*q[2]*q[2]*q[3] - 4*cur_accel[1]*q[1] - 4*cur_accel[2]*q[2]
    };
}

/* 高斯牛顿迭代 */
/* Gauss Newton Iteration Method */
quaternion gauss_newton_method(const quaternion& q){
    quaternion error_function{
        0,
        2*q[0]*q[2] - 2*q[1]*q[3] -cur_accel[1],
        -2*q[0]*q[1] - 2*q[2]*q[3] -cur_accel[2],
        1 - 2*q[1]*q[1] - 2*q[2]*q[2] -cur_accel[3]
    };

    return q - \
            quaternion{2*q[2],-2*q[3],2*q[0],-2*q[1]} * error_function[1] - \
            quaternion{-2*q[1],-2*q[0],-2*q[3],-2*q[2]} * error_function[2] - \
            quaternion{0,-4*q[1],-4*q[2],0} * error_function[3];
}

void TEST_Task_RK1_Attitude_Calculate(void* arg){

    mpu6050 mpu6050_dev;
    for(;;){
        //更新mpu6050
        mpu6050_dev.update();
        //第一次获取IMU数据
        if(__euler_angle_told == 0)
        {
            __euler_angle_told = Get_TimeStamp();
            last_gyro = mpu6050_dev.get_gyro();
            OSTimeDlyHMSM(0, 0, 0, 100);
        }
        //2-n次获取IMU数据
        cur_gyro = mpu6050_dev.get_gyro();
        gyro_sampling_duty = ((double)Get_TimeStamp()-__euler_angle_told) / HAL_RCC_GetSysClockFreq();
        //Runge Kutta求解姿态
        last_attitude = RK1(0,
                            last_attitude, 
                            gyro_sampling_duty,
                            quaternion_derivative);
        last_attitude.normalization();

        //高斯牛顿迭代获取加速度计姿态
        cur_accel = mpu6050_dev.get_accelero();
        // for(int i=0;i<2;++i)
        //     last_attitude = gradient_decent(last_attitude,error_function_gradient,0.5);
        last_attitude = gauss_newton_method(last_attitude);
        last_attitude.normalization();
        
        //更新数据
        last_gyro = cur_gyro;
        __euler_angle_told = Get_TimeStamp();
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void TEST_Task_Attitude(void* arg){
    for(;;)
    {
        print_quaternion2EulerAngle(last_attitude);
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

void TEST_RK4_Init()
{
    OSTaskCreate(TEST_Task_RK1_Attitude_Calculate, (void*)0, &Stk_RK4[1023], TASK_RK4_PRIO);
    //OSTaskCreate(TEST_Task_Attitude, NULL, &Stk_Rk4_Print[1023], TASK_RK4_PRINT_PRIO);
}



#endif      