#include "config.h"

#if TEST_MADGRICK_EN > 0u

#include <iostream>

#include "quaternion.hpp"
#include "myMath.hpp"
#include "mpu6050.hpp"
#include "hmc_5583l.hpp"
#include "gradient_decent.hpp"

#include <math.h>

#include "os.h"
#include "delay.h"

#define BETA 0.033
#define TASK_MADGWICK_READ_PRIO 19u
#define TASK_MADGWICK_PRINT_PRIO 18u

using namespace std;

OS_STK Stk_Madgwick[512];
OS_STK Stk_PrintMadgwick[512];

quaternion attitude = {1,0,0,0};

quaternion earth_magnetic{0,0.66436113595878288,0,0.74741172122703259};
// quaternion earth_magnetic{0,1,0,0};

/* Madgrick */
quaternion accelator_gradient(const quaternion& q,const quaternion& acceleration){

    quaternion func_g{
        2*(q[1]*q[3] - q[0]*q[2]) - acceleration[1],
        2*(q[0]*q[1] + q[2]*q[3]) - acceleration[2],
        1 - 2*q[1]*q[1] - 2*q[2]*q[2] - acceleration[3] 
    };
    return 
        quaternion{-2*q[2],2*q[3],-2*q[0],2*q[1]} * func_g[0] + \
        quaternion{2*q[1],2*q[0],2*q[3],2*q[2]} * func_g[1] + \
        quaternion{0,-4*q[1],-4*q[2],0} * func_g[2];
}

/* 高斯牛顿迭代 */
/* Gauss Newton Iteration Method */
quaternion gauss_newton_method(const quaternion& q,const quaternion& acceleration){
    quaternion error_function{
        0,
        2*q[0]*q[2] - 2*q[1]*q[3] -acceleration[1],
        -2*q[0]*q[1] - 2*q[2]*q[3] -acceleration[2],
        1 - 2*q[1]*q[1] - 2*q[2]*q[2] -acceleration[3]
    };

    return 
            quaternion{2*q[2],-2*q[3],2*q[0],-2*q[1]} * error_function[1] + \
            quaternion{-2*q[1],-2*q[0],-2*q[3],-2*q[2]} * error_function[2] + \
            quaternion{0,-4*q[1],-4*q[2],0} * error_function[3];
}

quaternion magnetic_gradient(const quaternion& q,const quaternion& magnetic){

    quaternion func_m{
        2*earth_magnetic[1]*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2*earth_magnetic[3]*(q[1]*q[3] - q[0]*q[2]) - magnetic[1],
        2*earth_magnetic[1]*(q[1]*q[2] - q[0]*q[3]) +2*earth_magnetic[3]*(q[0]*q[1] + q[2]*q[3]) - magnetic[2],
        2*earth_magnetic[1]*(q[0]*q[2] + q[1]*q[3]) + 2*earth_magnetic[3]*(0.5 - q[1]*q[1] - q[2]*q[2]) - magnetic[3]
    };

    return
        quaternion{-2*earth_magnetic[3]*q[2],2*earth_magnetic[3]*q[3],-4*earth_magnetic[1]*q[2]-2*earth_magnetic[3]*q[0],-4*earth_magnetic[1]*q[3]+2*earth_magnetic[3]*q[1]}*func_m[0]+\
        quaternion{-2*earth_magnetic[1]*q[3]+2*earth_magnetic[3]*q[1],2*earth_magnetic[1]*q[2]+2*earth_magnetic[3]*q[0],2*earth_magnetic[1]*q[1]+2*earth_magnetic[3]*q[3],-2*earth_magnetic[1]*q[0]+2*earth_magnetic[3]*q[2]}*func_m[1]+\
        quaternion{2*earth_magnetic[1]*q[2],2*earth_magnetic[1]*q[3]-4*earth_magnetic[3]*q[1],2*earth_magnetic[1]*q[0]-4*earth_magnetic[3]*q[2],2*earth_magnetic[1]*q[1]}*func_m[2];
}

// quaternion madgwick(const quaternion& attitude,float deltaT,const quaternion& acceleration,const quaternion& magnetic){

//     //获得陀螺仪近似处理得到的姿态
//     //获得加速度计和磁力计得到的融合姿态
//     return attitude + RK1(0,attitude, deltaT,quaternion_derivative) - BETA*(accelator_gradient(attitude, acceleration) + magnetic_gradient(attitude, magnetic))*deltaT;

// }

mpu6050* mpu6050_ptr;
hmc_558l* hmc_ptr;

void TEST_Madgwick(void* arg){
    mpu6050 mpu6050_dev;
    hmc_558l hmc_dev;
    mpu6050_ptr = &mpu6050_dev;
    hmc_ptr = &hmc_dev;

    //校准陀螺仪
    mpu6050_dev.alignment(100);

    for(;;){
        mpu6050_dev.update();
        hmc_dev.update();
        //quaternion mag = hmc_dev.getMagnetic();
        float deltaT = mpu6050_dev.getSamplePeriod();

        quaternion attitude_gyro = RK4(0,
                attitude, 
                deltaT,
                [](double t,quaternion q)->quaternion{
                        return 0.5*q*  ((t/mpu6050_ptr->getSamplePeriod())*mpu6050_ptr->get_current_gyro() + (1-t/mpu6050_ptr->getSamplePeriod())*mpu6050_ptr->get_last_gyro());
                    });
        
        quaternion acceleration_err_fun_gradient = gauss_newton_method(attitude, mpu6050_dev.get_acceleration());
        quaternion magnetic_err_func_gradient = magnetic_gradient(attitude, hmc_dev.getMagnetic().normalization());
        quaternion attitude_mixed = (acceleration_err_fun_gradient+magnetic_err_func_gradient);

        attitude = (attitude_gyro - 0.3*acceleration_err_fun_gradient.normalization()*deltaT).normalization();
        //attitude = (attitude_gyro).normalization();
        //attitude = attitude_gyro;
        //cout << attitude_mixed.length() << endl;
        //attitude = (attitude_gyro - attitude_mixed).normalization();
        //attitude = attitude.normalization();     

        //是否是1/2
        //attitude = 0.5*(attitude - attitude_acceleration).normalization() + 0.5*attitude_gyro.normalization();           

        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

void TEST_Task_Print_Attitude(void* arg){
    for(;;)
    {
        if(quat_get_Pitch(attitude) > 3.1415926 || quat_get_Pitch(attitude) < - 3.1415926)
            cout << 999 << endl;
        cout << quat_get_Roll(attitude) << endl;
        OSTimeDlyHMSM(0, 0, 0, 150);
    }
}

// quaternion My_Madgwick_Update(const quaternion& quat, float ax, float ay, float az,\
//                         float gx, float gy, float gz,\
//                         float mx, float my, float mz);



// void TEST_Madgwick(void* arg){
//     mpu6050 mpu6050_dev;
//     hmc_558l hmc_dev;
//     mpu6050_ptr = &mpu6050_dev;
//     hmc_ptr = &hmc_dev;

//     for(;;){
//         mpu6050_dev.update();
//         hmc_dev.update();
//         attitude = My_Madgwick_Update(attitude, mpu6050_dev.get_acceleration()[1], mpu6050_dev.get_acceleration()[2], mpu6050_dev.get_acceleration()[3],\
//                 0, 0, 0.98,\
//                 hmc_dev.getMagnetic()[1], hmc_dev.getMagnetic()[2], hmc_dev.getMagnetic()[3]);

//         cout << quat_get_Pitch(attitude) << endl;
//         OSTimeDlyHMSM(0, 0, 0, 100);
//     }
// }




void TEST_Madgwick_Init()
{
    OSTaskCreate(TEST_Task_Print_Attitude,NULL,&Stk_PrintMadgwick[511],TASK_MADGWICK_PRINT_PRIO);
    OSTaskCreate(TEST_Madgwick, NULL, &Stk_Madgwick[511], TASK_MADGWICK_READ_PRIO);
}


#endif