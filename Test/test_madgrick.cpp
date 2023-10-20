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

using namespace std;

OS_STK Stk_Madgwick[1024];

quaternion attitude = {1,0,0,0};

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

quaternion magnetic_gradient(const quaternion& q,const quaternion& magnetic){

    quaternion func_m{
        2*magnetic[1]*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2*magnetic[3]*(q[1]*q[3] - q[0]*q[2]) - magnetic[1],
        2*magnetic[1]*(q[1]*q[2] - q[0]*q[3]) +2*magnetic[3]*(q[0]*q[1] + q[2]*q[3]) - magnetic[2],
        2*magnetic[1]*(q[0]*q[2] + q[1]*q[3]) + 2*magnetic[3]*(0.5 - q[1]*q[1] - q[2]*q[2]) - magnetic[3]
    };

    return
        quaternion{-2*magnetic[3]*q[2],2*magnetic[3]*q[3],-4*magnetic[1]*q[2]-2*magnetic[3]*q[0],-4*magnetic[1]*q[3]+2*magnetic[3]*q[1]}*func_m[0]+\
        quaternion{-2*magnetic[1]*q[3]+2*magnetic[3]*q[1],2*magnetic[1]*q[2]+2*magnetic[3]*q[0],2*magnetic[1]*q[1]+2*magnetic[3]*q[3],-2*magnetic[1]*q[0]+2*magnetic[3]*q[2]}*func_m[1]+\
        quaternion{2*magnetic[1]*q[2],2*magnetic[1]*q[3]-4*magnetic[3]*q[1],2*magnetic[1]*q[0]-4*magnetic[3]*q[2],2*magnetic[1]*q[1]}*func_m[2];
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

    for(;;){
        mpu6050_dev.update();
        hmc_dev.update();
        float deltaT = mpu6050_dev.getSamplePeriod();

        quaternion attitude_gyro = RK1(0,
                attitude, 
                deltaT,
                [](double t,quaternion q)->quaternion{
                        return 0.5*q*  ((t/mpu6050_ptr->getSamplePeriod())*mpu6050_ptr->get_current_gyro() + (1-t/mpu6050_ptr->getSamplePeriod())*mpu6050_ptr->get_last_gyro());
                    });
        
        quaternion attitude_acceleration = accelator_gradient(attitude, mpu6050_dev.get_acceleration());
        quaternion attitude_magnetic = magnetic_gradient(attitude, hmc_dev.getMagnetic());
        quaternion attitude_mixed = (attitude_acceleration + attitude_magnetic).normalization();

        attitude = attitude_gyro - BETA*attitude_mixed*deltaT;                

        cout << quat_get_Roll(attitude) << endl;
        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

void TEST_Madgwick_Init()
{
    OSTaskCreate(TEST_Madgwick, NULL, &Stk_Madgwick[1023], TASK_MADGWICK_READ_PRIO);
}


#endif