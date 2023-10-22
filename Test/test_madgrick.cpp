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

quaternion GuassianFilter(const quaternion& gyro){
    static quaternion historyList[5];
    static size_t index = 0;
    quaternion average = {0,0,0,0};

    historyList[index++ % 5] = gyro;
    for(int i=0;i<5;++i)
        average += historyList[i];
    // static vector<quaternion> v5(5,quaternion{0,0,0,0});
    return index>=5?average/5:average/(index+1);
}

mpu6050* mpu6050_ptr;
hmc_558l* hmc_ptr;

quaternion attitude_gyro;
quaternion acceleration_err_fun_gradient;
quaternion magnetic_err_func_gradient;
quaternion attitude_mixed;
float deltaT;
quaternion attitude;
quaternion old_attitude = {1,0,0,0};

void TEST_Madgwick(void* arg){
    mpu6050 mpu6050_dev;
    hmc_558l hmc_dev;
    quaternion gyroData;
    mpu6050_ptr = &mpu6050_dev;
    hmc_ptr = &hmc_dev;

    //校准陀螺仪的零偏误差
    mpu6050_dev.alignment(100);

    for(;;){
        mpu6050_dev.update();
        hmc_dev.update();
        deltaT = mpu6050_dev.getSamplePeriod();

        //一阶龙格图塔计算姿态
        attitude_gyro = RK1(0,
                old_attitude, 
                deltaT,
                [](double t,quaternion q)->quaternion{
                        quaternion gyro = mpu6050_ptr->get_last_gyro();
                        gyro = GuassianFilter(gyro);
                        return 0.5*q*  ((t/mpu6050_ptr->getSamplePeriod())*mpu6050_ptr->get_current_gyro() + (1-t/mpu6050_ptr->getSamplePeriod())*gyro);
                    });
        
        //为加速度计的误差函数计算梯度
        acceleration_err_fun_gradient = accelator_gradient(old_attitude, mpu6050_dev.get_acceleration());
        //为磁力计的误差函数计算梯度
        magnetic_err_func_gradient = magnetic_gradient(old_attitude, hmc_dev.getMagnetic().normalization());
        
        attitude_mixed = (acceleration_err_fun_gradient+magnetic_err_func_gradient);

        //Madgwick
        attitude = (attitude_gyro - 0.05*acceleration_err_fun_gradient.normalization()*deltaT).normalization();
        
        //if(quat_get_Roll(attitude) )s
        //attitude = (attitude_gyro).normalization();
        //attitude = (attitude - acceleration_err_fun_gradient*0.5).normalization();
        //cout << attitude_mixed.length() << endl;
        //attitude = (attitude_gyro - attitude_mixed).normalization();
        //attitude = attitude.normalization();     

        //测试置信度是否是1/2
        //attitude = 0.5*(attitude - attitude_acceleration).normalization() + 0.5*attitude_gyro.normalization();    

        //为四元数添加低通滤波去除尖刺
        if(abs(quat_get_Pitch(old_attitude) - quat_get_Pitch((attitude))) > 0.2) attitude = old_attitude;
        old_attitude = attitude;
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

void TEST_Task_Print_Attitude(void* arg){
    for(;;)
    {
        if(quat_get_Pitch(attitude) > 3.1415926 || quat_get_Pitch(attitude) < - 3.1415926)
            cout << 999 << endl;
        // cout << "M " << quat_get_Roll(attitude) << endl;
        // cout << "A " << quat_get_Roll(attitude - 0.3*acceleration_err_fun_gradient.normalization()*deltaT) << endl;
        //cout << quat_get_Roll(attitude_gyro) << endl;
        // cout << "G " << mpu6050_ptr->get_current_gyro()[1] << endl;
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