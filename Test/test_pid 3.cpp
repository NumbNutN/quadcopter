#include "config.h"

#if TEST_PID3_EN > 0u

#define TASK_PID_PITCH_EXTERNAL_PRIO 20u
#define TASK_PID_ROLL_EXTERNAL_PRIO 21u
#define TASK_PID_PITCH_INTERNAL_PRIO 22u
#define TASK_PID_ROLL_INTERNAL_PRIO 23u

#include <iostream>

#include "pid.hpp"
#include "quaternion.hpp"

#include "myMath.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "hmc_5583l.hpp"

#include "os.h"
#include <math.h>

using namespace std;

extern quaternion cur_gyro;

extern quaternion attitude;

extern mpu6050* mpu6050_ptr;
extern hmc_558l* hmc_ptr;

OS_STK Stk_PID_Interal[1024];
OS_STK Stk_PID_Exteral[1024];


extern motor* motorObjList[4];

const float expK = 10000000;
float omega_g = 441.708435;

float k = 0.001;

void pitch_set_pwm(float alpha){
    motorObjList[0]->setAddDuty(- k*alpha);
    motorObjList[1]->setAddDuty(- k*alpha);
    motorObjList[3]->setAddDuty(k*alpha);
    motorObjList[2]->setAddDuty(k*alpha);
}

void roll_set_pwm(float alpha){
    motorObjList[1]->setAddDuty(- k*alpha);
    motorObjList[3]->setAddDuty(- k*alpha);
    motorObjList[2]->setAddDuty(k*alpha);
    motorObjList[0]->setAddDuty(k*alpha);
}

void yaw_set_pwm(float beta){

}

//内环PID p i d 获取当前角速度 计算出当前角加速度后的回调 获取当前导数（角加速度）
// 外环PID p i d 获取当前角度 计算出当前角速度后的回调  获取当前导数（角速度）


pid ipid_pitch_controller(0.3,0.1,0.2,
                        []()->float{return mpu6050_ptr->get_current_gyro()[1];},
                        pitch_set_pwm);

pid ipid_roll_controller(0.3,0.1,0.2,
                        []()->float{return mpu6050_ptr->get_current_gyro()[2];},
                        roll_set_pwm);

pid ipid_yaw_controller(0.3,0.1,0.2,
                        []()->float{return mpu6050_ptr->get_current_gyro()[3];},
                        yaw_set_pwm);

pid epid_pitch_controller(0.9,0,0,
                    []() -> float{return quat_get_Pitch(attitude);},
                    [](float tar)->void{ipid_pitch_controller.setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[1];});

pid epid_roll_controller(0.9,0,0,
                    []() -> float{return quat_get_Roll(attitude);},
                    [](float tar)->void{ipid_roll_controller.setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[2];});

pid epid_yaw_controller(0.9,0,0,
                    []() -> float{return quat_get_Yaw(attitude);},
                    [](float tar)->void{ipid_yaw_controller.setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[3];});


OS_EVENT* pitch_internal_get_ref;
OS_EVENT* yaw_internal_get_ref;
OS_EVENT* roll_internal_get_ref;

#define THRESHOLE 0.02

void TEST_PID_EXTERNAL(void* arg){

    pid* controller = (pid*)arg;
    controller->setTarget(0);
    // OS_EVENT* sem = OSSemCreate(1);
    // INT8U err;
    for(;;)
    {
        //OSSemPend(sem,0,&err);
        controller->run();
        // cout << controller->getLabel() << ' '<< controller->getOutput() << endl;
        OSTimeDlyHMSM(0, 0, 0, 200);
    }
}

void TEST_PID_INTERNAL(void* arg){

    pid* controller = (pid*)arg;
    for(;;)
    {
        controller->run();
        // if(abs(controller->getCurError()) < THRESHOLE)
        //     OSSemPost();
        // if(controller->getLabel() == 'r')
        //     cout << controller->getOutput() << endl;
        OSTimeDlyHMSM(0, 0, 0, 20);
    }
}

void TEST_PID_Init()
{
    ipid_pitch_controller.setLabel('p');
    ipid_roll_controller.setLabel('r');
    epid_pitch_controller.setLabel('P');
    epid_roll_controller.setLabel('R');
    OSTaskCreate(TEST_PID_INTERNAL, &ipid_pitch_controller, &Stk_PID_Interal[1023], TASK_PID_PITCH_INTERNAL_PRIO);
    OSTaskCreate(TEST_PID_INTERNAL, &ipid_roll_controller, &Stk_PID_Interal[511], TASK_PID_ROLL_INTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, &epid_pitch_controller, &Stk_PID_Exteral[1023], TASK_PID_PITCH_EXTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, &epid_roll_controller, &Stk_PID_Exteral[511], TASK_PID_ROLL_EXTERNAL_PRIO);
}

#endif