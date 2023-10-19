#include "config.h"

#if TEST_PID3_EN > 0u

#define TASK_PID_INTERNAL_PRIO 20u
#define TASK_PID_EXTERNAL_PRIO 21u

#include <iostream>

#include "pid.hpp"
#include "quaternion.hpp"

#include "myMath.hpp"
#include "motor.hpp"

#include "os.h"
#include <math.h>

using namespace std;

extern quaternion last_gyro;
extern quaternion cur_gyro;
extern quaternion cur_accel;
extern quaternion last_attitude;

OS_STK Stk_PID_Interal[1024];
OS_STK Stk_PID_Exteral[1024];


extern motor* motorObjList[4];

const float expK = 0.01;
float omega_g = 802.04859078;

void pitch_set_pwm(float alpha){
    motorObjList[0]->setAngularVehicle(sqrt(omega_g*omega_g - expK*alpha));
    motorObjList[3]->setAngularVehicle(sqrt(omega_g*omega_g + expK*alpha));

}

void roll_set_pwm(float alpha){
    motorObjList[1]->setAngularVehicle(sqrt(omega_g*omega_g - expK*alpha));
    motorObjList[2]->setAngularVehicle(sqrt(omega_g*omega_g + expK*alpha));
}

void yaw_set_pwm(float beta){

}



//内环PID p i d 获取当前角速度 计算出当前角加速度后的回调 获取当前导数（角加速度）
// 外环PID p i d 获取当前角度 计算出当前角速度后的回调  获取当前导数（角速度）


pid ipid_pitch_controller(0.5,0.2,0.1,
                        []()->float{return cur_gyro[0];},
                        pitch_set_pwm);

pid ipid_roll_controller(0.5,0.2,0.1,
                        []()->float{return cur_gyro[1];},
                        roll_set_pwm);

pid ipid_yaw_controller(0.5,0.2,0.1,
                        []()->float{return cur_gyro[2];},
                        yaw_set_pwm);

pid epid_pitch_controller(0.5,0,0,
                    []() -> float{return quat_get_Pitch(last_attitude);},
                    [](float tar)->void{ipid_pitch_controller.setTarget(tar);},
                    [](pid*)->float{return cur_gyro[0];});

pid epid_roll_controller(0.5,0,0,
                    []() -> float{return quat_get_Roll(last_attitude);},
                    [](float tar)->void{ipid_roll_controller.setTarget(tar);},
                    [](pid*)->float{return cur_gyro[1];});

pid epid_yaw_controller(0.5,0,0,
                    []() -> float{return quat_get_Yaw(last_attitude);},
                    [](float tar)->void{ipid_yaw_controller.setTarget(tar);},
                    [](pid*)->float{return cur_gyro[2];});

void epid_run(pid& controller){
    float threshold = 0.05;
    if(abs(controller.getCurError()) > threshold)
    //获取当前的等效偏移角
            controller.setTarget(0);
    controller.run();
}

void TEST_PID_EXTERNAL(void* arg){
    epid_pitch_controller.setTarget(0);
    epid_roll_controller.setTarget(0);
    epid_yaw_controller.setTarget(0);
    for(;;)
    {
        epid_run(epid_pitch_controller);
        epid_run(epid_roll_controller);
        epid_run(epid_yaw_controller);
        //绘制当前角速度期望值
        //cout << epid_pitch_controller.getOutput() << endl;
        OSTimeDlyHMSM(0, 0, 0, 200);
    }
}

void TEST_PID_INTERNAL(void* arg){
    for(;;)
    {
        ipid_yaw_controller.run();
        ipid_pitch_controller.run();
        ipid_roll_controller.run();
        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

void TEST_PID_Init()
{
    OSTaskCreate(TEST_PID_INTERNAL, (void*)0, &Stk_PID_Interal[1023], TASK_PID_INTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, NULL, &Stk_PID_Exteral[1023], TASK_PID_EXTERNAL_PRIO);
}

#endif