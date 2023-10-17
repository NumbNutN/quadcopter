#include "config.h"

#if TEST_PID_EN > 0u

#define TASK_PID_INTERNAL_PRIO 11u
#define TASK_PID_EXTERNAL_PRIO 12u

#include "pid.hpp"
#include "quaternion.hpp"

#include "os.h"

extern quaternion last_gyro;
extern quaternion cur_gyro;
extern quaternion cur_accel;
extern quaternion last_attitude;

OS_STK Stk_PID_Interal[1024];
OS_STK Stk_PID_Exteral[1024];

float get_Theta(){
    return cur_gyro.getTheta();
}

float get_Omega(){
    // quaternion cur_gyro_e = last_attitude * cur_gyro * last_attitude.conjugate();
    // quaternion axis = last_attitude.getAxis();
    // return 1.0f/3*(cur_gyro_e[1] / axis[1] + cur_gyro_e[2] / axis[2] + cur_gyro_e[3] / axis[3]);
    return cur_gyro.length();
}

void set_pwm(float theta){

}

float get_gyro() {

}

//内环PID p i d 获取当前角速度 计算出当前角加速度后的回调 获取当前导数（角加速度）
pid ipid_controller(0.5,0.5,0.5,get_Omega,set_pwm);
// 外环PID p i d 获取当前角度 计算出当前角速度后的回调  获取当前导数（角速度）
pid epid_controller(0.5,0.5,0.5,
                    get_Theta,
                    [](float tar)->void{ipid_controller.setTarget(tar);},
                    get_Omega);


void TEST_PID_EXTERNAL(void* arg){
    float threshold = 0.2;
    for(;;)
    {
        if(abs(epid_controller.getCurError()) < threshold)
            //获取当前的等效偏移角
            epid_controller.setTarget(-last_attitude.getTheta());
        epid_controller.run();
        OSTimeDlyHMSM(0, 0, 0, 200);
    }
}

void TEST_PID_INTERNAL(void* arg){
    for(;;)
    {
        ipid_controller.run();
        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

void TEST_PID_Init()
{
    OSTaskCreate(TEST_PID_INTERNAL, (void*)0, &Stk_PID_Interal[1023], TASK_PID_INTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, NULL, &Stk_PID_Exteral[1023], TASK_PID_EXTERNAL_PRIO);
}

#endif