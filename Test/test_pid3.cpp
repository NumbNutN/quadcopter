#include "config.h"

#if TEST_PID3_EN > 0u

#include <iostream>

#include <math.h>

#include "pid.hpp"
#include "quaternion.hpp"

#include "quat_math.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "hmc_5583l.hpp"

#include "usart.h"
#include "os.h"
#include "test_tasks.h"

using namespace std;

extern quaternion cur_gyro;

extern quaternion attitude;

extern mpu6050* mpu6050_ptr;
extern hmc_558l* hmc_ptr;

OS_STK Stk_PID_Interal[1024];
OS_STK Stk_PID_Exteral[1024];


//内环PID p i d 获取当前角速度 计算出当前角加速度后的回调 获取当前导数（角加速度）
// 外环PID p i d 获取当前角度 计算出当前角速度后的回调  获取当前导数（角速度）

#if PID3_SERIE > 0u

vector<pid> InternalControllerList = {
    pid(0.05,0,0.05,
                        []()->float{return mpu6050_ptr->get_current_gyro()[1];},
                        pitch_set_pwm),
    pid(0.05,0,0.05,
                        []()->float{return mpu6050_ptr->get_current_gyro()[2];},
                        roll_set_pwm),
    pid(0.05,0,0.05,
                        []()->float{return mpu6050_ptr->get_current_gyro()[3];},
                        yaw_set_pwm)
};

vector<pid> ExternalControllerList = {
    pid(5,0,0,
                    []() -> float{return quat_get_Pitch(attitude);},
                    [](float tar)->void{InternalControllerList[0].setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[1];}),
    pid(5,0,0,
                    []() -> float{return quat_get_Roll(attitude);},
                    [](float tar)->void{InternalControllerList[1].setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[2];}),
    pid(5,0,0,
                    []() -> float{return quat_get_Yaw(attitude);},
                    [](float tar)->void{InternalControllerList[2].setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[3];})
};

#elif PID3_SINGLE > 0u

pid ExternalControllerList[3] = {
    pid(0.5,0,0.05,
                    []() -> float{return quat_get_Pitch(attitude);},
                    pitch_set_pwm,
                    []()->float{return mpu6050_ptr->get_current_gyro()[1];}),
    pid(0.5,0,0.05,
                    []() -> float{return quat_get_Roll(attitude);},
                    roll_set_pwm,
                    []()->float{return mpu6050_ptr->get_current_gyro()[2];}),
    pid(0.5,0,0.05,
                    []() -> float{return quat_get_Yaw(attitude);},
                    yaw_set_pwm,
                    []()->float{return mpu6050_ptr->get_current_gyro()[3];})
};

#endif

#define THRESHOLE 0.02

void TEST_PID_EXTERNAL(void* idx){

    pid& controller = ExternalControllerList[size_t(idx)];
    controller.setTarget(0);
    for(;;)
    {
        controller.run();
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}
#if PID3_SERIE > 0u
void TEST_PID_INTERNAL(void* idx){

    pid& controller = InternalControllerList[size_t(idx)];
    for(;;)
    {
        controller.run();
        OSTimeDlyHMSM(0, 0, 0, 5);
    }
}
#endif

void TEST_PID_Init()
{
    //设置PID 参数调节接收
    OS_ERR err;
#if PID3_SERIE > 0u

    InternalControllerList[0].setLabel('p');
    InternalControllerList[1].setLabel('r');

    OSTaskCreate(TEST_PID_INTERNAL, (void*)1, &Stk_PID_Interal[511], TASK_PID_ROLL_INTERNAL_PRIO);
    OSTaskNameSet(TASK_PID_ROLL_INTERNAL_PRIO,(INT8U*)"IPID_ROLL",&err);
    OSTaskCreate(TEST_PID_INTERNAL,(void*)0,&Stk_PID_Interal[1023],TASK_PID_PITCH_INTERNAL_PRIO);
    OSTaskNameSet(TASK_PID_PITCH_INTERNAL_PRIO,(INT8U*)"IPID_PITCH",&err);

#endif

    ExternalControllerList[0].setLabel('P');
    ExternalControllerList[1].setLabel('R');

    OSTaskCreate(TEST_PID_EXTERNAL, (void*)1, &Stk_PID_Exteral[511], TASK_PID_ROLL_EXTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, (void*)0, &Stk_PID_Exteral[1023], TASK_PID_PITCH_EXTERNAL_PRIO);
    
    OSTaskNameSet(TASK_PID_ROLL_EXTERNAL_PRIO,(INT8U*)"EPID_ROLL",&err);
    OSTaskNameSet(TASK_PID_PITCH_EXTERNAL_PRIO,(INT8U*)"EPID_PITCH",&err);

    //使能IDLE中断以在数据传输完成后陷入中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

}


#endif