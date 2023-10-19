#include "config.h"

#if TEST_MOTOR_EN > 0u

#include "motor.hpp"
#include "os.h"

OS_STK Stk_Motor1Init[256];
OS_STK Stk_Motor2Init[256];
OS_STK Stk_Motor3Init[256];
OS_STK Stk_Motor4Init[256];

#define TASK_MOTOR1_PRIO 14u
#define TASK_MOTOR2_PRIO 15u
#define TASK_MOTOR3_PRIO 16u
#define TASK_MOTOR4_PRIO 17u

motor* motorObjList[4];


void TEST_Task_Motor(void* channel){
    float duty = 0.06;
    motor m(&htim3,(uint32_t)channel);
    if((uint32_t)channel == TIM_CHANNEL_1)
        motorObjList[0] = &m;
    else if ((uint32_t)channel == TIM_CHANNEL_2)
        motorObjList[1] = &m;
    else if ((uint32_t)channel == TIM_CHANNEL_3)
        motorObjList[2] = &m;
    else
        motorObjList[3] = &m;
    for(;;){
        // duty += 0.001;
        // m.setDuty(duty);
        m.updateDuty();
        OSTimeDlyHMSM(0, 0, 5, 0);
    }
}

void TEST_Motor_Init()
{
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_1, &Stk_Motor1Init[255], TASK_MOTOR1_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_2, &Stk_Motor2Init[255], TASK_MOTOR2_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_3, &Stk_Motor3Init[255], TASK_MOTOR3_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_4, &Stk_Motor4Init[255], TASK_MOTOR4_PRIO);
}

#endif