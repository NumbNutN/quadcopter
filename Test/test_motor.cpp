#include "config.h"

#if TEST_MOTOR_EN > 0u

#include "motor.hpp"
#include "os.h"
#include "delay.h"

#include <iostream>

#define MOTOR_STACK_SIZE 256

OS_STK Stk_Motor1Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor2Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor3Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor4Init[MOTOR_STACK_SIZE];

#define TASK_MOTOR1_PRIO 14u
#define TASK_MOTOR2_PRIO 15u
#define TASK_MOTOR3_PRIO 16u
#define TASK_MOTOR4_PRIO 17u

motor* motorObjList[4];

using namespace std;


void TEST_Task_Motor(void* channel){
    motor m(&htim3,(uint32_t)channel);
    uint64_t timeStamp;
    if((uint32_t)channel == TIM_CHANNEL_1){
        motorObjList[0] = &m;
        m.setLabel('a');
    }
    else if ((uint32_t)channel == TIM_CHANNEL_2){
        motorObjList[1] = &m;
        m.setLabel('b');
    }
    else if ((uint32_t)channel == TIM_CHANNEL_3){
        motorObjList[2] = &m;
        m.setLabel('c');
    }
    else{
        motorObjList[3] = &m;
        m.setLabel('d');
    }

    for(;;){
        float deltaT = (float)(Get_TimeStamp() - timeStamp) / HAL_RCC_GetSysClockFreq();
        timeStamp = Get_TimeStamp();
        m.updateDuty();
        OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

void TEST_Motor_Init()
{

    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_1, &Stk_Motor1Init[MOTOR_STACK_SIZE-1], TASK_MOTOR1_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_2, &Stk_Motor2Init[MOTOR_STACK_SIZE-1], TASK_MOTOR2_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_3, &Stk_Motor3Init[MOTOR_STACK_SIZE-1], TASK_MOTOR3_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_4, &Stk_Motor4Init[MOTOR_STACK_SIZE-1], TASK_MOTOR4_PRIO);
}

#endif