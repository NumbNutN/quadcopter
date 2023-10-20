#include "config.h"

#if TEST_MOTOR_EN > 0u

#include "motor.hpp"
#include "os.h"

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
    float duty = 0.06;
    motor m(&htim3,(uint32_t)channel);
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
        // duty += 0.001;
        // m.setDuty(duty);
        // if(m.getLabel() == 'a')
        //     cout <<m.get_dutyCycle() << endl;
        m.updateDuty();
        OSTimeDlyHMSM(0, 0, 0, 200);
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