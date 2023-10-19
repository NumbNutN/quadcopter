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

float motorDutyList[4] = {0.06,0.06,0.06,0.06};

void TEST_Task_Motor(void* channel){
    motor motor(&htim3,(uint32_t)channel);
    for(;;){
        // if((uint32_t)channel == TIM_CHANNEL_1)
        //     motor.setDuty(motorDutyList[0]);
        // else if ((uint32_t)channel == TIM_CHANNEL_2)
        //     motor.setDuty(motorDutyList[1]);
        // else if ((uint32_t)channel == TIM_CHANNEL_3)
        //     motor.setDuty(motorDutyList[2]);
        // else
        //     motor.setDuty(motorDutyList[3]);
        OSTimeDlyHMSM(0, 0, 0, 200);
    }
}

void TEST_Motor_Init()
{
    // OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_1, &Stk_Motor1Init[255], TASK_MOTOR1_PRIO);
    // OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_2, &Stk_Motor2Init[255], TASK_MOTOR2_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_3, &Stk_Motor3Init[255], TASK_MOTOR3_PRIO);
    // OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_4, &Stk_Motor4Init[255], TASK_MOTOR4_PRIO);
}

#endif