#include "config.h"
#include "motor.hpp"
#if TEST_MOTOR_EN > 0u

#include <vector>

#include "stm32f4xx.h"
#include <assert.h>
#include <iostream>
#include "os.h"
#include "delay.h"
#include "test_tasks.h"

using namespace std;

#define MOTOR_STACK_SIZE 256
OS_STK Stk_Motor1Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor2Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor3Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor4Init[MOTOR_STACK_SIZE];

vector<motor> motorList = {
  motor(&htim3,(uint32_t)TIM_CHANNEL_1,'1'),
  motor(&htim3,(uint32_t)TIM_CHANNEL_2,'2'),
  motor(&htim3,(uint32_t)TIM_CHANNEL_3,'3'),
  motor(&htim3,(uint32_t)TIM_CHANNEL_4,'4')
};

/*
/ H \       / H \
\___/\     /\___/
     /     \  
/ N \       / N \
\___/       \___/
*/
void pitch_set_pwm(float pid_out){
    //float dutyCycle = pid_out*mpu6050_ptr->getSamplePeriod() * 26.166f;
    motorList[0].setAddDuty(pid_out);
    motorList[1].setAddDuty(pid_out);
    motorList[2].setAddDuty(- pid_out);
    motorList[3].setAddDuty(- pid_out);
}

/*
/ H \       / N \
\___/\     /\___/
     /     \  
/ H \       / N \
\___/       \___/
*/
void roll_set_pwm(float pid_out){
    //float dutyCycle = pid_out*mpu6050_ptr->getSamplePeriod() * 26.166f;
    motorList[0].setAddDuty2(pid_out);
    motorList[1].setAddDuty2(- pid_out);
    motorList[2].setAddDuty2(pid_out);
    motorList[3].setAddDuty2(- pid_out);
}

/*
/ H \       / N \
\___/\     /\___/
     /     \  
/ N \       / H \
\___/       \___/
*/
void yaw_set_pwm(float pid_out){
    motorList[0].setAddDuty3(pid_out);
    motorList[1].setAddDuty3(- pid_out);
    motorList[2].setAddDuty3(- pid_out);
    motorList[3].setAddDuty3(pid_out);
}

using namespace std;
void TEST_Task_Motor(void* index){
    /* 解锁信号出现前，该任务阻塞 */
    OS_ERR err;
    motor& m = motorList[(size_t)index];
    //OSSemPend(sem_esc_unlock, 0, &err);
    m.init();
    for(;;){
        m.updateDuty();
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}  
void TEST_Motor_Init()
{
    OS_ERR err;
    OSTaskCreate(TEST_Task_Motor, (void*)0, &Stk_Motor1Init[MOTOR_STACK_SIZE-1], TASK_MOTOR1_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)1, &Stk_Motor2Init[MOTOR_STACK_SIZE-1], TASK_MOTOR2_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)2, &Stk_Motor3Init[MOTOR_STACK_SIZE-1], TASK_MOTOR3_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)3, &Stk_Motor4Init[MOTOR_STACK_SIZE-1], TASK_MOTOR4_PRIO);
    OSTaskNameSet(TASK_MOTOR1_PRIO,(INT8U*)"MOTOR1",&err);
    OSTaskNameSet(TASK_MOTOR2_PRIO,(INT8U*)"MOTOR2",&err);
    OSTaskNameSet(TASK_MOTOR3_PRIO,(INT8U*)"MOTOR3",&err);
    OSTaskNameSet(TASK_MOTOR4_PRIO,(INT8U*)"MOTOR4",&err);
    //使能接收机输入捕获中断
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
}

#endif