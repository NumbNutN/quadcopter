#include "config.h"
#include "motor.hpp"
#if TEST_MOTOR_EN > 0u
#include "stm32f4xx.h"
#include <assert.h>
#include <iostream>
#include "os.h"
#include "delay.h"
#include "test_tasks.h"

#define MOTOR_STACK_SIZE 256
OS_STK Stk_Motor1Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor2Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor3Init[MOTOR_STACK_SIZE];
OS_STK Stk_Motor4Init[MOTOR_STACK_SIZE];

motor motorList[4] = {
  motor(&htim3,(uint32_t)TIM_CHANNEL_1,'1'),
  motor(&htim3,(uint32_t)TIM_CHANNEL_2,'2'),
  motor(&htim3,(uint32_t)TIM_CHANNEL_3,'3'),
  motor(&htim3,(uint32_t)TIM_CHANNEL_4,'4')
};

using namespace std;
void pedal_update(motor& m);

void TEST_Task_Motor(void* index){
    motor& m = motorList[(size_t)index];
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

uint32_t channel_signal[8];

void pedal_update(motor& m){
    float duty = channel_signal[2] /(float)200;
    if(duty < 0.5)duty=0.5;
    if(duty > 1)duty = 1;
    m.setDuty(duty);
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  *        这个中断只响应上升沿触发
  */
extern "C" void TIM1_CC_IRQHandler(void)
{
  uint32_t cnt;
  static size_t idx = 0;
  static float ppm_startsig_threshole = HAL_RCC_GetPCLK2Freq() / (float)(TIM1->PSC+1)*6e-3;
  OSIntEnter();
  
  if(TIM1->SR & TIM_SR_CC1IF)
  {
    cnt = TIM1->CCR1;         /* 获取当前上升沿的持续时间*/
    TIM1->CR1 &= ~TIM_CR1_CEN;/* 关闭定时器 */
    TIM1->CNT = 0;            /* 清空定时器 */
    TIM1->CR1 |= TIM_CR1_CEN; /* 使能定时器 */
    if(cnt > ppm_startsig_threshole){
      idx = 0;            /* 说明当前是ppm起始信号 */
      for(int i=0;i<4;++i)pedal_update(motorList[i]);
    }
    else if(idx<8){
      channel_signal[idx++] = cnt;
    }
    TIM1->SR = 0;             /* 清除中断标志位 */
  }
  /* 错误检测 */
  //assert(idx <= 8 && "Err:Overflow");
  OSIntExit();
}
#endif
