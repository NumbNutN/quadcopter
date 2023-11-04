#include "config.h"
#include "motor.hpp"
motor* motorObjList[4];
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
        m.updateDuty();
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}
void TEST_Motor_Init()
{
    OS_ERR err;
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_1, &Stk_Motor1Init[MOTOR_STACK_SIZE-1], TASK_MOTOR1_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_2, &Stk_Motor2Init[MOTOR_STACK_SIZE-1], TASK_MOTOR2_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_3, &Stk_Motor3Init[MOTOR_STACK_SIZE-1], TASK_MOTOR3_PRIO);
    OSTaskCreate(TEST_Task_Motor, (void*)TIM_CHANNEL_4, &Stk_Motor4Init[MOTOR_STACK_SIZE-1], TASK_MOTOR4_PRIO);
    OSTaskNameSet(TASK_MOTOR1_PRIO,(INT8U*)"MOTOR1",&err);
    OSTaskNameSet(TASK_MOTOR2_PRIO,(INT8U*)"MOTOR2",&err);
    OSTaskNameSet(TASK_MOTOR3_PRIO,(INT8U*)"MOTOR3",&err);
    OSTaskNameSet(TASK_MOTOR4_PRIO,(INT8U*)"MOTOR4",&err);
    //使能接收机输入捕获中断
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
}

uint32_t channel_signal[8];
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
