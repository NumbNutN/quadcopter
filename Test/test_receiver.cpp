#include "config.h"

#if TEST_RECEIVER_EN > 0u

#include "test_tasks.h"

#include "motor.hpp"
#include "receiver.hpp"

#include "os.h"

#include <vector>

extern std::vector<motor> motorList;

#include "pid.hpp"

void controller_throttle(float sig){
    for(motor m:motorList)
        m.setDuty(sig);
}

extern pid ExternalControllerList[3];
void controller_move_forward(float sig){
    //定义映射空间
    constexpr float k_forward = 3.14*1/9/0.25;
    ExternalControllerList[0].setTarget((sig - 0.75)*k_forward);
}

void controller_move_laternal(float sig){
    constexpr float k_laternal   = 3.14/9/0.25;
    ExternalControllerList[1].setTarget((sig - 0.75)*k_laternal);
}

void controller_direction(float sig){
    yaw_set_pwm(sig / 10);
}

void unlock(){
  for(auto m : motorList){
    m.init();
  }
}

OS_STK Stk_Receiver[256];
receiver receiver_manager;
void task_controller_run(void* arg){
    while(1){
        receiver_manager.run();
        OSTimeDlyHMSM(0, 0, 0, 200);
    }
}

void TEST_Receiver_Init(){
  /*thruttle*/
  receiver_manager.register_handler(receiver::THRUTTLE_CHANNEL,controller_throttle);
  /*forward backward*/
  receiver_manager.register_handler(receiver::FORWARD_BACK, controller_move_forward);
  /*bend left bend right*/
  receiver_manager.register_handler(receiver::LATERAL_MOVE, controller_move_laternal);
  /*rotation*/
  receiver_manager.register_handler(receiver::DIRECTION, controller_direction);
  OSTaskCreate(task_controller_run, (void*)0, &Stk_Receiver[255], TASK_RECEIVER_PRIO);
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
  auto sig_buf = receiver_manager.getReceiverBuf();
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
      sig_buf[idx++] = cnt/(float)200;
    }
    TIM1->SR = 0;             /* 清除中断标志位 */
  }
  /* 错误检测 */
  //assert(idx <= 8 && "Err:Overflow");
  OSIntExit();
}

#endif