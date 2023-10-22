#include "stm32f4xx.h"

#include "motor.hpp"

static uint8_t TIM1_CH1_CAPTURE_STATUS = 0;   //捕获下降沿后置为0，上升沿后置为1
static uint16_t TIM1_CH1_CAPTURE_VAL=0;
float CAPTURE_PWM = 0.0f;

extern motor* motorObjList[4];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		
		if(TIM1_CH1_CAPTURE_STATUS)  //捕获到下降沿
		{
				TIM1_CH1_CAPTURE_VAL = TIM1->CCR1;    //记录当前捕获值
				TIM1->CCER &= ~(3<<1);									//设置为上升沿触发
				TIM1_CH1_CAPTURE_STATUS = 0;						
		}
		else												
		{
			TIM1_CH1_CAPTURE_STATUS = 1;					//ÉÏÉýÑØÖÃ1
			TIM1->CR1 &= ~(1<<0);									//¹Ø±Õ¼ÆÊ±Æ÷
			
			CAPTURE_PWM = (float)TIM1_CH1_CAPTURE_VAL / TIM1->CCR1;	//¼ÆËãPWMÖÜÆÚ
			for(int i=0;i<4;++i)
				motorObjList[i]->setDuty(CAPTURE_PWM / 10.0f);
			
			TIM1->CNT = 0;												//Çå¿Õ¼ÆÊ±Æ÷
			TIM1->CCER |= (1<<1);									//ÉèÖÃÎªÏÂ½µÑØ´¥·¢
			
			TIM1->CR1 |= (1<<0);									//¿ªÆô¼ÆÊýÆ÷
			
		}	
		TIM1->SR = 0;	
	}
}