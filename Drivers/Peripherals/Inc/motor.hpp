#include "stm32f4xx.h"
#include "tim.h"

#include "os.h"

class motor{

private:
    GPIO_TypeDef * port;
    uint16_t pin;
    TIM_TypeDef * timer;
	TIM_HandleTypeDef* htim;
	uint32_t channel;

public:
    // motor(GPIO_TypeDef * port,uint16_t pin,TIM_TypeDef * timer) :port(port),pin(pin),timer(timer) {
	// 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    // };

	motor(TIM_HandleTypeDef* htim,uint32_t channel) :htim(htim),channel(channel){
		
		//设定motor最高阈值
		setDuty(0.1);
		HAL_TIM_PWM_Start(htim, channel);
		OSTimeDlyHMSM(0, 0, 3, 0);

		//设定最高阈值
		setDuty(0.05);
		OSTimeDlyHMSM(0, 0, 3, 0);

		//回到最低阈值
		setDuty(0.08);

	}

	void setDuty(float dutyRate){
		__HAL_TIM_SET_COMPARE(htim,channel,dutyRate*htim->Instance->ARR);
	}

	void setAngularVehicle(float omega){
		setDuty(8.607e-5 * omega + 0.05);
	}

};