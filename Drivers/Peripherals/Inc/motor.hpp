#pragma once

#include "stm32f4xx.h"
#include "tim.h"

#include "os.h"

class motor{

private:

	TIM_HandleTypeDef* htim;
	uint32_t channel;
	float _dutyCycle = 0.05f;  /* danger */

public:

	motor(TIM_HandleTypeDef* htim,uint32_t channel) :htim(htim),channel(channel){
		
		//设定motor最高阈值
		setDuty(0.1);updateDuty();
		HAL_TIM_PWM_Start(htim, channel);
		OSTimeDlyHMSM(0, 0, 3, 0);

		//设定最低阈值
		setDuty(0.05);updateDuty();
		OSTimeDlyHMSM(0, 0, 3, 0);

	}

	void setDuty(float dutyCycle){
		if(dutyCycle > 0.08) dutyCycle = 0.08;
		_dutyCycle = dutyCycle;
	}

	void updateDuty(){
		__HAL_TIM_SET_COMPARE(htim,channel,_dutyCycle*htim->Instance->ARR);
	}

	void setAngularVehicle(float omega){
		setDuty(4.30148e-5 * omega + 0.05);
	}

};