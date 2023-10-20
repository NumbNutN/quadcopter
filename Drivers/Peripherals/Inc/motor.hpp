#pragma once

#include "stm32f4xx.h"
#include "tim.h"

#include "os.h"

class motor{

private:

	TIM_HandleTypeDef* htim;
	uint32_t channel;
	float _dutyCycle = 0.05f;  /* danger */
	int8_t _label = '?';

public:

	motor(TIM_HandleTypeDef* htim,uint32_t channel) :htim(htim),channel(channel){
		
		//设定motor最高阈值
		__HAL_TIM_SET_COMPARE(htim,channel,0.1*htim->Instance->ARR);
		HAL_TIM_PWM_Start(htim, channel);
		OSTimeDlyHMSM(0, 0, 3, 0);

		//设定最低阈值
		__HAL_TIM_SET_COMPARE(htim,channel,0.05*htim->Instance->ARR);
		OSTimeDlyHMSM(0, 0, 3, 0);

		setDuty(0.067);updateDuty();

	}

	void setLabel(int8_t label) {
		_label = label;
	}

	void setDuty(float dutyCycle){
		if(dutyCycle > 0.071) dutyCycle = 0.071;
		else if(dutyCycle < 0.06) dutyCycle = 0.06;
		else _dutyCycle = dutyCycle;
	}

	void setAddDuty(float deltaDutyCycle){
		if(_dutyCycle + deltaDutyCycle > 0.071) _dutyCycle = 0.071;
		else if(_dutyCycle + deltaDutyCycle < 0.06) _dutyCycle = 0.06;
		else _dutyCycle += deltaDutyCycle;
	}

	void updateDuty(){
		__HAL_TIM_SET_COMPARE(htim,channel,_dutyCycle*htim->Instance->ARR);
	}

	void setAngularVehicle(float omega){
		setDuty(4.30148e-5 * omega + 0.05);
	}

	int8_t getLabel() const {
		return _label;
	}

	float get_dutyCycle() const {
		return _dutyCycle;
	}

};