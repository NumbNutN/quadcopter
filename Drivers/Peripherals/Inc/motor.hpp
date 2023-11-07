#pragma once
#include "stm32f4xx.h"
#include "tim.h"
#include "os.h"
#define MAX_CYCLE_DUTY 0.8
#define MIN_CYCLE_DUTY 0.5

#define MAX_ADD_CYCLE_DUTY 0.15
#define MIN_ADD_CYCLE_DUTY 0
class motor{
private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    float _dutyCycleBase = 0.5f;  /* danger */
    float _dutyCycleAdd = 0.0f;
	float _dutyCycleAdd2 = 0.0f;
    int8_t _label = '?';
public:
    motor(TIM_HandleTypeDef* htim,uint32_t channel) :htim(htim),channel(channel){
        
        //设定motor最高阈值
        __HAL_TIM_SET_COMPARE(htim,channel,1*htim->Instance->ARR);
        HAL_TIM_PWM_Start(htim, channel);
        OSTimeDlyHMSM(0, 0, 3, 0);
        //设定最低阈值
        __HAL_TIM_SET_COMPARE(htim,channel,0.5*htim->Instance->ARR);
        OSTimeDlyHMSM(0, 0, 3, 0);
        setDuty(0.5);updateDuty();
    }
    void setLabel(int8_t label) {
        _label = label;
    }
    void setDuty(float dutyCycle){
        if(dutyCycle+_dutyCycleAdd > MAX_CYCLE_DUTY) _dutyCycleBase = MAX_CYCLE_DUTY-_dutyCycleAdd;
        else if(dutyCycle+_dutyCycleAdd < MIN_CYCLE_DUTY) _dutyCycleBase = MIN_CYCLE_DUTY-_dutyCycleAdd;
        else _dutyCycleBase = dutyCycle;
    }
    void setAddDuty(float deltaDutyCycle){
        // if(_dutyCycleBase + deltaDutyCycle > MAX_CYCLE_DUTY) _dutyCycleAdd = MAX_CYCLE_DUTY - _dutyCycleBase;
        // else if(_dutyCycleBase + deltaDutyCycle < MIN_CYCLE_DUTY) _dutyCycleAdd = MIN_CYCLE_DUTY - _dutyCycleBase;
		if(deltaDutyCycle > MAX_ADD_CYCLE_DUTY)_dutyCycleAdd=MAX_ADD_CYCLE_DUTY;
		else if(deltaDutyCycle < -MAX_ADD_CYCLE_DUTY)_dutyCycleAdd=-MAX_ADD_CYCLE_DUTY;
    	else _dutyCycleAdd = deltaDutyCycle;
    }

	void setAddDuty2(float deltaDutyCycle){
		if(deltaDutyCycle > MAX_ADD_CYCLE_DUTY)_dutyCycleAdd2=MAX_ADD_CYCLE_DUTY;
		else if(deltaDutyCycle < -MAX_ADD_CYCLE_DUTY)_dutyCycleAdd2=-MAX_ADD_CYCLE_DUTY;
    	else _dutyCycleAdd2 = deltaDutyCycle;
	}
    void updateDuty(){
        __HAL_TIM_SET_COMPARE(htim,channel,(_dutyCycleBase+_dutyCycleAdd+_dutyCycleAdd2)*htim->Instance->ARR);
    }
    void setAngularVehicle(float omega){
        setDuty(4.30148e-5 * omega + 0.05);
    }
    int8_t getLabel() const {
        return _label;
    }
    float get_dutyCycle() const {
        return _dutyCycleBase+_dutyCycleAdd;
    }
};