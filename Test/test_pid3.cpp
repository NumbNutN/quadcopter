#include "config.h"

#if TEST_PID3_EN > 0u

#define TASK_PID_PITCH_EXTERNAL_PRIO 20u
#define TASK_PID_ROLL_EXTERNAL_PRIO 21u
#define TASK_PID_PITCH_INTERNAL_PRIO 22u
#define TASK_PID_ROLL_INTERNAL_PRIO 23u

#include <iostream>

#include "pid.hpp"
#include "quaternion.hpp"

#include "myMath.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "hmc_5583l.hpp"

#include "usart.h"
#include "os.h"
#include <math.h>

using namespace std;

extern quaternion cur_gyro;

extern quaternion attitude;

extern mpu6050* mpu6050_ptr;
extern hmc_558l* hmc_ptr;

OS_STK Stk_PID_Interal[1024];
OS_STK Stk_PID_Exteral[1024];


extern motor* motorObjList[4];

const float expK = 10000000;
float omega_g = 441.708435;

float k = 0.01;

void pitch_set_pwm(float alpha){
    motorObjList[0]->setAddDuty(- k*alpha);
    motorObjList[1]->setAddDuty(- k*alpha);
    motorObjList[3]->setAddDuty(k*alpha);
    motorObjList[2]->setAddDuty(k*alpha);
}

void roll_set_pwm(float alpha){
    motorObjList[1]->setAddDuty(- k*alpha);
    motorObjList[3]->setAddDuty(- k*alpha);
    motorObjList[2]->setAddDuty(k*alpha);
    motorObjList[0]->setAddDuty(k*alpha);
}

void yaw_set_pwm(float beta){

}

//内环PID p i d 获取当前角速度 计算出当前角加速度后的回调 获取当前导数（角加速度）
// 外环PID p i d 获取当前角度 计算出当前角速度后的回调  获取当前导数（角速度）

pid ExternalControllerList[3] = {
    pid(0.5,0.1,1,
                        []()->float{return mpu6050_ptr->get_current_gyro()[1];},
                        pitch_set_pwm),
    pid(0.5,0.1,1,
                        []()->float{return mpu6050_ptr->get_current_gyro()[2];},
                        roll_set_pwm),
    pid(0.5,0.1,1,
                        []()->float{return mpu6050_ptr->get_current_gyro()[3];},
                        yaw_set_pwm)
};

pid InternalControllerList[3] = {
    pid(0.5,0,0,
                    []() -> float{return quat_get_Pitch(attitude);},
                    [](float tar)->void{ExternalControllerList[0].setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[1];}),
    pid(0.5,0,0,
                    []() -> float{return quat_get_Roll(attitude);},
                    [](float tar)->void{ExternalControllerList[1].setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[2];}),
    pid(0.5,0,0,
                    []() -> float{return quat_get_Yaw(attitude);},
                    [](float tar)->void{ExternalControllerList[2].setTarget(tar);},
                    []()->float{return mpu6050_ptr->get_current_gyro()[3];})
};




OS_EVENT* pitch_internal_get_ref;
OS_EVENT* yaw_internal_get_ref;
OS_EVENT* roll_internal_get_ref;

#define THRESHOLE 0.02

void TEST_PID_EXTERNAL(void* arg){

    pid* controller = (pid*)arg;
    controller->setTarget(0);
    // OS_EVENT* sem = OSSemCreate(1);
    // INT8U err;
    for(;;)
    {
        //OSSemPend(sem,0,&err);
        controller->run();
        // cout << controller->getLabel() << ' '<< controller->getOutput() << endl;
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void TEST_PID_INTERNAL(void* arg){

    pid* controller = (pid*)arg;
    for(;;)
    {
        controller->run();
        // if(abs(controller->getCurError()) < THRESHOLE)
        //     OSSemPost();
        // if(controller->getLabel() == 'r')
        //     cout << controller->getOutput() << endl;
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

//命令格式要求 <e|i><p|i|d>"number"[+<e|i><p|i|d>"number"]*
char pid_command[17] = {'\0'};


void TEST_PID_Init()
{
    InternalControllerList[0].setLabel('p');
    InternalControllerList[1].setLabel('r');
    ExternalControllerList[0].setLabel('P');
    ExternalControllerList[1].setLabel('R');
    OSTaskCreate(TEST_PID_INTERNAL, &InternalControllerList[0], &Stk_PID_Interal[1023], TASK_PID_PITCH_INTERNAL_PRIO);
    OSTaskCreate(TEST_PID_INTERNAL, &InternalControllerList[1], &Stk_PID_Interal[511], TASK_PID_ROLL_INTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, &ExternalControllerList[0], &Stk_PID_Exteral[1023], TASK_PID_PITCH_EXTERNAL_PRIO);
    OSTaskCreate(TEST_PID_EXTERNAL, &ExternalControllerList[1], &Stk_PID_Exteral[511], TASK_PID_ROLL_EXTERNAL_PRIO);

    //设置PID 参数调节接收
    HAL_UART_Receive_IT(&huart1, (uint8_t*)pid_command, 16);
}



//串口回调函数
//严格格式  <e|i>' 'x.xx' 'x.xx' 'x.xx'
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
        pid_command[6] = '\0';
        pid_command[11] = '\0';
        pid_command[16] = '\0';
        char type = pid_command[0];
        if(type == 'e'){
            ExternalControllerList[0].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
            ExternalControllerList[1].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
        }
        else if(type == 'i'){
            InternalControllerList[0].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
            InternalControllerList[0].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
        }

        uint16_t data = USART1->DR;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)pid_command, 16);
	}
}

/**
  * @brief This function handles USART1 global interrupt.
  */
extern "C" void USART1_IRQHandler(void)
{
  size_t index = 0;
  /* USER CODE BEGIN USART1_IRQn 0 */
  OSIntEnter();
  /* USER CODE END USART1_IRQn 0 */
    do{
        pid_command[index++] = USART1->DR;
        while(!(USART1->SR & USART_SR_RXNE) && index<16){}
    }while(index != 16);
    
    pid_command[6] = '\0';
    pid_command[11] = '\0';  
    pid_command[16] = '\0';
    char type = pid_command[0];
    if(type == 'e'){
        ExternalControllerList[0].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
        ExternalControllerList[1].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
    }
    else if(type == 'i'){
        InternalControllerList[0].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
        InternalControllerList[0].setParam(strtod(&pid_command[2],NULL),strtod(&pid_command[7], NULL), strtod(&pid_command[12], NULL));
    }
  /* USER CODE BEGIN USART1_IRQn 1 */
  OSIntExit();
  /* USER CODE END USART1_IRQn 1 */
}

#endif