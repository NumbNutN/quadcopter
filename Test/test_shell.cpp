#include "config.h"

#if TEST_SHELL_EN > 0u

#include "stm32f4xx.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "shell.hpp"

/**
 * @brief 修改pid控制器参数 
 *        格式：pid <e|i> FLOAT FLOAT FLOAT
*/
#include "pid.hpp"
extern pid ExternalControllerList[3];
#if PID3_SERIE > 0u
extern pid InternalControllerList[3];
#endif
int Pid_Param_Update(int argc,char** argv){
    char type = argv[1][0];
    float kp = strtod(argv[2],NULL);
    float ki = strtod(argv[3],NULL);
    float kd = strtod(argv[4],NULL);
    if(type == 'e'){
        ExternalControllerList[1].setParam(kp,ki,kd);
        ExternalControllerList[0].setParam(kp,ki,kd);
    }
#if PID3_SERIE > 0u
    else if(type == 'i'){
        InternalControllerList[1].setParam(kp,ki,kd);
        InternalControllerList[0].setParam(kp,ki,kd);
    }
#endif
    return 0;
}

#if TEST_PID3_EN > 0u
extern uint8_t internal_pid_outputSig_AngularAcceleration_en;

int InternalPID_Data_Capture(int argc,char** argv){
    internal_pid_outputSig_AngularAcceleration_en = 1;
    return 0;
}
#endif

sh shell;
void TEST_shell(){
    /* 注册shell指令 */
#if TEST_PID3_EN > 0u
    shell.registerCmd((char*)"pid", Pid_Param_Update);
    shell.registerCmd((char*)"intcap", InternalPID_Data_Capture);
#endif
    //使能IDLE中断以在数据传输完成后陷入中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //使能DMA传输
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)shell.getCmdBuffer(), 23);
}

#endif