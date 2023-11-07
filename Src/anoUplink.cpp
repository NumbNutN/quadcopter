#include "quaternion.hpp"
#include "quat_math.hpp"

#include "stm32f4xx.h"
#include "usart.h"
#include "os.h"

void ano_send_dataFrame(quaternion& attitude){
    static uint8_t frame[13] = {0xAA,0xFF,0x03,0x07,0,0,0,0,0,0,0x01};

    int16_t roll = quat_get_Roll(attitude) * 100 * 57.3;
    int16_t pitch = quat_get_Pitch(attitude) * 100 * 57.3;
    int16_t yaw = quat_get_Yaw(attitude) * 100 * 57.3;

    *(int16_t*)(frame + 4) = roll;
    *(int16_t*)(frame + 6) = pitch;
    *(int16_t*)(frame + 8) = yaw;

    frame[11] = 0;
    frame[12] = 0;

    for(int i=0;i<4+7;i++){
        frame[11] += frame[i];
        frame[12] += frame[11];
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)frame, 13, HAL_MAX_DELAY);

}

void ano_send_quaternion(quaternion& attiitude){
    OS_CPU_SR cpu_sr;
    static uint8_t frame[15] = {0xAA,0xFF,0x04,0x09,0,0,0,0,0,0,0,0,0x01};
    *(int16_t*)(frame + 4) = attiitude[0]*10000;
    *(int16_t*)(frame + 6) = attiitude[1]*10000;
    *(int16_t*)(frame + 8) = attiitude[2]*10000;
    *(int16_t*)(frame + 10) = attiitude[3]*10000;

    frame[13] = 0;
    frame[14] = 0;
    for(int i=0;i<4+9;i++){
        frame[13] += frame[i];
        frame[14] += frame[13];
    }
    OS_ENTER_CRITICAL();
    HAL_UART_Transmit(&huart1, (uint8_t*)frame, 15, HAL_MAX_DELAY);
    OS_EXIT_CRITICAL();
}


