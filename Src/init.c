#include <stdint.h>

#include "os.h"
#include "stm32f4xx.h"
#include "test_tasks.h"

#include "i2c.h"


void Task_Init(void){

    //TASK_BEFORE_OS_INIT

}

/**
 * @brief Initial the peripherals hardware 
*/
void Peripherals_Init(void){

    I2C_Init((uint32_t)I2C1);

}

void Test_Task_Init(void){
    //MPU6050 Read Data
    Test_Task_MPU6050_Get_Data_Init();
}

void Test_Bf_OS(void){
    TEST_uart();
    TEST_SSD1306_PutChar();
}
