#include <stdint.h>

#include "os.h"
#include "stm32f4xx.h"
#include "test_tasks.h"

#include "i2c.h"


void Task_Init(void){

    //TASK_BEFORE_OS_INIT

    /* task for testing initialization */
    
    //MPU6050 Read Data
    Test_Task_MPU6050_Get_Data_Init();
}

/**
 * @brief Initial the peripherals hardware 
*/
void Peripherals_Init(void){

    I2C_Init((uint32_t)I2C1);

}

