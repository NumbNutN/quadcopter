#include <stdint.h>

#include "os.h"
#include "stm32f4xx.h"
#include "test_tasks.h"

#include "i2c.h"
#include "mpu6050.h"
#include "ssd1306_i2c.h"



void Task_Init(void){

    //TASK_BEFORE_OS_INIT

}

/**
 * @brief Initial the peripherals hardware 
*/
void Peripherals_Init(void){

    I2C_Init((uint32_t)I2C1);
    // SSD1306_Init();
    // OLED_Clean();

}

void Test_Task_Init(void){
    //MPU6050 Read Data
    Test_Task_MPU6050_Get_Data_Init();
    Test_RK4_Init();
    Test_Print_Accel_Init();
    Test_PID_Init();
    Test_Motor_Init();
    Test_Madgwick_Init();
    Test_Mahony_Init();
    Test_Anotc_Conn_Init();
}

void Test_Bf_OS(void){
    test_uart();
    test_stream();
    test_quaternion();
    test_hmc();
}
