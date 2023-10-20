#include "os.h"
#include "common.h"
#include "config.h"

extern void doNothing(void);

#if TEST_I2C_MPU6050_EN > 0u
    #define Test_Task_MPU6050_Get_Data_Init TEST_Task_MPU6050_Get_Data_Init

    #define TASK_MPU6050_GET_DATA_STACK_SIZE 1024
    #define TASK_MPU6050_GET_DATA_PRIO 10u
    #define TASK_COUNT_EULERANGLE_PRIO 11u
    #define TASK_PRINT_EULERANGLE_PRIO 9u

#else
    #define Test_Task_MPU6050_Get_Data_Init doNothing
#endif

#if TEST_RK4_EN > 0
    #define Test_RK4_Init TEST_RK4_Init
#else
    #define Test_RK4_Init doNothing
#endif

#if TEST_UART_EN > 0u
    #define test_uart TEST_uart
#else
    #define test_uart doNothing
#endif

#if TEST_COUT_EN > 0u
    #define test_stream TEST_stream
#else 
    #define test_stream doNothing
#endif

#if TEST_QUAT_EN > 0u
    #define test_quaternion TEST_quaternion
#else
    #define test_quaternion doNothing
#endif

#if TEST_HMC_EN > 0u
    #define test_hmc TEST_hmc
#else
    #define test_hmc doNothing
#endif

#if TEST_PRINT_ACCEL_EN > 0u
    #define Test_Print_Accel_Init TEST_Print_Accel_Init
#else
    #define Test_Print_Accel_Init doNothing
#endif

#if (TEST_PID_EN > 0u) || (TEST_PID3_EN > 0)
    #define Test_PID_Init TEST_PID_Init
#else
    #define Test_PID_Init doNothing
#endif

#if TEST_MOTOR_EN > 0u
    #define Test_Motor_Init TEST_Motor_Init
#else
    #define Test_Motor_Init doNothing
#endif

#if TEST_MADGRICK_EN > 0u
    #define Test_Madgwick_Init TEST_Madgwick_Init
#else
    #define Test_Madgwick_Init doNothing
#endif

/**
* @brief gy-86数据采集
*/
extern void TEST_Task_MPU6050_Get_Data_Init(void);

extern void test_cpp(void);

extern void test_uart(void);

extern void test_stream(void); 

extern void test_quaternion(void);

extern void test_hmc(void);

extern void TEST_RK4_Init(void);

extern void TEST_Print_Accel_Init(void);

extern void TEST_PID_Init(void);

extern void TEST_Motor_Init(void);

extern void TEST_HMC_Init(void);

extern void TEST_Madgwick_Init(void);