#include "os.h"
#include "common.h"
#include "config.h"

extern void doNothing(void);

#if (TEST_PID_EN > 0u) || (TEST_PID3_EN > 0)
    #define Test_PID_Init TEST_PID_Init
    #define TASK_PID_PITCH_EXTERNAL_PRIO 18u
    #define TASK_PID_ROLL_EXTERNAL_PRIO 19u
    #define TASK_PID_PITCH_INTERNAL_PRIO 20u
    #define TASK_PID_ROLL_INTERNAL_PRIO 21u
#else
    #define Test_PID_Init doNothing
#endif

#if TEST_MOTOR_EN > 0u
    #define Test_Motor_Init TEST_Motor_Init
    #define TASK_MOTOR1_PRIO 14u
    #define TASK_MOTOR2_PRIO 15u
    #define TASK_MOTOR3_PRIO 16u
    #define TASK_MOTOR4_PRIO 17u
    void pitch_set_pwm(float pid_out);
    void roll_set_pwm(float pid_out);
    void yaw_set_pwm(float pid_out);
#else
    #define Test_Motor_Init doNothing
#endif

#if TEST_MADGRICK_EN > 0u
    #define Test_Madgwick_Init TEST_Madgwick_Init
    #define TASK_MADGWICK_READ_PRIO 22u
#else
    #define Test_Madgwick_Init doNothing
#endif

#if TEST_MAHONY_EN > 0u
    #define Test_Mahony_Init TEST_Mahony_Init
    #define TASK_MAHONY_READ_PRIO 22u

#else
    #define Test_Mahony_Init doNothing
#endif

#if TEST_ANOTC_EN > 0u
    #define TASK_ANOTC_INFO_TRAN_PRIO 23u
    #define Test_Anotc_Conn_Init TEST_Anotc_Conn_Init
#else
    #define Test_Anotc_Conn_Init doNothing
#endif

#if TEST_PRINT_EN > 0u
    #define TASK_PRINT_PRIO 9u
    #define Test_Print_Init TEST_Print_Init
#else
    #define Test_Print_Init doNothing
#endif

#if TEST_RECEIVER_EN > 0u
    #define TASK_RECEIVER_PRIO 13u
    #define Test_Receiver_Init TEST_Receiver_Init
#else
    #define Test_Receiver_Init doNothing
#endif

/**
* @brief gy-86数据采集
*/

extern void TEST_RK4_Init(void);

extern void TEST_PID_Init(void);

extern void TEST_Motor_Init(void);

extern void TEST_Madgwick_Init(void);

extern void TEST_Mahony_Init(void);

extern void TEST_Anotc_Conn_Init(void);

extern void TEST_shell(void);

extern void TEST_Print_Init(void);

extern void TEST_Fs(void);

extern void TEST_Receiver_Init(void);