#ifndef _CONFIG_H
#define _CONFIG_H

/* define used i2c interface */
#define I2C_USE_SOFT


#define TEST_EN 1u

#if TEST_EN > 0u
    #define TEST_PID3_EN        1u

    #ifdef TEST_PID3_EN
        #define PID3_SERIE      1u
        #define PID3_SINGLE     0u
    #endif
    
    #define TEST_MOTOR_EN       1u
    #define TEST_MADGRICK_EN    0u
    #define TEST_MAHONY_EN      1u

    #define TEST_ANOTC_EN       0u
    #define TEST_SHELL_EN       1u
    #define TEST_PRINT_EN       1u

    #define TEST_STREAM_EN      1u


#endif

#endif