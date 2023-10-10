#ifndef _CONFIG_H
#define _CONFIG_H

/* define used i2c interface */
#define I2C_USE_SOFT




#define TEST_EN 1u

#if TEST_EN > 0u
    #define TEST_I2C_MPU6050_EN    0u
    #define TEST_RK4_EN         0u
    #define TEST_PRINT_ACCEL_EN         1u

    #define TEST_UART_EN        0u
    #define TEST_COUT_EN        0u
    #define TEST_QUAT_EN        0u
    #define TEST_HMC_EN         0u

#endif

#endif