#ifndef _CONFIG_H
#define _CONFIG_H

/* define used i2c interface */
#define I2C_USE_HARD




#define TEST_EN 1u

#if TEST_EN > 0u
    #define TEST_I2C_MPU6050_EN    1u
#endif

#endif