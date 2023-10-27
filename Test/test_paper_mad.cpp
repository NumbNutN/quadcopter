#include "config.h"

#if TEST_PAPER_MADGRICK_EN > 0u

#include <iostream>

#include "quaternion.hpp"
#include "myMath.hpp"
#include "mpu6050.hpp"
#include "hmc_5583l.hpp"
#include "gradient_decent.hpp"

#include "myMath.hpp"
#include <math.h>
#include <stdint.h>

#include "os.h"
#include "delay.h"

#define BETADEf 0.1f
#define RAD_TO_DEG 57.295780
#define DEG_TO_RAD 0.017453
#define ACCEL_CORRECTOR 0.000061
#define GYRO_CORRECTOR 0.007634

uint32_t madgwick_timer=0;

#define BETA 0.033
#define TASK_MADGWICK_READ_PRIO 19u
#define TASK_MADGWICK_PRINT_PRIO 18u

using namespace std;

OS_STK Stk_Madgwick[512];
OS_STK Stk_PrintMadgwick[512];

quaternion earth_magnetic{0,0.66436113595878288,0,0.74741172122703259};
mpu6050* mpu6050_ptr;
hmc_558l* hmc_ptr;

quaternion attitude_gyro;
quaternion acceleration_err_fun_gradient;
quaternion magnetic_err_func_gradient;
quaternion attitude_mixed;
float deltaT;
quaternion attitude = {1,0,0,0};
quaternion old_attitude = {1,0,0,0};

void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{

    float norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-attitude[1] * gx - attitude[2] * gy - attitude[3] * gz);
    qDot2 = 0.5f * (attitude[0] * gx + attitude[2] * gz - attitude[3] * gy);
    qDot3 = 0.5f * (attitude[0] * gy - attitude[1] * gz + attitude[3] * gx);
    qDot4 = 0.5f * (attitude[0] * gz + attitude[1] * gy - attitude[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * attitude[0];
        _2q1 = 2.0f * attitude[1];
        _2q2 = 2.0f * attitude[2];
        _2q3 = 2.0f * attitude[3];
        _4q0 = 4.0f * attitude[0];
        _4q1 = 4.0f * attitude[1];
        _4q2 = 4.0f * attitude[2];
        _8q1 = 8.0f * attitude[1];
        _8q2 = 8.0f * attitude[2];
        q0q0 = attitude[0] * attitude[0];
        q1q1 = attitude[1] * attitude[1];
        q2q2 = attitude[2] * attitude[2];
        q3q3 = attitude[3] * attitude[3];

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * attitude[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * attitude[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * attitude[3] - _2q1 * ax + 4.0f * q2q2 * attitude[3] - _2q2 * ay;
        norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 = s0 / norm;
        s1 = s1 / norm;
        s2 = s2 / norm;
        s3 = s3 / norm;

        // Apply feedback step
        qDot1 -= BETADEf * s0;
        qDot2 -= BETADEf * s1;
        qDot3 -= BETADEf * s2;
        qDot4 -= BETADEf * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    attitude[0] += qDot1 * dt;
    attitude[1] += qDot2 * dt;
    attitude[2] += qDot3 * dt;
    attitude[3] += qDot4 * dt;

    // Normalise quaternion
    norm = sqrt(attitude[0] * attitude[0] + attitude[1] * attitude[1] + attitude[2] * attitude[2] + attitude[3] * attitude[3]);
    attitude[0] = attitude[0] / norm;
    attitude[1] = attitude[1] / norm;
    attitude[2] = attitude[2] / norm;
    attitude[3] = attitude[3] / norm;

}

void TEST_Madgwick(void* arg){
    mpu6050 mpu6050_dev;
    hmc_558l hmc_dev;
    quaternion gyroData;
    mpu6050_ptr = &mpu6050_dev;
    hmc_ptr = &hmc_dev;

    // //校准陀螺仪的零偏误差
    // mpu6050_dev.alignment(100);

    for(;;){
        mpu6050_dev.update();
        hmc_dev.update();
        deltaT = mpu6050_dev.getSamplePeriod();
        
        quaternion accel = mpu6050_dev.get_acceleration();
        quaternion gyro = mpu6050_dev.get_current_gyro();

        MadgwickAHRSupdateIMU(accel[1],accel[2],accel[3],gyro[1],gyro[2],gyro[3],deltaT);
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

void TEST_Task_Print_Attitude(void* arg){
    for(;;)
    {
        ano_send_dataFrame(attitude);
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void TEST_Madgwick_Init()
{
    OSTaskCreate(TEST_Task_Print_Attitude,NULL,&Stk_PrintMadgwick[511],TASK_MADGWICK_PRINT_PRIO);
    OSTaskCreate(TEST_Madgwick, NULL, &Stk_Madgwick[511], TASK_MADGWICK_READ_PRIO);
}

#endif