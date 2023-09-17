#ifndef _MPU6050_H
#define _MPU6050_H

#define MPU6050_ADDRESS_W   0xD0
#define MPU6050_ADDRESS_R   0xD1

#define MPU6050_GYRO_XYZ_BASE 0x43

#include "stdint.h"

void MPU6050_Init();

void MPU6050_Read(uint8_t* buf,uint8_t pos,uint8_t size);

void MPU6050_Read_Scaler(uint8_t* buf,uint8_t* pos,uint8_t size);

/**
 * @brief read Gyroscope data from mpu6050
*/
uint8_t MPU6050_Get_Gyroscope(double* alpha,double* beta,double* gama);

#endif