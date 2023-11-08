#ifndef _MPU6050_H
#define _MPU6050_H

#include "stdint.h"

extern uint8_t MPU6050_Read_Buffer[14];

void MPU6050_Init();

void MPU6050_Read(uint8_t* buf,uint8_t pos,uint8_t size);

void MPU6050_Read_Scaler(uint8_t* buf,uint8_t* pos,uint8_t size)__attribute__((deprecated));

/**
 * @brief Count Gyroscope data from mpu6050
*/
uint8_t MPU6050_Get_Gyroscope(double* alpha,double* beta,double* gama);

/**
 * @brief Count Accelerometer data from mpu6050
*/
int MPU6050_Get_Accelerometer(double* ax,double* ay,double* az);

/**
 * @brief perfrom a read buffer
*/
int MPU6050_Read_Data();

#endif