#ifndef _MPU6050_H
#define _MPU6050_H

#include "stdint.h"

void MPU6050_Init();

void MPU6050_Read(uint8_t* buf,uint8_t pos,uint8_t size);

void MPU6050_Read_Scaler(uint8_t* buf,uint8_t* pos,uint8_t size);

#endif