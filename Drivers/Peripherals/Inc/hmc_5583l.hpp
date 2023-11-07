#pragma once

#include "stm32f4xx.h"
#include "i2c.h"
#include "common.h"
#include "delay.h"

#include <iostream>

#include "quaternion.hpp"

using namespace std;

#define HMC5883L_ADDRESS_W 0x3C
#define HMC5883L_ADDRESS_R 0x3D

using namespace std;

class hmc_558l{
private:

    int8_t buffer[6];

public:
    hmc_558l(){
        uint8_t config[2] = {0x02,0x00};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config,2);
        delay_ms(67);
        uint8_t config2[2] = {0x00,0x70};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config2,2);
        delay_ms(67);
        update();
    }

    void update(){
        uint8_t reg = 0x03;
        I2C_Transfer((uint32_t)I2C1,HMC5883L_ADDRESS_W,&reg,1,HMC5883L_ADDRESS_R,(uint8_t*)buffer,6);
    }

    quaternion getMagnetic(){
        return quaternion{
            0,
            ((buffer[0] << 8) | buffer[1]) / 4096.0f * 1.76,
            ((buffer[4] << 8) | buffer[5]) / 4096.0f * 1.76,
            ((buffer[2] << 8) | buffer[3]) / 4096.0f * 1.76,
        };
    }

    quaternion getMagneticWithoutZ(){
        return quaternion{
            0,
            ((buffer[0] << 8) | buffer[1]) / 4096.0f * 1.76,
            ((buffer[4] << 8) | buffer[5]) / 4096.0f * 1.76,
            0
        }.normalization();
    }

    ~hmc_558l() = default;

};