#pragma once

#include "stm32f4xx.h"
#include "i2c.h"
#include "common.h"
#include "delay.h"
#include "ssd1306_i2c.h"

#include <iostream>

#include <math.h>

#include "quaternion.hpp"


#define HMC5883L_ADDRESS_W 0x3C
#define HMC5883L_ADDRESS_R 0x3D

using namespace std;

class hmc_558l{
private:

    uint8_t buffer[6];
    quaternion _measured_earth_magnetic = {0,0,0,0};

    enum{
        CFG_REG1 = 0,
        CFG_REG2 = 1,
        MODER_REG = 2,
        STAT_REG = 9
    };

    enum{
        MODER_REG_RDY = (1 << 0u),
        MODER_REG_LOCK = (1 << 1u)
    };

    void _init(){
        uint8_t config[2] = {MODER_REG,0x00};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config,2);
        uint8_t config2[2] = {CFG_REG1,0x70};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config2,2);
        uint8_t config3[2] = {CFG_REG2,0x20};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config3,2);
        delay_ms(67); 
    }

    void _handle_stat(){
        //获得标志位
        uint8_t stat;
        uint8_t reg = STAT_REG;
        do{
            I2C_Transfer((uint32_t)I2C1,HMC5883L_ADDRESS_W,&reg,1,HMC5883L_ADDRESS_R,(uint8_t*)&stat,1);
        }while(!(stat & MODER_REG_RDY));
    }

public:
    hmc_558l(){
        _init();
        update();
    }

    void update(){
        _handle_stat();
        uint8_t reg = 0x03;
        I2C_Transfer((uint32_t)I2C1,HMC5883L_ADDRESS_W,&reg,1,HMC5883L_ADDRESS_R,(uint8_t*)buffer,6);
    }

    void test(){
        OS_CPU_SR cpu_sr;
        OS_ENTER_CRITICAL();
        //执行自测模式  
        //选择出场默认LSB
        uint8_t config3[2] = {CFG_REG2,0x40};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config3,2);
        //选择缺省 采集周期 打开自测位
        uint8_t config2[2] = {CFG_REG1,0x71};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config2,2);
        //选择模式为单次
        uint8_t config1[2] = {MODER_REG,0x01};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config1,2);
        delay_ms(67);

        //第一次读取，获得正偏置之前的地磁场
        update();
        quaternion test_dat = {
            0,
            (float)((buffer[0] << 8) | buffer[1]) / 820.0f,
            (float)((buffer[4] << 8) | buffer[5]) / 820.0f,
            (float)((buffer[2] << 8) | buffer[3]) / 820.0f,            
        };

        OLED_Clean();
        cout << "T " << test_dat << endl;
        cout << "Test over" << endl;

        _init();
        OS_EXIT_CRITICAL();
    }

    quaternion getMagnetic(){
        return quaternion{
            0,
            (float)((int16_t)((buffer[0] << 8) | buffer[1])) / 1090.0f,
            (float)((int16_t)((buffer[4] << 8) | buffer[5])) / 1090.0f,
            (float)((int16_t)((buffer[2] << 8) | buffer[3])) / 1090.0f,
        };
    }

    void measure_earth_magnetic(size_t times) {
        int64_t totalX = 0, totalY = 0, totalZ = 0;
        for(int i=0;i<times;++i){
            update();
            totalX += (short)(buffer[0] << 8 | buffer[1]);
            totalY += (short)(buffer[4] << 8 | buffer[5]);
            totalZ += (short)(buffer[2] << 8 | buffer[3]);
        }
        _measured_earth_magnetic[0] = 0;
        //_measured_earth_magnetic[1] = sqrt(pow(totalX / (float)times / 4096.0f * 1.76,2)+pow(totalY / (float)times / 4096.0f * 1.76,2));
        _measured_earth_magnetic[1] = totalX / (float)times / 4096.0f * 1.76;
        _measured_earth_magnetic[2] = totalY / (float)times / 4096.0f * 1.76;
        _measured_earth_magnetic[3] = totalZ / (float)times / 4096.0f * 1.76;
    }

    const quaternion& get_measured_earth_magnetic(){
        return _measured_earth_magnetic;
    }

    ~hmc_558l() = default;

};