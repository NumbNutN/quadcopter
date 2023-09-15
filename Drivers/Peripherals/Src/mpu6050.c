#include "stm32f4xx.h"
#include "i2c.h"  /* hi2c1 */
#include "hard_i2c.h"
#include "mpu6050.h"

void MPU6050_Init()
{
    uint8_t cfgFreq[2] = {0x6B,0x00};   //内部晶振设为8Mhz
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgFreq,2);
    uint8_t cfgSam[2] = {0x1A,0x06};    //低采样率
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgSam,2);
    uint8_t cfgScaler[2] = {0x19,0x09}; //采样率分频
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgScaler,2);
    uint8_t cfgTuo[2] = {0x1B,0x18};    //陀螺仪
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgTuo,2);
    uint8_t cfgAccler[2] = {0x1C,0x00};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgAccler,2);
}


void MPU6050_Read(uint8_t* buf,uint8_t pos,uint8_t size)
{
    I2C_Transfer((uint32_t)I2C1,MPU6050_ADDRESS_W,&pos,1,MPU6050_ADDRESS_R,buf,size);
}


void MPU6050_Read_Scaler(uint8_t* buf,uint8_t* pos,uint8_t size)
{
    for(size_t i=0;i<size;++i)
    {
        I2C_Transfer((uint32_t)I2C1, MPU6050_ADDRESS_W, &pos[i], 1, MPU6050_ADDRESS_R, &buf[i], 1);
    }

}