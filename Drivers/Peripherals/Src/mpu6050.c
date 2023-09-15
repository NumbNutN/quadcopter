#include "stm32f4xx.h"
#include "i2c.h"  /* hi2c1 */
#include "hard_i2c.h"
#include "mpu6050.h"

enum FULL_Scale_Range{
    FS_SEL_250 = 250,
    FS_SEL_500 = 500,
    FS_SEL_1000 = 1000,
    FS_SEL_2000 = 2000
} _FS_SEL = FS_SEL_500;

void MPU6050_Init()
{
    //FS_SEL 
    uint8_t cfgFreq[2] = {0x6B,0x00};   //内部晶振设为8Mhz
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgFreq,2);
    uint8_t cfgSam[2] = {0x1A,0x06};    //低采样率
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgSam,2);
    uint8_t cfgScaler[2] = {0x19,0x09}; //采样率分频
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgScaler,2);
    uint8_t cfgTuo[2] = {0x1B,0x08};    //陀螺仪
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

uint8_t MPU6050_Get_Gyroscope(float* alpha,float* beta,float* gama)
{
    short xout,yout,zout;
    uint8_t buffer[6];
    MPU6050_Read(buffer,MPU6050_GYRO_XYZ_BASE,6);
    xout = buffer[0]<<8 | buffer[1];
    yout = buffer[2]<<8 | buffer[3];
    zout = buffer[4]<<8 | buffer[5];

    *alpha = xout / (65536.0f / (_FS_SEL*2));
    *beta = yout / (65536.0f / (_FS_SEL*2));
    *gama = zout / (65536.0f / (_FS_SEL*2));

    return 0;
}