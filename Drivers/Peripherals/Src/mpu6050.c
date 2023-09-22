#include "stm32f4xx.h"
#include "i2c.h"  /* hi2c1 */
#include "hard_i2c.h"
#include "mpu6050.h"

enum FULL_Scale_Range{
    FS_SEL_250 = 250,
    FS_SEL_500 = 500,
    FS_SEL_1000 = 1000,
    FS_SEL_2000 = 2000,
    AFS_SEL_2g = 2,
    AFS_SEL_4g = 4,
    AFS_SEL_8g = 8,
    AFS_SEL_16g = 16
} _FS_SEL = FS_SEL_500, _AFS_SEL = AFS_SEL_4g;


#define MPU6050_CONFIG 0x1A //低通滤波器 和 external Frame Synchronization (FSYNC)
#define MPU6050_SMPRT_DIV 0x19 //分频采样
#define MPU6050_GYROSCOPE_CONFIG 0x1B //陀螺仪自检和量程
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_POWER_MANAGER_1 0x6B   //设置晶振 电源模式 重设设备
#define MPU6050_POWER_MANAGER_2 0x6C   //所有传感器的挂起管理

#define MPU6050_PWR_MGMT_1_TEMP_DIS (1u << 3)

#define MPU6050_ADDRESS_W   0xD0
#define MPU6050_ADDRESS_R   0xD1

#define MPU6050_GYRO_XYZ_BASE 0x43
#define MPU6050_ACCELEROMETER 0x3B

#define PI 3.141592654

uint8_t MPU6050_Read_Buffer[14];
uint8_t* _accelerometer_data_ptr;
uint8_t* _gyroscope_data_ptr;

void MPU6050_Init()
{
    //内部晶振设为8Mhz 不睡眠 不循环 关闭温度采样
    uint8_t cfgFreq[2] = {MPU6050_POWER_MANAGER_1,MPU6050_PWR_MGMT_1_TEMP_DIS};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgFreq,2);

    //分频采样 陀螺仪采样率降低为1kHz
    uint8_t cfgScaler[2] = {MPU6050_SMPRT_DIV,0x01};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgScaler,2);

    //不低通滤波
    uint8_t cfgSam[2] = {MPU6050_CONFIG,0x00};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgSam,2);

    //陀螺仪量程选用+-500°
    uint8_t cfgTuo[2] = {MPU6050_GYROSCOPE_CONFIG,0x08};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgTuo,2);

    //加速度计量程选用+-4g
    uint8_t cfgAccler[2] = {MPU6050_ACCEL_CONFIG,0x08};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,cfgAccler,2);

    //所有设备不进入挂起模式(standby mode)
    uint8_t pwr_mgmt2[2] = {MPU6050_POWER_MANAGER_2,0x00};
    I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,pwr_mgmt2,2);

    //定义采集数据指针
    _accelerometer_data_ptr = &MPU6050_Read_Buffer[0];
    _gyroscope_data_ptr = &MPU6050_Read_Buffer[8];
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

uint8_t MPU6050_Get_Gyroscope(double* alpha,double* beta,double* gama)
{
    short xout,yout,zout;
    xout = _gyroscope_data_ptr[0]<<8 | _gyroscope_data_ptr[1];
    yout = _gyroscope_data_ptr[2]<<8 | _gyroscope_data_ptr[3];
    zout = _gyroscope_data_ptr[4]<<8 | _gyroscope_data_ptr[5];

    *alpha = (xout) / (65536.0f / (_FS_SEL*2)) / 180 * PI;
    *beta = (yout) / (65536.0f / (_FS_SEL*2)) / 180 * PI;
    *gama = (zout) / (65536.0f / (_FS_SEL*2)) / 180 * PI;

    return 0;
}

int MPU6050_Read_Data()
{
    MPU6050_Read(MPU6050_Read_Buffer, MPU6050_ACCELEROMETER, 14);
    return 0;
}

int MPU6050_Get_Accelerometer(double* ax,double* ay,double* az)
{
    short xout,yout,zout;
    xout = (_accelerometer_data_ptr[0] << 8) | _accelerometer_data_ptr[1];
    yout = (_accelerometer_data_ptr[2] << 8) | _accelerometer_data_ptr[3];
    zout = (_accelerometer_data_ptr[4] << 8) | _accelerometer_data_ptr[5];

    *ax = xout / ((65535.0f) / (_AFS_SEL*2));
    *ay = yout / ((65535.0f) / (_AFS_SEL*2));
    *az = zout / ((65535.0f) / (_AFS_SEL*2));
}