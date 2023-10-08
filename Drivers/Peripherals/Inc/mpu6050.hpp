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

#include <stdint.h>
#include "i2c.h"
#include "quaternion.hpp"

class mpu6050{

private:

    uint8_t buffer[14];
    uint8_t* _accelerometer_data_ptr;
    uint8_t* _gyroscope_data_ptr;
    double gk;
    double ak;

public:
    mpu6050()
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

        uint8_t user_mode[2] = {0x6A,0x00};
        I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,user_mode,2);
        uint8_t user_mode2[2] = {0x37,0x02};
        I2C_Write((uint32_t)I2C1,MPU6050_ADDRESS_W,user_mode2,2);

        //定义采集数据指针
        _accelerometer_data_ptr = &buffer[0];
        _gyroscope_data_ptr = &buffer[8];

        //定义转换系数
        /* gyro = readData * k */
        gk = PI / 180 / 65536.0f * 1000;

        /* acce = readData * k */
        ak = 1 / 65536.f * 8;
    }

    void read(uint8_t* buf,uint8_t pos,uint8_t size){
        I2C_Transfer((uint32_t)I2C1,MPU6050_ADDRESS_W,&pos,1,MPU6050_ADDRESS_R,buf,size);
    }

    void update(){
        read(buffer, MPU6050_ACCELEROMETER, 14);
    }

    quaternion get_gyro(){
        short xout,yout,zout;
        xout = _gyroscope_data_ptr[0]<<8 | _gyroscope_data_ptr[1];
        yout = _gyroscope_data_ptr[2]<<8 | _gyroscope_data_ptr[3];
        zout = _gyroscope_data_ptr[4]<<8 | _gyroscope_data_ptr[5];
        return quaternion{0,xout * gk,yout * gk,zout * gk};
    }

    quaternion get_accelero(){
        short xout,yout,zout;
        xout = (_accelerometer_data_ptr[0] << 8) | _accelerometer_data_ptr[1];
        yout = (_accelerometer_data_ptr[2] << 8) | _accelerometer_data_ptr[3];
        zout = (_accelerometer_data_ptr[4] << 8) | _accelerometer_data_ptr[5];
        return quaternion{0,xout * ak,yout * ak,zout * ak};
    }
};