#ifndef _HARD_I2C_H
#define _HARD_I2C_H

#define MPU6050_ADDRESS_W   0xD0
#define MPU6050_ADDRESS_R   0xD1

enum I2C_RECV_SEQ{
    I2C_NORMAL_RECV,
    I2C_READ_AFTER_WRITE_SEQUENCE
};

#define I2C_EV5     do{}while(!Hard_I2CStatusCheck(I2C_SR1_SB,0))     //读取SR1后将地址写至DR清除
#define I2C_EV6     do{}while(!Hard_I2CStatusCheck(I2C_SR1_ADDR,0))   //读取SR1后读取SR2清除
//在写入数据前检测
#define I2C_EV8_1   do{}while(!Hard_I2CStatusCheck(I2C_SR1_TXE,0))     //向DR写入值清0?
//在写入数据后检测
#define I2C_EV8     do{}while(!Hard_I2CStatusCheck(I2C_SR1_TXE,0))     //不必清除
//停止位发生前判断总线无数据
#define I2C_EV8_2   do{}while(!Hard_I2CStatusCheck(I2C_SR1_TXE|I2C_SR1_BTF,0))     //发�?�停止位后清�?

//读取前判断数据非空
#define I2C_EV7     do{}while(!Hard_I2CStatusCheck(I2C_SR1_RXNE,0))        //读取数据位清�?
#define I2C_EV7_1   do{I2C1->CR1 &= ~I2C_CR1_ACK;}while(!Hard_I2CStatusCheck(I2C_SR1_RXNE,0))



#endif