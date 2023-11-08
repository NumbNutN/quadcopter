#include "i2c.h"
#include "os.h"
#include "delay.h"

#define DELAY_TIME  500
#define I2C_Delay delay_ns

/**
  * @brief SDA线输入模式配置
  * @param None
  * @retval None
  */
void SDA_Input_Mode()
{
    GPIOB->MODER &= ~(GPIO_MODER_MODER9);
}

/**
  * @brief SDA线输出模式配置
  * @param None
  * @retval None
  */
void SDA_Output_Mode()
{
    GPIOB->BSRR |= GPIO_PIN_9;
    GPIOB->MODER |= GPIO_MODER_MODE9_0;
}

/**
  * @brief SDA线输出一个位
  * @param val 输出的数据
  * @retval None
  */
void SDA_Output( uint16_t val )
{
    if ( val )
    {
        GPIOB->BSRR |= GPIO_PIN_9;
    }
    else
    {
        GPIOB->BSRR |= (GPIO_PIN_9 << sizeof(uint16_t)*8);
    }
}

/**
  * @brief SCL线输出一个位
  * @param val 输出的数据
  * @retval None
  */
void SCL_Output( uint16_t val )
{
    if ( val )
    {
        GPIOB->BSRR |= GPIO_PIN_8;
    }
    else
    {
        GPIOB->BSRR |= (GPIO_PIN_8 << sizeof(uint16_t)*8);
    }
}

/**
  * @brief SDA输入一位
  * @param None
  * @retval GPIO读入一位
  */
uint8_t SDA_Input(void)
{
    if(GPIOB->IDR & GPIO_IDR_ID9)
        return 1;
    else
        return 0;
}

/**
  * @brief I2C起始信号
  * @param None
  * @retval None
  */
void I2CStart(void)
{
    SDA_Output(1);
    I2C_Delay(DELAY_TIME);
    SCL_Output(1);
    I2C_Delay(DELAY_TIME);
    SDA_Output(0);
    I2C_Delay(DELAY_TIME);
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);
}

/**
  * @brief I2C结束信号
  * @param None
  * @retval None
  */
void I2CStop(void)
{
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);
    SDA_Output(0);
    I2C_Delay(DELAY_TIME);
    SCL_Output(1);
    I2C_Delay(DELAY_TIME);
    SDA_Output(1);
    I2C_Delay(DELAY_TIME);

}

/**
  * @brief I2C等待确认信号
  * @param None
  * @retval None
  */
unsigned char I2CWaitAck(void)
{
    unsigned short cErrTime = 5;
    SDA_Input_Mode();
    I2C_Delay(DELAY_TIME);
    SCL_Output(1);
    I2C_Delay(DELAY_TIME);
    while(SDA_Input())
    {
        cErrTime--;
        I2C_Delay(DELAY_TIME);
        if (0 == cErrTime)
        {
            SDA_Output_Mode();
            I2CStop();
            return ERROR;
        }
    }
    SDA_Output_Mode();
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);
    return SUCCESS;
}

/**
  * @brief I2C发送确认信号
  * @param None
  * @retval None
  */
void I2CSendAck(void)
{
    SDA_Output(0);
    I2C_Delay(DELAY_TIME);
    I2C_Delay(DELAY_TIME);
    SCL_Output(1);
    I2C_Delay(DELAY_TIME);
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);

}

/**
  * @brief I2C发送非确认信号
  * @param None
  * @retval None
  */
void I2CSendNotAck(void)
{
    SDA_Output(1);
    I2C_Delay(DELAY_TIME);
    I2C_Delay(DELAY_TIME);
    SCL_Output(1);
    I2C_Delay(DELAY_TIME);
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);

}

/**
  * @brief I2C发送一个字节
  * @param cSendByte 需要发送的字节
  * @retval None
  */
void I2CSendByte(unsigned char cSendByte)
{
    unsigned char  i = 8;
    while (i--)
    {
        SCL_Output(0);
        I2C_Delay(DELAY_TIME);
        SDA_Output(cSendByte & 0x80);
        I2C_Delay(DELAY_TIME);
        cSendByte += cSendByte;
        I2C_Delay(DELAY_TIME);
        SCL_Output(1);
        I2C_Delay(DELAY_TIME);
    }
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);
    //ADD 发送字节结束后将SDA重新拉高
    SDA_Output(1);
}

/**
  * @brief I2C接收一个字节
  * @param None
  * @retval 接收到的字节
  */
unsigned char I2CReceiveByte(void)
{
    unsigned char i = 8;
    unsigned char cR_Byte = 0;
    SDA_Input_Mode();
    while (i--)
    {
        cR_Byte += cR_Byte;
        SCL_Output(0);
        I2C_Delay(DELAY_TIME);
        I2C_Delay(DELAY_TIME);
        SCL_Output(1);
        I2C_Delay(DELAY_TIME);
        cR_Byte |=  SDA_Input();
    }
    SCL_Output(0);
    I2C_Delay(DELAY_TIME);
    SDA_Output_Mode();
    return cR_Byte;
}


void SOFT_I2C_Init(uint32_t i2c)
{
    if(i2c == (uint32_t)I2C1)
    {
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_8;
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStructure.Pull = GPIO_PULLUP;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
        SDA_Output(1);
        SCL_Output(1);
    }
}

/**
 * @brief I2C_Wrtie interfce implemented by "SOFT"
 */
void SOFT_I2C_Write(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t n){
    size_t idx = 0;
    OS_CPU_SR cpu_sr;
    if(n <= 0)return;
    OS_ENTER_CRITICAL();
    if(i2c == (uint32_t)I2C1){
        I2CStart();
        I2CSendByte(addr);         
        I2CWaitAck();                
        while (n--) {
            I2CSendByte(data[idx++]);
            I2CWaitAck();      
        }
        I2CStop();
    }
    OS_EXIT_CRITICAL();
}

/**
 * @brief I2C_Read interface implemented by "SOFT"
 */
void SOFT_I2C_Read(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t n){
    size_t idx = 0;
    if(n <= 0)return;
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    if(i2c == (uint32_t)I2C1){
        I2CStart();        
        I2CSendByte(addr);         
        I2CWaitAck();                
        while (n--) {
            buf[idx++] = I2CReceiveByte();
            if(n)I2CSendAck();      
            else I2CSendNotAck();
        }
        I2CStop();
    }
    OS_EXIT_CRITICAL();
}

/**
 * @brief I2C_Transfer interface implemented by "SOFT"
 */
void SOFT_I2C_Transfer(uint32_t i2c, uint8_t waddr, const uint8_t *w, size_t wn,uint8_t raddr,
                       uint8_t *r, size_t rn)   {
    size_t idx = 0;
    OS_CPU_SR cpu_sr;
    uint8_t tmp_wn = wn;
    uint8_t tmp_rn = rn;
    // OS_ENTER_CRITICAL();
    if(tmp_wn > 0){
        I2CStart();
        I2CSendByte(waddr);        
        I2CWaitAck();                
        while (tmp_wn--) {
            I2CSendByte(w[idx++]);
            I2CWaitAck();
        }
    }
    idx = 0;
    if(tmp_rn > 0){
        I2CStart();
        I2CSendByte(raddr);    
        I2CWaitAck();                  
        while (tmp_rn--) {
            r[idx++] = I2CReceiveByte();
            if(tmp_rn)I2CSendAck();
            else I2CSendNotAck();
        }
    }
    if(rn > 0 || wn > 0)I2CStop();
    // OS_EXIT_CRITICAL();
}