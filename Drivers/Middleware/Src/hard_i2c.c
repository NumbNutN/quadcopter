#include "stm32f4xx.h"
#include <stdarg.h>

#include "main.h"

#include "hard_i2c.h"

#include "os.h"

void HARD_I2C_Init(uint32_t i2c) {
  if (i2c == (uint32_t)I2C1) {
    // 时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8 I2C1_SCL PB9 I2C1_SDA
    GPIOB->MODER &= ~(0x03 << (8 * 2));
    GPIOB->MODER |= (0x02 << (8 * 2));
    GPIOB->MODER &= ~(0x03 << (9 * 2));
    GPIOB->MODER |= (0x02 << (9 * 2));

    // 开漏输出
    GPIOB->OTYPER |= (1 << 8);
    GPIOB->OTYPER |= (1 << 9);

    // 设置速度
    GPIOB->OSPEEDR &= ~(0x03 << (2 * 8));
    GPIOB->OSPEEDR &= ~(0x03 << (2 * 9));
    GPIOB->OSPEEDR |= (0x03 << (8 * 2));
    GPIOB->OSPEEDR |= (0x03 << (9 * 2));

    // 无上拉下拉
    // 新加
    GPIOB->PUPDR &= ~(0x0F << (4 * 5));

    // 复用
    GPIOB->AFR[1] &= ~(0xFF);
    GPIOB->AFR[1] |= (0x44);
  } else {
    assert_param(false && "have not support i2c else yet");
  }

  // I2C引脚初始化
  // 选择I2C模式
  I2C1->CR1 &= ~I2C_CR1_SMBUS;

  // FREQ  42M
  I2C1->CR2 |= (0x2A);

  // 设置自身地址
  I2C1->OAR1 = 0x0A;

  // 设置I2C总线速度
  I2C1->CCR |= 0xFFF;
  I2C1->CCR &= 0xD2;

  // TRISE
  I2C1->TRISE &= ~(0x1F);
  I2C1->TRISE |= I2C1->CR2 + 1;

  // 允许应答
  I2C1->CR1 |= I2C_CR1_ACK;

  // 使能I2C
  I2C1->CR1 |= I2C_CR1_PE;
}

// MyI2CStatusCheck
uint8_t Hard_I2CStatusCheck(uint16_t SR1, uint16_t SR2) {
  if ((I2C1->SR1 & SR1) == SR1 && (I2C1->SR2 & SR2) == SR2) {
    return 1;
  }
  return 0;
}


void HARD_i2c_read_Seq(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t n)
{
  if (n <= 0)return;
  // 发送起始位
  I2C1->CR1 |= I2C_CR1_START;
  I2C_EV5;
  // 发送地址(读)
  I2C1->DR = addr + 1;
  I2C_EV6;
  int i = 0;
  // 接收数据
  do {
    // 检查数据
    if (i < n - 1)I2C_EV7;
    else I2C_EV7_1;
    buf[i++] = I2C1->DR;
  } while (i < n);
}

/**
 * @brief I2C_Read interface implemented by "SOFT"
 */
void HARD_I2C_Read(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t n) {
  OS_CPU_SR cpu_sr;
  if (n <= 0) return;
  OS_ENTER_CRITICAL();
  HARD_i2c_read_Seq(i2c,addr,buf,n);
  //发送停止位
  I2C1->CR1 |= I2C_CR1_STOP;
  //开启应答位
  I2C1->CR1 |= I2C_CR1_ACK;
  OS_EXIT_CRITICAL();
}

void HARD_i2c_write_Seq(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t n)
{
  if (n <= 0)return;
  I2C_TypeDef *pi2c = (I2C_TypeDef *)i2c;
  // 发送起始位
  pi2c->CR1 |= I2C_CR1_START;
  I2C_EV5;
  // 发送地址
  pi2c->DR = addr;
  // 判断事件EV6 EV8
  I2C_EV6;
  I2C_EV8_1;
  // 发送下一个数据
  int i = 0;
  do {
    pi2c->DR = data[i++];
    I2C_EV8;
  } while (i < n);
}

/**
 * @brief I2C_Wrtie interfce implemented by "HARD"
 */
void HARD_I2C_Write(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t n) {
  OS_CPU_SR cpu_sr;
  if(n<=0)return;
  OS_ENTER_CRITICAL();
  HARD_i2c_write_Seq(i2c,addr,data,n);
  // 判断终止条件
  I2C_EV8_2;
  //产生结束信号
	I2C1->SR1 |= I2C_CR1_STOP;
  OS_EXIT_CRITICAL();
}

/**
 * @brief I2C_Transfer interface implemented by "HARD"
 */
void HARD_I2C_Transfer(uint32_t i2c, uint8_t waddr, const uint8_t *w, size_t wn,uint8_t raddr,
                       uint8_t *r, size_t rn) {
    OS_CPU_SR cpu_sr;   
    OS_ENTER_CRITICAL();
    HARD_i2c_write_Seq(i2c, waddr, w, wn);
    //判断终止条件
    I2C_EV8_2;
    HARD_i2c_read_Seq(i2c, raddr, r, rn);
    //发送停止位
    ((I2C_TypeDef *)i2c)->SR1 |= I2C_CR1_STOP;
    // 开启应答位
    ((I2C_TypeDef *)i2c)->CR1 |= I2C_CR1_ACK;
    OS_EXIT_CRITICAL();
}
