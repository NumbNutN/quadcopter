/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx.h"
#include "common.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

#define I2C1_GPIO_BASE GPIOA
#define I2C1_SCK_PIN GPIO_PIN_8
#define I2C1_SDA_PIN GPIO_PIN_9
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

extern void I2C_Init(uint32_t i2c);
/**
 * @brief I2C_Write routine perform the function that ...
 *        OS related:I2C_Write has no responsibility to create a critical area,
 *                   since it is not a complete I2C duty
 * @param data ...
 * @param n ...
 * @param addr ...
 */
extern void I2C_Write(uint32_t i2c,uint8_t addr,const uint8_t* data,size_t n);

extern void I2C_Read(uint32_t i2c,uint8_t addr,uint8_t* buf,size_t n);

/**
 * @brief I2C_Transfer is a good idea for implementation for a "repeated start/stop conditions"
 *        Reference in [libopencm3 - GitHub](https://github.com/libopencm3/libopencm3/blob/master/lib/stm32/common/i2c_common_v1.c)
 *        OS related:I2C_Transfer do create a critical area inside to assure a complete duty will not be interupt by OS
 */
extern void I2C_Transfer(uint32_t i2c,uint8_t waddr, const uint8_t* w,size_t wn,uint8_t raddr,uint8_t* r,size_t rn);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

