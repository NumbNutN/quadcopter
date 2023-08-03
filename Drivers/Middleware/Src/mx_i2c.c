#include "stm32f4xx.h"
#include "i2c.h"

#include <stdint.h>

/**
 * @brief */
void MX_I2C_Init(uint32_t i2c){
    if(i2c == (uint32_t)I2C1){
        MX_I2C1_Init();
    }
}

/**
 * @brief I2C_Wrtie interfce implemented by "MX"
 */
void MX_I2C_Write(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t n){
    if(i2c == (uint32_t)I2C1){
        HAL_I2C_Master_Transmit(&hi2c1,addr,data,n,0);
    }
}

