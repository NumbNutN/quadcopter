#include "stm32f4xx.h"
#include "i2c.h"
#include "common.h"
#include "delay.h"

#include <iostream>

#include "quaternion.hpp"

using namespace std;

#define HMC5883L_ADDRESS_W 0x3C
#define HMC5883L_ADDRESS_R 0x3D

using namespace std;

class hmc_558l{
private:

    uint8_t buffer[6];

public:
    hmc_558l(){
        uint8_t config[2] = {0x02,0x00};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config,2);
        delay_ms(67);
        uint8_t config2[2] = {0x00,0x70};
        I2C_Write((uint32_t)I2C1,HMC5883L_ADDRESS_W,config2,2);
        delay_ms(67);
    }

    void read(){
        uint8_t reg = 0x03;
        I2C_Read((uint32_t)I2C1,HMC5883L_ADDRESS_R,buffer,6);
    }

    // friend ostream& operator<<(ostream& out,hmc_5581& sensor);

    void print() {
        uint16_t x = buffer[0] * 256 + buffer[1];
        uint16_t z = buffer[2] * 256 + buffer[3];
        uint16_t y = buffer[4] * 256 + buffer[5]; 
        printf("X:%d\nY:%d\nZ:%d\n",x,y,z);
    }



    ~hmc_558l() = default;

};


// ostream& operator<<(ostream& out,hmc_5581& sensor){
//     uint16_t x = sensor.buffer[0] * 256 + sensor.buffer[1];
//     uint16_t z = sensor.buffer[2] * 256 + sensor.buffer[3];
//     uint16_t y = sensor.buffer[4] * 256 + sensor.buffer[5];
//     out << "X:" << x << endl << "Y:" << y << endl << "Z:" << z << endl;
// }