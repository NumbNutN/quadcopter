#include "quaternion.hpp"
#include "usart.hpp"
#include "hmc_5583l.hpp"

#include <iostream>


using namespace std;

#include "test_attitude_calculate.cpp"
#include "test_print_accel.cpp"
#include "test_pid.cpp"
#include "test_pid 3.cpp"
#include "test_motor.cpp"

void TEST_uart()
{
    // usart usart_obj = usart(GPIOA,GPIO_PIN_9,GPIO_PIN_10,115200);
    // usart_obj.write((uint8_t*)"hello\n",6);
    // printf("hello\n");
}

void TEST_stream()
{
    cout << "hello" << endl;
}

void TEST_quaternion()
{
    quaternion a= {1,0,0,0};
    quaternion b{1,1,1,1};
    quaternion c(1,0,0,0);
    c += b;
}

// void TEST_hmc()
// {
//     hmc_558l dev;
//     while(1)
//     {
//         dev.read();
//         dev.print();
//     }

// }