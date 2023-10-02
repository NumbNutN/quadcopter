#include "quaternion.hpp"
#include "usart.hpp"

void TEST_cpp()
{
    quaternion a;
    quaternion b;
    a = a*b;
}

void TEST_uart()
{
    usart usart_obj = usart(GPIOA,GPIO_PIN_9,GPIO_PIN_10,115200);
    usart_obj.write((uint8_t*)"hello\n",6);
}