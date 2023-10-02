#include <stdint.h>

#include "stm32f4xx.h"
#include "delay.h"

class usart{

private:
    GPIO_TypeDef * port;
    uint16_t pinTx;
    uint16_t pinRx;
    uint32_t _delay_ns;
    uint32_t _delay_us;

    void _tx_pin_init(GPIO_TypeDef * port,uint16_t pin){
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* GPIO Ports Clock Enable */
        if(port == GPIOA)
            __HAL_RCC_GPIOA_CLK_ENABLE();
        if(port == GPIOB)
            __HAL_RCC_GPIOB_CLK_ENABLE();
        if(port == GPIOC)
            __HAL_RCC_GPIOC_CLK_ENABLE();
        if(port == GPIOH)
            __HAL_RCC_GPIOH_CLK_ENABLE();

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

        /*Configure GPIO pin : TxPin */
        GPIO_InitStruct.Pin = pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    }

    void _uart_start()
    {
        HAL_GPIO_WritePin(port, pinTx, GPIO_PIN_RESET);
        delay_us(_delay_us);
    }

    void _uart_send(uint8_t byte)
    {
        for(int i=0;i<8;++i)
        {
            HAL_GPIO_WritePin(port, pinTx, GPIO_PinState(byte & 1u));
            byte = byte >> 1;
            delay_us(_delay_us);
        }
    }

    void _uart_stop(){
        HAL_GPIO_WritePin(port, pinTx, GPIO_PIN_SET);
        delay_us(_delay_us);
        delay_us(_delay_us);
    }

public:

    usart(GPIO_TypeDef * port,uint16_t tx,uint16_t rx,uint32_t baudrate) :port(port),pinTx(tx),pinRx(rx),_delay_ns(1e9 / baudrate),_delay_us(1e6 / baudrate)
    {
        _tx_pin_init(port, tx);
    }

    void write(const uint8_t *pData, uint16_t Size){
        for(int i=0;i<Size;++i)
        {
            _uart_start();
            _uart_send(pData[i]);
            _uart_stop();
        }
    }
};