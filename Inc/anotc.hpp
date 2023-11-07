#pragma once
/**
 * @brief 对匿名上位机数据发送帧的支持
*/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "usart.h"

#include <vector>


using namespace std;

// template <typename T>
// concept arrayLike = requires(T a){a[0];}

template <size_t datalen>
class anotcDataFrame{

private:
    constexpr static int8_t functionCodeIdx = 2;
    constexpr static int8_t dataIdx = 4;
    constexpr static int8_t frameRestLen = 6;
    constexpr static int8_t frameLen = datalen + frameRestLen;

    constexpr static int8_t scIdx = dataIdx+datalen;
    constexpr static int8_t acIdx = scIdx+1;


    uint8_t _header[dataIdx] = {0xAA,0xFF,0x0,datalen};

    vector<int8_t> _frame = vector<int8_t>(datalen);
    
public:

    anotcDataFrame(uint8_t id){
        memcpy(&_frame[0],&_header[0],dataIdx);
        _frame[functionCodeIdx] = (int8_t)id;
    }

    template <typename T>
    void pack(T data){
        int i=0;
        _frame[scIdx]=0;_frame[acIdx]=0;
        memcpy(&_frame[dataIdx], &data[0], datalen);
        for(;i<dataIdx+datalen;i++){
            _frame[scIdx] += _frame[i];
            _frame[acIdx] += _frame[scIdx];
        }
    }

    void send(){
        HAL_UART_Transmit(&huart1, (uint8_t*)&_frame[0], frameLen, HAL_MAX_DELAY);
    }

};