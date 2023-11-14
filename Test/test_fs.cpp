#include "config.h"

#if TEST_STREAM_EN > 0u

#include <iostream>
#include <fstream>

#include "fs.hpp"

#include "usart.h"
#include "ssd1306_i2c.h"

using namespace std;

int usart_write (const void * ptr, size_t len)
{    
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

int oled_write(const void * ptr, size_t len)
{
    int i;
    char ch;
    char* idx = (char*)ptr;
    for(i=0;i<len;++i)
    {
        OLED_ScreenSwitchCheck();
        ch = *(char*)idx;
        if(ch == '\n')
            OLED_Newline();
        else
        {
            OLED_Update_Pos();
            OLED_ShowChar(_ssd1306_pos_x,_ssd1306_pos_y,*(idx++));
        }
    }
}

fs filesys = fs(
    {(char*)"null",NULL},
    {(char*)"usart",usart_write},
    {(char*)"oled",oled_write}
);

/* 初始化数组 */
uint8_t fs::idx = 0;

void TEST_Fs(){

    /* 装载stdin stdout stderr */
    filesys.exec();
    
    /* 测试标准输出 */
    //cout << "hello world" << endl;

    /* 测试文件输出流 */
    //ofstream oled;
    //oled.open("oled");
    //oled << "hello world" << endl;

}

#endif