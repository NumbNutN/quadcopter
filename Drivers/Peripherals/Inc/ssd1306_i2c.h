#ifndef _SSD1306_H
#define _SSD1306_H

#define SSD1306_WADDR 0x78

#define SSD1306_CMD 0x0
#define SSD1306_DATA 0x40

#include <stdint.h>

/* 参考ssd1306手册 
   ssd1306具有8个Page 
   每个page 8*128 
   假设英文字符为8*6尺寸
   则一行可排布21个字符*/
extern uint8_t _ssd1306_pos_x;
extern uint8_t _ssd1306_pos_y;


void SSD1306_Init();
 
/**
 * @brief 清空屏幕
*/
void OLED_Clean();
void OLED_Fill();

/**
 * @brief 设置当前指针
*/
void OLED_Set_Pos(uint16_t x,uint16_t y);

/**
 * @brief 显示字符
*/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);

void OLED_Update_Pos();
void OLED_Newline();
void OLED_ScreenSwitchCheck();

#endif