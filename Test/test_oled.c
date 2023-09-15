#include <stdio.h>

#include "ssd1306_i2c.h"

void TEST_SSD1306_PutChar(void){

    SSD1306_Init();

    OLED_Clean();

    // OLED_ShowChar(0, 0, 'H');
    // OLED_ShowChar(1*6, 0, 'e');
    // OLED_ShowChar(2*6, 0, 'l');
    // OLED_ShowChar(3*6, 0, 'l');
    // OLED_ShowChar(4*6, 0, 'o');

    printf("%c",'H');
}