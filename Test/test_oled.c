#include <stdio.h>

#include "ssd1306_i2c.h"

void TEST_SSD1306_PutChar(void){

    SSD1306_Init();

    OLED_Clean();

    printf("%c",'H');
    printf("hello\n");
    printf("%d\n",77);
    printf("%f\n",3.1415926);
}