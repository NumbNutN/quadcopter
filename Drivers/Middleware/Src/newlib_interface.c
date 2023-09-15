#include "ssd1306_i2c.h"

// void putchar(int c){
//     OLED_ShowChar(0,0,c);
// }

/* A minimal write */
int write(int file,char* ptr,int len){
    int i;
    for(i=0;i<len;++i)
        OLED_ShowChar(0,0,*(ptr++));
    return len;
}