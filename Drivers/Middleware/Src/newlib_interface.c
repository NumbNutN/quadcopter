#include "ssd1306_i2c.h"
#include <stdint.h>
#include <stddef.h>
#include <reent.h>

// void putchar(int c){
//     OLED_ShowChar(0,0,c);
// }

/* A minimal write */
// int write(int file,char* ptr,int len){
//     int i;
//     for(i=0;i<len;++i)
//         OLED_ShowChar(0,0,*(ptr++));
//     return len;
// }

int
_write (int file, const void * ptr, size_t len)
{
    int i;
    for(i=0;i<len;++i)
        OLED_ShowChar(0,0,*((char*)ptr++));
    return len;
}

_ssize_t
_write_r (struct _reent *ptr,
     int fd,
     const void *buf,
     size_t cnt)
{
    int i;
    char ch;
    for(i=0;i<cnt;++i)
    {
        OLED_ScreenSwitchCheck();
        ch = *(char*)buf;
        if(ch == '\n')
            OLED_Newline();
        else
        {
            OLED_Update_Pos();
            OLED_ShowChar(_ssd1306_pos_x,_ssd1306_pos_y,*((char*)buf++));
        }
    }
    return cnt;
}