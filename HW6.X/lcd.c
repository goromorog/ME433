#include "ili9341.h"

LCD_drawLetter(char letter, unsigned short x, unsigned short y, unsigned short fc, unsigned short bc){
    int j;
    int i; 
    for (i = 0; i < 5; i++){
        char col = ASCII[letter-0x20][i];
        for (j = 0; j<8; j++){
            char bit = (col >> j)&1;
            if (bit == 1){
                LCD_drawPixel(x+i, y+j, fc);
            } else{
                LCD_drawPixel(x+i, y+j, bc);
            }
        }
    }
}
