#include "ili9341.h"
#include<stdio.h>

void LCD_drawLetter(char letter, unsigned short x, unsigned short y, unsigned short fc, unsigned short bc){
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

void LCD_print(char *m, unsigned short x, unsigned short y, unsigned short c1, unsigned short c2){
    int t = 0;
    while(m[t]){
        LCD_drawLetter(m[t], x + t*5, y, c1, c2);
        t++;
    }
}
