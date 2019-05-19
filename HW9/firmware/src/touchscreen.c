#include <xc.h>
#include "touchscreen.h"
#include "ili9341.h"


void XPT2046_read(unsigned short *x, unsigned short *y, unsigned int *z){
    unsigned short low, high, z1pos, z2pos;
    CS2 = 0;
    spi_io(0b11010001); //X-pos: A2=1, A1=0, A0=1;
    high = spi_io(0x00);
    low = spi_io(0x00);
    *x = (high << 5) | (low >> 3);
    
    spi_io(0b10010001); //Y-pos: A2=0, A1=0, A0=1;
    high = spi_io(0x00);
    low = spi_io(0x00);
    *y = (high << 5) | (low >> 3);
    
    spi_io(0b10110001); //Z1-pos: A2=0, A1=1, A0=1;
    high = spi_io(0x00);
    low = spi_io(0x00);
    z1pos = (high << 5) | (low >> 3);
    
    spi_io(0b11000001); //Z1-pos: A2=1, A1=0, A0=0;
    high = spi_io(0x00);
    low = spi_io(0x00);
    z2pos = (high << 5) | (low >> 3);
    
    *z = z1pos - z2pos + 4095;
    CS2 = 1;
            
}

void initButtons(){
    int i;
    int ii;
    for (i = 0; i < 30; i++){
        for (ii = 0; ii <30; ii++){
            LCD_drawPixel(120+i, 160+ii, ILI9341_BLUE);
            LCD_drawPixel(120+i, 240+ii, ILI9341_BLUE);
        }
    }
    char s[100];
    sprintf(s, "I+");
    LCD_print(s, 130, 170, ILI9341_WHITE, ILI9341_BLUE);
    sprintf(s, "I-");
    LCD_print(s, 130, 250, ILI9341_WHITE, ILI9341_BLUE);
}

signed int incrementButton(unsigned short x, unsigned short y){
    if ((x > 120) && (x < 165)){
        if ((y > 145) && (y < 190)){
            return 1;
        }
        else if ((y > 230) && (y < 265)){
            return -1;
        }
    }
}

