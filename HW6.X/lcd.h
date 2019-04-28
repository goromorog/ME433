#ifndef LCD_H__
#define LCD_H__

void LCD_drawLetter(char letter, int x, int y, char fc, char bc);

void LCD_print(char* m, unsigned short x, unsigned short y, unsigned short c1, unsigned short c2);

#endif