#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"
#include "lcd.h"
#include<stdio.h>

// DEVCFG0
#pragma config DEBUG = 0b11 // no debugging
#pragma config JTAGEN = 0b0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b111111111 // no write protect
#pragma config BWP = 0b0 // no boot write protect
#pragma config CP = 0b1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0b0 // turn off secondary oscillator
#pragma config IESO = 0b0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 0b1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b11 // do not enable clock switch
#pragma config WDTPS = 0b10100 // use slowest wdt
#pragma config WINDIS = 0b1 // wdt no window mode
#pragma config FWDTEN = 0b0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0b0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0b0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 0b1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 0b1 // USB BUSON controlled by USB module


int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here

    __builtin_enable_interrupts();

    TRISB = 0xFFFF;
    TRISA = 0xFFEF;
    LATAbits.LATA4 = 1;
    

    SPI1_init();
    LCD_init();

    _CP0_SET_COUNT(0);
    LCD_clearScreen(ILI9341_RED);
            
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        static int i = 0;
        char m[100];
        char b[100];
        char f[100];
        static float fps;
        /*
        if (_CP0_GET_COUNT() > 1200000) { //  .0005/((1/48000000)*2){
            _CP0_SET_COUNT(0);
            LATAINV = 0b10000;
        }
         
        
        if (PORTBbits.RB4 == 0){
          LATAbits.LATA4 = 1;  
        }
         */
        
        _CP0_SET_COUNT(0);
        if (i == 0){
            sprintf(m, "Hello world!   ");
        }else{
            sprintf(m, "Hello world! %d", i); 
        }
        LCD_print(m, 28, 32, ILI9341_WHITE, ILI9341_BLACK);
        
        int j;
        int jj;
        for (j = 0; j < i; j++){
            for (jj = 0; jj < 5; jj++){
                LCD_drawPixel(20+j, 42+jj, ILI9341_BLUE);
            }
        }
        
        int k;
        int kk;
        for (k = i; k < 100; k++){
            for (kk = 0; kk < 5; kk++){
                LCD_drawPixel(20+k, 42+kk, ILI9341_WHITE);
            }
        }
        
        
        
        fps = 24000000.0/_CP0_GET_COUNT();
        if (i > 98){
            i=0;
        }else{
            i++;
        }
        
        
        
        sprintf(f, "FPS = %4.2f", fps);
        LCD_print(f, 28, 100, ILI9341_WHITE, ILI9341_BLACK);
        
        while (!(_CP0_GET_COUNT() > 2400000)) { 
            ;
        }
        LATAINV = 0b10000;
         

    }
}