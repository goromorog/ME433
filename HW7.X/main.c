#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_imu.h"
#include "i2c_master.h"
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

    i2c_master_setup();
    initIMU();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_RED);
    
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        _CP0_SET_COUNT(0);
        static signed short temperature;
        static signed short gyroX;
        static signed short gyroY;
        static signed short gyroZ;
        static signed short accelX;
        static signed short accelY;
        static signed short accelZ;
        
        char m[100];
        unsigned char data[14];


        signed short who = readWho();

        /*
        I2C_read_multiple(0b01101011, 0b00100010, data, 4);
        temperature = (data[1]<<8) | (data[0]);
        gyroX = (data[2]<<8) | (data[3]);
        gyroY = (data[4]<<8) | (data[5]);
        gyroZ = (data[6]<<8) | (data[7]);
        accelX = (data[8]<<8) | (data[9]);
        accelY = (data[10]<<8) | (data[11]);
        accelZ = (data[12]<<8) | (data[13]);
        */
        
        signed int templ = I2C_read(0b01101011, 0b00100000);
        signed int temph = I2C_read(0b01101011, 0b00100001);
        temperature = temph<<8 | templ;
        
        signed int gyroXl = I2C_read(0b01101011, 0b00100010);
        signed int gyroXh = I2C_read(0b01101011, 0b00100011);
        gyroX = gyroXh<<8 | gyroXl;
        
        signed int gyroYl = I2C_read(0b01101011, 0b00100100);
        signed int gyroYh = I2C_read(0b01101011, 0b00100101);
        gyroY = gyroYh<<8 | gyroYl;
        
        signed int gyroZl = I2C_read(0b01101011, 0b00100110);
        signed int gyroZh = I2C_read(0b01101011, 0b00100111);
        gyroZ = gyroZh<<8 | gyroZl;
        
        signed int accelXl = I2C_read(0b01101011, 0b00101000);
        signed int accelXh = I2C_read(0b01101011, 0b00101001);
        accelX = accelXh<<8 | accelXl;
        
        signed int accelYl = I2C_read(0b01101011, 0b00101010);
        signed int accelYh = I2C_read(0b01101011, 0b00101011);
        accelY = accelYh<<8 | accelYl;
        
        signed int accelZl = I2C_read(0b01101011, 0b00101100);
        signed int accelZh = I2C_read(0b01101011, 0b00101101);
        accelZ = accelZh<<8 | accelZl;
        
        sprintf(m, "WHO_AM_I? %d", who); 
        LCD_print(m, 28, 30, ILI9341_WHITE, ILI9341_BLACK);
        

        
        sprintf(m, "Temperature: %5d", temperature); 
        LCD_print(m, 28, 50, ILI9341_WHITE, ILI9341_BLACK);
        sprintf(m, "gyroX: %5d", gyroX); 
        LCD_print(m, 28, 60, ILI9341_WHITE, ILI9341_BLACK);
        sprintf(m, "gyroY: %5d", gyroY); 
        LCD_print(m, 28, 70, ILI9341_WHITE, ILI9341_BLACK);
        sprintf(m, "gyroZ: %5d", gyroZ); 
        LCD_print(m, 28, 80, ILI9341_WHITE, ILI9341_BLACK);
        
        sprintf(m, "accelX: %5d", accelX); 
        LCD_print(m, 28, 90, ILI9341_WHITE, ILI9341_BLACK);
        sprintf(m, "accelY: %5d", accelY); 
        LCD_print(m, 28, 100, ILI9341_WHITE, ILI9341_BLACK);
        sprintf(m, "accelZ: %5d", accelZ); 
        LCD_print(m, 28, 110, ILI9341_WHITE, ILI9341_BLACK);
        
        signed int propX = accelX/170;
        signed int propY = accelY/170;
        
        if (propX>0){
            int j;
            int jj;
            for (j = 0; j < propX; j++){
                for (jj = 0; jj < 5; jj++){
                    LCD_drawPixel(120+j, 160+jj, ILI9341_BLUE);
                }
            }

            int k;
            int kk;
            for (k = propX; k < 100; k++){
                for (kk = 0; kk < 5; kk++){
                    LCD_drawPixel(120+k, 160+kk, ILI9341_WHITE);
                }
            }
            
            int l;
            int ll;
            for (l = -100; l <0; l++){
                for (ll = 0; ll < 5; ll++){
                    LCD_drawPixel(120+l, 160+ll, ILI9341_WHITE);
                }
            }
            
        }
        
        if (propX<0){
            int j;
            int jj;
            for (j = -100; j < propX; j++){
                for (jj = 0; jj < 5; jj++){
                    LCD_drawPixel(120+j, 160+jj, ILI9341_WHITE);
                }
            }

            int k;
            int kk;
            for (k = propX; k < 0; k++){
                for (kk = 0; kk < 5; kk++){
                    LCD_drawPixel(120+k, 160+kk, ILI9341_BLUE);
                }
            }
            
            int l;
            int ll;
            for (l = 0; l < 100; l++){
                for (ll = 0; ll < 5; ll++){
                    LCD_drawPixel(120+l, 160+ll, ILI9341_WHITE);
                }
            }
        }
        
        
        if (propY>0){
            int j;
            int jj;
            for (j = 0; j < propY; j++){
                for (jj = 0; jj < 5; jj++){
                    LCD_drawPixel(120+jj, 160+j, ILI9341_BLUE);
                }
            }

            int k;
            int kk;
            for (k = propY; k < 100; k++){
                for (kk = 0; kk < 5; kk++){
                    LCD_drawPixel(120+kk, 160+k, ILI9341_WHITE);
                }
            }
            
            int l;
            int ll;
            for (l = -100; l <0; l++){
                for (ll = 0; ll < 5; ll++){
                    LCD_drawPixel(120+ll, 160+l, ILI9341_WHITE);
                }
            }
            
        }
        
        if (propY<0){
            int j;
            int jj;
            for (j = -100; j < propY; j++){
                for (jj = 0; jj < 5; jj++){
                    LCD_drawPixel(120+jj, 160+j, ILI9341_WHITE);
                }
            }

            int k;
            int kk;
            for (k = propY; k < 0; k++){
                for (kk = 0; kk < 5; kk++){
                    LCD_drawPixel(120+kk, 160+k, ILI9341_BLUE);
                }
            }
            
            int l;
            int ll;
            for (l = 0; l < 100; l++){
                for (ll = 0; ll < 5; ll++){
                    LCD_drawPixel(120+ll, 160+l, ILI9341_WHITE);
                }
            }
        }


        
        
        while (!(_CP0_GET_COUNT() > 1200000)) { 
            ;
        }
        LATAINV = 0b10000;
        
        
    }
}