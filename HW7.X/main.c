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
        static signed short temperature;
        static signed short gyroX;
        static signed short gyroY;
        static signed short gyroZ;
        static signed short accelX;
        static signed short accelY;
        static signed short accelZ;
        
        char m[100];
        unsigned char data[14];
        
        if ((_CP0_GET_COUNT() > 1200000) && (readWho()==0b01101001)) { //  .0005/((1/48000000)*2){
            _CP0_SET_COUNT(0);
            LATAINV = 0b10000;
        }
        
        if (PORTBbits.RB4 == 0){
          LATAbits.LATA4 = 1;  
        }
        signed short who = readWho();
        
        
        
        /*
        
        I2C_read_multiple(0b11010110, 0b00100000, data, 14);
        temperature = (data[1]<<8) | (data[0]);
        gyroX = (data[2]<<8) | (data[3]);
        gyroY = (data[4]<<8) | (data[5]);
        gyroZ = (data[6]<<8) | (data[7]);
        accelX = (data[8]<<8) | (data[9]);
        accelY = (data[10]<<8) | (data[11]);
        accelZ = (data[12]<<8) | (data[13]);

        */
        
        sprintf(m, "WHO_AM_I? %d", who); 
        LCD_print(m, 28, 30, ILI9341_WHITE, ILI9341_BLACK);
        
        
        signed short test = I2C_read(0b11010110, 0b00100000);
        sprintf(m, "test: %d", test); 
        LCD_print(m, 28, 60, ILI9341_WHITE, ILI9341_BLACK);
        
        signed short test2 = data[1];
        sprintf(m, "test2: %d", test2); 
        LCD_print(m, 28, 70, ILI9341_WHITE, ILI9341_BLACK);
        
        signed short test3 = data[2];
        sprintf(m, "test3: %d", test3); 
        LCD_print(m, 28, 80, ILI9341_WHITE, ILI9341_BLACK);
        
        /*
        sprintf(m, "gyroX: %d", gyroX); 
        LCD_print(m, 28, 100, ILI9341_WHITE, ILI9341_BLACK);
        */
        
        
    }
}