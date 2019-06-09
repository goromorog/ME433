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



void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Timer3ISR(void){
	static int counter = 0;			//initialize counter once
	
	OC4RS = 2399*counter/200;
	LATAINV = 0b10000;
	counter++;							//add one to counter every time ISR is entered
	if (counter >= 200){
		counter = 0;	//rollover counter over when end of waveform reached
       
    }

	IFS0bits.T3IF = 0; 

}

//USING OC4 INSTEAD OF OC1 BECAUSE NO OPEN PINS
int main() {
    T2CONbits.TCKPS = 0;     // Timer2 prescaler N=1 (1:1)
    PR2 = 2399;              // PR = PBCLK / N / desiredF - 1
    //48000000 = pbclk, desiredf = 20000
    TMR2 = 0;                // initial TMR2 count is 0
    OC4CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
    OC4RS = 0;             // duty cycle = OC1RS/(PR2+1)
    OC4R = 0;              // initialize before turning OC1 on; afterward it is read-only
    T2CONbits.ON = 1;        // turn on Timer2
    OC4CONbits.ON = 1;       // turn on OC1
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
    
  //INT step 3: setup peripheral
    //set OC4 to pin RPB13
    RPB13Rbits.RPB13R = 0b0101;
    
    T3CONbits.TCKPS = 0b100; //timer3 prescaler of 32
    //PR = 48000000/32/100 -1 = 14999
    PR3 = 14999;                    //             set period register
    TMR3 = 0;                       //             initialize count to 0
   

    T3CONbits.ON = 1;               //             turn on Timer2
    IPC3bits.T3IP = 5;              // INT step 4: priority
    IPC3bits.T3IS = 0;              //             subpriority
    IFS0bits.T3IF = 0;              // INT step 5: clear interrupt flag
    IEC0bits.T3IE = 1;              // INT step 6: enable interrupt



    
    

    __builtin_enable_interrupts();

    TRISB = 0xFFFF;
    TRISA = 0xFFEF;
    LATAbits.LATA4 = 1;
    

    SPI1_init();
    LCD_init();

    _CP0_SET_COUNT(0);
    LCD_clearScreen(ILI9341_RED);
    srand(time(NULL)); 
    unsigned char a[240];
    unsigned char b[240];
    unsigned char c[240];
    int i;
    for (i = 0; i < 240; i++){
        a[i] = i;
        b[i] = 120;
        c[i] = rand();


    }

    _CP0_SET_COUNT(0);


    int j;
    for (j = 0; j < 240; j++){
        LCD_drawPixel(j, (a[j]>>5)+50, ILI9341_BLUE);
    }

    int k;
    for (k = 0; k < 240; k++){
        LCD_drawPixel(k, (b[k]>>5)+100, ILI9341_BLUE);

    }


    int l;
    for (l = 0; l < 240; l++){
        LCD_drawPixel(l, (c[l]>>5) +150, ILI9341_BLUE);

    }

    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk


        
        
         

    }
}