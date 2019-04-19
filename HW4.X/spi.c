#include "spi.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

#define CS LATAbits.LATA0

void setVoltage(char channel, int voltage) {
	unsigned short t = 0;
	t= channel << 15; //channel is at the very end of the data transfer
	t = t | 0b01110000000000000;
	t = t | ((voltage&0b1111111111) <<2); //rejecting excessive bits (above 10)

	CS = 0;
	SPI1_IO (t>>8);
	SPI1_IO(t&0b11111111);
    CS = 1;
            
}

void initSPI1(){
    RPA1Rbits.RPA1R = 0b0011; // assigning pin RPA1 to SD01
    SDI1Rbits.SDI1R = 0b0100; // assigning SDI1 to pin RPB8
    RPA0Rbits.RPA0R = 0b0011;//assigning pin RPA0 to SS1
  // set up the chip select pin as an output
  // the chip select pin is used by the pic to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISAbits.TRISA0 = 0;
  CS = 1;
  
    // setup spi1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  
}

unsigned char SPI1_IO(unsigned char write){
    SPI1BUF = write;
    while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;

}










