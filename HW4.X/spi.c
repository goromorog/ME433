#include "spi.h"


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
    SS1Rbits.SS1R = 0b0000; // assigning SS1 to pin RPA0
  // set up the chip select pin as an output
  // the chip select pin is used by the pic to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB8 = 0;
  CS = 1;
  
    // setup spi1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
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








int main(void) {
  unsigned short addr1 = 0x1234;                  // the address for writing the ram
  char data[] = "Help, I'm stuck in the RAM!";    // the test message
  char read[] = "***************************";    // buffer for reading from ram
  char buf[100];                                  // buffer for comm. with the user
  unsigned char status;                           // used to verify we set the status 
  NU32_Startup();   // cache on, interrupts on, LED/button init, UART init
  ram_init(); 

  // check the ram status
  CS = 0;
  spi_io(0x5);                                      // ram read status command
  status = spi_io(0);                               // the actual status
  CS = 1;

  sprintf(buf, "Status 0x%x\r\n",status);
  NU32_WriteUART3(buf);

  sprintf(buf,"Writing \"%s\" to ram at address 0x%x\r\n", data, addr1);
  NU32_WriteUART3(buf);
                                                    // write the data to the ram
  ram_write(addr1, data, strlen(data) + 1);         // +1, to send the '\0' character
  ram_read(addr1, read, strlen(data) + 1);          // read the data back
  sprintf(buf,"Read \"%s\" from ram at address 0x%x\r\n", read, addr1);
  NU32_WriteUART3(buf);


	init_spi();

  while(1) {
	_CPO_SET_COUNT(0);
	float f = 512 +512*sin(i*2*3.1415/1000*10);  //should make a 10Hz sin wave)
	i++;



	setVoltage(0,512);		//test
	setVoltage(1,256);		//test

	while(_CPO_GET_COUNT() < 2400000000/1000) {}  //check this is 24Million
    ;
  }
  return 0;
}


