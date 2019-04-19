#ifndef _SPI_H    /* Guard against multiple inclusion */
#define _SPI_H



void setVoltage(char channel, int voltage);
void initSPI1();
unsigned char SPI1_IO(unsigned char write);



#endif 


