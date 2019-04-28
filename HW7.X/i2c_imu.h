#ifndef I2C_IMU_H__
#define I2C_IMU_H__
// Header file for i2c_master_noint.c
// helps implement use I2C1 as a master without using interrupts
void initIMU(void);
void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length);

void setExpander(char pin, char level);
char getExpander();



#endif