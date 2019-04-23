#ifndef I2C_EXPANDER_H__
#define I2C_EXPANDER_H__
// Header file for i2c_master_noint.c
// helps implement use I2C1 as a master without using interrupts
void initExpander(void);
void setExpander(char pin, char level);
char getExpander();



#endif