#include "i2c_master.h"

void initIMU(void){
    i2c_master_start();
    i2c_master_send(0b11010110); //0b1101011 chip address, writing
    i2c_master_send(0b00010000); //CTRL1_XL register address
    i2c_master_send(0b10000010); //ODR_XL=1000 (1.66kHz), FS_XL=00 (2g), BW_XL=00 (400Hz)
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b11010110); //0b1101011 chip address, writing
    i2c_master_send(0b00010001); //CTRL2_G register address
    i2c_master_send(0b10001000); //ODR_G3=1000 (1.66kHz), FS_G=10 (1000dps), FS_125=0
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b11010110); //0b1101011 chip address, writing
    i2c_master_send(0b00010010); //CTRL3_G register address
    i2c_master_send(0b00000100); //default values with IF_INC=1
    i2c_master_stop();
}

unsigned char readWho(){
    i2c_master_start();
    i2c_master_send(0b11010110); //0b1101011 chip address, writing
    i2c_master_send(0b00001111); //GPIO ADDR 0x09
    i2c_master_restart();
    i2c_master_send(0b11010111); //0b1101011 chip address, reading
    unsigned char r = i2c_master_recv(); //save returned value
    i2c_master_ack(1);
    i2c_master_stop();
    
    return r;
}

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length){
    
    
}


void setExpander(char pin, char level){
    //level is 0-1
    //pin is 0-7
    //per Nick, we're just setting one pin, so ignore reading and just writing
    
    i2c_master_start();
    i2c_master_send(0b01000000);//A0=0, A1=0, A2=0, writing
    i2c_master_send(0x0A); //OLAT ADDR 0x0A
    i2c_master_send(level << pin);
    i2c_master_stop();
}

char getExpander(){
    i2c_master_start();
    i2c_master_send(0b01000000); //A0=0, A1=0, A2=0, writing
    i2c_master_send(0x09); //GPIO ADDR 0x09
    i2c_master_restart();
    i2c_master_send(0b01000001); //A0=0, A1=0, A2=0, reading
    char r = i2c_master_recv(); //save returned value
    i2c_master_ack(1);
    i2c_master_stop();
    
    return r;
    
}