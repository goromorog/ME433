#include "i2c_master.h"

void initExpander(void){
    i2c_master_start();
    i2c_master_send(0b01000000); //A0=0, A1=0, A2=0, writing
    i2c_master_send(0x00); //iodir ADDR 0x00
    i2c_master_send(0b11110000); //gp0-3 outputs, gp4-7 inputs
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b01000000);
    i2c_master_send(0x0A); //OLAT ADDR 0x0A
    i2c_master_send(0b00001111);
    i2c_master_stop();
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