/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_imu.h"
#include "i2c_master.h"
#include "ili9341.h"
#include "lcd.h"
#include<stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    TRISB = 0xFFFF;
    TRISA = 0xFFEF;
    LATAbits.LATA4 = 1;

    i2c_master_setup();
    initIMU();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_RED);
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
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
        
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
