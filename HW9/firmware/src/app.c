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
#include "ili9341.h"
#include "lcd.h"
#include<stdio.h>
#include "touchscreen.h"

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
    

    SPI1_init();
    LCD_init();

    _CP0_SET_COUNT(0);
    LCD_clearScreen(ILI9341_RED);
    initButtons();
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
            unsigned short x, y; 
            int z; 
            XPT2046_read(&x, &y, &z);
            char m[100];
            sprintf(m, "xraw: %5d", x);
            LCD_print(m, 20, 20, ILI9341_WHITE, ILI9341_BLACK);
            sprintf(m, "yraw: %5d", y);
            LCD_print(m, 20, 30, ILI9341_WHITE, ILI9341_BLACK);
            sprintf(m, "zraw: %5d", z);
            LCD_print(m, 20, 40, ILI9341_WHITE, ILI9341_BLACK);
            
            unsigned short xpix = ((x - 200)/15.4);//x: 200 => 0, 3900 => 240      
            unsigned short ypix = ((3900-y)/11.25);//y: 3900 => 0, 300 => 320
            if (z > 100){

                sprintf(m, "x-coordinate: %5d", xpix);
                LCD_print(m, 20, 50, ILI9341_WHITE, ILI9341_BLACK);
                
                sprintf(m, "y-coordinate: %5d", ypix);
                LCD_print(m, 20, 60, ILI9341_WHITE, ILI9341_BLACK);
            } else{
                sprintf(m, "                    ");
                LCD_print(m, 20, 60, ILI9341_WHITE, ILI9341_RED);
                LCD_print(m, 20, 50, ILI9341_WHITE, ILI9341_RED);
            }
            
            static signed int count = 0;
            static int currentPress = 0;
            static int pastPress = 0;
            static signed int xpixpast;
            static signed int ypixpast;
            if (z > 100){
                currentPress = 1;
                xpixpast = xpix;
                ypixpast = ypix;
                        
            }
            if (z < 100){
                currentPress = 0;
            }
            if ((pastPress == 1) && (currentPress == 0)){
                count = count + incrementButton(xpixpast, ypixpast);
            }
            
            pastPress = currentPress;

            sprintf(m, "I = %3d", count);
            LCD_print(m, 120, 210, ILI9341_WHITE, ILI9341_BLACK);
            
            while (!(_CP0_GET_COUNT() > 2400000)) { 
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
