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

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0; // to remember the loop time
int maf[10];
int fir[10];
float fir_weights[6];
float iir_old_weight;
float iir_new_weight;


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

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    /* PUT YOUR LCD, IMU, AND PIN INITIALIZATIONS HERE */
    TRISB = 0xFFFF;
    TRISA = 0xFFEF;
    LATAbits.LATA4 = 1;

    i2c_master_setup();
    initIMU();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_RED);
    startTime = _CP0_GET_COUNT();
    int jj;
    for (jj = 0; jj < FILTERCOUNT; jj++)
        maf[jj] = 16000;
        fir[jj] = 16000;

    //5Hz cutoff: 100 Hz sample rate -> 50Hz Nyquist -> 0.1 of Nyquist is 5Hz
    // from Matlab: fir1(5, 0.1):
    fir_weights[0] = 0.0264;
    fir_weights[1] = 0.1405;
    fir_weights[2] = 0.3331;
    fir_weights[3] = 0.3331;
    fir_weights[4] = 0.1405;
    fir_weights[5] = 0.0264;
    
    iir_old_weight = 0.8;
    iir_new_weight = 0.2;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                        /* AT THIS POINT, appData.readBuffer[0] CONTAINS A LETTER
                        THAT WAS SENT FROM THE COMPUTER */
                        /* YOU COULD PUT AN IF STATEMENT HERE TO DETERMINE WHICH LETTER
                        WAS RECEIVED (USUALLY IT IS THE NULL CHARACTER BECAUSE NOTHING WAS
                      TYPED) */

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

             /* WAIT FOR 100HZ TO PASS OR UNTIL A LETTER IS RECEIVED */
            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }


            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            /* PUT THE TEXT YOU WANT TO SEND TO THE COMPUTER IN dataOut
            AND REMEMBER THE NUMBER OF CHARACTERS IN len */
            /* THIS IS WHERE YOU CAN READ YOUR IMU, PRINT TO THE LCD, ETC */
            
            /*

            static signed short temperature;
            static signed short gyroX;
            static signed short gyroY;
            static signed short gyroZ;
            static signed short accelX;
            static signed short accelY;
             */
            static signed short accelZ;

            char m[100];
            unsigned char data[14];

            signed short who = readWho();
/*
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

 */
            signed int accelZl = I2C_read(0b01101011, 0b00101100);
            signed int accelZh = I2C_read(0b01101011, 0b00101101);
            accelZ = accelZh<<8 | accelZl;

            sprintf(m, "WHO_AM_I? %d", who); 
            LCD_print(m, 28, 30, ILI9341_WHITE, ILI9341_BLACK);


/*
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
 */
            sprintf(m, "accelZ: %5d", accelZ); 
            LCD_print(m, 28, 110, ILI9341_WHITE, ILI9341_BLACK);
/*
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
            while (!(_CP0_GET_COUNT() - startTime > 240000)) { 
                        ;
                    }*/
            maf[i%FILTERCOUNT] = accelZ;
            fir[i%FILTERCOUNT] = accelZ * fir_weights[i%FILTERCOUNT];
            
            float maf_result = 0;
            float fir_result = 0;
            static float iir_result = 16000;
            int ii;
            for (ii = 0; ii < FILTERCOUNT; ii ++){
                maf_result += maf[ii];
                fir_result += fir[ii];
            }
            maf_result = maf_result/FILTERCOUNT;
            
            iir_result = iir_result*iir_old_weight + accelZ*iir_new_weight;
            
            
            
            LATAINV = 0b10000;
            
                    
            len = sprintf(dataOut, "%d %d %d %d %d\r\n", i, accelZ, maf_result, iir_result, fir_result);
            i++; // increment the index so we see a change in the text
            /* IF A LETTER WAS RECEIVED, ECHO IT BACK SO THE USER CAN SEE IT */
           
            /*
            if (appData.isReadComplete) {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            }
            /* ELSE SEND THE MESSAGE YOU WANTED TO SEND
            else {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT(); // reset the timer for acurate delays
            }
            */
            if ((appData.readBuffer[0] == 'r') && (i < 101)) {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT();
            }
            /* ELSE SEND THE MESSAGE YOU WANTED TO SEND */
            else {
                len = 1;
                dataOut[0] = 0;
                appData.readBuffer[0] = ' ';
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                i = 0;
                startTime = _CP0_GET_COUNT(); // reset the timer for acurate delays
            }
            
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}



/*******************************************************************************
 End of File
 */