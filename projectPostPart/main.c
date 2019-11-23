//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Timer Demo
// Application Overview - This application is to showcases the usage of Timer 
//                        DriverLib APIs. The objective of this application is 
//                        to showcase the usage of 16 bit timers to generate 
//                        interrupts which in turn toggle the state of the GPIO 
//                        (driving LEDs).
//                        Two timers with different timeout value(one is twice 
//                        the other) are set to toggle two different GPIOs which 
//                        in turn drives two different LEDs, which will give a 
//                        blinking effect.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Timer_Demo_Application
// or
// docs\examples\CC32xx_Timer_Demo_Application.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup timer_demo
//! @{
//
//*****************************************************************************

// Standard include
#include <stdio.h>
//#include "string.h"

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"

// Common interface includes
#include "gpio_if.h"

// Simplelink includes
//#include "simplelink.h"
#include "uart.h"
#include "common.h"
#include "uart_if.h"

#include "timer_if.h"
#include "timer.h"
#include "interrupt.h"
#include "pinmux.h"
#include "hw_adc.h"
#include "hw_gprcm.h"
#include "pin.h"
#include "adc.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
//#define APPLICATION_VERSION        "1.1.1"
#define FOREVER                    1
#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "a1r5zeocu68fac-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                14    /* Current Date */
#define MONTH               16     /* Month 1-12 */
#define YEAR                2019  /* Current year */
#define HOUR                01    /* Time - hours */
#define MINUTE              31    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/EEC172_P5/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: a1r5zeocu68fac-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \""
#define DATA2 "\"\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;

//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;

int id = 0;
char *message;
SlDateTime g_time;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
int counterHRZT = 0;
int servoLimitHRZT = 70;
int counterVTC = 0;
int servoLimitVTC = 40;
int counterGRIP = 0;
int servoLimitGRIP = 100;
int UpDown = 2; //up 1->+2 && down 0->-2 && 2 do nothing
int LeftRight = 2; //left 1->+2 && right 0->-2 && 2 do nothing
int Grip = 2;
int x;
int y;
int SW2;
int SW3;


//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static void
TimerA0IntHandler(void)
{
    Timer_IF_InterruptClear(TIMERA0_BASE);

    x = MAP_ADCFIFORead(ADC_BASE, ADC_CH_2);
    y = MAP_ADCFIFORead(ADC_BASE, ADC_CH_1);
    x = (x >> 2) & 0x0FFF;
    y = (y >> 2) & 0x0FFF;
    printf("x = %d    y = %d\r\n", x, y);
    if(x < 200){
        if(y < 200){
            //printf("Up -> Left\r\n");
            UpDown = 1;
            LeftRight = 1;
        }
        else{
            LeftRight = 1;//printf("Left\r\n");
            UpDown = 2;
        }
    }
    else if(y < 1700 && y > 1400){
        //printf("Up -> Right\r\n");
        UpDown = 1;
        LeftRight = 0;
    }
    else if(x < 2350 && x > 2200){
        if(y < 2350 && y > 2200){
            //printf("Down -> Right\r\n");
            UpDown = 0;
            LeftRight = 0;
        }
        else{
            LeftRight = 0;//printf("Right\r\n");
            UpDown = 2;
        }
    }
    else if(x < 1000 && x > 300){
        //printf("Down -> Left\r\n");
        UpDown = 0;
        LeftRight = 1;
    }
    else if(y < 200){
        UpDown = 1;//printf("Up\r\n");
        LeftRight = 2;
    }
    else if(y < 2300 && y > 2200){
        UpDown = 0;//printf("Down\r\n");
        LeftRight = 2;
    }
    else{
        UpDown = 2;
        LeftRight = 2;
    }

    SW2 =  MAP_GPIOPinRead(GPIOA2_BASE, 0x40);
    SW3 =  MAP_GPIOPinRead(GPIOA1_BASE, 0x20);
    if(SW2 != 0 && SW3 != 0){
        Grip = 2;
    }
    else if(SW2 != 0){
        Grip = 1;
    }
    else if(SW3 != 0){
        Grip = 0;
    }
    else{
        Grip = 2;
    }
}

static void
TimerA1IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(TIMERA1_BASE);
    MAP_TimerDisable(TIMERA1_BASE, TIMER_BOTH);
    if(counterHRZT < servoLimitHRZT){
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    }
    else{
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
    }
    counterHRZT++;
    if(counterHRZT == 999)
        counterHRZT = 0;

    if(counterVTC < servoLimitVTC){
        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
    }
    else{
        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
    }
    counterVTC++;
    if(counterVTC == 999)
        counterVTC = 0;

    if(counterGRIP < servoLimitGRIP){
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    }
    else{
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    }
    counterGRIP++;
    if(counterGRIP == 999)
        counterGRIP = 0;


    MAP_TimerIntEnable(TIMERA1_BASE, TIMER_BOTH);
    MAP_TimerEnable(TIMERA1_BASE, TIMER_BOTH);
}

void
ADCSetUp(void)
{
    //
    // Enable ADC and channel
    //
    MAP_ADCEnable(ADC_BASE);
    MAP_ADCChannelEnable(ADC_BASE, ADC_CH_2);
    MAP_ADCChannelEnable(ADC_BASE, ADC_CH_1);
}

void
LEDSetUp(void)
{
    //
    // configure the LED RED and GREEN and ORANGE
    //
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
}

void
TimerIntSetUp(void)
{
    unsigned long ulStatus;

    // Init Timer A0
    PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA0);
    TimerIntRegister(TIMERA0_BASE, TIMER_A, TimerA0IntHandler);
    TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMERA0_BASE, TIMER_A, 80000000);
    ulStatus = TimerIntStatus(TIMERA0_BASE, false);
    TimerIntClear(TIMERA0_BASE, ulStatus);

    // Init Timer A1
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC, TIMER_BOTH, 0);
    MAP_TimerLoadSet(TIMERA1_BASE, TIMER_BOTH, 1140);
    MAP_TimerIntRegister(TIMERA1_BASE, TIMER_BOTH, TimerA1IntHandler);
    ulStatus = TimerIntStatus(TIMERA1_BASE, false);
    TimerIntClear(TIMERA1_BASE, ulStatus);
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//!    main function demonstrates the use of the timers to generate
//! periodic interrupts.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int
main(void)
{
    //
    // Initialize board configurations
    BoardInit();
    //
    // Pinmuxing for LEDs
    //
    PinMuxConfig();
    LEDSetUp();
    TimerIntSetUp();
    ADCSetUp();

    MAP_TimerIntEnable(TIMERA0_BASE, TIMER_A);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);

    MAP_TimerIntEnable(TIMERA1_BASE, TIMER_BOTH);
    MAP_TimerEnable(TIMERA1_BASE, TIMER_BOTH);

    //
    // Loop forever while the timers run.
    //
    while(FOREVER)
    {
        if(LeftRight == 1 && servoLimitHRZT <= 110)
            servoLimitHRZT += 2;
        else if(LeftRight == 0 && servoLimitHRZT >= 30)
            servoLimitHRZT -= 2;

        if(UpDown == 1 && servoLimitVTC <= 110)
            servoLimitVTC += 2;
        else if(UpDown == 0 && servoLimitVTC >= 30)
            servoLimitVTC -= 2;

        if(Grip == 1 && servoLimitGRIP <= 95)
            servoLimitGRIP += 5;
        else if(Grip == 0 && servoLimitGRIP >= 30)
            servoLimitGRIP -= 5;

        MAP_UtilsDelay(800000);
    }
}

//*****************************************************************************
//
// POST
//
//*****************************************************************************

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(message);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, message);
    pcBufHeaders += strlen(message);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);

    UART_PRINT("cc \n\r a \n\r");

    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        //UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        //UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        //UART_PRINT(acRecvbuff);
    }
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
