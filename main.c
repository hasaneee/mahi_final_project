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
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"

// lab3 includes
#include "gpio.h"
#include "timer.h"
#include "spi.h"
#include "pin.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

// Custom includes
#include "utils/network_utils.h"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                7    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2026  /* Current year */
#define HOUR                2    /* Time - hours */
#define MINUTE              49    /* Time - minutes */
#define SECOND              0     /* Time - seconds */


#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a1yd6cishud0ro-ats.iot.us-east-1.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT        8443


#define POSTHEADER "POST /things/MahiandTeni_CC3200/shadow HTTP/1.1\r\n"             // CHANGE ME
#define HOSTHEADER "Host: a1yd6cishud0ro-ats.iot.us-east-1.amazonaws.com\r\n"  // CHANGE ME
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"




//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif

#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************

//
//  Lab 3 variables and functions:
//

// SPI/OLED
#define SPI_BITRATE        100000

#define TIMER_TICKS_PER_SEC      80000000UL
#define MULTITAP_TIMEOUT_MS      800UL
#define MULTITAP_TIMEOUT_TICKS   ((TIMER_TICKS_PER_SEC / 1000UL) * MULTITAP_TIMEOUT_MS)

#define MAX_MSG_LEN              64

// Measured pulse ranges from Part III implementation.
#define MEASURED_DATA_TICKS           ((uint32_t)(0.070 * TIMER_TICKS_PER_SEC))
#define MEASURED_NEW_DATA_WAIT_TICKS  ((uint32_t)(0.033 * TIMER_TICKS_PER_SEC))
#define MEASURED_0_TICKS_LOWER        60000
#define MEASURED_0_TICKS_UPPER        68000
#define MEASURED_1_TICKS_LOWER        190000
#define MEASURED_1_TICKS_UPPER        200000
#define MEASURED_OUTER_TICKS_LOWER    70000
#define MEASURED_OUTER_TICKS_UPPER    74000

// IR receiver: GPIOA3 pin 0x10 (LaunchPad pin 18 in your mapping)
#define IR_GPIO_PORT_BASE        GPIOA3_BASE
#define IR_GPIO_PIN              0x10

// UART1 base for board-to-board channel
#define MSG_UART_BASE            UARTA1_BASE

typedef enum { // temp
    KEY_NONE = 0,
    KEY_0, KEY_1, KEY_2, KEY_3, KEY_4,
    KEY_5, KEY_6, KEY_7, KEY_8, KEY_9,
    KEY_ENTER, KEY_DELETE,
    KEY_VOLUP, KEY_VOLDOWN,
    KEY_PGUP, KEY_PGDOWN
} IRKey;

typedef enum {
    None = 0,
    Disarm,
    Arm,
    Alert,
    Reset,
    Settings,
    PasswordSet
} LockState;

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define CLOUD            0x77F7

#define SCREEN_HEIGHT 128
#define SCREEN_WIDTH 128
#define CHAR_HEIGHT 9
#define CHAR_WIDTH 6

static volatile uint32_t g_ulIRData = 0;
static volatile uint32_t g_ulPreviousTick = 0;
static volatile uint32_t g_ulFirstTick = 0;
static volatile uint32_t g_ulEndTick = 0;
static volatile uint32_t g_ulDelta = 0;

static volatile int g_iEdgeType = 0;
static volatile int g_bDataReady = 0;
static volatile int g_bStartDetected = 0;
static volatile int g_bNewDataWord = 1;
static volatile int g_bDataCooldown = 0;
static volatile int g_bTotalEdges = 0;

static char g_composeBuf[MAX_MSG_LEN + 1] = {0};
static uint32_t g_composeLen = 0;
static char g_incomingBuf[MAX_MSG_LEN + 1] = {0};
static volatile uint32_t g_incomingLen = 0;
static char g_lastIncoming[MAX_MSG_LEN + 1] = {0};
static volatile bool g_bDisplayRefreshPending = false;
static volatile bool updated = false;
static volatile bool passwordSet = false;


static LockState currentState = None;
static LockState lastState = None;
static IRKey g_lastTapKey = KEY_NONE;
static uint32_t g_lastTapTick = 0;
static uint32_t g_lastTapCycle = 0;



/*
 *
 *  GRAPHICS FUNCTIONS:
 *  For start up, loading screen, menus and state notifications!
 *
 */


__attribute__((weak)) void OLED_DrawBackgroundGuardianGraphic(void) {
    drawCircle(64,64,60,YELLOW);
    fillRoundRect(36,48,56,16,8,WHITE);
    fillRoundRect(36,72,56,16,8,WHITE);
    fillRect(48,0,32,128,WHITE);
    fillRect(0,0,128,24,RED);
    unsigned char* title = "  BACKPACK       GUARDIAN   ";
    int i = 0;
    while(i < 27){
            drawChar(17+((i*8)% 120), 1 + 11*((int)(i/15)), title[i], WHITE, RED, 1);
            i++;
    }
    MAP_UtilsDelay(30000000);
}



bool connected = false;
__attribute__((weak)) void OLED_DrawLoadingScreen(void) {
    fillRect(0,52,128,24,BLACK);
    drawCircle(64,64,24,WHITE);
    drawCircle(64,64,25,WHITE);
    drawCircle(64,64,26,WHITE);

    connected ? drawCircle(64,64,60,YELLOW):drawCircle(64,64,60,RED);
    fillRect(0,52,128,24,BLACK);
    char* connection = connected ? "CONNECTED!":"CONNECTING";

    int i = 0;
    while(i < 10){
            drawChar(28 + 8*i, 60, connection[i], YELLOW, 0x0000, 1);
            i++;
    }
    MAP_UtilsDelay(20000000);
}

__attribute__((weak)) void OLED_DrawMenuBackground(void) {
    drawRect(0,0,SCREEN_WIDTH,1,WHITE);
    drawRect(0,0,SCREEN_WIDTH,8,WHITE);
    drawRect(0,0,SCREEN_WIDTH, 120,WHITE);
    drawRect(0,0,SCREEN_WIDTH,127,WHITE);
}

__attribute__((weak)) void OLED_RemoveGraphic(void) {
    fillRect(1,8,126,108,BLACK);
}

__attribute__((weak)) void OLED_StatusUpdate(unsigned char* message, int currentLength) {
    int currentChar = 0;
    while(currentChar < currentLength){
            drawChar(6 + ((currentChar*8)% 120), 96, message[currentChar], CLOUD, 0x0000, 1);
            currentChar++;
    }

}

__attribute__((weak)) void OLED_DrawCurrentState(LockState state) {
    //Replacing Current Graphic:
    //

    unsigned char* statusReport = "";
    int currentLength = 8;

    switch (state) { //fixed switch issues :D
        case Arm:
            OLED_RemoveGraphic();
            statusReport = " STATUS: ARMED";
            currentLength += 6;
            OLED_StatusUpdate(statusReport, currentLength);

            fillRoundRect(36,48,56,16,8,WHITE);
            fillRoundRect(36,72,56,16,8,WHITE);
            fillRoundRect(44,24,40,8,8,WHITE);
            fillRoundRect(46,28,8,16,1,WHITE);
            fillRoundRect(74,28,8,16,1,WHITE);
            return;
        case Disarm:
            // If this runs when pressing armed there's a problem
            OLED_RemoveGraphic();
            Report("Disarmed\n\r");
            statusReport = "STATUS:DISARMED";
            currentLength += 7;
            OLED_StatusUpdate(statusReport, currentLength);

            fillRoundRect(36,48,56,16,8,WHITE);
            fillRoundRect(36,72,56,16,8,WHITE);
            fillRoundRect(44,20,40,8,8,WHITE);
            fillRoundRect(46,24,8,16,1,WHITE);
            fillRoundRect(74,22,8,12,1,WHITE);

            return;

        case Settings:
            fillScreen(BLACK);
            Report("Settings\n\r");
            statusReport = "    SETTINGS";
            currentLength += 4;
            OLED_StatusUpdate(statusReport, currentLength);
            return;

        case Alert:
            fillScreen(BLACK);
            setCursor(0, 0);
            setTextSize(2);
            setTextColor(RED, BLACK);
            Outstr("ALERT\n");

            setTextSize(1);
            setTextColor(WHITE, BLACK);
            Outstr("Movement detected\n");
            return;
        case Reset:
            fillScreen(BLACK);
            statusReport = " STATUS: RESET";
            currentLength += 6;
            OLED_StatusUpdate(statusReport, currentLength);
            OLED_RemoveGraphic();
            return;
        case PasswordSet:
            return;
        default:
            Report("Error");
            return;
    }
}

volatile int currentOption = 0;
__attribute__((weak)) void OLED_PrintMenuPage(LockState state) {
    //Replacing Current Graphic:
    OLED_RemoveGraphic();
    //None for Reset and Alert
    switch (state) { //fixed switch issues :D
        case Arm:
            fillScreen(BLACK);
            setCursor(4, 32);
            setTextSize(4);
            setTextColor(WHITE, YELLOW);
            fillRect(0,32,128,24,YELLOW);
            Outstr("ARMED");

            setCursor(16, 16);
            setTextSize(1);
            setTextColor(WHITE, BLACK);
            Outstr("Press 1 to Disarm");

            setCursor(0, 72);
            setTextSize(1);
            setTextColor(WHITE, BLACK);
            Outstr("Press 2 to Test Alert");

            return;
        case Disarm:
            //fillScreen(BLACK);
            setCursor(24, 0);
            setTextSize(1);
            setTextColor(WHITE, BLUE);
            fillRect(0,0,128,8,BLUE);
            Outstr("DISARMED MENU");
            OLED_DrawMenuBackground();

            setCursor(2, 20);
            setTextSize(1);
            setTextColor(WHITE, BLACK);
            Outstr("Press 1 to Arm");

            setCursor(2, 30);
            setTextSize(1);
            setTextColor(WHITE, BLACK);
            Outstr("Press 0 for Settings");
            return;
        case Settings:
            fillScreen(BLACK);
            setCursor(36,0);
            setTextSize(1);
            setTextColor(WHITE, CLOUD);
            OLED_DrawMenuBackground();
            fillRect(0,0,128,8,CLOUD);
            Outstr("SETTINGS");

            setCursor(2, 20);
            setTextSize(1);
            setTextColor(WHITE, BLACK);
            Outstr("Press 1 to Set New");
            setCursor(2, 30);
            Outstr("Password");

            setCursor(2, 40);
            Outstr("Press 2 to Remove");
            setCursor(2, 50);
            Outstr("Password");

            setCursor(2, 70);
            Outstr("Press 0 to go Back");
            return;

        default:
            return;
    }
}
/*
 *  End of graphics!
 */

static inline uint32_t GetTick(void) {
    return MAP_TimerValueGet(TIMERA0_BASE, TIMER_A);
}

// Wrap-safe elapsed check for free-running 32-bit up-counter.
static inline bool HasElapsed(uint32_t now, uint32_t then, uint32_t interval) {
    return (uint32_t)(now - then) >= interval;
}

static void RefreshDisplay(void) {

    if(lastState != currentState && !updated){
        OLED_PrintMenuPage(currentState);
        updated = 1;
        MAP_UtilsDelay(2000000);
    }

}


static void CommitCurrentCharacter(void) {
    g_lastTapKey = KEY_NONE;
    g_lastTapCycle = 0;
}

/*
static const char *KeyCharset(IRKey key) {
    lastState = currentState;
    updated = 0;

    switch (key) {
        case KEY_1: return ".,?!1"; // Leave for settings
        case KEY_2: return "abc2";
        case KEY_3: return "def3";
        case KEY_4: return "ghi4";
        case KEY_5: return "jkl5";
        case KEY_6: return "mno6";
        case KEY_7: return "pqrs7";
        case KEY_8: return "tuv8";
        case KEY_9: return "wxyz9";
        case KEY_0: return " 0";     // space then 0

        case KEY_VOLUP: return "+";
        case KEY_VOLDOWN: return "-";
        default:    return "";
    }
}
*/

static const char *NumberKeyCode(IRKey key) {
    //LockState prev = lastState;
    lastState = currentState;


    /* Key : -------  Button 1            - Button 2 - Button 0 - Channel++ - Channel-- - Vol++ - Vol--
     *      Disarmed |S->Arm                           Settings   Option+
     *      Armed    |S->Reset+Password    S->Alert     Settings   Option-
     *      Settings |SetPass+Password                 Back
     *      Alert    |                     No input, S-> Armed
     *      Reset    |No input, S-> Disarmed
     */

    switch (key) {
        case KEY_0:
                currentState = (currentState == Settings) ? Disarm : (currentState == Disarm) ? Settings : currentState;
                if(currentState != lastState){
                    updated = 0;
                }
                return " 0";
        case KEY_1:
            currentState = (currentState == Disarm) ? Arm : ((currentState == Arm) ? Reset: currentState == Settings? PasswordSet : currentState);
            if(currentState != lastState){
                updated = 0;
            }
            return "1";
        case KEY_2:
            currentState = (currentState == Arm) ? Alert: (currentState == Alert) ? Arm : currentState;
            if(currentState != lastState){
                updated = 0;
            }
            return "2";

        case KEY_PGUP:
            currentOption++;
            currentOption = currentOption % 3;
            updated = 0;
            return "(";
        case KEY_PGDOWN:
            currentOption--;
            currentOption = currentOption < 0 ? 0 : currentOption;
            updated = 0;
            return ")";
        default:    return "";
    }
}
static void MultiTapHandleKey(IRKey key) {
    uint32_t now = GetTick();

    if (key < KEY_0 || key > KEY_9) {
        return;
    }

    const char *chars = NumberKeyCode(key);
    uint32_t charsetLen = (uint32_t)strlen(chars);
    if (charsetLen == 0 || g_composeLen >= MAX_MSG_LEN) {
        return;
    }

    bool sameKey = (key == g_lastTapKey);
    bool withinThreshold = !HasElapsed(now, g_lastTapTick, MULTITAP_TIMEOUT_TICKS);

    if (sameKey && withinThreshold && g_composeLen > 0) {
        g_lastTapCycle = (g_lastTapCycle + 1) % charsetLen;
        g_composeBuf[g_composeLen - 1] = chars[g_lastTapCycle];
    } else {
        if (g_composeLen < MAX_MSG_LEN) {
            g_lastTapCycle = 0;
            g_composeBuf[g_composeLen++] = chars[g_lastTapCycle];
            g_composeBuf[g_composeLen] = '\0';
        }
    }

    g_lastTapKey = key;
    g_lastTapTick = now;
    //RefreshDisplay();
}

static IRKey DecodeIRToKey(uint32_t ulData) {

    switch (ulData) {
        case 0x8167E9: return KEY_1;
        case 0x796869: return KEY_2;
        case 0x7568A9: return KEY_3;
        case 0x7168E9: return KEY_4;
        case 0x6D6929: return KEY_5;
        case 0x696969: return KEY_6;
        case 0x6569A9: return KEY_7;
        case 0x6169E9: return KEY_8;
        case 0x5D6A29: return KEY_9;
        case 0x596A69: return KEY_0;
        case 0x316CE9: return KEY_ENTER;  // MUTE used as ENTER
        case 0xC563A9: return KEY_DELETE; // LAST used as DELETE
        case 0x3D6C29: return KEY_VOLUP;  // Vol+
        case 0x396C69: return KEY_VOLDOWN;// Vol-
        case 0xBD6429: return KEY_PGUP;
        case 0xDD6229: return KEY_PGDOWN;
        default:       return KEY_NONE;
    }
}
static void GPIOA3IntHandler(void) {
    uint32_t status = MAP_GPIOIntStatus(IR_GPIO_PORT_BASE, true);
    if ((status & IR_GPIO_PIN) == 0) {
        return;
    }
    MAP_GPIOIntClear(IR_GPIO_PORT_BASE, IR_GPIO_PIN);

    uint32_t ulCurrentTick = GetTick();
    g_ulDelta = ulCurrentTick - g_ulPreviousTick;
    g_ulPreviousTick = ulCurrentTick;

    if (g_bDataCooldown && HasElapsed(ulCurrentTick, g_ulEndTick, MEASURED_NEW_DATA_WAIT_TICKS)) {
        g_bDataCooldown = 0;
    }

    if (g_bNewDataWord && !g_bDataCooldown) {
        g_bNewDataWord = 0;
        g_ulFirstTick = ulCurrentTick;
    }

    if ((ulCurrentTick - g_ulFirstTick) >= MEASURED_DATA_TICKS && !g_bDataCooldown) {
        g_bNewDataWord = 1;
        g_ulEndTick = ulCurrentTick;
        g_bDataCooldown = 1;
        g_bDataReady = 1;
        g_bStartDetected = 0;
    }

    if (g_bTotalEdges == 3) {
        g_bStartDetected = 1;
    }

    if (g_bStartDetected && g_iEdgeType) {
        int currentBit = (g_bTotalEdges - 3) / 2;

        if (MEASURED_OUTER_TICKS_LOWER < g_ulDelta && g_ulDelta < MEASURED_OUTER_TICKS_UPPER) {
            g_iEdgeType = !g_iEdgeType;
        }

        if (MEASURED_1_TICKS_LOWER < g_ulDelta && g_ulDelta < MEASURED_1_TICKS_UPPER) {
            g_ulIRData |= (1UL << currentBit);
        }
    }

    g_bTotalEdges++;
    g_iEdgeType = !g_iEdgeType;
}

static void UART1IntHandler(void) {
    uint32_t status = MAP_UARTIntStatus(MSG_UART_BASE, true);
    MAP_UARTIntClear(MSG_UART_BASE, status);

    while (MAP_UARTCharsAvail(MSG_UART_BASE)) {
        char c = (char)MAP_UARTCharGetNonBlocking(MSG_UART_BASE);

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            g_incomingBuf[g_incomingLen] = '\0';
            strncpy(g_lastIncoming, g_incomingBuf, MAX_MSG_LEN);
            g_lastIncoming[MAX_MSG_LEN] = '\0';
            g_incomingLen = 0;
            g_incomingBuf[0] = '\0';
            g_bDisplayRefreshPending = true;
            continue;
        }

        if (g_incomingLen < MAX_MSG_LEN) {
            g_incomingBuf[g_incomingLen++] = c;
            g_incomingBuf[g_incomingLen] = '\0';
        }
    }
}

static void InitTimer0FreeRunning(void) {
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC_UP);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
}

static void InitIRGPIO(void) {
    // Keep IR input in a known idle state to avoid spurious interrupt storms.
    MAP_PinConfigSet(PIN_18, PIN_STRENGTH_2MA, PIN_TYPE_STD_PU);
    MAP_GPIOIntDisable(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
    MAP_GPIOIntRegister(IR_GPIO_PORT_BASE, GPIOA3IntHandler);
    MAP_GPIOIntTypeSet(IR_GPIO_PORT_BASE, IR_GPIO_PIN, GPIO_BOTH_EDGES);
    MAP_GPIOIntClear(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
    MAP_GPIOIntEnable(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
}

static void InitUART1InterruptMode(void) {
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_UARTA1);

    // UART1 pins and mode must be assigned in pinmux.c / SysConfig.
    // Keep this init non-blocking during boot: avoid runtime vector registration.
    MAP_UARTDisable(MSG_UART_BASE);
    MAP_UARTConfigSetExpClk(MSG_UART_BASE,
                            MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                            115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    MAP_UARTFIFOEnable(MSG_UART_BASE);
    MAP_UARTIntDisable(MSG_UART_BASE, UART_INT_RX | UART_INT_RT);
    MAP_UARTEnable(MSG_UART_BASE);
}

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_post(int);

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
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
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

void ButtonPressControl(void){
    IRKey key = DecodeIRToKey(g_ulIRData);
    if (key != KEY_NONE) {
        MultiTapHandleKey(key);
    }

    g_ulIRData = 0;
    g_bDataReady = 0;
    g_bTotalEdges = 0;
}
//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    long lRetVal = -1;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");

    UART_PRINT("Remote Control Setup\n\r");
    UART_PRINT(" - Timer init...\n\r");
    InitTimer0FreeRunning();
    UART_PRINT(" - IR GPIO init...\n\r");
    InitIRGPIO();
    UART_PRINT(" - UART1 init...\n\r");
    InitUART1InterruptMode();
    UART_PRINT("Remote Control Setup Complete\n\r");


    UART_PRINT("OLED Setup\n\r");
    UART_PRINT(" - GSPI clock enable...\n\r");
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    UART_PRINT(" - GSPI reset...\n\r");
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);

    UART_PRINT(" - GSPI config...\n\r");
    MAP_SPIConfigSetExpClk(
        GSPI_BASE,
        MAP_PRCMPeripheralClockGet(PRCM_GSPI),
        SPI_BITRATE,
        SPI_MODE_MASTER,
        SPI_SUB_MODE_0,
        SPI_SW_CTRL_CS |
        SPI_4PIN_MODE |
        SPI_TURBO_OFF |
        SPI_CS_ACTIVELOW |
        SPI_WL_8
    );

    UART_PRINT(" - GSPI enable...\n\r");
    MAP_SPIEnable(GSPI_BASE);
    UART_PRINT(" - OLED init...\n\r");
    Adafruit_Init();
    UART_PRINT(" - OLED clear...\n\r");
    fillScreen(BLACK);

    UART_PRINT("Board and Remote Setup Complete!\n\r");

    // Startup screen
    OLED_DrawBackgroundGuardianGraphic();
    fillScreen(BLACK);


    UART_PRINT("Connecting!\n\r");
    OLED_DrawLoadingScreen();
    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
    //UART_PRINT("Connection attempt");
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    //UART_PRINT("Time attempt");
    if(lRetVal < 0) {
        UART_PRINT("Unable to connect to Access Point \n\r");
        LOOP_FOREVER();
    } else {
        lRetVal = set_time();
    }

    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device \n\r");
        LOOP_FOREVER();
    } else {
        //Connect to the website with TLS encryption
        lRetVal = tls_connect();
    }


    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
        fillScreen(RED);
        Outstr("Error Connecting to AWS \n\r");
    } else {
        UART_PRINT("Connected!!!\n\r");
        connected = true;
        OLED_DrawLoadingScreen();
    }


    //START

    fillScreen(BLACK);

    Report("Bag Guardian Debugger! [UART not included in final]\n\r");
    Report("\n\r");

    Report("Key:\n\r");
    Report("\n\r");


    currentState = Disarm;
    while (1) {
        // LockState Control

       if(currentState == Disarm){
            CommitCurrentCharacter();
            OLED_DrawCurrentState(Disarm);
            Report("Disarm/Carry Mode On:\n\r");
            Report("(1) For Stationary/Alarm Mode \n\r");
            Report("(0) For Settings \n\r");
            Report("\n\r");
            MAP_UtilsDelay(10000000);
            while(currentState == Disarm){
                RefreshDisplay();
                if (g_bDataReady) {
                    ButtonPressControl();
                }
            }
        } else if(currentState == Arm){
                CommitCurrentCharacter();
                OLED_DrawCurrentState(Arm);
                Report("Stationary Mode On:\n\r");
                Report("Settings Cannot Be Accessed\n\r");
                Report("(0) *Debug* Password Reset  \n\r");
                Report("(1) For Disarm/Carry Mode [Checks for Password as well] \n\r");
                Report("(2) *Debug* Alarm Mode \n\r");
                Report("\n\r");
                MAP_UtilsDelay(12000000);
                while(currentState == Arm){
                  RefreshDisplay();
                  if (g_bDataReady) {
                      ButtonPressControl();
                  }

                }
        }
        else if(currentState == Settings){
            CommitCurrentCharacter();
            OLED_DrawCurrentState(Settings);
            Report("Settings Menu:\n\r");
            Report("(0) Back to current state \n\r");
            Report("(1) Password Input [Checks for Password as well] \n\r");
            Report("(2) Remove Alarm Buzzer \n\r");
            Report("\n\r");
            MAP_UtilsDelay(12000000);
            while(currentState == Settings){
                RefreshDisplay();
                if (g_bDataReady) {
                    ButtonPressControl();
                  }
            }
            fillScreen(BLACK);
        }
         else if(currentState == Alert){
            CommitCurrentCharacter();
            OLED_DrawCurrentState(Alert);
            Report("Alert Mode On:\n\r");
            Report("Settings Cannot Be Accessed\n\r");
            Report("(1) For Reset Mode \n\r");
            Report("Do nothing for Alarm Mode");
            Report("\n\r");
            http_post(lRetVal);
            MAP_UtilsDelay(12000000);
            int alertTicks = 0;
            while(currentState == Alert && alertTicks < 200){
                // Finalize pending character if threshold elapsed with no additional same-key presses.
                if (g_bDataReady) {
                    ButtonPressControl();
                  }
                alertTicks++;
            }

            fillScreen(BLACK);
            currentState = Reset;
        }
         else if(currentState == Reset){
            CommitCurrentCharacter();
            OLED_DrawCurrentState(Reset);
            Report("Reset Mode On:\n\r");
            Report("Displays Graphic for Reset before returning to Disarm/Carry \n\r");
            Report("1 to disarm \n\r");
            Report("\n\r");
            MAP_UtilsDelay(12000000);
            currentState = Disarm;
         }
         else if(currentState == PasswordSet){

         }
       /*
        if (g_bDisplayRefreshPending) {

            RefreshDisplay();
            g_bDisplayRefreshPending = false;
            http_post(lRetVal);
        }

        // Finalize pending character if threshold elapsed with no additional same-key presses.
        if (g_lastTapKey != KEY_NONE && HasElapsed(GetTick(), g_lastTapTick, MULTITAP_TIMEOUT_TICKS)) {
            CommitCurrentCharacter();
            RefreshDisplay();
        }

        if (g_bDataReady) {
            IRKey key = DecodeIRToKey(g_ulIRData);
            if (key != KEY_NONE) {
                MultiTapHandleKey(key);
            }

            g_ulIRData = 0;
            g_bDataReady = 0;
            g_bTotalEdges = 0;
        }
        */

    }

}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"newMessage\" :\""
#define DUMMYMESSAGE    "Hello phone, this is a test message from your BACKPACK GUARDIAN"
#define DATA2           "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    //UART_PRINT("Check Post Data...");
    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1) + strlen(DUMMYMESSAGE) + strlen(DATA2);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);
    /*
    strcpy(pcBufHeaders, g_lastIncoming);
    pcBufHeaders += strlen(g_lastIncoming);
    */
    strcpy(pcBufHeaders, DUMMYMESSAGE);
    pcBufHeaders += strlen(DUMMYMESSAGE);

    strcpy(pcBufHeaders, DATA2);
    pcBufHeaders += strlen(DATA2);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);

    UART_PRINT("Message Sent!\r\n");
    GPIOPinWrite(GPIOA1_BASE, 0x2, 0x2);
    MAP_UtilsDelay(12000000);
    GPIOPinWrite(GPIOA1_BASE, 0x2, 0x0);

    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}
