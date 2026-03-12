//*****************************************************************************
//
// Anti-Theft Backpack Guardian (CC3200)
// Simplified: 4-state machine, minimal OLED, working IR decoder
//
// States:  DISARMED (blue) -> ARMED (green) -> ALERT (red) -> RESET (white) -> DISARMED
// IR:      KEY_1 = arm / disarm / reset
// Sensors: BMA222 accel (shake) + BH1750 light (bag open) -> Alert
// OLED:    fillScreen color + text + icon, redraws ONCE per state change
//
// Pin assignments:
//   PIN_01 / PIN_02       : I2C SCL/SDA -> BMA222 (0x18) + BH1750 (0x23)
//   PIN_15 / GPIOA2 0x40 : Buzzer (active-LOW)
//   PIN_03 / GPIOA1 0x10 : IR receiver
//   PIN_55 / PIN_57       : UART0 TX/RX (debug, 115200)
//   PIN_58 / PIN_59       : UART1 TX/RX (GPS, 9600)
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// SimpleLink
#include "simplelink.h"

// Driverlib
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_nvic.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "i2c.h"
#include "hw_i2c.h"
#include "gpio.h"
#include "timer.h"
#include "spi.h"
#include "pin.h"
#include "systick.h"

// Common interface
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"

// OLED
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

// Network
#include "utils/network_utils.h"

//*****************************************************************************
// AWS / WiFi -- EXACT from Lab 4 working main.c
//*****************************************************************************
#define DATE                7
#define MONTH               3
#define YEAR                2026
#define HOUR                2
#define MINUTE              49
#define SECOND              0

#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a1yd6cishud0ro-ats.iot.us-east-1.amazonaws.com"
#define GOOGLE_DST_PORT        8443

#define POSTHEADER "POST /things/MahiandTeni_CC3200/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: a1yd6cishud0ro-ats.iot.us-east-1.amazonaws.com\r\n"
#define CHEADER    "Connection: Keep-Alive\r\n"
#define CTHEADER   "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1  "Content-Length: "
#define CLHEADER2  "\r\n\r\n"

//*****************************************************************************
// Vector table -- EXACT from Lab 4 working main.c
//*****************************************************************************
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
// OLED / SPI
//*****************************************************************************
#define SPI_BITRATE   100000

// RGB565 colors
#define BLACK   0x0000
#define BLUE    0x001F
#define GREEN   0x07E0
#define RED     0xF800
#define WHITE   0xFFFF
#define YELLOW  0xFFE0

//*****************************************************************************
// IR decoder -- EXACT from Lab 4 working main.c (zero changes)
//*****************************************************************************
#define TIMER_TICKS_PER_SEC          80000000UL
#define MULTITAP_TIMEOUT_MS          800UL
#define MULTITAP_TIMEOUT_TICKS       ((TIMER_TICKS_PER_SEC / 1000UL) * MULTITAP_TIMEOUT_MS)

#define MEASURED_DATA_TICKS          ((uint32_t)(0.070 * TIMER_TICKS_PER_SEC))
#define MEASURED_NEW_DATA_WAIT_TICKS ((uint32_t)(0.033 * TIMER_TICKS_PER_SEC))
#define MEASURED_0_TICKS_LOWER       60000
#define MEASURED_0_TICKS_UPPER       68000
#define MEASURED_1_TICKS_LOWER       190000
#define MEASURED_1_TICKS_UPPER       200000
#define MEASURED_OUTER_TICKS_LOWER   70000
#define MEASURED_OUTER_TICKS_UPPER   74000

#define IR_GPIO_PORT_BASE   GPIOA1_BASE
#define IR_GPIO_PIN         0x10

//*****************************************************************************
// Buzzer -- PIN_15 / GPIOA2 0x40, active-LOW
//*****************************************************************************
#define BUZZER_PORT               GPIOA2_BASE
#define BUZZER_PIN                0x40
#define BUZZER_TRIGGER_ACTIVE_LOW 1
#define BUZZER_FORCE_OFF          0

//*****************************************************************************
// Accelerometer (BMA222 at I2C 0x18)
//*****************************************************************************
#define ACCEL_I2C_ADDR         0x18
#define ACCEL_REG_X            0x05
#define ACCEL_REG_Y            0x03
#define MOTION_ABS_THRESHOLD   7
#define MOTION_DELTA_THRESHOLD 4
#define MOTION_HITS_REQUIRED   4
#define MOTION_SAMPLE_MS       200

//*****************************************************************************
// BH1750 light sensor (I2C 0x23)
//*****************************************************************************
#define BH1750_I2C_ADDR           0x23
#define BH1750_CMD_POWER_ON       0x01
#define BH1750_CMD_RESET          0x07
#define BH1750_CMD_CONT_HRES      0x10
#define LIGHT_LUX_DELTA_THRESHOLD 50
#define LIGHT_HITS_REQUIRED       3

//*****************************************************************************
// GPS (UART1) -- PIN_58=TX->GPS_RXD, PIN_59=RX<-GPS_TXD, 9600 baud
//*****************************************************************************
#define GPS_UART_BAUD           9600
#define GPS_CONNECT_TIMEOUT_MS  180000
#define GPS_COORD_PERIOD_MS     5000
#define GPS_DIAG_ECHO_LINES     8

//*****************************************************************************
// SysTick -- 40ms tick for sensor timing
//*****************************************************************************
#define SYSTICK_RELOAD_VAL  3200000UL   // ~40ms @ 80MHz

//*****************************************************************************
// State machine
//
// FIX: Added ST_NONE = 255 as sentinel for g_prevState initial value.
// Previously g_prevState = ST_RESET (3) which is a real reachable state.
// If g_state ever became ST_RESET first, OLEDUpdateIfChanged() would see
// g_state == g_prevState and skip the draw -- blank screen.
// ST_NONE = 255 is never a real state so the very first call always draws.
//*****************************************************************************
typedef enum {
    ST_DISARMED = 0,
    ST_ARMED    = 1,
    ST_ALERT    = 2,
    ST_RESET    = 3,
    ST_NONE     = 255   // sentinel only -- never assigned to g_state at runtime
} AppState;

//*****************************************************************************
// IR key -- only KEY_1 used; all other buttons silently ignored
// FIX: Kept as a minimal enum matching what DecodeIRToKey() returns.
//*****************************************************************************
typedef enum {
    KEY_NONE = 0,
    KEY_1
} IRKey;

//*****************************************************************************
// Globals
//*****************************************************************************

// IR decoder (volatile - written in ISR) -- EXACT from Lab 4 working main.c
static volatile uint32_t g_ulIRData       = 0;
static volatile uint32_t g_ulPreviousTick = 0;
static volatile uint32_t g_ulFirstTick    = 0;
static volatile uint32_t g_ulEndTick      = 0;
static volatile uint32_t g_ulDelta        = 0;
static volatile int      g_iEdgeType      = 0;
static volatile int      g_bDataReady     = 0;
static volatile int      g_bStartDetected = 0;
static volatile int      g_bNewDataWord   = 1;
static volatile int      g_bDataCooldown  = 0;
static volatile int      g_bTotalEdges    = 0;

// State machine
// FIX: g_prevState = ST_NONE (255) -- guarantees OLED draws on first loop
static AppState g_state     = ST_DISARMED;
static AppState g_prevState = ST_NONE;

// SysTick 40ms counter
static volatile uint32_t g_tick40ms = 0;

// I2C / sensors
static bool     g_i2c_available      = false;
static int      g_prev_x             = 0;
static int      g_prev_y             = 0;
static uint8_t  g_motion_hits        = 0;
static uint32_t g_motion_next_sample = 0;

static bool     g_bh1750_ready       = false;
static uint16_t g_light_baseline_lux = 0;
static uint8_t  g_light_hits         = 0;
static bool     g_last_trigger_light = false;
static bool     g_alert_posted       = false;

// GPS
typedef struct { bool valid; float lat; float lon; } gps_fix_t;
static gps_fix_t g_fix               = { false, 0.0f, 0.0f };
static bool      g_gps_lock_reported = false;
static bool      g_gps_fail_reported = false;
static uint32_t  g_gps_connect_deadline = 0;
static bool      g_gps_rx_seen       = false;
static uint32_t  g_gps_diag_lines    = 0;
static uint32_t  g_gps_coord_next    = 0;

// AWS
// FIX: tls_connect() returns int (socket fd). Store as int to match.
// Lab 4 stored in long lRetVal then passed to http_post(int) -- same width
// on CC3200 but int is cleaner and avoids implicit narrowing warnings.
static bool connected   = false;
static int  g_lRetVal   = -1;   // TLS socket, named to match Lab 4 usage

//*****************************************************************************
// Function prototypes (Lab 4 style forward declarations)
//*****************************************************************************
static int  set_time(void);
static void BoardInit(void);
static int  http_post(int iTLSSockID);

//*****************************************************************************
// Local I2C_IF fallback
// Fixes linker errors when SDK common i2c_if.c is not part of the CCS project.
//*****************************************************************************
#define LOCAL_I2C_BASE I2CA0_BASE
#define LOCAL_I2C_SYSCLK 80000000UL
#define LOCAL_I2C_TIMEOUT 0x7D

static int local_i2c_transact(unsigned long cmd)
{
    MAP_I2CMasterIntClear(LOCAL_I2C_BASE);
    MAP_I2CMasterTimeoutSet(LOCAL_I2C_BASE, LOCAL_I2C_TIMEOUT);
    MAP_I2CMasterControl(LOCAL_I2C_BASE, cmd);

    while ((MAP_I2CMasterIntStatusEx(LOCAL_I2C_BASE, false) & I2C_MASTER_INT_DATA) == 0) {
        if ((MAP_I2CMasterIntStatusEx(LOCAL_I2C_BASE, false) & I2C_MASTER_INT_TIMEOUT) != 0) {
            return -1;
        }
    }

    if (MAP_I2CMasterErr(LOCAL_I2C_BASE) != I2C_MASTER_ERR_NONE) {
        switch (cmd) {
            case I2C_MASTER_CMD_BURST_SEND_START:
            case I2C_MASTER_CMD_BURST_SEND_CONT:
            case I2C_MASTER_CMD_BURST_SEND_STOP:
                MAP_I2CMasterControl(LOCAL_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
                break;
            case I2C_MASTER_CMD_BURST_RECEIVE_START:
            case I2C_MASTER_CMD_BURST_RECEIVE_CONT:
            case I2C_MASTER_CMD_BURST_RECEIVE_FINISH:
                MAP_I2CMasterControl(LOCAL_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
                break;
            default:
                break;
        }
        return -1;
    }
    return 0;
}

int I2C_IF_Open(unsigned long ulMode)
{
    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_I2CA0);

    MAP_I2CMasterEnable(LOCAL_I2C_BASE);
    MAP_I2CMasterIntClear(LOCAL_I2C_BASE);
    MAP_I2CMasterIntEnableEx(LOCAL_I2C_BASE, I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA);

    if (ulMode == I2C_MASTER_MODE_STD) {
        MAP_I2CMasterInitExpClk(LOCAL_I2C_BASE, LOCAL_I2C_SYSCLK, false);
    } else {
        MAP_I2CMasterInitExpClk(LOCAL_I2C_BASE, LOCAL_I2C_SYSCLK, true);
    }
    return SUCCESS;
}

int I2C_IF_Close(void)
{
    MAP_PRCMPeripheralClkDisable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    return SUCCESS;
}

int I2C_IF_Write(unsigned char ucDevAddr, unsigned char *pucData,
                 unsigned char ucLen, unsigned char ucStop)
{
    if (!pucData || ucLen == 0) return -1;

    MAP_I2CMasterSlaveAddrSet(LOCAL_I2C_BASE, ucDevAddr, false);
    MAP_I2CMasterDataPut(LOCAL_I2C_BASE, *pucData++);
    if (local_i2c_transact(I2C_MASTER_CMD_BURST_SEND_START) < 0) return -1;
    ucLen--;

    while (ucLen) {
        MAP_I2CMasterDataPut(LOCAL_I2C_BASE, *pucData++);
        if (local_i2c_transact(I2C_MASTER_CMD_BURST_SEND_CONT) < 0) return -1;
        ucLen--;
    }

    if (ucStop) {
        if (local_i2c_transact(I2C_MASTER_CMD_BURST_SEND_STOP) < 0) return -1;
    }
    return SUCCESS;
}

int I2C_IF_Read(unsigned char ucDevAddr, unsigned char *pucData, unsigned char ucLen)
{
    unsigned long cmd;
    if (!pucData || ucLen == 0) return -1;

    MAP_I2CMasterSlaveAddrSet(LOCAL_I2C_BASE, ucDevAddr, true);
    cmd = (ucLen == 1) ? I2C_MASTER_CMD_SINGLE_RECEIVE : I2C_MASTER_CMD_BURST_RECEIVE_START;
    if (local_i2c_transact(cmd) < 0) return -1;
    ucLen--;

    while (ucLen) {
        *pucData++ = (unsigned char)MAP_I2CMasterDataGet(LOCAL_I2C_BASE);
        ucLen--;
        if (ucLen) {
            if (local_i2c_transact(I2C_MASTER_CMD_BURST_RECEIVE_CONT) < 0) return -1;
        } else {
            if (local_i2c_transact(I2C_MASTER_CMD_BURST_RECEIVE_FINISH) < 0) return -1;
        }
    }
    *pucData = (unsigned char)MAP_I2CMasterDataGet(LOCAL_I2C_BASE);
    return SUCCESS;
}

//*****************************************************************************
// SysTick timebase
// Use polled COUNTFLAG (no SysTick interrupt) to avoid vector-table issues
// that can trap into default handlers on some CC3200 project setups.
//*****************************************************************************
void SysTickIntHandler(void) { g_tick40ms++; } // kept for compatibility
void SysTickHandler(void)    { SysTickIntHandler(); }

static void SysTickInitApp(void)
{
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
    MAP_SysTickIntDisable();
    MAP_SysTickEnable();
}

static inline void AppTickPoll(void)
{
    // Reading COUNTFLAG clears it; this increments once per elapsed SysTick period.
    if (HWREG(NVIC_ST_CTRL) & NVIC_ST_CTRL_COUNT) {
        g_tick40ms++;
    }
}

static uint32_t ms_to_ticks40(uint32_t ms) { return (ms + 39) / 40; }

//*****************************************************************************
// BoardInit -- EXACT from Lab 4 working main.c
//*****************************************************************************
static void BoardInit(void)
{
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);
    PRCMCC3200MCUInit();
}

//*****************************************************************************
// set_time -- EXACT from Lab 4 working main.c (field order preserved as-is)
//*****************************************************************************
static int set_time(void)
{
    long retVal;

    g_time.tm_day  = DATE;
    g_time.tm_mon  = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec  = HOUR;       // preserved exactly from Lab 4
    g_time.tm_hour = MINUTE;     // preserved exactly from Lab 4
    g_time.tm_min  = SECOND;     // preserved exactly from Lab 4

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                       SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                       sizeof(SlDateTime), (unsigned char *)(&g_time));
    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
// IR decoder -- EXACT from Lab 4 working main.c (zero changes to ISR logic)
//*****************************************************************************
static inline uint32_t GetTick(void)
{
    return MAP_TimerValueGet(TIMERA0_BASE, TIMER_A);
}

static inline bool HasElapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}

static void GPIOA3IntHandler(void)
{
    uint32_t status = MAP_GPIOIntStatus(IR_GPIO_PORT_BASE, true);
    MAP_GPIOIntClear(IR_GPIO_PORT_BASE, status);

    uint32_t ulCurrentTick = GetTick();
    g_ulDelta        = ulCurrentTick - g_ulPreviousTick;
    g_ulPreviousTick = ulCurrentTick;

    if (g_bDataCooldown && HasElapsed(ulCurrentTick, g_ulEndTick, MEASURED_NEW_DATA_WAIT_TICKS)) {
        g_bDataCooldown = 0;
    }

    if (g_bNewDataWord && !g_bDataCooldown) {
        g_bNewDataWord = 0;
        g_ulFirstTick  = ulCurrentTick;
    }

    if ((ulCurrentTick - g_ulFirstTick) >= MEASURED_DATA_TICKS && !g_bDataCooldown) {
        g_bNewDataWord   = 1;
        g_ulEndTick      = ulCurrentTick;
        g_bDataCooldown  = 1;
        g_bDataReady     = 1;
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

static void InitTimer0FreeRunning(void)
{
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC_UP);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
}

static void InitIRGPIO(void)
{
    MAP_GPIOIntRegister(IR_GPIO_PORT_BASE, GPIOA3IntHandler);
    MAP_GPIOIntTypeSet(IR_GPIO_PORT_BASE, IR_GPIO_PIN, GPIO_BOTH_EDGES);
    MAP_GPIOIntClear(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
    MAP_GPIOIntEnable(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
}

// Only KEY_1 hex code needed -- all others return KEY_NONE and are ignored
static IRKey DecodeIRToKey(uint32_t ulData)
{
    switch (ulData) {
        case 0x8167E9: return KEY_1;
        default:       return KEY_NONE;
    }
}

//*****************************************************************************
// IRPollAndConsume -- atomic read + clear of IR frame
//
// FIX (race condition): The old code did:
//     key = DecodeIRToKey(g_ulIRData);   // read
//     IRConsumeFrame();                  // clear -- TWO separate operations
// The GPIOA3 ISR can fire between these two lines and write a new valid frame
// into g_ulIRData, which then gets wiped by IRConsumeFrame(), losing a press.
// Fix: disable the IR interrupt for the ~5 instructions of read+clear.
// This matches how Lab 4 main loop clears atomically:
//     g_ulIRData = 0; g_bDataReady = 0; g_bTotalEdges = 0;
// all in the same if-block with no function call gap.
//*****************************************************************************
static IRKey IRPollAndConsume(void)
{
    IRKey key = KEY_NONE;
    if (!g_bDataReady) return KEY_NONE;

    MAP_GPIOIntDisable(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
    key           = DecodeIRToKey(g_ulIRData);
    g_ulIRData    = 0;
    g_bDataReady  = 0;
    g_bTotalEdges = 0;
    MAP_GPIOIntEnable(IR_GPIO_PORT_BASE, IR_GPIO_PIN);

    return key;
}

//*****************************************************************************
// OLED -- 4 minimal draw functions, called ONCE per state entry only.
// No refresh loops, no menus, no polling.
//*****************************************************************************
static void drawLockBody(uint16_t color)
{
    fillRoundRect(44, 68, 40, 32, 4, color);
}

static void drawOpenShackle(uint16_t color)
{
    drawRoundRect(60, 44, 20, 28, 6, color);
}

static void drawClosedShackle(uint16_t color)
{
    drawRoundRect(48, 48, 20, 28, 6, color);
    fillRect(52, 62, 12, 8, color);
}

static void OLED_DrawDisarmed(void)
{
    fillScreen(BLUE);
    setCursor(4, 10);
    setTextSize(2);
    setTextColor(WHITE, BLUE);
    Outstr("DISARMED");
    drawLockBody(WHITE);
    drawOpenShackle(WHITE);
    setCursor(8, 114);
    setTextSize(1);
    setTextColor(WHITE, BLUE);
    Outstr("KEY1 to ARM");
}

static void OLED_DrawArmed(void)
{
    fillScreen(GREEN);
    setCursor(20, 10);
    setTextSize(2);
    setTextColor(WHITE, GREEN);
    Outstr("ARMED");
    drawLockBody(WHITE);
    drawClosedShackle(WHITE);
    setCursor(8, 114);
    setTextSize(1);
    setTextColor(WHITE, GREEN);
    Outstr("KEY1 to DISARM");
}

static void OLED_DrawAlert(void)
{
    fillScreen(RED);
    setCursor(14, 30);
    setTextSize(3);
    setTextColor(WHITE, RED);
    Outstr("ALERT");
    setCursor(56, 70);
    setTextSize(2);
    setTextColor(WHITE, RED);
    Outstr("!");
    setCursor(8, 114);
    setTextSize(1);
    setTextColor(WHITE, RED);
    Outstr("KEY1 to RESET");
}

static void OLED_DrawReset(void)
{
    fillScreen(WHITE);
    setCursor(20, 54);
    setTextSize(2);
    setTextColor(BLACK, WHITE);
    Outstr("RESET");
}

// Called once per main loop iteration.
// Redraws only when state has changed.
// g_prevState = ST_NONE (255) guarantees first-iteration draw.
static void OLEDUpdateIfChanged(void)
{
    if (g_state == g_prevState) return;
    g_prevState = g_state;

    switch (g_state) {
        case ST_DISARMED: OLED_DrawDisarmed(); break;
        case ST_ARMED:    OLED_DrawArmed();    break;
        case ST_ALERT:    OLED_DrawAlert();    break;
        case ST_RESET:    OLED_DrawReset();    break;
        default:          break;               // ST_NONE never reached
    }
}

//*****************************************************************************
// Buzzer -- active-LOW module
//*****************************************************************************
static void buzzer_on(void)
{
#if BUZZER_FORCE_OFF
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
    return;
#endif
#if BUZZER_TRIGGER_ACTIVE_LOW
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
#else
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
#endif
}

static void buzzer_off(void)
{
#if BUZZER_TRIGGER_ACTIVE_LOW
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
#else
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
#endif
}

//*****************************************************************************
// I2C pin config
//*****************************************************************************
static void ConfigureI2CPins(void)
{
    PinTypeI2C(PIN_01, PIN_MODE_1); // SCL
    PinTypeI2C(PIN_02, PIN_MODE_1); // SDA
}

//*****************************************************************************
// Accelerometer (BMA222 at 0x18)
//*****************************************************************************
static int accel_read_i8(uint8_t reg, int *out)
{
    uint8_t u8 = 0;
    if (!g_i2c_available) return -1;
    if (I2C_IF_Write(ACCEL_I2C_ADDR, &reg, 1, 0) != SUCCESS) return -1;
    if (I2C_IF_Read(ACCEL_I2C_ADDR,  &u8,  1)    != SUCCESS) return -1;
    *out = (int)((int8_t)u8);
    return SUCCESS;
}

static bool motion_detect_sample(void)
{
    int x = 0, y = 0, ax, ay, dx, dy;
    bool moved = false;

    if (accel_read_i8(ACCEL_REG_X, &x) != SUCCESS) return false;
    if (accel_read_i8(ACCEL_REG_Y, &y) != SUCCESS) return false;

    ax = (x < 0) ? -x : x;
    ay = (y < 0) ? -y : y;
    dx = x - g_prev_x; if (dx < 0) dx = -dx;
    dy = y - g_prev_y; if (dy < 0) dy = -dy;
    g_prev_x = x;
    g_prev_y = y;

    if (ax >= MOTION_ABS_THRESHOLD   || ay >= MOTION_ABS_THRESHOLD)   moved = true;
    if (dx >= MOTION_DELTA_THRESHOLD || dy >= MOTION_DELTA_THRESHOLD) moved = true;

    if (moved) { if (g_motion_hits < 255) g_motion_hits++; }
    else       { if (g_motion_hits > 0)   g_motion_hits--; }

    return (g_motion_hits >= MOTION_HITS_REQUIRED);
}

//*****************************************************************************
// BH1750 light sensor (I2C 0x23)
//*****************************************************************************
static bool bh1750_write_cmd(uint8_t cmd)
{
    if (!g_i2c_available) return false;
    return (I2C_IF_Write(BH1750_I2C_ADDR, &cmd, 1, 1) == SUCCESS);
}

static bool bh1750_init(void)
{
    if (!bh1750_write_cmd(BH1750_CMD_POWER_ON))  return false;
    if (!bh1750_write_cmd(BH1750_CMD_RESET))     return false;
    if (!bh1750_write_cmd(BH1750_CMD_CONT_HRES)) return false;
    g_bh1750_ready = true;
    Report("Light: BH1750 init OK at 0x23\r\n");
    return true;
}

static uint16_t bh1750_read_lux(void)
{
    uint8_t buf[2];
    if (!g_bh1750_ready || !g_i2c_available) return 0;
    if (I2C_IF_Read(BH1750_I2C_ADDR, buf, 2) != SUCCESS) return 0;
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (uint16_t)((uint32_t)raw * 10 / 12);
}

static void light_sensor_arm_baseline(void)
{
    g_light_baseline_lux = bh1750_read_lux();
    g_light_hits = 0;
    Report("Light baseline lux: %u\r\n", (unsigned int)g_light_baseline_lux);
}

static bool light_sensor_open_sample(void)
{
    uint16_t now   = bh1750_read_lux();
    uint16_t delta = (now > g_light_baseline_lux) ? (now - g_light_baseline_lux) : 0;

    if (delta >= LIGHT_LUX_DELTA_THRESHOLD) {
        if (g_light_hits < 255) g_light_hits++;
        Report("Light: lux=%u base=%u delta=%u hits=%u\r\n",
               (unsigned int)now, (unsigned int)g_light_baseline_lux,
               (unsigned int)delta, (unsigned int)g_light_hits);
    } else {
        if (g_light_hits > 0) g_light_hits--;
    }
    return (g_light_hits >= LIGHT_HITS_REQUIRED);
}

//*****************************************************************************
// GPS (UART1 polled, non-blocking)
//*****************************************************************************
static void ConfigureGpsPins(void)
{
    PinTypeUART(PIN_58, PIN_MODE_6); // UART1_TX -> GPS RXD
    PinTypeUART(PIN_59, PIN_MODE_6); // UART1_RX <- GPS TXD
}

static void GPSUartInit(void)
{
    MAP_UARTDisable(UARTA1_BASE);
    MAP_UARTConfigSetExpClk(UARTA1_BASE,
                            MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                            GPS_UART_BAUD,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    MAP_UARTEnable(UARTA1_BASE);
}

static char *csv_next(char **p)
{
    char *s, *c;
    if (!p || !*p) return 0;
    s = *p;
    c = strchr(s, ',');
    if (c) { *c = '\0'; *p = c + 1; }
    else   { *p = 0; }
    return s;
}

static float nmea_to_decimal(const char *val, char hemi)
{
    float v, dec;
    int   deg;
    float min;
    if (!val || !val[0]) return 0.0f;
    v   = (float)atof(val);
    deg = (int)(v / 100.0f);
    min = v - ((float)deg * 100.0f);
    dec = (float)deg + (min / 60.0f);
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

static void gps_try_parse_rmc(char *line)
{
    char *p, *hdr, *time_f, *status, *lat, *latH, *lon, *lonH;
    bool was_valid;

    if (!line) return;
    if (strncmp(line, "$GPRMC", 6) != 0 && strncmp(line, "$GNRMC", 6) != 0) return;

    p         = line;
    was_valid = g_fix.valid;
    hdr    = csv_next(&p); (void)hdr;
    time_f = csv_next(&p); (void)time_f;
    status = csv_next(&p);
    lat    = csv_next(&p);
    latH   = csv_next(&p);
    lon    = csv_next(&p);
    lonH   = csv_next(&p);

    if (!status || status[0] != 'A') {
        g_fix.valid = false;
        if (was_valid && g_gps_lock_reported) {
            Report("GPS: lock lost\r\n");
            g_gps_lock_reported = false;
        }
        return;
    }
    if (!lat || !latH || !lon || !lonH) { g_fix.valid = false; return; }

    g_fix.lat   = nmea_to_decimal(lat, latH[0]);
    g_fix.lon   = nmea_to_decimal(lon, lonH[0]);
    g_fix.valid = true;
    if (!g_gps_lock_reported) {
        Report("GPS: fix acquired lat=%.5f lon=%.5f\r\n",
               (double)g_fix.lat, (double)g_fix.lon);
        g_gps_lock_reported = true;
        g_gps_fail_reported = false;
    }
}

static void GPSPoll(void)
{
    static char     buf[128];
    static uint32_t idx = 0;
    int c;
    unsigned int budget = 64; // hard cap per main-loop iteration

    while (budget-- && MAP_UARTCharsAvail(UARTA1_BASE)) {
        c = MAP_UARTCharGetNonBlocking(UARTA1_BASE);
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            buf[idx] = '\0';
            if (idx > 0) {
                char tmp[128];
                char *rmc = 0;
                strncpy(tmp, buf, sizeof(tmp) - 1);
                tmp[sizeof(tmp) - 1] = '\0';
                g_gps_rx_seen = true;
                if (g_gps_diag_lines < GPS_DIAG_ECHO_LINES) {
                    Report("GPS: %s\r\n", tmp);
                    g_gps_diag_lines++;
                }
                // Some modules/noisy links prepend junk before "$GPRMC".
                // Parse from first RMC token found in the received line.
                rmc = strstr(tmp, "$GPRMC");
                if (!rmc) rmc = strstr(tmp, "$GNRMC");
                if (rmc) gps_try_parse_rmc(rmc);
            }
            idx = 0;
        } else {
            if (idx < sizeof(buf) - 1) buf[idx++] = (char)c;
            else idx = 0;
        }
    }

    if (!g_fix.valid && !g_gps_fail_reported &&
        ((int32_t)(g_tick40ms - g_gps_connect_deadline) >= 0)) {
        Report(g_gps_rx_seen ?
               "GPS: no fix (indoors?), continuing\r\n" :
               "GPS: no UART data -- check PIN_59 wiring, baud=9600\r\n");
        g_gps_fail_reported = true;
    }
}

//*****************************************************************************
// http_post -- structure EXACT from Lab 4 working main.c
// Message body: dynamic alert string with GPS coords (replaces DUMMYMESSAGE)
//
// FIX vs previous version:
//   1. Added missing  strcpy(pcBufHeaders, "\r\n\r\n")  after CHEADER --
//      Lab 4 line 653 has this; omitting it makes the HTTP header malformed.
//   2. acRecvbuff[lRetVal+1] = '\0'  matches Lab 4 line 701 exactly.
//      (Previous version used acRecvbuff[lRetVal] = '\0'.)
//*****************************************************************************
#define DATA1 "{" \
                  "\"state\": {\r\n"                                              \
                      "\"desired\" : {\r\n"                                       \
                          "\"newMessage\" :\""
#define DATA2             "\"\r\n"                                                \
                  "}"                                                             \
              "}"                                                                 \
          "}\r\n\r\n"

static int http_post(int iTLSSockID)
{
    char  acSendBuff[512];
    char  acRecvbuff[1460];
    char  cCLLength[200];
    char *pcBufHeaders;
    int   lRetVal = 0;

    pcBufHeaders = acSendBuff;

    // Headers -- EXACT order from Lab 4
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    // FIX: Lab 4 writes "\r\n\r\n" here before Content-Type (line 653)
    strcpy(pcBufHeaders, "\r\n\r\n");

    // Build alert message with GPS coordinates if available
    char alertMsg[128];
    if (g_fix.valid)
        sprintf(alertMsg, "ALERT! Backpack moved. Lat:%.6f Lon:%.6f",
                (double)g_fix.lat, (double)g_fix.lon);
    else
        sprintf(alertMsg, "ALERT! Backpack moved. No GPS fix available.");

    int dataLength = strlen(DATA1) + strlen(alertMsg) + strlen(DATA2);

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
    strcpy(pcBufHeaders, alertMsg);
    pcBufHeaders += strlen(alertMsg);
    strcpy(pcBufHeaders, DATA2);
    pcBufHeaders += strlen(DATA2);

    UART_PRINT(acSendBuff);
    UART_PRINT("Message Sent!\r\n");

    // Send -- EXACT from Lab 4
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if (lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r", lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if (lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r", lRetVal);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    } else {
        // FIX: Lab 4 uses lRetVal+1 (line 701) -- matched exactly
        acRecvbuff[lRetVal + 1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

//*****************************************************************************
// main
//*****************************************************************************
void main(void)
{
    BoardInit();
    PinMuxConfig();

    // Buzzer: drive HIGH immediately (active-LOW -- prevents boot buzz)
    MAP_GPIODirModeSet(BUZZER_PORT, BUZZER_PIN, GPIO_DIR_MODE_OUT);
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);

    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");
    UART_PRINT("Remote Control Setup\n\r");

//    // IR: free-running timer + GPIO interrupt -- EXACT from Lab 4
//    InitTimer0FreeRunning();
//    InitIRGPIO();

    // SysTick for sensor timing (40ms tick)
    SysTickInitApp();

    // I2C: accelerometer (0x18) + BH1750 (0x23)
    ConfigureI2CPins();
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    g_i2c_available = true;
    Report("I2C: init OK\r\n");

    if (!bh1750_init()) {
        Report("Light: BH1750 not found -- light detection disabled\r\n");
        g_bh1750_ready = false;
    }

    // GPS on UART1 at 9600 baud (polled, non-blocking)
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    ConfigureGpsPins();
    GPSUartInit();
    g_gps_connect_deadline = g_tick40ms + ms_to_ticks40(GPS_CONNECT_TIMEOUT_MS);
    Report("GPS: UART1 init 9600 baud\r\n");

    // OLED SPI init -- EXACT from Lab 4
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(
        GSPI_BASE,
        MAP_PRCMPeripheralClockGet(PRCM_GSPI),
        SPI_BITRATE,
        SPI_MODE_MASTER,
        SPI_SUB_MODE_0,
        SPI_SW_CTRL_CS |
        SPI_4PIN_MODE  |
        SPI_TURBO_OFF  |
        SPI_CS_ACTIVELOW |
        SPI_WL_8
    );
    MAP_SPIEnable(GSPI_BASE);
    Adafruit_Init();
    UART_PRINT("Board and Remote Setup Complete!\n\r");

    // WiFi + TLS -- EXACT from Lab 4
//    g_app_config.host = SERVER_NAME;
//    g_app_config.port = GOOGLE_DST_PORT;
//
//    g_lRetVal = connectToAccessPoint();
//
//    UART_PRINT("Time attempt\n\r");
//    g_lRetVal = set_time();
//    if (g_lRetVal < 0) {
//        UART_PRINT("Unable to set time in the device");
//        LOOP_FOREVER();
//    }
//
//    g_lRetVal = tls_connect();
//    if (g_lRetVal < 0) {
//        ERR_PRINT(g_lRetVal);
//        Report("AWS connect failed -- alerts disabled\r\n");
//        connected = false;
//    } else {
//        connected = true;
//    }

    //fillScreen(BLACK);
    Report("Boot complete. State machine starting.\r\n");

    g_gps_coord_next = g_tick40ms + ms_to_ticks40(GPS_COORD_PERIOD_MS);

    // Start DISARMED; g_prevState=ST_NONE forces OLED draw on first loop
    g_state = ST_DISARMED;

    // =========================================================================
    // Main loop
    // =========================================================================
    while (1) {
        AppTickPoll();

        // 1. Non-blocking GPS poll -- always runs
        GPSPoll();

        // 2. Periodic GPS coordinate log
        if ((int32_t)(g_tick40ms - g_gps_coord_next) >= 0) {
            g_gps_coord_next += ms_to_ticks40(GPS_COORD_PERIOD_MS);
            if (g_fix.valid)
                Report("GPS: lat=%.6f lon=%.6f\r\n",
                       (double)g_fix.lat, (double)g_fix.lon);
            else
                Report("GPS: no fix\r\n");
        }

        // 3. OLED: redraw only when state has changed
        OLEDUpdateIfChanged();

//        // 4. IR: atomically read + consume frame if ready
//        //    FIX: interrupt gated during read+clear to prevent ISR race
//        IRKey key = IRPollAndConsume();

        IRKey key = KEY_NONE;

        // =====================================================================
        // State machine
        // =====================================================================
        switch (g_state) {

            // -----------------------------------------------------------------
            // DISARMED -- blue screen, KEY_1 to arm
            // -----------------------------------------------------------------
            case ST_DISARMED:
                if (key == KEY_1) {
                    Report("-> ARMED\r\n");
                    accel_read_i8(ACCEL_REG_X, &g_prev_x);
                    accel_read_i8(ACCEL_REG_Y, &g_prev_y);
                    g_motion_hits        = 0;
                    g_light_hits         = 0;
                    g_alert_posted       = false;
                    g_last_trigger_light = false;
                    light_sensor_arm_baseline();
                    g_motion_next_sample = g_tick40ms + ms_to_ticks40(MOTION_SAMPLE_MS);
                    g_state = ST_ARMED;
                }
                break;

            // -----------------------------------------------------------------
            // ARMED -- green screen, sensor monitoring, KEY_1 to disarm
            // -----------------------------------------------------------------
            case ST_ARMED:
                if (key == KEY_1) {
                    Report("-> DISARMED\r\n");
                    buzzer_off();
                    g_state = ST_DISARMED;
                    break;
                }

                // Non-blocking timed sensor poll
                if ((int32_t)(g_tick40ms - g_motion_next_sample) >= 0) {
                    g_motion_next_sample = g_tick40ms + ms_to_ticks40(MOTION_SAMPLE_MS);

                    bool motion_alarm = motion_detect_sample();
                    bool light_alarm  = g_bh1750_ready
                                        ? light_sensor_open_sample()
                                        : false;

                    if (motion_alarm || light_alarm) {
                        g_last_trigger_light = light_alarm;
                        Report("Alert trigger: %s\r\n",
                               light_alarm ? "BAG_OPEN" : "MOTION");

                        // Buzzer ON once here -- NOT repeated in ST_ALERT
                        // FIX: calling buzzer_on() every loop in ST_ALERT
                        // hammered GPIO and starved the IR ISR of CPU cycles
                        buzzer_on();

                        // POST to AWS once per alert
                        if (!g_alert_posted && connected) {
                            http_post(g_lRetVal);
                            g_alert_posted = true;
                        }
                        g_state = ST_ALERT;
                    }
                }
                break;

            // -----------------------------------------------------------------
            // ALERT -- red screen, buzzer already on, KEY_1 to reset
            // FIX: removed buzzer_on() from this case -- it was being called
            // every single loop iteration, thousands of times per second,
            // hammering GPIO and blocking CPU cycles the IR decoder needs.
            // The buzzer was already turned on when entering this state above.
            // -----------------------------------------------------------------
            case ST_ALERT:
                if (key == KEY_1) {
                    Report("-> RESET\r\n");
                    buzzer_off();
                    g_state = ST_RESET;
                }
                break;

            // -----------------------------------------------------------------
            // RESET -- white screen (drawn by OLEDUpdateIfChanged above),
            // short delay for user feedback, then back to DISARMED
            // -----------------------------------------------------------------
            case ST_RESET:
                MAP_UtilsDelay(40000000); // ~0.5s @ 80MHz
                Report("-> DISARMED\r\n");
                g_state = ST_DISARMED;
                break;

            // ST_NONE is a sentinel value -- never reached at runtime
            default:
                break;
        }
    }
}
