//*****************************************************************************
//
// Anti-Theft Backpack Guardian (CC3200)
//
// Standalone behavior (no UART terminal needed for control):
//   - Arm/Disarm using IR remote
//   - Continuous motion/lift detection using I2C accelerometer
//   - Audible alarm using buzzer (GPIO)
//   - OLED status + alert feedback (SPI SSD1351)
//   - Optional GPS capture (UART1)
//   - HTTPS POST notification to AWS IoT Thing Shadow (for email routing)
//
// IMPORTANT: Update these macros before running:
//   - WIFI_SSID, WIFI_PASS, WIFI_SEC
//   - SERVER_NAME, HOSTHEADER, POSTHEADER (Thing name)
//
//*****************************************************************************

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "simplelink.h"
#include "netcfg.h"

#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_nvic.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom_map.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "systick.h"
#include "utils.h"

#include "common.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "pinmux.h"
#include "i2c_if.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "oled_test.h"

/*
 * Provide a RAM-based vector table so driverlib IntRegister() can install
 * SimpleLink/NWP ISR handlers at runtime.
 */
#define APP_VTABLE_NUM_ENTRIES 256U
#pragma DATA_ALIGN(g_pfnVectors, 1024)
#pragma DATA_SECTION(g_pfnVectors, ".vtable")
void (*g_pfnVectors[APP_VTABLE_NUM_ENTRIES])(void);

static void AppIntDefaultHandler(void)
{
    while (1) {
    }
}

static void InitVectorTable(void)
{
    unsigned long old_vtor;
    unsigned long *old_vectors;
    unsigned int i;

    old_vtor = HWREG(NVIC_VTABLE);
    old_vectors = (unsigned long *)old_vtor;

    for (i = 0; i < APP_VTABLE_NUM_ENTRIES; i++) {
        unsigned long v = old_vectors[i];
        g_pfnVectors[i] = (v != 0UL) ? (void (*)(void))v : AppIntDefaultHandler;
    }
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
}

static void BoardInit(void)
{
    PRCMCC3200MCUInit();
    InitVectorTable();
    MAP_IntEnable(FAULT_SYSTICK);
    MAP_IntMasterEnable();
}


//*****************************************************************************
// User WiFi credentials (EDIT THESE)
//*****************************************************************************
#define WIFI_SSID       "Hasanph"
#define WIFI_PASS       "12345678"

// Security types: SL_SEC_TYPE_OPEN, SL_SEC_TYPE_WEP, SL_SEC_TYPE_WPA, SL_SEC_TYPE_WPA_WPA2
#define WIFI_SEC        SL_SEC_TYPE_WPA_WPA2

//*****************************************************************************
// Cloud config (EDIT THESE)
//*****************************************************************************
#define THING_NAME      "YOUR_THING_NAME"
#define SERVER_NAME     "YOUR_AWS_ENDPOINT"  // e.g. a1b2c3d4e5-ats.iot.us-west-2.amazonaws.com
#define TLS_DST_PORT    8443

#define POSTHEADER      "POST /things/" THING_NAME "/shadow HTTP/1.1\r\n"
#define HOSTHEADER      "Host: " SERVER_NAME "\r\n"
#define CHEADER         "Connection: Keep-Alive\r\n"
#define CTHEADER        "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1       "Content-Length: "
#define CLHEADER2       "\r\n\r\n"

// Upload these files to CC3200 serial flash (user files) via UniFlash/ImageCreator.
// Convert downloaded PEM files to DER and upload using these names.
#define AWS_CA_FILE         "/cert/rootCA.der"
#define AWS_CLIENT_CERT     "/cert/client.der"
#define AWS_PRIVATE_KEY     "/cert/private.der"

// TLS time (keep it recent)
#define DATE            28
#define MONTH           3
#define YEAR            2026
#define HOUR            10
#define MINUTE          0
#define SECOND          0

//*****************************************************************************
// SysTick timing
//*****************************************************************************
#define SYSCLK_HZ               80000000ULL
#define SYSTICK_RELOAD_VAL      3200000UL     // ~40ms @ 80MHz
#define TICKS_TO_US(ticks)      ((ticks) / (SYSCLK_HZ / 1000000ULL))
#define GPS_UART_BAUD            9600
#define STATUS_PERIOD_MS         1000
#define GPS_CONNECT_TIMEOUT_MS   180000
#define GPS_DIAG_ECHO_LINES      8
// Set to 1 only when GPS wiring is connected and does not interfere with NWP boot pins.
#define ENABLE_GPS               1
// Set to 1 only when OLED hardware is connected and verified.
#define ENABLE_OLED              1
// Set to 1 only when IR receiver hardware is connected and stable.
#define ENABLE_IR                1
// Set to 1 only when accelerometer/I2C hardware is connected and required.
#define ENABLE_I2C               1
// Set to 1 only after NWP/Wi-Fi bring-up is verified on your board.
#define ENABLE_WIFI              1

// IR command codes (update after observing printed received codes, if needed).
#define IR_CMD_UNLOCK            0x8889
#define IR_CMD_LOCK              0x1819

// Test fallback: long-press reset button to toggle arm/disarm without IR.
#define BUTTON_TOGGLE_HOLD_MS    1500

static volatile uint32_t g_tick40ms = 0;

static void AppSysTickHandler(void)
{
    g_tick40ms++;
}

static void SysTickInitApp(void)
{
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
    MAP_SysTickIntRegister(AppSysTickHandler);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();
}

static uint32_t ms_to_ticks40(uint32_t ms)
{
    const uint32_t period_ms = 40;
    return (ms + (period_ms - 1)) / period_ms;
}

//*****************************************************************************
// State machine
//*****************************************************************************
typedef enum {
    ST_DISARMED = 0,
    ST_ARMING,
    ST_ARMED,
    ST_ALERT
} system_state_t;

static volatile system_state_t g_state = ST_DISARMED;

static int  g_tls_sock = -1;
static bool g_alert_sent = false;
static bool g_wifi_connected = false;
static bool g_oled_available = false;
static bool g_i2c_available = false;

static uint32_t g_arming_deadline = 0;
static uint32_t g_ui_last_countdown = 0;

static uint32_t g_motion_next_sample = 0;
static int g_prev_x = 0;
static int g_prev_y = 0;
static uint8_t g_motion_hits = 0;

static const char *state_to_str(system_state_t st)
{
    if (st == ST_DISARMED) return "DISARMED";
    if (st == ST_ARMING)   return "ARMING";
    if (st == ST_ARMED)    return "ARMED";
    return "ALERT";
}

//*****************************************************************************
// GPIO assignments (match pinmux.c I gave you)
//*****************************************************************************
#define IR_GPIO_PORT            GPIOA1_BASE
#define IR_GPIO_PIN             0x10          // PIN_03

#define RESET_BTN_PORT          GPIOA1_BASE
#define RESET_BTN_PIN           0x20          // PIN_04
#define RESET_BTN_ACTIVE_LOW    1

#define BUZZER_PORT             GPIOA1_BASE
#define BUZZER_PIN              0x02          // PIN_64

//*****************************************************************************
// IR decoder (48-bit frame, last 16-bit command)
//*****************************************************************************
#define IR_START_HI_MIN_US      8500
#define IR_START_HI_MAX_US      9500
#define IR_START_LO_MIN_US      4000
#define IR_START_LO_MAX_US      5000

#define IR_BIT_0_LO_MAX_US      900
#define IR_BIT_1_LO_MIN_US      1200
#define IR_BIT_1_LO_MAX_US      1900

#define IR_FRAME_BITS           48

static volatile uint32_t g_ir_last_tick = 0;
static volatile uint32_t g_ir_pulse_us = 0;
static volatile bool     g_ir_edge_seen = false;

static volatile uint64_t g_ir_shift = 0;
static volatile uint8_t  g_ir_bitcount = 0;
static volatile bool     g_ir_frame_ready = false;

static uint32_t systick_delta_ticks(uint32_t prev, uint32_t now)
{
    if (prev >= now) return (prev - now);
    return (prev + (SYSTICK_RELOAD_VAL - now));
}

static void IRIntHandler(void)
{
    unsigned long status;
    uint32_t now, dticks;

    status = MAP_GPIOIntStatus(IR_GPIO_PORT, true);
    MAP_GPIOIntClear(IR_GPIO_PORT, status);

    now = MAP_SysTickValueGet();
    dticks = systick_delta_ticks(g_ir_last_tick, now);
    g_ir_last_tick = now;

    g_ir_pulse_us = (uint32_t)TICKS_TO_US(dticks);
    g_ir_edge_seen = true;
}

static void IRInit(void)
{
    MAP_GPIOIntDisable(IR_GPIO_PORT, IR_GPIO_PIN);
    MAP_GPIOIntClear(IR_GPIO_PORT, IR_GPIO_PIN);
    MAP_GPIOIntRegister(IR_GPIO_PORT, IRIntHandler);
    MAP_GPIOIntTypeSet(IR_GPIO_PORT, IR_GPIO_PIN, GPIO_BOTH_EDGES);
    MAP_GPIOIntEnable(IR_GPIO_PORT, IR_GPIO_PIN);

    g_ir_last_tick = MAP_SysTickValueGet();
}

static void ir_decoder_reset(void)
{
    g_ir_shift = 0;
    g_ir_bitcount = 0;
    g_ir_frame_ready = false;
}

static void ir_decoder_process_pulse(uint32_t pulse_us)
{
    if ((pulse_us >= IR_START_HI_MIN_US && pulse_us <= IR_START_HI_MAX_US) ||
        (pulse_us >= IR_START_LO_MIN_US && pulse_us <= IR_START_LO_MAX_US)) {
        ir_decoder_reset();
        return;
    }

    if (pulse_us <= IR_BIT_0_LO_MAX_US) {
        g_ir_shift = (g_ir_shift << 1) | 0ULL;
        g_ir_bitcount++;
    } else if (pulse_us >= IR_BIT_1_LO_MIN_US && pulse_us <= IR_BIT_1_LO_MAX_US) {
        g_ir_shift = (g_ir_shift << 1) | 1ULL;
        g_ir_bitcount++;
    } else {
        ir_decoder_reset();
        return;
    }

    if (g_ir_bitcount >= IR_FRAME_BITS) {
        g_ir_frame_ready = true;
    }
}

static bool IRPollCommand16(uint16_t *out_cmd)
{
    if (g_ir_edge_seen) {
        g_ir_edge_seen = false;
        ir_decoder_process_pulse(g_ir_pulse_us);
    }

    if (g_ir_frame_ready) {
        uint16_t cmd;
        cmd = (uint16_t)(g_ir_shift & 0xFFFF);

        g_ir_frame_ready = false;
        g_ir_bitcount = 0;

        if (out_cmd) *out_cmd = cmd;
        return true;
    }
    return false;
}

//*****************************************************************************
// Accelerometer (I2C) - BMA222 typical mapping
//*****************************************************************************
#define ACCEL_I2C_ADDR          0x18
#define ACCEL_REG_X             0x05
#define ACCEL_REG_Y             0x03

#define MOTION_ABS_THRESHOLD    7
#define MOTION_DELTA_THRESHOLD  4
#define MOTION_HITS_REQUIRED    4

static int accel_read_i8(uint8_t reg, int *out)
{
    uint8_t u8;
    int ret;

    if (!g_i2c_available) return -1;

    u8 = 0;
    ret = I2C_IF_Write(ACCEL_I2C_ADDR, &reg, 1, 0);
    if (ret != SUCCESS) return ret;

    ret = I2C_IF_Read(ACCEL_I2C_ADDR, &u8, 1);
    if (ret != SUCCESS) return ret;

    *out = (int)((int8_t)u8);
    return SUCCESS;
}

static bool motion_detect_sample(void)
{
    int x, y;
    int ax, ay;
    int dx, dy;
    bool moved;

    x = 0; y = 0;
    if (accel_read_i8(ACCEL_REG_X, &x) != SUCCESS) return false;
    if (accel_read_i8(ACCEL_REG_Y, &y) != SUCCESS) return false;

    ax = (x < 0) ? -x : x;
    ay = (y < 0) ? -y : y;

    dx = x - g_prev_x; if (dx < 0) dx = -dx;
    dy = y - g_prev_y; if (dy < 0) dy = -dy;

    g_prev_x = x;
    g_prev_y = y;

    moved = false;
    if (ax >= MOTION_ABS_THRESHOLD || ay >= MOTION_ABS_THRESHOLD) moved = true;
    if (dx >= MOTION_DELTA_THRESHOLD || dy >= MOTION_DELTA_THRESHOLD) moved = true;

    if (moved) {
        if (g_motion_hits < 255) g_motion_hits++;
    } else {
        if (g_motion_hits > 0) g_motion_hits--;
    }

    return (g_motion_hits >= MOTION_HITS_REQUIRED);
}

//*****************************************************************************
// GPS (UART1) - parse RMC
//*****************************************************************************
typedef struct {
    bool valid;
    float lat;
    float lon;
} gps_fix_t;

static gps_fix_t g_fix = { false, 0.0f, 0.0f };
static bool g_gps_lock_reported = false;
static bool g_gps_fail_reported = false;
static uint32_t g_gps_connect_deadline = 0;
static bool g_gps_rx_seen = false;
static uint32_t g_gps_diag_lines = 0;

static void report_status_line(void)
{
    Report("Status: state=%s wifi=%s gps_uart=%s gps_fix=%s\r\n",
           state_to_str(g_state),
           g_wifi_connected ? "UP" : "DOWN",
           g_gps_rx_seen ? "RX" : "NO-RX",
           g_fix.valid ? "YES" : "NO");
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

static char* csv_next(char **p)
{
    char *s;
    char *c;

    if (p == 0 || *p == 0) return 0;
    s = *p;

    c = strchr(s, ',');
    if (c) {
        *c = '\0';
        *p = c + 1;
    } else {
        *p = 0;
    }
    return s;
}

static float nmea_to_decimal(const char *val, const char hemi)
{
    float v;
    int deg;
    float min;
    float dec;

    if (!val || !val[0]) return 0.0f;

    v = (float)atof(val);
    deg = (int)(v / 100.0f);
    min = v - ((float)deg * 100.0f);
    dec = (float)deg + (min / 60.0f);

    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

static void gps_try_parse_rmc(char *line)
{
    char *p;
    char *hdr;
    char *time;
    char *status;
    char *lat;
    char *latH;
    char *lon;
    char *lonH;
    bool was_valid;

    if (!line) return;
    if (strncmp(line, "$GPRMC", 6) != 0 && strncmp(line, "$GNRMC", 6) != 0) return;

    p = line;
    was_valid = g_fix.valid;

    hdr    = csv_next(&p);
    time   = csv_next(&p);
    status = csv_next(&p);
    lat    = csv_next(&p);
    latH   = csv_next(&p);
    lon    = csv_next(&p);
    lonH   = csv_next(&p);

    (void)hdr;
    (void)time;

    if (!status || status[0] != 'A') {
        g_fix.valid = false;
        if (was_valid && g_gps_lock_reported) {
            Report("GPS disconnected (lock lost)\r\n");
            g_gps_lock_reported = false;
        }
        return;
    }
    if (!lat || !latH || !lon || !lonH) {
        g_fix.valid = false;
        if (was_valid && g_gps_lock_reported) {
            Report("GPS disconnected (lock lost)\r\n");
            g_gps_lock_reported = false;
        }
        return;
    }

    g_fix.lat = nmea_to_decimal(lat, latH[0]);
    g_fix.lon = nmea_to_decimal(lon, lonH[0]);
    g_fix.valid = true;
    if (!g_gps_lock_reported) {
        Report("GPS connected (lock acquired)\r\n");
        g_gps_lock_reported = true;
        g_gps_fail_reported = false;
    }
}

static void GPSPoll(void)
{
    static char buf[128];
    static uint32_t idx = 0;
    int c;

    while (MAP_UARTCharsAvail(UARTA1_BASE)) {
        c = MAP_UARTCharGetNonBlocking(UARTA1_BASE);
        if (c < 0) break;

        if (c == '\r') continue;

        if (c == '\n') {
            buf[idx] = '\0';
            if (idx > 0) {
                char tmp[128];
                strncpy(tmp, buf, sizeof(tmp) - 1);
                tmp[sizeof(tmp) - 1] = '\0';
                g_gps_rx_seen = true;
                if (g_gps_diag_lines < GPS_DIAG_ECHO_LINES) {
                    Report("GPS NMEA: %s\r\n", tmp);
                    g_gps_diag_lines++;
                }
                gps_try_parse_rmc(tmp);
            }
            idx = 0;
        } else {
            if (idx < (sizeof(buf) - 1)) {
                buf[idx++] = (char)c;
            } else {
                idx = 0;
            }
        }
    }

    if (!g_fix.valid && !g_gps_fail_reported &&
        ((int32_t)(g_tick40ms - g_gps_connect_deadline) >= 0)) {
        if (g_gps_rx_seen) {
            Report("GPS no fix within %lus (likely indoors/weak sky view), continuing\r\n",
                   (unsigned long)(GPS_CONNECT_TIMEOUT_MS / 1000));
        } else {
            Report("GPS UART no data within %lus: check TX->PIN_59, GND, baud=9600\r\n",
                   (unsigned long)(GPS_CONNECT_TIMEOUT_MS / 1000));
        }
        g_gps_fail_reported = true;
    }
}

//*****************************************************************************
// Buzzer
//*****************************************************************************
static void buzzer_on(void)  { MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN); }
static void buzzer_off(void) { MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0); }

static void buzzer_beep_pattern_step(void)
{
    static uint32_t last = 0;
    static bool on = false;
    static uint8_t phase = 0;

    if (g_tick40ms == last) return;
    last = g_tick40ms;

    phase++;
    if (phase >= 4) {
        phase = 0;
        on = !on;
        if (on) buzzer_on();
        else buzzer_off();
    }
}

//*****************************************************************************
// UI helpers (IMPORTANT: setTextColor uses 2 args in your library)
//*****************************************************************************
static void ui_show_disarmed(void)
{
    if (!g_oled_available) return;
    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("Backpack\n");

    setTextColor(GREEN, BLACK);
    Outstr("DISARMED\n");

    setTextSize(1);
    setTextColor(WHITE, BLACK);
    Outstr("\nIR: LOCK to arm\n");
}

static void ui_show_arming(uint32_t remaining_ms)
{
    char tmp[32];
    if (!g_oled_available) return;

    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("Arming\n");

    setTextColor(YELLOW, BLACK);
    Outstr("WAIT\n");

    setTextSize(1);
    setTextColor(WHITE, BLACK);
    Outstr("\nGrace: ");
    snprintf(tmp, sizeof(tmp), "%lu ms\n", (unsigned long)remaining_ms);
    Outstr(tmp);

    Outstr("\nIR: UNLOCK cancels\n");
}

static void ui_show_armed(void)
{
    if (!g_oled_available) return;
    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("Backpack\n");

    setTextColor(CYAN, BLACK);
    Outstr("ARMED\n");

    setTextSize(1);
    setTextColor(WHITE, BLACK);
    Outstr("\nMonitoring motion\n");
    Outstr("IR: UNLOCK to stop\n");
}

static void ui_show_alert(void)
{
    if (!g_oled_available) return;
    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(RED, BLACK);
    Outstr("ALERT\n");

    setTextSize(1);
    setTextColor(WHITE, BLACK);
    Outstr("Movement detected\n");
    Outstr("\nIR: UNLOCK to disarm\n");
    Outstr("RESET: disarm\n");
}

static void ui_show_cloud_sent(bool ok)
{
    if (!g_oled_available) return;
    setCursor(0, 96);
    setTextSize(1);
    setTextColor(ok ? GREEN : YELLOW, BLACK);
    Outstr(ok ? "Cloud: sent\n" : "Cloud: failed\n");
}

static void ui_show_gps_line(void)
{
    char tmp[64];
    if (!g_oled_available) return;

    setCursor(0, 112);
    setTextSize(1);
    setTextColor(WHITE, BLACK);

    if (g_fix.valid) {
        snprintf(tmp, sizeof(tmp), "GPS: %.5f %.5f\n", g_fix.lat, g_fix.lon);
    } else {
        snprintf(tmp, sizeof(tmp), "GPS: no fix\n");
    }
    Outstr(tmp);
}

static void ui_oled_self_test(void)
{
    if (!g_oled_available) return;

    fillScreen(RED);
    MAP_UtilsDelay(500000);
    fillScreen(GREEN);
    MAP_UtilsDelay(500000);
    fillScreen(BLUE);
    MAP_UtilsDelay(500000);

    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("OLED OK\n");
    setTextSize(1);
    setTextColor(CYAN, BLACK);
    Outstr("SSD1351 128x128\n");
    MAP_UtilsDelay(700000);
}

//*****************************************************************************
// Reset button
//*****************************************************************************
static bool reset_pressed(void)
{
    unsigned long val;
    val = MAP_GPIOPinRead(RESET_BTN_PORT, RESET_BTN_PIN);
#if RESET_BTN_ACTIVE_LOW
    return (val == 0);
#else
    return (val != 0);
#endif
}

//*****************************************************************************
// WiFi + TLS time (SimpleLink only, no external helpers)
//*****************************************************************************
static int wifi_connect_simple(void)
{
    SlSecParams_t secParams;
    SlDateTime_t dt;
    long ret;
    long mode_ret;
    long stop_ret;
    int start_try;

    ret = -1;
    for (start_try = 1; start_try <= 3; start_try++) {
        Report("WiFi: sl_Start attempt %d...\r\n", start_try);
        ret = sl_Start(0, 0, 0);
        Report("WiFi: sl_Start ret=%ld\r\n", ret);
        if (ret >= 0) {
            break;
        }
        if (ret == SL_API_ABORTED) {
            stop_ret = sl_Stop(0xFF);
            Report("WiFi: sl_Stop after abort ret=%ld\r\n", stop_ret);
            MAP_UtilsDelay(1600000);
            continue;
        }
        return (int)ret;
    }
    if (ret < 0) return (int)ret;

    if (ret != ROLE_STA) {
        mode_ret = sl_WlanSetMode(ROLE_STA);
        Report("WiFi: sl_WlanSetMode(STA) ret=%ld\r\n", mode_ret);
        stop_ret = sl_Stop(0xFF);
        Report("WiFi: sl_Stop done ret=%ld, restarting...\r\n", stop_ret);
        MAP_UtilsDelay(800000);
        ret = sl_Start(0, 0, 0);
        Report("WiFi: sl_Start (after mode set) ret=%ld\r\n", ret);
        if (ret < 0) return (int)ret;
    }

    // Set date/time for TLS
    memset(&dt, 0, sizeof(dt));
    dt.sl_tm_day  = DATE;
    dt.sl_tm_mon  = MONTH;
    dt.sl_tm_year = YEAR;
    dt.sl_tm_hour = HOUR;
    dt.sl_tm_min  = MINUTE;
    dt.sl_tm_sec  = SECOND;

    sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
              SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
              sizeof(SlDateTime_t),
              (unsigned char *)&dt);

    // Connect to AP
    memset(&secParams, 0, sizeof(secParams));
    secParams.Key = (signed char *)WIFI_PASS;
    secParams.KeyLen = (unsigned char)strlen(WIFI_PASS);
    secParams.Type = WIFI_SEC;

    ret = sl_WlanConnect((signed char *)WIFI_SSID, strlen(WIFI_SSID), 0, &secParams, 0);
    Report("WiFi: sl_WlanConnect ret=%ld\r\n", ret);
    if (ret < 0) return (int)ret;

    // Wait until IP assigned (simple poll)
    // Wait until IP assigned (simple poll)
    {
        uint32_t start;
        SlNetCfgIpV4Args_t ipV4;
        long rc;

        start = g_tick40ms;

        while ((g_tick40ms - start) < ms_to_ticks40(15000)) {

            memset(&ipV4, 0, sizeof(ipV4));

        #if defined(SL_NETCFG_IPV4_STA_ADDR_MODE)
                    unsigned char len = (unsigned char)sizeof(ipV4);
                    unsigned char opt = 0;
                    rc = sl_NetCfgGet(SL_NETCFG_IPV4_STA_ADDR_MODE, &opt, &len, (unsigned char *)&ipV4);
        #elif defined(SL_IPV4_STA_P2P_CL_GET_INFO)
                    unsigned char len = (unsigned char)sizeof(ipV4);
                    unsigned char opt = 0;
                    rc = sl_NetCfgGet(SL_IPV4_STA_P2P_CL_GET_INFO, &opt, &len, (unsigned char *)&ipV4);
        #else
                    rc = -1;   // No known NetCfg constant in this SDK
        #endif

                    if ((rc >= 0) && (ipV4.ipV4 != 0)) {
                        Report("WiFi: IP acquired\r\n");
                        return 0;   // got an IPv4 address
                    }

                    MAP_UtilsDelay(800000);
                }
            }

        Report("WiFi: IP wait timeout\r\n");
        return -1;
}

//*****************************************************************************
// Cloud notify
//*****************************************************************************
static bool cloud_config_valid(void)
{
    bool ok = true;
    if (strcmp(SERVER_NAME, "YOUR_AWS_ENDPOINT") == 0) {
        Report("Cloud config error: set SERVER_NAME to your AWS IoT endpoint\r\n");
        ok = false;
    }
    if (strcmp(THING_NAME, "YOUR_THING_NAME") == 0) {
        Report("Cloud config error: set THING_NAME to your AWS IoT thing name\r\n");
        ok = false;
    }
    return ok;
}

static int tls_connect(void)
{
    SlSockAddrIn_t addr;
    unsigned long ip;
    int ret;
    int sock;
    unsigned int cipher;

    if (g_tls_sock >= 0) return g_tls_sock;

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = SL_AF_INET;
    addr.sin_port = sl_Htons(TLS_DST_PORT);

    ip = 0;
    ret = sl_NetAppDnsGetHostByName((signed char *)SERVER_NAME, strlen(SERVER_NAME), &ip, SL_AF_INET);
    if (ret < 0) return ret;

    addr.sin_addr.s_addr = sl_Htonl(ip);

    sock = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
    if (sock < 0) return sock;

    {
        SlSockSecureMethod method;
        method.secureMethod = SL_SO_SEC_METHOD_TLSV1_2;
        ret = sl_SetSockOpt(sock, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(method));
        if (ret < 0) {
            sl_Close(sock);
            return ret;
        }
    }

    // AWS IoT requires mutual TLS (server validation + client cert/key).
    cipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
    ret = sl_SetSockOpt(sock, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &cipher, sizeof(cipher));
    if (ret < 0) {
        sl_Close(sock);
        return ret;
    }

    ret = sl_SetSockOpt(sock, SL_SOL_SOCKET,
                        SL_SO_SECURE_FILES_CA_FILE_NAME,
                        AWS_CA_FILE, strlen(AWS_CA_FILE));
    if (ret < 0) {
        Report("TLS: CA file missing/invalid: %s (ret=%d)\r\n", AWS_CA_FILE, ret);
        sl_Close(sock);
        return ret;
    }

    ret = sl_SetSockOpt(sock, SL_SOL_SOCKET,
                        SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME,
                        AWS_CLIENT_CERT, strlen(AWS_CLIENT_CERT));
    if (ret < 0) {
        Report("TLS: client cert missing/invalid: %s (ret=%d)\r\n", AWS_CLIENT_CERT, ret);
        sl_Close(sock);
        return ret;
    }

    ret = sl_SetSockOpt(sock, SL_SOL_SOCKET,
                        SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME,
                        AWS_PRIVATE_KEY, strlen(AWS_PRIVATE_KEY));
    if (ret < 0) {
        Report("TLS: private key missing/invalid: %s (ret=%d)\r\n", AWS_PRIVATE_KEY, ret);
        sl_Close(sock);
        return ret;
    }

    ret = sl_Connect(sock, (SlSockAddr_t *)&addr, sizeof(addr));
    if (ret < 0) {
        sl_Close(sock);
        return ret;
    }

    g_tls_sock = sock;
    return sock;
}

static int cloud_post_event(const char *event_name)
{
    char json[256];
    char hdr[256];
    char rsp[128];
    int sock;
    int ret;
    unsigned int now_ms;

    if (!g_wifi_connected) {
        return -1;
    }
    if (!cloud_config_valid()) {
        return -3;
    }

    if (!event_name) event_name = "ALERT";
    now_ms = g_tick40ms * 40U;

    if (g_fix.valid) {
        snprintf(json, sizeof(json),
                 "{\"state\":{\"reported\":{\"event\":\"%s\",\"gps_fix\":true,\"lat\":%.6f,\"lon\":%.6f,\"uptime_ms\":%lu}}}",
                 event_name, (double)g_fix.lat, (double)g_fix.lon, (unsigned long)now_ms);
    } else {
        snprintf(json, sizeof(json),
                 "{\"state\":{\"reported\":{\"event\":\"%s\",\"gps_fix\":false,\"uptime_ms\":%lu}}}",
                 event_name, (unsigned long)now_ms);
    }

    snprintf(hdr, sizeof(hdr), "%s%s%s%s%s%u%s",
             POSTHEADER, HOSTHEADER, CHEADER, CTHEADER,
             CLHEADER1, (unsigned int)strlen(json), CLHEADER2);

    sock = tls_connect();
    if (sock < 0) return sock;

    ret = sl_Send(sock, hdr, strlen(hdr), 0);
    if (ret < 0) {
        sl_Close(sock);
        g_tls_sock = -1;
        return ret;
    }

    ret = sl_Send(sock, json, strlen(json), 0);
    if (ret < 0) {
        sl_Close(sock);
        g_tls_sock = -1;
        return ret;
    }

    memset(rsp, 0, sizeof(rsp));
    ret = sl_Recv(sock, rsp, sizeof(rsp) - 1, 0);
    if (ret < 0) {
        sl_Close(sock);
        g_tls_sock = -1;
        return ret;
    }
    if (ret > 0 && strstr(rsp, "200") == 0 && strstr(rsp, "204") == 0) {
        Report("Cloud: HTTP response: %s\r\n", rsp);
        return -2;
    }

    return 0;
}

//*****************************************************************************
// App logic
//*****************************************************************************
#define ARM_GRACE_MS        7000
#define MOTION_SAMPLE_MS    200

static void enter_state(system_state_t st)
{
    g_state = st;

    if (st == ST_DISARMED) {
        buzzer_off();
        g_motion_hits = 0;
        g_alert_sent = false;
        ui_show_disarmed();

    } else if (st == ST_ARMING) {
        buzzer_off();
        g_motion_hits = 0;
        g_alert_sent = false;

        g_arming_deadline = g_tick40ms + ms_to_ticks40(ARM_GRACE_MS);
        g_ui_last_countdown = 0;

        ui_show_arming(ARM_GRACE_MS);

    } else if (st == ST_ARMED) {
        buzzer_off();
        g_motion_hits = 0;
        g_alert_sent = false;

        accel_read_i8(ACCEL_REG_X, &g_prev_x);
        accel_read_i8(ACCEL_REG_Y, &g_prev_y);

        g_motion_next_sample = g_tick40ms + ms_to_ticks40(MOTION_SAMPLE_MS);
        ui_show_armed();

    } else {
        buzzer_on();
        ui_show_alert();
    }
}

static void service_button_toggle(void)
{
    static bool was_pressed = false;
    static bool action_fired = false;
    static uint32_t press_start = 0;
    bool now_pressed;

    now_pressed = reset_pressed();

    if (now_pressed && !was_pressed) {
        press_start = g_tick40ms;
        action_fired = false;
    }

    if (now_pressed && !action_fired &&
        ((g_tick40ms - press_start) >= ms_to_ticks40(BUTTON_TOGGLE_HOLD_MS))) {
        action_fired = true;
        if (g_state == ST_DISARMED) {
            Report("Button long-press: ARM\r\n");
            enter_state(ST_ARMING);
        } else {
            Report("Button long-press: DISARM\r\n");
            enter_state(ST_DISARMED);
        }
    }

    if (!now_pressed) {
        action_fired = false;
    }

    was_pressed = now_pressed;
}

static void handle_ir_command(uint16_t code)
{
    if (code == IR_CMD_UNLOCK) {
        enter_state(ST_DISARMED);
        return;
    }

    if (code == IR_CMD_LOCK) {
        if (g_state == ST_DISARMED) {
            enter_state(ST_ARMING);
        }
        return;
    }

    Report("IR unknown code: 0x%04X\r\n", (unsigned int)code);
}

static void update_arming(void)
{
    uint32_t now;
    uint32_t remaining_ticks;
    uint32_t remaining_ms;

    if (g_tick40ms >= g_arming_deadline) {
        enter_state(ST_ARMED);
        return;
    }

    now = g_tick40ms;
    if ((now - g_ui_last_countdown) >= ms_to_ticks40(200)) {
        g_ui_last_countdown = now;

        remaining_ticks = (g_arming_deadline > now) ? (g_arming_deadline - now) : 0;
        remaining_ms = remaining_ticks * 40;

        ui_show_arming(remaining_ms);
    }
}

static void update_armed(void)
{
    if (g_tick40ms >= g_motion_next_sample) {
        g_motion_next_sample = g_tick40ms + ms_to_ticks40(MOTION_SAMPLE_MS);

        if (motion_detect_sample()) {
            enter_state(ST_ALERT);
        }
    }
}

static void update_alert(void)
{
    buzzer_beep_pattern_step();

    if (!g_alert_sent) {
        int ret = cloud_post_event("BACKPACK_ALERT");
        g_alert_sent = true;
        ui_show_cloud_sent(ret == 0);
    }

    ui_show_gps_line();

    if (reset_pressed()) {
        enter_state(ST_DISARMED);
    }
}

//*****************************************************************************
// main
//*****************************************************************************
int main(void)
{
    int ret;
    uint16_t cmd;
    uint32_t next_status_tick;

    BoardInit();
    PinMuxConfig();

    InitTerm();
    // Keep boot logs visible in terminals that don't parse ANSI clear codes.
    Report("\r\n=== Anti-Theft Backpack Guardian Boot ===\r\n");
    Report("UART0: 115200 8N1 | UART1(GPS): 9600 8N1\r\n");
 #if ENABLE_GPS
    GPSUartInit();
    Report("UART1 GPS init: 9600 8N1\r\n");
 #else
    Report("UART1 GPS disabled by config\r\n");
 #endif

    SysTickInitApp();
 #if ENABLE_GPS
    g_gps_connect_deadline = g_tick40ms + ms_to_ticks40(GPS_CONNECT_TIMEOUT_MS);
    Report("GPS: waiting for fix (%lus timeout)\r\n",
           (unsigned long)(GPS_CONNECT_TIMEOUT_MS / 1000));
 #else
    g_gps_connect_deadline = 0;
 #endif

    Report("Init: OLED start\r\n");
#if ENABLE_OLED
    if (Adafruit_Init()) {
        g_oled_available = true;
        ui_oled_self_test();
        ui_show_disarmed();
        Report("Init: OLED done\r\n");
    } else {
        g_oled_available = false;
        Report("Init: OLED missing or not responding, continuing without display\r\n");
    }
#else
    g_oled_available = false;
    Report("Init: OLED disabled by config, continuing without display\r\n");
#endif

    Report("Init: I2C start\r\n");
#if ENABLE_I2C
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    g_i2c_available = true;
    Report("Init: I2C done\r\n");
#else
    g_i2c_available = false;
    Report("Init: I2C disabled by config, continuing\r\n");
#endif

    Report("Init: IR start\r\n");
#if ENABLE_IR
    IRInit();
    Report("Init: IR done\r\n");
#else
    Report("Init: IR disabled by config, continuing\r\n");
#endif

    MAP_GPIODirModeSet(RESET_BTN_PORT, RESET_BTN_PIN, GPIO_DIR_MODE_IN);
    MAP_GPIODirModeSet(BUZZER_PORT, BUZZER_PIN, GPIO_DIR_MODE_OUT);
    buzzer_off();

    Report("Init: WiFi connect start\r\n");
#if ENABLE_WIFI
    ret = wifi_connect_simple();
    if (ret < 0) {
        g_wifi_connected = false;
        Report("WiFi connect failed: %d\r\n", ret);
    } else {
        g_wifi_connected = true;
        Report("WiFi connected\r\n");
    }
    cloud_config_valid();
#else
    g_wifi_connected = false;
    Report("Init: WiFi disabled by config, continuing\r\n");
#endif

    next_status_tick = g_tick40ms + ms_to_ticks40(STATUS_PERIOD_MS);
    report_status_line();

    while (1) {
 #if ENABLE_GPS
        GPSPoll();
 #endif

        if ((int32_t)(g_tick40ms - next_status_tick) >= 0) {
            report_status_line();
            next_status_tick += ms_to_ticks40(STATUS_PERIOD_MS);
        }

        service_button_toggle();

        cmd = 0;
 #if ENABLE_IR
        if (IRPollCommand16(&cmd)) {
            Report("IR code: 0x%04X\r\n", (unsigned int)cmd);
            handle_ir_command(cmd);
        }
 #endif

        if (g_state == ST_ARMING) {
            update_arming();
        } else if (g_state == ST_ARMED) {
            update_armed();
        } else if (g_state == ST_ALERT) {
            update_alert();
        }

        MAP_UtilsDelay(40000);
    }
}
//hello vs code
// hi
