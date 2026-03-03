#include <stdint.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "spi.h"
#include "prcm.h"
#include "rom_map.h"
#include "utils.h"

#include "Adafruit_SSD1351.h"

// GPIO pins (match your pinmux.c wiring)
static inline void OLED_CS_Low(void)   { MAP_GPIOPinWrite(GPIOA3_BASE, 0x10, 0x00); }
static inline void OLED_CS_High(void)  { MAP_GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10); }
static inline void OLED_DC_Cmd(void)   { MAP_GPIOPinWrite(GPIOA3_BASE, 0x80, 0x00); }
static inline void OLED_DC_Data(void)  { MAP_GPIOPinWrite(GPIOA3_BASE, 0x80, 0x80); }
static int g_oled_ready = 0;

static inline void OLED_ResetPulse(void)
{
    MAP_GPIOPinWrite(GPIOA2_BASE, 0x02, 0x00);
    MAP_UtilsDelay(800000);
    MAP_GPIOPinWrite(GPIOA2_BASE, 0x02, 0x02);
    MAP_UtilsDelay(800000);
}

static int OLED_SPI_Write8(unsigned char b)
{
    long wrote;
    long got;
    unsigned long dummy;
    int guard;

    if (!g_oled_ready) return 0;

    wrote = 0;
    for (guard = 0; guard < 20000; guard++) {
        wrote = MAP_SPIDataPutNonBlocking(GSPI_BASE, (unsigned long)b);
        if (wrote) break;
    }
    if (!wrote) return 0;

    got = 0;
    for (guard = 0; guard < 20000; guard++) {
        got = MAP_SPIDataGetNonBlocking(GSPI_BASE, &dummy);
        if (got) break;
    }
    if (!got) return 0;

    return 1;
}

static void OLED_SPI_Init(void)
{
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE,
                           MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           1000000,
                           SPI_MODE_MASTER,
                           SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS |
                            SPI_4PIN_MODE |
                            SPI_TURBO_OFF |
                            SPI_CS_ACTIVELOW |
                            SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);
    MAP_SPICSDisable(GSPI_BASE);
}

void writeCommand(unsigned char c)
{
    if (!g_oled_ready) return;
    OLED_DC_Cmd();
    OLED_CS_Low();
    if (!OLED_SPI_Write8(c)) {
        g_oled_ready = 0;
    }
    OLED_CS_High();
}

void writeData(unsigned char d)
{
    if (!g_oled_ready) return;
    OLED_DC_Data();
    OLED_CS_Low();
    if (!OLED_SPI_Write8(d)) {
        g_oled_ready = 0;
    }
    OLED_CS_High();
}

unsigned int Color565(unsigned char r, unsigned char g, unsigned char b)
{
    return (unsigned int)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

int Adafruit_Init(void)
{
    g_oled_ready = 1;
    OLED_SPI_Init();

    OLED_CS_High();
    OLED_DC_Data();

    OLED_ResetPulse();

    writeCommand(SSD1351_CMD_COMMANDLOCK);
    writeData(0x12);
    writeCommand(SSD1351_CMD_COMMANDLOCK);
    writeData(0xB1);

    writeCommand(SSD1351_CMD_DISPLAYOFF);

    writeCommand(SSD1351_CMD_CLOCKDIV);
    writeData(0xF1);

    writeCommand(SSD1351_CMD_MUXRATIO);
    writeData(127);

    writeCommand(SSD1351_CMD_SETREMAP);
    writeData(0x74);

    writeCommand(SSD1351_CMD_STARTLINE);
    writeData(0x00);

    writeCommand(SSD1351_CMD_DISPLAYOFFSET);
    writeData(0x00);

    writeCommand(SSD1351_CMD_SETGPIO);
    writeData(0x00);

    writeCommand(SSD1351_CMD_FUNCTIONSELECT);
    writeData(0x01);

    writeCommand(SSD1351_CMD_PRECHARGE);
    writeData(0x32);

    writeCommand(SSD1351_CMD_VCOMH);
    writeData(0x05);

    writeCommand(SSD1351_CMD_NORMALDISPLAY);

    writeCommand(SSD1351_CMD_CONTRASTABC);
    writeData(0xC8);
    writeData(0x80);
    writeData(0xC8);

    writeCommand(SSD1351_CMD_CONTRASTMASTER);
    writeData(0x0F);

    writeCommand(SSD1351_CMD_SETVSL);
    writeData(0xA0);
    writeData(0xB5);
    writeData(0x55);

    writeCommand(SSD1351_CMD_PRECHARGE2);
    writeData(0x01);

    writeCommand(SSD1351_CMD_DISPLAYON);

    MAP_UtilsDelay(800000);
    return g_oled_ready;
}

void goTo(int x, int y)
{
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData((unsigned char)x);
    writeData(127);

    writeCommand(SSD1351_CMD_SETROW);
    writeData((unsigned char)y);
    writeData(127);
}

static void setWindow(int x0, int y0, int x1, int y1)
{
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData((unsigned char)x0);
    writeData((unsigned char)x1);

    writeCommand(SSD1351_CMD_SETROW);
    writeData((unsigned char)y0);
    writeData((unsigned char)y1);

    writeCommand(SSD1351_CMD_WRITERAM);
}

void drawPixel(int x, int y, unsigned int color)
{
    if (!g_oled_ready) return;
    if (x < 0 || x >= 128 || y < 0 || y >= 128) return;

    setWindow(x, y, x, y);

    OLED_DC_Data();
    OLED_CS_Low();
    if (!OLED_SPI_Write8((unsigned char)(color >> 8)) ||
        !OLED_SPI_Write8((unsigned char)(color & 0xFF))) {
        g_oled_ready = 0;
    }
    OLED_CS_High();
}

void fillScreen(unsigned int fillcolor)
{
    int x;
    int y;
    if (!g_oled_ready) return;

    setWindow(0, 0, 127, 127);

    OLED_DC_Data();
    OLED_CS_Low();

    for (y = 0; y < 128; y++) {
        for (x = 0; x < 128; x++) {
            if (!OLED_SPI_Write8((unsigned char)(fillcolor >> 8)) ||
                !OLED_SPI_Write8((unsigned char)(fillcolor & 0xFF))) {
                g_oled_ready = 0;
                OLED_CS_High();
                return;
            }
        }
    }

    OLED_CS_High();
}

void drawFastHLine(int x, int y, int w, unsigned int color)
{
    int i;
    if (!g_oled_ready) return;

    if (y < 0 || y >= 128 || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if ((x + w) > 128) w = 128 - x;
    if (w <= 0) return;

    setWindow(x, y, x + w - 1, y);

    OLED_DC_Data();
    OLED_CS_Low();
    for (i = 0; i < w; i++) {
        if (!OLED_SPI_Write8((unsigned char)(color >> 8)) ||
            !OLED_SPI_Write8((unsigned char)(color & 0xFF))) {
            g_oled_ready = 0;
            OLED_CS_High();
            return;
        }
    }
    OLED_CS_High();
}

void drawFastVLine(int x, int y, int h, unsigned int color)
{
    int i;
    if (!g_oled_ready) return;

    if (x < 0 || x >= 128 || h <= 0) return;
    if (y < 0) { h += y; y = 0; }
    if ((y + h) > 128) h = 128 - y;
    if (h <= 0) return;

    setWindow(x, y, x, y + h - 1);

    OLED_DC_Data();
    OLED_CS_Low();
    for (i = 0; i < h; i++) {
        if (!OLED_SPI_Write8((unsigned char)(color >> 8)) ||
            !OLED_SPI_Write8((unsigned char)(color & 0xFF))) {
            g_oled_ready = 0;
            OLED_CS_High();
            return;
        }
    }
    OLED_CS_High();
}

void fillRect(unsigned int x0, unsigned int y0, unsigned int w, unsigned int h, unsigned int color)
{
    unsigned int x;
    unsigned int y;
    if (!g_oled_ready) return;

    if (w == 0 || h == 0) return;
    if (x0 >= 128 || y0 >= 128) return;

    if ((x0 + w) > 128U) w = 128U - x0;
    if ((y0 + h) > 128U) h = 128U - y0;
    if (w == 0 || h == 0) return;

    setWindow((int)x0, (int)y0, (int)(x0 + w - 1U), (int)(y0 + h - 1U));

    OLED_DC_Data();
    OLED_CS_Low();
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            if (!OLED_SPI_Write8((unsigned char)(color >> 8)) ||
                !OLED_SPI_Write8((unsigned char)(color & 0xFF))) {
                g_oled_ready = 0;
                OLED_CS_High();
                return;
            }
        }
    }
    OLED_CS_High();
}
