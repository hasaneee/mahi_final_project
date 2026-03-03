#include "oled_test.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

#include "rom_map.h"
#include "utils.h"

void delay(unsigned long ulCount)
{
    MAP_UtilsDelay(ulCount);
}

void testlines(unsigned int color)
{
    int x;
    int y;

    fillScreen(BLACK);

    for (x = 0; x < 128; x += 6) {
        drawLine(0, 0, x, 127, color);
    }
    for (y = 0; y < 128; y += 6) {
        drawLine(0, 0, 127, y, color);
    }
}

void testfastlines(unsigned int color1, unsigned int color2)
{
    int x;
    int y;

    fillScreen(BLACK);

    for (y = 0; y < 128; y += 5) {
        drawFastHLine(0, y, 128, color1);
    }
    for (x = 0; x < 128; x += 5) {
        drawFastVLine(x, 0, 128, color2);
    }
}

void testdrawrects(unsigned int color)
{
    int i;

    fillScreen(BLACK);
    for (i = 0; i < 128; i += 6) {
        drawRect(i / 2, i / 2, 128 - i, 128 - i, color);
    }
}

void testfillrects(unsigned int color1, unsigned int color2)
{
    int i;

    fillScreen(BLACK);
    for (i = 0; i < 128; i += 6) {
        fillRect(i / 2, i / 2, 128 - i, 128 - i, (i % 12) ? color1 : color2);
    }
}

void testfillcircles(unsigned char radius, unsigned int color)
{
    int x;
    int y;

    fillScreen(BLACK);
    for (x = radius; x < 128; x += (int)radius * 2) {
        for (y = radius; y < 128; y += (int)radius * 2) {
            fillCircle(x, y, radius, color);
        }
    }
}

void testdrawcircles(unsigned char radius, unsigned int color)
{
    int x;
    int y;

    fillScreen(BLACK);
    for (x = radius; x < 128; x += (int)radius * 2) {
        for (y = radius; y < 128; y += (int)radius * 2) {
            drawCircle(x, y, radius, color);
        }
    }
}

void testtriangles(void)
{
    int t;
    int x;
    int y;
    int z;
    unsigned int color;

    fillScreen(BLACK);

    color = 0xF800;
    x = 64;
    y = 72;
    z = 32;

    for (t = 0; t <= 15; t++) {
        drawTriangle(x, y - z, x - z, y + z, x + z, y + z, color);
        z += 4;
        color = (unsigned int)(color + 1000);
    }
}

void testroundrects(void)
{
    int i;
    unsigned int color;

    fillScreen(BLACK);
    color = 100;

    for (i = 0; i < 128; i += 8) {
        drawRoundRect(i / 2, i / 2, 128 - i, 128 - i, 10, color);
        color = (unsigned int)(color + 1100);
    }
}

void lcdTestPattern(void)
{
    int x;
    unsigned int c;

    for (x = 0; x < 128; x++) {
        if (x < 16) c = RED;
        else if (x < 32) c = YELLOW;
        else if (x < 48) c = GREEN;
        else if (x < 64) c = CYAN;
        else if (x < 80) c = BLUE;
        else if (x < 96) c = MAGENTA;
        else if (x < 112) c = WHITE;
        else c = ORANGE;

        drawFastVLine(x, 0, 128, c);
    }
}

void lcdTestPattern2(void)
{
    int y;
    unsigned int c;

    for (y = 0; y < 128; y++) {
        if (y < 16) c = RED;
        else if (y < 32) c = YELLOW;
        else if (y < 48) c = GREEN;
        else if (y < 64) c = CYAN;
        else if (y < 80) c = BLUE;
        else if (y < 96) c = MAGENTA;
        else if (y < 112) c = WHITE;
        else c = ORANGE;

        drawFastHLine(0, y, 128, c);
    }
}

void testfullfont(void)
{
    int i;
    char s[2];

    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(1);
    setTextColor(WHITE, BLACK);

    for (i = 32; i < 127; i++) {
        s[0] = (char)i;
        s[1] = '\0';
        Outstr(s);

        if ((i % 16) == 15) {
            setCursor(0, (i / 16) * 8);  // crude line wrap
        }
    }
}

void testVertical(void)
{
    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("Vertical");
}

void testHorizontal(void)
{
    fillScreen(BLACK);
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("Horizontal");
}

void testHelloWorld(void)
{
    fillScreen(BLACK);

    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);
    Outstr("Hello");

    setCursor(0, 24);
    Outstr("World");
}
