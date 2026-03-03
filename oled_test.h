#ifndef OLED_TEST_H_
#define OLED_TEST_H_

// ---------------------------------------------------------------------------
// Basic 16-bit (RGB565) color constants and simple OLED test helpers.
// ---------------------------------------------------------------------------

#define BLACK    0x0000
#define BLUE     0x001F
#define GREEN    0x07E0
#define CYAN     0x07FF
#define RED      0xF800
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

// Extra colors (RGB565)
#define ORANGE   0xFD20
#define INDIGO   0x4810
#define PURPLE   0x69D5

void delay(unsigned long ulCount);

void testfastlines(unsigned int color1, unsigned int color2);
void testdrawrects(unsigned int color);
void testfillrects(unsigned int color1, unsigned int color2);
void testfillcircles(unsigned char radius, unsigned int color);
void testdrawcircles(unsigned char radius, unsigned int color);
void testtriangles(void);
void testroundrects(void);
void testlines(unsigned int color);
void lcdTestPattern(void);
void lcdTestPattern2(void);
void testfullfont(void);
void testVertical(void);
void testHorizontal(void);
void testHelloWorld(void);

#endif /* OLED_TEST_H_ */
