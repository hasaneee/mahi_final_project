#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"

/*
 * Keep optional peripheral pinmux disabled unless that hardware is in use.
 * Touching extra pins can interfere with NWP bring-up on some CC3200 boards.
 */
#define PINMUX_ENABLE_IR      1
#define PINMUX_ENABLE_OLED    1
#define PINMUX_ENABLE_I2C     1
#define PINMUX_ENABLE_GPS     1

/*
 * Pin map used by the Backpack Anti-Theft Guardian firmware
 *
 * I2C0 (Accelerometer)
 *   - PIN_01 : I2C0 SCL
 *   - PIN_02 : I2C0 SDA
 *
 * OLED (SSD1351) over GSPI
 *   - PIN_05 : GSPI CLK
 *   - PIN_07 : GSPI MOSI
 *   - PIN_18 : OLED CS   (GPIOA3, 0x10)
 *   - PIN_45 : OLED DC   (GPIOA3, 0x80)
 *   - PIN_08 : OLED RST  (GPIOA2, 0x02)
 *
 * GPS (UART1)
 *   - PIN_58 : UART1 TX
 *   - PIN_59 : UART1 RX
 *
 * IR Receiver input
 *   - PIN_03 : GPIO input (GPIOA1, 0x10)
 *
 * Reset/Disarm button
 *   - PIN_04 : GPIO input (GPIOA1, 0x20)
 *
 * Buzzer output
 *   - PIN_64 : GPIO output (GPIOA1, 0x02)
 */

//*****************************************************************************
void PinMuxConfig(void)
{
    //
    // Avoid forcing "unused" pin modes here.
    // On CC3200, changing certain pins can interfere with NWP/flash behavior
    // and cause sl_Start() to hang.
    // JTAG pins 16,17,19,20 must remain untouched.
    //

    //
    // Enable peripheral clocks used by this project
    //
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
#if PINMUX_ENABLE_OLED
    PRCMPeripheralClkEnable(PRCM_GSPI,  PRCM_RUN_MODE_CLK);
#endif

#if PINMUX_ENABLE_IR
    //
    // IR receiver input on PIN_03 -> GPIOA1, bit 0x10
    //
    PinTypeGPIO(PIN_03, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_IN);
#endif

    //
    // Reset/Disarm button on PIN_04 -> GPIOA1, bit 0x20
    //
    PinTypeGPIO(PIN_04, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

    //
    // Buzzer output on PIN_64 -> GPIOA1, bit 0x02
    //
    PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, 0x02, GPIO_DIR_MODE_OUT);

#if PINMUX_ENABLE_OLED
    //
    // OLED RESET on PIN_08 -> GPIOA2, bit 0x02
    //
    PinTypeGPIO(PIN_08, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA2_BASE, 0x02, GPIO_DIR_MODE_OUT);

    //
    // OLED CS on PIN_18 -> GPIOA3, bit 0x10
    //
    PinTypeGPIO(PIN_18, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);

    //
    // OLED DC on PIN_45 -> GPIOA3, bit 0x80
    //
    PinTypeGPIO(PIN_45, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA3_BASE, 0x80, GPIO_DIR_MODE_OUT);

    //
    // GSPI pins
    //
    PinTypeSPI(PIN_05, PIN_MODE_7); // GSPI_CLK
    PinTypeSPI(PIN_07, PIN_MODE_7); // GSPI_MOSI
#endif

#if PINMUX_ENABLE_GPS
    //
    // UART1 pins for GPS
    //
    PinTypeUART(PIN_58, PIN_MODE_6); // UART1_TX
    PinTypeUART(PIN_59, PIN_MODE_6); // UART1_RX
#endif

    //
    // UART0 pins (debug)
    //
    PinTypeUART(PIN_55, PIN_MODE_3); // UART0_TX
    PinTypeUART(PIN_57, PIN_MODE_3); // UART0_RX

#if PINMUX_ENABLE_I2C
    //
    // I2C0 pins for accelerometer
    //
    PinTypeI2C(PIN_01, PIN_MODE_1); // I2C_SCL
    PinTypeI2C(PIN_02, PIN_MODE_1); // I2C_SDA
#endif
}
