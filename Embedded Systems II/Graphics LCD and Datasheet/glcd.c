// Graphics LCD Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Keyboard Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE5 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE4 drives an NPN transistor that powers the blue LED
// ST7565R Graphics LCD Display Interface:
//   MOSI on PD3 (SSI1Tx)
//   SCLK on PD0 (SSI1Clk)
//   ~CS on PD1 (SSI1Fss)
//   A0 connected to PD2

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "graphics_lcd.h"
#include "wait.h"
#include "tm4c123gh6pm.h"

// Pin bitbands
#define RED_BL_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define GREEN_BL_LED (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define BLUE_BL_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

// PortB masks
#define RED_BL_LED_MASK 32

// PortE masks
#define GREEN_BL_LED_MASK 32
#define BLUE_BL_LED_MASK 16

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= RED_BL_LED_MASK;               // make bit5 an output
    GPIO_PORTB_DR2R_R |= RED_BL_LED_MASK;              // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= RED_BL_LED_MASK;               // enable bit5 for digital
    GPIO_PORTE_DIR_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;  // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK; // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;  // enable bits 4 and 5 for digital
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();

    // Turn-on all LEDs to create white backlight
    RED_BL_LED = 1;
    GREEN_BL_LED = 1;
    BLUE_BL_LED = 1;

    // Initialize graphics LCD
    initGraphicsLcd();

    // Draw X in left half of screen
    uint8_t i;
    for (i = 0; i < 64; i++)
        drawGraphicsLcdPixel(i, i, SET);
    for (i = 0; i < 64; i++)
        drawGraphicsLcdPixel(63-i, i, INVERT);

    // Draw text on screen
    setGraphicsLcdTextPosition(84, 5);
    putsGraphicsLcd("Text");

    // Draw flashing block around the text
    while(true)
    {
        drawGraphicsLcdRectangle(83, 39, 25, 9, INVERT);
        waitMicrosecond(500000);
    }
}
