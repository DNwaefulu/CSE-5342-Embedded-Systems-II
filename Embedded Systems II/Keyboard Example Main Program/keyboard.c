// Keyboard Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// 4x4 Keyboard
//   Column 0-3 outputs on PA6, PA7, PD2, PD3 are connected to cathode of diodes whose anode connects to column of keyboard
//   Rows 0-3 inputs connected to PE1, PE2, PE3, PF1 which are pulled high
//   To locate a key (r, c), the column c is driven low so the row r reads as low

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "kb.h"
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer

    // Configure LED pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 3, 2, and 1 are outputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0E;  // enable LEDs

    // Configure keyboard
    // Columns 0-3 connected to PA6, PA7, PD2, PD3
    // Rows 0-3 connected to PE1, PE2, PE3, PF1
    GPIO_PORTA_DIR_R |= 0xC0;  // bits 6 and 7 are outputs
    GPIO_PORTD_DIR_R |= 0x0C;  // bits 2 and 3 are outputs
    GPIO_PORTA_DEN_R |= 0xC0;  // bits 6 and 7 are digital
    GPIO_PORTD_DEN_R |= 0x0C;  // bits 2 and 3 are digital
    GPIO_PORTE_DEN_R |= 0x0E;  // bits 1-3 are digital
    GPIO_PORTF_DEN_R |= 0x02;  // bit 1 is digital
    GPIO_PORTA_ODR_R |= 0xC0;  // bits 6 and 7 are outputs
    GPIO_PORTD_ODR_R |= 0x0C;  // bits 2 and 3 are outputs
    GPIO_PORTE_PUR_R = 0x0E;  // enable internal pull-up for rows 0-2
    GPIO_PORTF_PUR_R = 0x02;  // enable internal pull-up for row 3

    // Configure UART0 pins
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
	UART0_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
    NVIC_EN0_R |= 1 << (INT_UART0-16);               // turn-on interrupt 21 (UART0)

    // Configure Timer 1 for keyboard service
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x30D40;                        // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// For each received character, toggle the green LED
// For each received "1", set the red LED
// For each received "0", clear the red LED
void Uart0Isr()
{
	char c = UART0_DR_R & 0xFF;
	if (c == '1')
	    RED_LED = 1;
	if (c == '0')
		RED_LED = 0;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();
	initKb();

	GREEN_LED = 1;
	waitMicrosecond(250000);
	GREEN_LED = 0;
	waitMicrosecond(250000);

	// Display greeting
    putsUart0("Keyboard Example\r\n");
    putsUart0("Press '0' or '1' to turn LED on and off\r\n");

    // Poll keyboard
	char c;
    while(1)
    {
    	// Send characters to UART0 if available
		if (kbhit())
		{
			GREEN_LED ^= 1;
			c = getKey();
			if (c != 'D')
                putcUart0(c);
			else
				putsUart0("\r\n");
		}
		// Emulate a foreground task that is running continuously
  		BLUE_LED ^= 1;
		waitMicrosecond(250000);
    }
}
