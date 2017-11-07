/*
 * File:   uart.c
 * Author: PC
 *
 * Created on November 28, 2016, 12:03 PM
 */
#define FCY 7372800UL


// DSPIC30F2010 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT_PLL4            // Primary Oscillator Mode (XT w/PLL 4x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = NONE            // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = PGD                // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include "xc.h"
#include "p30F2010.h"
//#include "uart.h"
#define UART_BAUD 9600
#define UBRG_VALUE (FCY/UART1_BAUD/16) - 1   // UBRG_VALUE=47

int RX_byte; // save received byte

void uart_init(void)
{
    U1MODEbits.UARTEN = 1; //UART Enable
    U1STAbits.UTX1EN=1;//UART transmitter enabled, UxTX pin controlled by UART
    U1BRG=UBRG_VALUE; ///baud rate 9600
    IEC0bits.U1RXIE=1; //enable RX1 interrupt
    //IEC0bits.U1TXIE=1; //enable TX1 interrupt
}
void WriteUART (unsigned int data)
{
    U1TXREG = data;
    while(!U1STAbits.TRMT){}
}
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    RX_byte = U1RXREG;
}
int main(void) 
{
    uart_init();
    WriteUART(23); 
    while(1)
    {
         
    }
    return 0;
}
