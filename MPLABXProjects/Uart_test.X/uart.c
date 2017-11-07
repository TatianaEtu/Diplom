/*
 * File:   uart.c
 * Author: PC
 *
 * Created on November 28, 2016, 12:03 PM
 */
#define FCY 7372800UL
#pragma config FPR = XT_PLL4            // Primary Oscillator Mode (XT w/PLL 4x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD     

// FICD


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h"
#include "libpic30.h"
#include "p30F4012.h"
#include <stdio.h>
#include <stdlib.h>


#define UART_BAUD 9600
#define UBRG_VALUE (FCY/UART_BAUD/16) - 1   // UBRG_VALUE=47

int RX_byte = 0,TX_byte = 0; // save received byte and transmitt byte

void uart_init(void)
{
    U1MODEbits.UARTEN = 1; //UART Enable
    U1STAbits.UTXEN=1;//UART transmitter enabled, UxTX pin controlled by UART
    U1BRG=UBRG_VALUE; ///baud rate 9600
    IEC0bits.U1RXIE=1; //enable RX1 interrupt
    //IEC0bits.U1TXIE=1; //enable TX1 interrupt
    U1STAbits.URXISEL1=1;//If URXISEL<1:0> = 11, an interrupt is generated when a word is transferred from the
    U1STAbits.URXISEL0=1;//Receive Shift register (UxRSR) to the receive buffer and as a result, the receive buffer
                         //contains 4 characters (i.e., becomes full).    
}
void WriteUART (unsigned int data)
{
    U1TXREG = data;
    while(!U1STAbits.TRMT){}
    //LATBbits.LATB0=!LATBbits.LATB0; // led
}

void WriteStringUART(const char * s)
{
    while(*s)
    {
       WriteUART(*s++);
    }
}
//void ReadUART (unsigned int data)
//{
//    U1TXREG = data;
//    while(!U1STAbits.TRMT){}
//}
void __attribute__((__interrupt__)) _U1RXInterrupt(void) // Recieve data interrupt
{
    IFS0bits.U1RXIF = 0;
    RX_byte = U1RXREG;
   // RX_byte = RX_byte + 1;
   // WriteUART(RX_byte);
        //WriteStringUART(" ");
    //LATBbits.LATB0=!LATBbits.LATB0; // led
}

void interrupt_init() // not used (Index pulse)
{ 
    INTCON2bits.INT0EP=0;// Interrupt on positive edge
    TRISEbits.TRISE8=1; // input (INT0)
    IEC0bits.INT0IE = 1;    /*Enable INT0 Interrupt Service Routine */
    //IPC0bits.INT0IP=0b00000111; // highest (7) priority   
    IFS0bits.INT0IF = 0;    /*Reset INT0 interrupt flag */         
}
void __attribute__ ((interrupt)) _INT0Interrupt(void) //button interrupt
{            
    LATBbits.LATB0=!LATBbits.LATB0; // led
    WriteUART(31); 
    IFS0bits.INT0IF = 0;// clear the interrupt flag
}

int main(void) 
{
    int a=0, i=0;
    uart_init();
    interrupt_init();
    //WriteUART(31); 
    TRISBbits.TRISB0=0; // output led
   // LATBbits.LATB0=1;
    while(1)
    {
//       WriteUART(a); 
//       a++;
//       __delay_ms(500);
    }
    return 0;
}
