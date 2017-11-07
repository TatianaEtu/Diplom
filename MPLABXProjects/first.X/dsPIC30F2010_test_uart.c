/*
 * File:   newmainXC16.c
 * Author: PC
 *
 * Created on October 16, 2016, 3:47 PM
 */


#define FCY 7372800UL

// DSPIC30F2010 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT_PLL4          // Primary Oscillator Mode (XT w/PLL 16x)
#pragma config FOS = FRC                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
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

#include <xc.h>
#include "libpic30.h"
#include "p30F2010.h"

#define UART_BAUD  9600
#define UBRG_VALUE  (FCY/UART_BAUD/16) - 1  // UBRG_VALUE=47

void uart_init(void) {   
    U1BRG = UBRG_VALUE; ///baud rate 9600
    IEC0bits.U1RXIE = 1; //enable RX1 interrupt
    //IEC0bits.U1TXIE=1; //enable TX1 interrupt
    U1STAbits.URXISEL1 = 1; //If URXISEL<1:0> = 11, an interrupt is generated when a word is transferred from the
    U1STAbits.URXISEL0 = 1; //Receive Shift register (UxRSR) to the receive buffer and as a result, the receive buffer
    //contains 4 characters (i.e., becomes full).
    U1MODEbits.UARTEN = 1; //UART Enable    
    U1STAbits.UTXEN = 1; //UART transmitter enabled, UxTX pin controlled by UART
}
void WriteUART(int data) //? unsigned char - 8 bits, or unsigned int = 16 bits, size of transmit buffer?
{
    U1TXREG = data; // 8 bit mode 
    while (!U1STAbits.TRMT) {
    }
}
void WriteTwoByte(int par) // find low and high bytes in 16bits variable and send it 
{
    char h = 0, l = 0; //     
    h = (char) ((par >> 8)); // find high byte 
    WriteUART(h); //send high byte
    l = (char) ((par)); //find low byte
    WriteUART(l); // send low byte !
}

int main(void) 
{   
    uart_init(); 
    TRISEbits.TRISE0 = 0;
    _TRISB0=0;
    while(1)
    {
       LATEbits.LATE0=!LATEbits.LATE0;
       //_LATB0=!_LATB0;
      // WriteUART(0b00001111);
       //__delay_ms(1000);
    }  
    return 0;     
}