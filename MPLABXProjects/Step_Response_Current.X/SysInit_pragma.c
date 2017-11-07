/*
 * File:   SysInit_pragma.c
 * Author: PC
 *
 * Created on April 19, 2017, 11:32 AM
 */

// DSPIC33FJ256MC710 Configuration Bit Settings

// 'C' source line config statements
#define FCY 16000000UL


#include "xc.h"

void sysInit(void)
{
// Disable Watch Dog Timer
    RCONbits.SWDTEN=0;	
	
// Configure Oscillator to operate the device at 40Mhz
// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
// Fosc= 7.37*M40/(2*2)=73.7Mhz for 7.37M input clock
	PLLFBD=38;					// M=40
	CLKDIVbits.PLLPOST=0;		// N1=2
	CLKDIVbits.PLLPRE=0;		// N2=2
	OSCTUN=0;					// Tune FRC oscillator, if FRC is used

// Perform Clock Switch to FRC+PLL
// Clock switching to incorporate PLL
	__builtin_write_OSCCONH(0x01);		// Initiate Clock Switch to
													// FRC with PLL (NOSC=0b001)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0b001);	// Wait for Clock switch to occur	

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {}; 
}
