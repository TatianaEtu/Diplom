/*
 * File:   adc.c
 * Author: PC
 *
 * Created on December 22, 2016, 2:45 PM
 */

#define FCY 7372800UL

// DSPIC30F4012 Configuration Bit Settings

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
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include "xc.h"
#include "libpic30.h"
#include "p30F4012.h"
#include <stdio.h>
#include <stdlib.h>

#define lcd_clear() lcd_command(1) 
#define lcd_origin() lcd_command(2) 
#define E_pulse_with 100 
#define LCD_Data4 LATE 

#define UART_BAUD 9600
#define UBRG_VALUE (FCY/UART_BAUD/16) - 1   // UBRG_VALUE=47

#define LCD_E LATCbits.LATC14
#define LCD_RS LATCbits.LATC13 

#define ADC_MASK 3 //0b10000000; // choose adc channel

unsigned int ADResult1 = 0;
unsigned int ADResult2 = 0;
int res = 0;

void lcd_clk(void)
{ 
  LCD_E = 1; 
  __delay_us(E_pulse_with); 
  LCD_E = 0; 
  __delay_us(E_pulse_with); 
} 
void lcd_command(unsigned char outbyte) 
{ 
  LCD_RS=0; 
  LCD_Data4=((LCD_Data4&0xf0)|((outbyte>>4)&0x0f));    
  lcd_clk(); 
  LCD_Data4=((LCD_Data4&0xf0)|(outbyte&0x0f));   
  lcd_clk(); 
  __delay_ms(10); 
} 
void lcd_putc(char outbyte) 
{ 
  LCD_RS=1; 
  LCD_Data4=((LCD_Data4&0xf0)|((outbyte>>4)&0x0f));   
  lcd_clk(); 
  LCD_Data4=((LCD_Data4&0xf0)|(outbyte&0x0f));   
  lcd_clk(); 
} 
void lcd_puts(unsigned char line,const char *p) 
{ 
 lcd_origin();        
 lcd_command(line);       
 while( *p )                 
 { 
    lcd_putc( *p );             
    p++;                      
 } 
} 
void inttolcd(unsigned char posi, signed int value) 
{ 
 char buff[16]; 
 sprintf(buff,"%5d",value); 
 lcd_puts(posi,buff);  
} 
void lcd_init() 
{ 
  TRISCbits.TRISC14=0; // E 
  TRISCbits.TRISC13=0; //RS
  
  TRISEbits.TRISE0=0;//data
  TRISEbits.TRISE1=0;
  TRISEbits.TRISE2=0;
  TRISEbits.TRISE3=0;
  
  LCD_Data4 &= 0b00000000; //clear
  LCD_E=0; 
  LCD_RS=0; 
  __delay_ms(100); // power up delay
  
  LCD_Data4=((LCD_Data4&0xf0)|((0x30>>4)&0x0f)); //reset
  lcd_clk(); 
  __delay_ms(1); 
  LCD_Data4=((LCD_Data4&0xf0)|((0x30>>4)&0x0f)); //reset
  lcd_clk(); 
  __delay_ms(1); 
  LCD_Data4=((LCD_Data4&0xf0)|((0x30>>4)&0x0f)); //reset
  lcd_clk(); 
  __delay_ms(1); 
/*---------------------------------*/ 
  LCD_Data4=((LCD_Data4&0xf0)|((0x20>>4)&0x0f));   //4bit mode
  lcd_clk(); 
  __delay_ms(1); 
  
  lcd_command(0x28); 
  lcd_command(0x01);   
  lcd_command(0x06);  
  lcd_command(0x0C);  
  lcd_command(0x02);   
  lcd_command(0x01);   
} 
void ADC_init()
{    
    //TRISBbits.TRISB3 = 1; //analog inputs
    ADCON1bits.FORM=0; //Data Output Format bits-   integer (DOUT = 0000 00dd dddd dddd)
    ADCON1bits.SSRC=0b111; //Internal counter ends sampling and starts conversion (auto convert)
    ADCON1bits.ASAM=1; // Sampling begins immediately after last conversion completes. SAMP bit is auto set   
    //ADCON1bits.SAMP=1; //At least one A/D sample/hold amplifier is sampling
        
    ADCON2bits.VCFG = 0b000;// Voltage Reference Configuration bits (AVdd, AVss)
    ADCON2bits.CSCNA=1;//Scan Input Selections for CH0+ S/H Input for MUX A Input Multiplexer Setting bit
    ADCON2bits.SMPI=0;// Interrupts at the completion of conversion for each sample/convert sequence
    ADCON2bits.BUFM=0; //Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.ALTS=0; //Always use MUX A input multiplexer settings
    
    ADCON3bits.SAMC=31; // =0b0001 1111 Auto Sample Time bits = 31 TAD
    ADCON3bits.ADRC=0; // Clock derived from system clock //A/D internal RC clock
    ADCON3bits.ADCS=5; //A/D Conversion Clock Select bits 111111 = TCY/2 • (ADCS<5:0> + 1) = 32 • TCY

   // ADPCFG = 0xFFFF; 
    ADPCFGbits.PCFG3 = 0;   // 0- Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage; 
                        //1- Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
    ADCHSbits.CH0SA = ADC_MASK ; //ADCHS: A/D Input Select Register  //Channel 0 positive input is AN3
    
    ADCSSLbits.CSSL3 = 1;//?? 0 = Skip ANx for input scan
    
    IFS0bits.ADIF=0; // clear interrupt flug
    //IEC0bits.ADIE=1; // enable adc done convert interrupt
    ADCON1bits.ADON=1; // turn on the adc module
}
unsigned long ADC_read(int channel)
{
    unsigned long ADC_val;
    ADCSSL = channel;
    while (!ADCON1bits.DONE)
    ADC_val = ADCBUF0;
    return (ADC_val);
}
void __attribute__((__interrupt__)) _ADCInterrupt(void)
{ 
    IFS0bits.ADIF = 0;
   // WriteTwoByte( ADCBUF0 );        
}
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
void WriteUART (int data)   //? unsigned char - 8 bits, or unsigned int = 16 bits, size of transmit buffer?
{
    U1TXREG = data; // 8 bit mode 
    while(!U1STAbits.TRMT){}
   // LATBbits.LATB0=!LATBbits.LATB0; // led
}
void WriteTwoByte( int par)  // find low and high bytes in 16bits variable and send it 
{
    char al=0; // 
    al=( char)((par));//find low byte
    WriteUART(al);  // send low byte !
    al=( char)((par >> 8)); // find high byte 
    WriteUART(al);   //send high byte
}
int main(void) 
{
    //unsigned long res;
    //TRIS ?
    
    uart_init();
    lcd_init();
    ADC_init();
    lcd_puts(128,"r=");
    while (1)
        
    {
       
//      res = ADC_read(3); 
     while (!ADCON1bits.DONE);
     inttolcd( 135, ADCBUF0 );
     WriteTwoByte( ADCBUF0 );
      __delay_ms(300);
//      
    }
    return 0;
}
