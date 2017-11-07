/*
 * File:   exp2.c
 * Author: PC
 *
 * Created on December 29, 2016, 2:01 PM
 */
#define FCY 7372800UL
// DSPIC30F2010 Configuration Bit Settings
// 'C' source line config statements

// FOSC
#pragma config FOSFPR = XT_PLL4              // Oscillator (XT)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
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
#include "p30F3010.h"
#include <stdio.h>
#include <stdlib.h>

#define UART_BAUD 9600
#define UBRG_VALUE (FCY/UART_BAUD/16) - 1   // UBRG_VALUE=47

#define lcd_clear() lcd_command(1) 
#define lcd_origin() lcd_command(2) 
#define E_pulse_with 100 
#define LCD_E LATCbits.LATC14
#define LCD_RS LATCbits.LATC13 
#define LCD_Data4 LATE 

//#define EXP_DELAY 300
#define ADC_MASK 3 // choose adc channel
#define chA PORTDbits.RD0 // INT 1 interrupt, channel A of encoder
#define chB PORTEbits.RE8 // INT 0 interrpt, channel B of encoder

// global veriables
int i=0,j=0;
unsigned long  RX_byte=0; // save received byte and transmitt byte //32 bits variable
unsigned int TX_byte=0; //16bits variable
int pos_counter = 0;// variable for enpoder position counter (interrupt mode, not QEI)
//int flag = 0;
int ms = 0; // TMR1 variables

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
 while(*p)                 
 { 
  lcd_putc(*p);             
  p++;                      
 } 
} 
void inttolcd(unsigned char posi, signed int value) 
{ 
 char buff[16]; 
 sprintf(buff,"%4d",value); 
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
void InitQEI(void)
{
ADPCFG |= 0b00111000; // Configure QEI pins as digital inputs
TRISB  |= 0b00111000;
//MAXCNT = 4000; //!not use Index pulse
POSCNT = 0; // Reset position counter
QEICONbits.QEIM = 0; // Disable QEI Module
QEICONbits.CNTERR = 0; // Clear any count errors
//QEICONbits.QEISIDL = 0; // Continue operation during sleep
QEICONbits.SWPAB = 1; // QEA and QEB not swapped      ???????
QEICONbits.PCDOUT = 1; // QEI logic controls state of I/O pin
QEICONbits.POSRES = 0; // !Index Pulse does not reset Position Counter (Bit only applies when QEIM<2:0> = 100 or 110)
//QEICONbits.TQGATE = 1;
DFLTCONbits.CEID = 1; // Count error interrupts disabled
//QEICONbits.QEOUT = 1; // Digital filters output enabled for QEn pins
//QEICONbits.QECK = 5; // 1:64 clock divide for digital filter for QEn
//QEICONbits.INDOUT = 1; // Digital filter output enabled for Index pin
//QEICONbits.INDCK = 5; // 1:64 clock divide for digital filter for Index
    QEICONbits.QEIM2 = 1;//!Quadrature Encoder Interface enabled (x4 mode) with MAXCNT reset of position counter
    QEICONbits.QEIM1 = 1;
    QEICONbits.QEIM0 = 1;
//QEICONbits.QEIM2 = 1;//!Quadrature Encoder Interface enabled (x2 mode) with position counter reset by match (MAXCNT)
//QEICONbits.QEIM0 = 1;// !Quadrature Encoder Interface enabled (x2 mode) with position counter reset by match (MAXCNT))
}//
void PWM_init()
{
   TRISEbits.TRISE5=0; // PWM3H output enable
   PTCONbits.PTOPS  = 0;  // PWM timer post-scale
   PWMCON1bits.PMOD3=0; // dosn't care (endependent, if 3L - pwm)
   PWMCON1bits.PEN3H=1;// PWM3H pin is enabled for PWM output
   PWMCON1bits.PEN3L=1;// PWM3H pin is enabled for PWM output
   PTCONbits.PTMOD  = 0b00;  // PWM operates in free-running mode continuously  
   PTCONbits.PTCKPS = 0b00; // prescale=1:64 (0=1:1, 1=1:4, 2=1:16, 3=1:64) //11 = PWM time base input clock period is 64 TCY (1:64 prescale)
   PTPER = 7360;         // 20ms PWM period (15-bit period value)
   PTMR = 0;             // Clear 15-bit PWM timer counter
   PTCONbits.PTEN = 1;   // Enable PWM time base
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
// јргумент этой ф-ции можно изменить на unsigned char!!!!
void WriteUART (int data)   //? unsigned char - 8 bits, or unsigned int = 16 bits, size of transmit buffer?
{
    U1TXREG = data; // 8 bit mode 
    while(!U1STAbits.TRMT){}
}
void WriteStringUART(unsigned char * s)
{
    while(*s)
    {
       WriteUART(*s++);
    }
}
void WriteTwoByte( int par)  // find low and high bytes in 16bits variable and send it 
{
    char h=0,l=0; //     
    h=( char)((par >> 8)); // find high byte 
    WriteUART(h);   //send high byte
    l=( char)((par));//find low byte
    WriteUART(l);  // send low byte !
}


void __attribute__((__interrupt__)) _U1RXInterrupt(void) // Receive data interrupt (interrupt occur after 4 bytes have received)
{
    IFS0bits.U1RXIF = 0; 
    int k=0, Num_of_repeats=10, Num_of_steps=600, PDC3_val=100, EXP_DELAY = 50, step= 1; // parameters of experiment
    RX_byte=0; // clear receiver buffer;
    for( k = 4; k >= 1; k-- )// 
    {
        RX_byte |= U1RXREG << 8*(k-1);       // every time  U1RXREG operation called,in U1RXREG write one byte extracted from memory
    } 
    Init_Timer1(); // start send data

 //  StartExp( PDC3_val, EXP_DELAY, Num_of_repeats, Num_of_steps, step );       
}
void interrupt_init() // not used (Index pulse)
{ 
    INTCON2bits.INT0EP=0;// Interrupt INT0 on positive edge
    INTCON2bits.INT1EP=0;// Interrupt INT1 on positive edge
    TRISEbits.TRISE8=1; // pin mode input (INT0)
    TRISDbits.TRISD0=1; // pin mode input (INT1)
    IFS0bits.INT0IF = 0;    /*Reset INT0 interrupt flag */
    IFS1bits.INT1IF = 0;    /*Reset INT1 interrupt flag */
    IEC0bits.INT0IE = 1;    /*Enable INT0 Interrupt Service Routine */ 
    IEC1bits.INT1IE = 1;    /*Enable INT0 Interrupt Service Routine */ 
}
void ADC_init()
{    
    //TRISBbits.TRISB3 = 1; //analog inputs
    ADCON1bits.FORM=0; //Data Output Format bits-   integer (DOUT = 0000 00dd dddd dddd)
    ADCON1bits.SSRC=0b111; //Internal counter ends sampling and starts conversion (auto convert)
    ADCON1bits.ASAM=1; // Sampling begins immediately after last conversion completes. SAMP bit is auto set   
    //ADCON1bits.SAMP=1; //At least one A/D sample/hold amplifier is sampling
        
    ADCON2bits.VCFG = 0b000;// Voltage Reference Configuration bits (Vref+,Vref-)// (AVdd, AVss)
    ADCON2bits.CSCNA=1;//Scan Input Selections for CH0+ S/H Input for MUX A Input Multiplexer Setting bit
    ADCON2bits.SMPI=0;// Interrupts at the completion of conversion for each sample/convert sequence
    ADCON2bits.BUFM=0; //Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.ALTS=0; //Always use MUX A input multiplexer settings
    
    ADCON3bits.SAMC=31; // =0b0001 1111 Auto Sample Time bits = 31 TAD
    ADCON3bits.ADRC=0; // Clock derived from system clock //A/D internal RC clock
    ADCON3bits.ADCS=2; //A/D Conversion Clock Select bits 111111 = TCY/2 Х (ADCS<5:0> + 1) = 32 Х TCY

   // ADPCFG = 0xFFFF; 
    ADPCFGbits.PCFG3 = 0;   // 0- Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage; 
                        //1- Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
    ADCHSbits.CH0SA = ADC_MASK ; //ADCHS: A/D Input Select Register  //Channel 0 positive input is AN3
    
    ADCSSLbits.CSSL3 = 1;//?? 0 = Skip ANx for input scan
    
    IFS0bits.ADIF=0; // clear interrupt flug
    //IEC0bits.ADIE=1; // enable adc done convert interrupt
    ADCON1bits.ADON=1; // turn on the adc module
}
void Init_Timer1( void )
{
	/* declare temp variable for CPU IPL storing */
	//int current_cpu_ipl;	
	/* ensure Timer 1 is in reset state */
	T1CON = 0;
	/* reset Timer 1 interrupt flag */
 	IFS0bits.T1IF = 0; 	
 	/* set Timer1 interrupt priority level to 4 */
	IPC0bits.T1IP = 4;
	/* enable Timer 1 interrupt */
 	IEC0bits.T1IE = 1; 	 
    /*Timer Input Clock Prescale Select bits = 1; */
    T1CONbits.TCKPS = 0b11; // 11 = 1:256 prescale value	
	/* set Timer 1 period register */
	PR1 = 300; //  1 / ( 7372800 / 256 ) * 300 = 10.4 ms (every 10.4 ms interrupt request is occur)
    /* enable Timer 1 and start the count */ 
    T1CONbits.TON = 1;
 	/* select external timer clock */
	//T1CONbits.TCS = 1;
	/* disable interrupts for unlock sequence below */
	//SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);		
	/* restore CPU IPL value after executing unlock sequence */
	//RESTORE_CPU_IPL(current_cpu_ipl);
}
void __attribute__((__interrupt__)) _T1Interrupt( void )
{
    
    while (!ADCON1bits.DONE);    
    WriteTwoByte( ADCBUF0 & 0b0000001111111111 );
    
 //   WriteTwoByte( ms );  // попробовать потом pos_counter
    
	ms++;
    if (ms >= 100)
    {
        ms = 0;
        LATDbits.LATD1=!LATDbits.LATD1;  // 2 times per second led is turn on
    }    
    IFS0bits.T1IF = 0; // clear an interrupt flag
}     
void StartExp( int PDC3_val, int EXP_DELAY, int Num_of_repeats, int Num_of_steps, int step )
{   
          for (  i = 0; i < Num_of_repeats; i++ )
          {                      
                PDC3 = PDC3_val; // start value PWM duty cycle
                __delay_ms (300);
                POSCNT = 0; // clear the encoder position counter
                __delay_ms(500);
                inttolcd( 128 , i ); // I can see, the number of cycle
            
                for ( j = 0; j < Num_of_steps; j++ )
                {
                    PDC3 = PDC3 + step;                              
                    __delay_ms( EXP_DELAY );
                    WriteTwoByte( POSCNT ); 
                    while (!ADCON1bits.DONE);
                    WriteTwoByte( ADCBUF0 );                 
                }            
                for ( j = Num_of_steps; j > 0; j-- )
                {
                    PDC3 = PDC3 - step;  // change PWM duty cycle
                    __delay_ms( EXP_DELAY );
                    WriteTwoByte( POSCNT ); 
                    while (!ADCON1bits.DONE);
                    WriteTwoByte( ADCBUF0 );
                }
          }
}
void __attribute__((__interrupt__)) _ADCInterrupt(void)
{ 
    IFS0bits.ADIF = 0;
   // WriteTwoByte( ADCBUF0 );        
}
void __attribute__ ((interrupt)) _INT0Interrupt(void) // chA interrupt 
{
    IFS0bits.INT0IF = 0;// clear the interrupt flag 
    //LATBbits.LATB0=!LATBbits.LATB0; // for debugging
    //INTCON2bits.INT0EP != INTCON2bits.INT0EP; // change interrupt INT0 detect edge
    if (( pos_counter >= 2000 )||( pos_counter <= -2000 ))
    {
        pos_counter = 0; 
    }
    if (chA == chB)
    {
        pos_counter ++;
    }
    else pos_counter --;
    
//    while (!ADCON1bits.DONE);
//    WriteTwoByte( ADCBUF0 );
//    WriteTwoByte( pos_counter );    
}
void __attribute__ ((interrupt)) _INT1Interrupt(void) // chB interrupt
{
    IFS1bits.INT1IF = 0;// clear the interrupt flag 
    //LATDbits.LATD1=!LATDbits.LATD1;         
    //INTCON2bits.INT1EP != INTCON2bits.INT1EP; // change interrupt INT1 detect edge
    if (( pos_counter >= 2000 )||( pos_counter <= -2000 ))
    {
        pos_counter = 0;
    }
    if (chA != chB)
    {
        pos_counter ++;
    }
    else 
    {
        pos_counter --;
    }
    
//    while (!ADCON1bits.DONE);
//    WriteTwoByte( ADCBUF0 );
//    WriteTwoByte( pos_counter ); 
}

int main(void) 
{        
    uart_init();   
    lcd_init(); // initialization LCD
    //interrupt_init();// initialization interrupt module
    InitQEI(); // initialization  Quadrature Encoder Interface
    PWM_init(); // initialization PWM in free running mode 
    ADC_init();   
   // Init_Timer1();
    TRISBbits.TRISB0=0; // output led 1
    TRISBbits.TRISB1=0; // output led 2
    PDC3 = 10000; 
   //PDC3 = 7360;
    LATBbits.LATB0=0;
    LATBbits.LATB1=0;
    while(1)
    {    
       
//        while (!ADCON1bits.DONE);
//        inttolcd( 133, ADCBUF0 );  // position counter to LCD  
//        WriteTwoByte( ADCBUF0 );
//        WriteTwoByte( POSCNT );        
//        __delay_ms (500);
    }
    return 0;
}
