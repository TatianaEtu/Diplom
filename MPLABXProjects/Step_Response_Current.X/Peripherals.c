/*
 * File:   Reripherals.c
 * Author: PC
 *
 * Created on April 19, 2017, 10:34 AM
 */


#include "xc.h"
#include "p33FJ256MC710.h"
#include <libpic30.h>
#include "define_heder.h"

//#define TMR1_PERIOD 2500 //T = 0.01
//#define TMR1_PRESCALER 0b10 // (0b00 = 1; 0b01 = 8; 0b10 = 64; 0b11 = 256)
//
//#define TMR3_PERIOD 60000
//#define TMR3_PRESCALER 0b11 // (0b00 = 1; 0b01 = 8; 0b10 = 64; 0b11 = 256)
//
//#define PWM_PERIOD 15999 //1kHz // PTPER =  Fcy/(Fpwm*PTMRprescaler)
//#define SET_PWM_DUTY(duty)  PDC3 = duty 


void interrupt_init() // not used (Index pulse)
{
    TRISFbits.TRISF6 = 1; // pin mode input (INT0)
    TRISDbits.TRISD0 = 1; // pin mode input (INT1)
    INTCON2bits.INT0EP = 0; // Interrupt INT0 on positive edge
    INTCON2bits.INT1EP = 0; // Interrupt INT1 on positive edge    
    IFS0bits.INT0IF = 0; /*Reset INT0 interrupt flag */
    IFS1bits.INT1IF = 0; /*Reset INT1 interrupt flag */
    IEC0bits.INT0IE = 1;    /*Enable INT0 Interrupt Service Routine */ 
    IEC1bits.INT1IE = 1;    /*Enable INT0 Interrupt Service Routine */  
}

void ADC_init() {
    TRISBbits.TRISB3 = 1; //analog inputs AN3
    TRISBbits.TRISB2 = 1; //analog inputs AN2
    ADPCFGbits.PCFG3 = 0; // 0- Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage; 
    //1- Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
    ADPCFGbits.PCFG2 = 0;
//    AD1CHS0bits.CH0SA = ADC_MASK; //ADCHS: A/D Input Select Register  //Channel 0 positive input is AN3
    AD1CON2bits.VCFG = 0b000; // Voltage Reference Configuration bits (Vref+,Vref-)// (AVdd, AVss)
    AD1CON3bits.ADCS = 10; //A/D Conversion Clock Select bits 111111 = TCY/2 • (ADCS<5:0> + 1) = 32 • TCY
    AD1CON1bits.AD12B = 1; //12 bit operation
    AD1CON1bits.FORM = 0; //Data Output Format bits-   integer (DOUT = 0000 00dd dddd dddd)
    AD1CON1bits.SSRC = 0b111; //Internal counter ends sampling and starts conversion (auto convert)
    AD1CON3bits.SAMC = 0b11111; //Auto Sample Time bits
    AD1CON1bits.ASAM = 0;//0 = Sampling begins when SAMP bit is set
    AD1CON1bits.ADON = 1;// enable adc module
}

int ReadAdc(int channel)
{
   AD1CHS0 = channel;
   AD1CON1bits.SAMP = 1; //ADC S&H amplifiers are sampling
   while (!AD1CON1bits.DONE);
   return ADC1BUF0;
}

void InitQEI(void) {
  //  ADPCFG |= 0b00110000; // Configure QEI pins as digital inputs
    TRISB |= 0b00110000;// RB4,RB5-inputs
    POSCNT = 7; // Reset position counter
    MAXCNT = 60000;    
    //QEI1CONbits.QEIM = 0; // Disable QEI Module
    QEICONbits.CNTERR = 0; // Clear any count errors
    QEICONbits.SWPAB = 1; // QEA and QEB not swapped      ???????
    QEICONbits.PCDOUT = 0; // QEI logic controls state of I/O pin
    QEICONbits.POSRES = 0; // !Index Pulse does not reset Position Counter (Bit only applies when QEIM<2:0> = 100 or 110)
    DFLTCONbits.CEID = 1; // Interrupts due to count errors are disabled
    QEICONbits.QEIM2 = 1; //!Quadrature Encoder Interface enabled (x4 mode) with MAXCNT reset of position counter
    QEICONbits.QEIM1 = 1;
    QEICONbits.QEIM0 = 1;
}

void PWM_init() {
    TRISEbits.TRISE5 = 0; // PWM3H output enable
    TRISEbits.TRISE4 = 0; // PWM3L output enable
    PTCONbits.PTOPS = 0; // PWM timer post-scale
    PWMCON1bits.PMOD3 = 1; //PWM I/O pin pair is in the Independent PWM Output mode
//    PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output
//    PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output
    PTCONbits.PTMOD = 0b00; // PWM time base operates in a free running mode 
    PTCONbits.PTCKPS = 0b00; // prescale=1:1 (0=1:1, 1=1:4, 2=1:16, 3=1:64) //11 = PWM time base input clock period is 64 TCY (1:64 prescale)
    PTPER = PWM_PERIOD; // 1 kHz PWM period (15-bit period value)  // PTPER =  Fcy/(Fpwm*PTMRprescaler) -1
    PTMR = 0; // Clear 15-bit PWM timer counter
    PTCONbits.PTEN = 1; // Enable PWM time base
} 

void PWM_run(int direction) // 0 - low channel, 1-high channel, 2 - both channel
{
    switch(direction)
    {
        case 0:
        {
            PWMCON1bits.PEN3H = 0; // PWM3H pin is enabled for PWM output
            PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output 
            break;
        }
        case 1:
        {
            PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output
            PWMCON1bits.PEN3L = 0; // PWM3L pin is enabled for PWM output 
            break;
        }
        case 2:
        {
            PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output
            PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output  
            break;
        }
        default:
        {
            PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output
            PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output
            break;
        }
    }
 }

void Set_DutyCycleValue(int duty)
{
    SET_PWM_DUTY(duty); 
}

void Init_Timer1(void) {
    T1CON = 0; // disable timer1
    TMR1 = 0; // TMR1 counter register = 0
    IFS0bits.T1IF = 0; // clear interrupt flag
    IPC0bits.T1IP = 4; // interrupt preority = 4
    T1CONbits.TCKPS = TMR1_PRESCALER; // 11 = 1:256 prescale value (1,8,64,256))
    PR1 = TMR1_PERIOD; // TMR1 period register 2^16       /
    //IEC0bits.T1IE = 1; // TMR1 interrupt enable
    T1CONbits.TON = 1; // enable timer1
}

void Init_Timer3(void) {
    T3CON = 0;
    TMR3 = 0;
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP=7;
    T3CONbits.TCKPS = TMR3_PRESCALER; // 00 = 1:8 prescale value (1,8,64,256))
    PR3 = TMR3_PERIOD; //T = 12 ms
   // IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}

void InputCaptureInit1(void) {
    _TRISD8 = 1;
    IC1CONbits.ICM = 0b000; // turn off 
    IC1CONbits.ICTMR = 0; //TMR3 contents are captured on capture event
    IC1CONbits.ICI = 0b00; // Interrupt on every  capture event
    IPC0bits.IC1IP = 1;
    IFS0bits.IC1IF = 0; // clear an interrupt flag
    //IEC0bits.IC1IE = 1; // capture channel interrupt enable    
    IC1CONbits.ICM = 0b011; //001 = Capture mode, every edge (rising and falling)  (ICI<1:0> does not control interrupt generation for this mode.)
                            //011 = Capture mode, every rising edge
}

void InputCaptureInit2(void) {
    _TRISD9 = 1;
    IC2CONbits.ICM = 0b000; // turn off 
    IC2CONbits.ICTMR = 0; //TMR3 contents are captured on capture event
    IC2CONbits.ICI = 0b00; // Interrupt on every  capture event
    IPC1bits.IC2IP = 1;
    IFS0bits.IC2IF = 0; // clear an interrupt flag
   // IEC0bits.IC2IE = 1; // capture channel interrupt enable    
    IC2CONbits.ICM = 0b011; //001 = Capture mode, every edge (rising and falling)  (ICI<1:0> does not control interrupt generation for this mode.)
                            //011 = Capture mode, every rising edge
}

void ConfigureCN(void)
{
    _TRISD6 = 1; // OC7//CN15
    _TRISD5 = 1; //OC6//CN14
    CNEN1bits.CN14IE = 1; // Enable CN3 pin for interrupt detection
    CNEN1bits.CN15IE = 1; // Enable CN3 pin for interrupt detection
    IEC1bits.CNIE = 1; // Enable CN interrupts
    IFS1bits.CNIF = 0; // Reset CN interrupt
}

