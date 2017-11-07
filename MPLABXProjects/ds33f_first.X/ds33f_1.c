/*
 * File:   ds33f_1.c
 * Author: PC
 *
 * Created on February 13, 2017, 12:53 PM
 */


// DSPIC33FJ256MC710 Configuration Bit Settings

// 'C' source line config statements
#define FCY 16000000UL
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure Segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Mode (Primary Oscillator (XT, HS, EC))
#pragma config IESO = ON                // Two-speed Oscillator Start-Up Enable (Start up with FRC, then switch)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config LPOL = ON                // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD3               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = ON              // JTAG Port Enable (JTAG is Enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "p33FJ256MC710.h"
#include <libpic30.h>

#define UART_BAUD  115200  //125000
#define UBRG_VALUE  (FCY/UART_BAUD/16) - 1  // UBRG_VALUE=47

#define ADC_MASK  3; // choose adc channel
#define BUFF  300

int NUM = BUFF, ms=0, flag_start = 0,uart_flag=0;
unsigned int pos_cnt=0,RX_byte = 0; // save received byte and transmitt byte //32 bits variable
int flag = 0,p_read = 0, p_write = 0, counter = 0, count_buff = 0,flag_w=0,flag_r=0;//, counter2 = 0;

unsigned int IC1_arr[BUFF]; // input capture array
//int IC2_arr[NUM]; // input capture array
unsigned int  t1=0,t2=0;//,period = 0,;//,t3 = 0,t4=0;
void uart_init(void) {
    TRISF=0xffff;
    TRISFbits.TRISF3 = 0;    
    U1MODEbits.BRGH = 0;    
    U1BRG = UBRG_VALUE; // baud rate 115200
    IEC0bits.U1RXIE = 1; //enable RX1 interrupt
    //IEC0bits.U1TXIE=1; //enable TX1 interrupt
    U1STAbits.URXISEL1 = 0; //If URXISEL<1:0> = 11, an interrupt is generated when a word is transferred from the
    U1STAbits.URXISEL0 = 0; //Receive Shift register (UxRSR) to the receive buffer and as a result, the receive buffer
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
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) // Receive data interrupt (interrupt occur after 4 bytes have received)
{
      int k = 0; 
       
    if(uart_flag == 0)
    {
       uart_flag++; 
       RX_byte = U1RXREG;
    }
    else 
    {
   
        RX_byte = (RX_byte<<8)|U1RXREG;     
//        for (k =0; k<100; k++)
//         {
//            WriteTwoByte( RX_byte );
//            RX_byte ++;
//         }         
//        Nop();
 //       Nop();
        uart_flag = 0;
        IEC1bits.CNIE = 1; // Enable CN interrupts
        NUM = RX_byte;
        pos_cnt = 0;
    
        _LATE4 = 0; 
        _LATE5 = 1; 
     
    }
        
    IFS0bits.U1RXIF = 0; // clear interrupt flag    
}
//void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) // Receive data interrupt (interrupt occur after 4 bytes have received)
//{
//    int k = 0; //, Num_of_repeats=10, Num_of_steps=600, PDC3_val=100, EXP_DELAY = 50, step= 1; // parameters of experiment
//    RX_byte = 0; // clear receiver buffer;
//    for (k = 4; k >= 1; k--)//  
//    {
//        RX_byte |= U1RXREG << 8 * (k - 1); // every time  U1RXREG operation called,in U1RXREG write one byte extracted from memory
//    }  
//    _LATG13 = !_LATG13;
////    Init_Timer3();
////    InputCaptureInit1();
////    InputCaptureInit2();
//    
////    if (flag_start == 0)
////    {
//////        IEC1bits.CNIE = 1; // Enable CN interrupts
//////        _LATE4 = 0; //switch off motor
//////        _LATE5 = 1; //
//////        NUM = RX_byte;
//////        //IEC0bits.T1IE = 1;
//////        counter = 0;
////        
////    }
////    else
////    {
////        _LATE4 = 0; //switch off motor
////        _LATE5 = 0; //
////        IEC1bits.CNIE = 0; // Enable CN interrupts
////    }
//   // flag_start++;
////    IEC0bits.IC1IE = 1; // capture channel interrupt enable 
////    IEC0bits.IC2IE = 1; // capture channel interrupt enable 
////  
////    NUM = RX_byte;
////    IEC0bits.T1IE = 1;
////    counter = 0;
////    ms = 0;
//    IFS0bits.U1RXIF = 0; // clear interrupt flag    
//}

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
void interrupt_init() // not used (Index pulse)
{
    TRISEbits.TRISE8 = 1; // pin mode input (INT0)
    //TRISDbits.TRISD0 = 1; // pin mode input (INT1)
    INTCON2bits.INT0EP = 0; // Interrupt INT0 on positive edge
    //INTCON2bits.INT1EP = 0; // Interrupt INT1 on positive edge    
    IFS0bits.INT0IF = 0; /*Reset INT0 interrupt flag */
    //IFS1bits.INT1IF = 0; /*Reset INT1 interrupt flag */
    IEC0bits.INT0IE = 1;    /*Enable INT0 Interrupt Service Routine */ 
    //IEC1bits.INT1IE = 1;    /*Enable INT0 Interrupt Service Routine */  
}
void __attribute__((interrupt, auto_psv)) _INT0Interrupt(void) {

    IFS0bits.INT0IF = 0; // clear ADC interrupt flag
}
void __attribute__((interrupt, auto_psv)) _INT1Interrupt(void) {
 
    IFS1bits.INT1IF = 0; // clear ADC interrupt flag
}
void ADC_init() {
    TRISBbits.TRISB3 = 1; //analog inputs    
    ADPCFGbits.PCFG3 = 0; // 0- Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage; 
    //1- Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
    AD1CHS0bits.CH0SA = ADC_MASK; //ADCHS: A/D Input Select Register  //Channel 0 positive input is AN3
    AD1CON2bits.VCFG = 0b000; // Voltage Reference Configuration bits (Vref+,Vref-)// (AVdd, AVss)
    AD1CON3bits.ADCS = 2; //A/D Conversion Clock Select bits 111111 = TCY/2 • (ADCS<5:0> + 1) = 32 • TCY
    AD1CON1bits.AD12B = 1; //12 bit operation
    AD1CON1bits.FORM = 0; //Data Output Format bits-   integer (DOUT = 0000 00dd dddd dddd)
    AD1CON1bits.SSRC = 0b111; //Internal counter ends sampling and starts conversion (auto convert)
    AD1CON3bits.SAMC = 0b11111; //Auto Sample Time bits
    AD1CON1bits.ASAM = 1;
    AD1CON1bits.ADON = 1;// enable adc module
}
void InitQEI(void) {
    //ADPCFG |= 0b00110000; // Configure QEI pins as digital inputs
    TRISB |= 0b00110000;// RB4,RB5-inputs
    POSCNT = 0; // Reset position counter
    QEICONbits.QEIM = 0; // Disable QEI Module
    QEICONbits.CNTERR = 0; // Clear any count errors
    QEICONbits.SWPAB = 1; // QEA and QEB not swapped      ???????
    QEICONbits.PCDOUT = 1; // QEI logic controls state of I/O pin
    QEICONbits.POSRES = 0; // !Index Pulse does not reset Position Counter (Bit only applies when QEIM<2:0> = 100 or 110)
    DFLTCONbits.CEID = 1; // Count error interrupts disabled
    QEICONbits.QEIM2 = 1; //!Quadrature Encoder Interface enabled (x4 mode) with MAXCNT reset of position counter
    QEICONbits.QEIM1 = 1;
    QEICONbits.QEIM0 = 1;
}
void PWM_init() {
    TRISEbits.TRISE5 = 0; // PWM3H output enable
    TRISEbits.TRISE4 = 0; // PWM3L output enable
    PTCONbits.PTOPS = 0; // PWM timer post-scale
    PWMCON1bits.PMOD3 = 1; //PWM I/O pin pair is in the Independent PWM Output mode
    PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output
    PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output
    PTCONbits.PTMOD = 0b00; // PWM time base operates in a free running mode 
    PTCONbits.PTCKPS = 0b00; // prescale=1:1 (0=1:1, 1=1:4, 2=1:16, 3=1:64) //11 = PWM time base input clock period is 64 TCY (1:64 prescale)
    PTPER = 15000; // 1 kHz PWM period (15-bit period value)  // PTPER =  Fcy/(Fpwm*PTMRprescaler) -1
    PTMR = 0; // Clear 15-bit PWM timer counter
    PTCONbits.PTEN = 1; // Enable PWM time base
} 
void Init_Timer1(void) {
    T1CON = 0;
    TMR1 = 0;
    IFS0bits.T1IF = 0;
    IPC0bits.T1IP = 4;
    T1CONbits.TCKPS = 0b10; // 11 = 1:256 prescale value
    PR1 = 625; //T = 480ms
    //IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}
void WriteCycleBuff( int data)
{
      if (counter <= NUM)
    {        
        if ( p_write <= BUFF )
        {
            IC1_arr [p_write] = data;
            p_write++;
            counter++;
        }
        else
        {
            p_write = 0;
            IC1_arr [p_write] = data;
            flag_w++;
        }    
    }
}
void ReadCycleBuff(void)
{
    if (flag_w == flag_r)
         {
             if (p_write > p_read)
             {
                 WriteTwoByte(IC1_arr [p_read]);
                 p_read++;
             }
             else 
             {
                 // _LATG13 =! _LATG13; 
             }
         }
         if (flag_w - flag_r == 1)
         {
             if (p_read > p_write)
             {
                 WriteTwoByte(IC1_arr [p_read]);
                 p_read++;
             }
         }
         else 
         {
             _LATG13 =! _LATG13;  // error
         }
        if (p_read > BUFF)
        {
            p_read = 0;
            flag_r ++;
        }
    
    if ( counter >= NUM )
    {
//        IEC0bits.T1IE = 0;  // timer 1 interrupt disable
//        IEC0bits.IC1IE = 0; // input capture channel interrupt disable 
//        IEC0bits.IC2IE = 0; // input capture channel interrupt disable 
       // counter = 0;
        _LATE4 = 0; //switch off motor
        _LATE5 = 0; //
        IEC1bits.CNIE = 0; // Enable CN interrupts
        counter = 0;
    }
}
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    ms++;
    WriteCycleBuff( ms );
    IFS0bits.T1IF = 0; // clear an interrupt flag
}
//void __attribute__((interrupt, auto_psv)) _T1Interrupt( void )
//{    
//    if ( counter < 900 )
//    {
//        pos_array[counter] = POSCNT; 
//        counter++;
//    }
//    else
//    {
//        if ( flag == 0)
//        {
//            for ( counter = 0; counter < 900; counter++)
//            {
//                WriteTwoByte ( pos_array[counter] );
//            } 
//            LATBbits.LATB0=1; //switch off motor
//            LATBbits.LATB1=1; //
//            flag = 1;
//            LATDbits.LATD1 = 1;//indicator
//           // inttolcd( 133,pos_array[899] );
//          //  inttolcd( 200,flag );           
//        }
//    }   
//    IFS0bits.T1IF = 0; // clear an interrupt flag
//} 

void Init_Timer3(void) {
    T3CON = 0;
    TMR3 = 0;
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP=3;
    T3CONbits.TCKPS = 0b10; // 00 = 1:64 prescale value
    PR3 = 3000; //T = 12 ms
   // IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}
void __attribute__((interrupt, auto_psv)) _T3Interrupt(void) {
    
  //  LATDbits.LATD1 =! LATDbits.LATD1;
    IFS0bits.T3IF = 0; // clear an interrupt flag
}

void InputCaptureInit1(void) {
    _TRISD8 = 1;
    IC1CONbits.ICM = 0b000; // turn off 
    IC1CONbits.ICTMR = 0; //TMR3 contents are captured on capture event
    IC1CONbits.ICI = 0b00; // Interrupt on every  capture event
    IPC0bits.IC1IP = 1;
    IFS0bits.IC1IF = 0; // clear an interrupt flag
    IEC0bits.IC1IE = 1; // capture channel interrupt enable    
    IC1CONbits.ICM = 0b001; //001 = Capture mode, every edge (rising and falling)  (ICI<1:0> does not control interrupt generation for this mode.)
                            //011 = Capture mode, every rising edge
}
void __attribute__((interrupt, auto_psv)) _IC1Interrupt(void) {
    t1 = IC1BUF;
    IFS0bits.IC1IF = 0;
}
void InputCaptureInit2(void) {
    _TRISD9 = 1;
    IC2CONbits.ICM = 0b000; // turn off 
    IC2CONbits.ICTMR = 0; //TMR3 contents are captured on capture event
    IC2CONbits.ICI = 0b00; // Interrupt on every  capture event
    IPC1bits.IC2IP = 1;
    IFS0bits.IC2IF = 0; // clear an interrupt flag
    IEC0bits.IC2IE = 1; // capture channel interrupt enable    
    IC2CONbits.ICM = 0b001; //001 = Capture mode, every edge (rising and falling)  (ICI<1:0> does not control interrupt generation for this mode.)
                            //011 = Capture mode, every rising edge
}
void __attribute__((interrupt, auto_psv)) _IC2Interrupt(void) {
    t2 = IC2BUF;  
    IFS0bits.IC2IF = 0;
}
void ConfigureCN(void)
{
    _TRISD6 = 0; // OC7//CN15
    _TRISD5 = 0; //OC6//CN14
    CNEN1bits.CN14IE = 1; // Enable CN3 pin for interrupt detection
    CNEN1bits.CN15IE = 1; // Enable CN3 pin for interrupt detection
    //IEC1bits.CNIE = 1; // Enable CN interrupts
    IFS1bits.CNIF = 0; // Reset CN interrupt
}
void __attribute__ ((interrupt, auto_psv)) _CNInterrupt(void)
{
   pos_cnt++;
   
   while (!AD1CON1bits.DONE);
   WriteCycleBuff(ADC1BUF0);
   WriteCycleBuff(pos_cnt);
   IFS1bits.CNIF = 0; // Reset CN interrupt
}
int main(void) {
    // sysInit();
     ADPCFG = 0xFFFF; // all AN as digital input?
     uart_init();
     ADC_init();
//     Init_Timer1();
//     Init_Timer3();
//     InputCaptureInit1();
//     InputCaptureInit2();
    // InitQEI();
     ConfigureCN();
     //PWM_init();
    _TRISE5 = 0; // PWM3H output enable
    _TRISE4 = 0; // PWM3L output enable
    _TRISG13 = 0; 
//    _LATE4 = 0; //switch off motor
//    _LATE5 = 1; //
    while (1)
    {
         ReadCycleBuff();
//       while (!AD1CON1bits.DONE);
//       WriteTwoByte( ADC1BUF0 );
// // WriteTwoByte( 0xAAAA );
//      __delay_ms(300);
////    
    }
    return 0;
}
