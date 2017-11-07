
#ifndef DEFINE_HEADER
#define	DEFINE_HEADER

///***************
#define FCY 16000000UL 

#define UART_BAUD  57600
#define UBRG_VALUE  (FCY/UART_BAUD/16) - 1  // rounding to the low integer
#define BUFF  500
//****************************
//#include <xc.h> // include processor files - each processor file is guarded.  
#define TMR1_PERIOD 2500 //250UI //T = 0.01
//#define SAMPLE_TIME  TMR1_PERIOD/250U // TMR1_PERIOD =  250 -> 1 ms 
#define TMR1_PRESCALER 0b10 // (0b00 = 1; 0b01 = 8; 0b10 = 64; 0b11 = 256)

#define TMR3_PERIOD 60000
#define TMR3_PRESCALER 0b11 // (0b00 = 1; 0b01 = 8; 0b10 = 64; 0b11 = 256)

#define PWM_PERIOD 15999 //1kHz // PTPER =  Fcy/(Fpwm*PTMRprescaler)
#define SET_PWM_DUTY(duty)  PDC3 = duty 
//********************

#define SET_ALL_AN_AS_DIGITAL_INPUT ADPCFG = 0xFFFF
#define TMR1_INT_EN IEC0bits.T1IE 
//#define ALLOWED_ANGLE(angle_feedback)  ( angle_feedback >= -9000)&&(angle_feedback <= 9000)
#define chA PORTF && (1<<6)
#define chB PORTD && (1<<0)
//**********************************



#endif	/* DEFINE_HEADER*/

