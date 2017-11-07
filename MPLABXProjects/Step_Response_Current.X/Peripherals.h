
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

  
#ifndef PERIPHERALS
#define	PERIPHERALS

//#define FCY 16000000UL
#include <xc.h> // include processor files - each processor file is guarded. 

void interrupt_init(); // External interrupts initialization

void ADC_init();
int ReadAdc(int channel);

void InitQEI(void); //Quadrature encoder 

void PWM_init();
void PWM_run(int direction); // 0 - low channel, 1-high channel, 2 - both channels
void Set_DutyCycleValue(int duty);

void Init_Timer1(void);
void Init_Timer3(void);
void InputCaptureInit1(void);
void InputCaptureInit2(void);
void ConfigureCN(void); // Change Notification 


#endif	/* PERIPHERALS */

