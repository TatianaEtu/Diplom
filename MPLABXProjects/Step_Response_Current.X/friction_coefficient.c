/*
 * File:   friction_coefficient.c
 * Author: PC
 *
 * Created on March 10, 2017, 1:10 PM
 */

// DSPIC33FJ256MC710 Configuration Bit Settings

//#define FCY 16000000UL 

#include "SysInit_pragma.h"
#include <xc.h>
#include "p33FJ256MC710.h"
#include <libpic30.h>
#include "Potentiometr.h"
#include "Peripherals.h"
#include "UART.h"
//#include "Math.h"
#include "define_heder.h"

//#define RELEY 
#define N_diff_filter 5


int pos_cnt=0, secondVal = 0, RX_byte = 0, uart_flag = 0;
unsigned int  t1 = 0,t2 = 0,i = 0; // Input Capture interrupts variables
char ident=0; // command identifier 

//int pot_adc_val = 0; // potentiomter adc values
int tmr1_int_counter = 0; 
int angle_feedback = 0;//, umin = 0, umax = 2*PTPER;
int duty = 0;
float ANG;
//float kp = 8.97,ki = 1.34,kd = 0; // degrees
float set_point=0, set_point2 = 0; // u -set point, p,i,d - PID coefficients
float kp = 0, ki = 0, kd = 0; // degreesint set_point=0; // u -set point, p,i,d - PID coefficients
// u - controller output (0 to 100%)
float u=0, error = 0, d_error=0, Prev_error = 0, i_error=0, d_error_filtr = 0;



void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) // Receive data interrupt (interrupt occur after 4 bytes have received)
{           
    if(uart_flag == 0)
    {
       uart_flag++; 
       RX_byte = U1RXREG;
      // WriteUART(RX_byte);
    }
    else  // second byte
    {   
        secondVal = U1RXREG ;
        if ( (secondVal ) == 35) // 0x23 = #
        {
            ident = RX_byte; // save  identificator
           // WriteUART(ident);
            RX_byte=0;
        }
        else 
        {
            RX_byte = (RX_byte<<8)|secondVal;
            switch(ident)
            {
                case 103 : set_point = (float)(RX_byte)*0.01; break; //103 -'g'
                case 112 : kp = (float)(RX_byte)*1; break; //'p'
                case 105 : ki = ((float)(RX_byte))*0.01; break; // 'i'
                case 100 : kd = ((float)(RX_byte))*(1/0.01); break; //'d'
                case 113 : {ClearCycleBuff(); tmr1_int_counter = 0; //113 - 'q'
                            TMR1_INT_EN = 1; break;}// timer 1 interrupt enable // 
               // case 116 : WriteTwoByte(SAMPLE_TIME); break; // 116 - 't'                
                default:    break;
            }
            // use just RX_byte, if there was no identifier (only number)
            //WriteUART(RX_byte);    
        }
        uart_flag = 0;    
    }       
 
    IFS0bits.U1RXIF = 0; // clear interrupt flag    
}

void __attribute__((interrupt, auto_psv)) _INT0Interrupt(void) {//chA

    if (chA !=chB)
    {
         pos_cnt ++;
    }
    else 
    {
        pos_cnt --;
    }
  
    IFS0bits.INT0IF = 0; // clear ADC interrupt flag
}

void __attribute__((interrupt, auto_psv)) _INT1Interrupt(void) {//chB
 
    if (chA ==chB)
    {
         pos_cnt++;
    }
    else 
    {
        pos_cnt--;
    }
    IFS1bits.INT1IF = 0; // clear ADC interrupt flag
}

float PIDcontroller(float set_point,float input, float kp,float ki,float kd)
{
    error = set_point - input;
    i_error = i_error + error;
    d_error = d_error + (error - Prev_error);    
  
//    d_error = d_error; 
    i++;
    if  ( i == (N_diff_filter - 1) )
    {
        d_error_filtr = d_error_filtr/N_diff_filter;
        d_error = 0;
        i = 0;
            
    }      
    if ( i_error < -PWM_PERIOD) // u_min = 0 ->> 0% pwm
    {
        i_error = -PWM_PERIOD;
    }
    if ( i_error > PWM_PERIOD) // u_max = 2*PTPER
    {
        i_error = PWM_PERIOD;
    }    
    if (( error <= 0.1 )&&( error >= -0.1 )) // dead zone
    {
        u = 0;
        return u;
    }
#ifdef RELEY
    if (( error <= 6 )&&( error >= -6 )) // dead zone
    {
        u = kd*d_error + ki * i_error;
    }
    else 
    {
        u = kp * error;
    }
#else

      u = kp * error + ki * i_error + kd* d_error_filtr;
#endif
      
 
    if ( u > 0 ) //dead zone
    {
        u = u;// + 0.5*PWM_PERIOD;
    }
    if ( u < 0 )
    {
        u = u;// - 0.5*PWM_PERIOD;
    }
    if ( u < -2*PWM_PERIOD) // u_min = 0 ->> 0% pwm
    {
        u = -2*PWM_PERIOD;
    }
    if ( u > 2*PWM_PERIOD) // u_max = 2*PTPER
    {
        u = 2*PWM_PERIOD;
    }
    Prev_error = error;
    return u; 
}

void MoveServo(int duty)
{
    if ( duty > 0)
    {
        PWM_run(1); // arguments: 0 - low, 1-high, 2 - both
        Set_DutyCycleValue(duty);
    }
    if ( duty < 0 )
    {
        PWM_run(0); // arguments: 0 - low, 1-high, 2 - both 
        Set_DutyCycleValue(-duty);
    }
    if (duty == 0)
    {
        PWM_run(2);
        Set_DutyCycleValue(0);
    }
}

void __attribute__((interrupt, auto_psv)) _T1Interrupt( void )
{   
    angle_feedback = ADC2DEG (ReadAdc(3)); // transform adc value to degrees value
    ANG = ((float)(angle_feedback))/100; 
    
    WriteTwoByte(angle_feedback);
   // WriteTwoByteFloat(u);
    //WriteTwoByteFloat(ANG); 
   // WriteTwoByteFloat(error); 
    //WriteTwoByteFloat(pos_cnt);
    
    u =  PIDcontroller( set_point, ANG, kp, ki, kd);  // count controller 
    duty = (int)u;
    MoveServo(-u);  
    
    if (IFS0bits.T1IF == 1) // interrupt occurred while interrupt program execution 
    {
        _LATG13 = 1;
    }
    IFS0bits.T1IF = 0; // clear an interrupt flag
} 

void __attribute__((interrupt, auto_psv)) _IC2Interrupt(void) {
    t2=IC2BUF;  
    IFS0bits.IC2IF = 0;
}

void __attribute__ ((interrupt, auto_psv)) _CNInterrupt(void)
{     
//   while (!AD1CON1bits.DONE);
//   WriteCycleBuff(ADC1BUF0);
//   WriteCycleBuff(pos_cnt);
   pos_cnt++; // count impulses 
   IFS1bits.CNIF = 0; // Reset CN interrupt
}

int ServoInit(void)
{
    if ( ( ADC2DEG (ReadAdc(3)) >= -250 )&&( ADC2DEG (ReadAdc(3)) <= 250 ) )
    {
        MoveServo(0);
        pos_cnt = 0;
        return 0;
    }
//    if (ReadAdc(3) < 2000)
    if ( ADC2DEG (ReadAdc(3)) < -100 ) // -5 deg
    {
        //while (ReadAdc(3) <= 2048)
        while ( ADC2DEG (ReadAdc(3)) < 0 )
        {
//            PWM_run(1); // arguments: 0 - low, 1-high, 2 - both
//            Set_DutyCycleValue(0.7*PTPER);
              MoveServo(-0.7*PTPER);
        }
//        Set_DutyCycleValue(0);
        MoveServo(0);
        pos_cnt = 0;
        return 0; 
    }     
    //if ( ReadAdc(3) > 2100 )
    if ( ADC2DEG (ReadAdc(3)) > 100 )
    {
        //while (ReadAdc(3) >= 2048)
        while ( ADC2DEG (ReadAdc(3)) > 0 )
        {
//            PWM_run(0); 
//            Set_DutyCycleValue(0.7*PTPER);
              MoveServo(0.7*PTPER);
        }
//        Set_DutyCycleValue(0);
        MoveServo(0);
        pos_cnt = 0;
        return 0;
    }
    return 0;
}

int main(void) {
    
    SET_ALL_AN_AS_DIGITAL_INPUT; // all AN as digital input?
    uart_init();
    interrupt_init(); //INT0 - Index pulse - reset encoder counter
    ADC_init();
    Init_Timer1();
    TMR1_INT_EN = 0; //TMR1 interrupts disable
    ConfigureCN();    
    PWM_init();
    ServoInit(); // move slowly to position 90 degrees
    
//    _TRISE5 = 0; // PWM3H output enable
//    _TRISE4 = 0; // PWM3L output enable
    _TRISG13 = 0;  //LED pin    
    while (1)
    {      
      
    }
    return 0;
}
