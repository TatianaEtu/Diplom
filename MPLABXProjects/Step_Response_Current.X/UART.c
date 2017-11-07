/*
 * File:   UART.c
 * Author: PC
 *
 * Created on April 19, 2017, 11:11 AM
 */


#include "xc.h"
#include "p33FJ256MC710.h"
#include <libpic30.h>
#include <string.h>
#include "define_heder.h"
//#define FCY 16000000UL 
//
//#define UART_BAUD  57600 
//#define UBRG_VALUE  (FCY/UART_BAUD/16) - 1  // rounding to the low integer
//#define BUFF  500 // cycle bufer demension

int NUM = 750, IC1_arr[BUFF];
int p_read = 0, p_write = 0, flag_r = 0, flag_w = 0, counter = 0;

void uart_init(void) {
    TRISF = 0xffff;
    TRISFbits.TRISF3 = 0;    
    U1MODEbits.BRGH = 0;    
    U1BRG = UBRG_VALUE; // baud rate 115200
    //U1BRG = 34; // 250000 real baud rate
    IEC0bits.U1RXIE = 1; //enable RX1 interrupt
    //IEC0bits.U1TXIE=1; //enable TX1 interrupt
    U1STAbits.URXISEL1 = 0; //If URXISEL<1:0> = 11,  an interrupt is generated
    //each time a data word is transferred from the UARTx Receive Shift Register 
    //(UxRSR) to the receive buffer. There may be one or more characters in the receive buffer.
    U1STAbits.URXISEL0 = 0;                         

    U1MODEbits.UARTEN = 1; //UART Enable    
    U1STAbits.UTXEN = 1; //UART transmitter enabled, UxTX pin controlled by UART
}

void WriteUART(char data) 
{
    while(U1STAbits.UTXBF==1) //UARTx Transmit Buffer Full Status bit (read-only)
    {
        Nop();
    }
    U1TXREG = data; // 8 bit mode 
//    while (!U1STAbits.TRMT) {
//    }
}
void WriteTwoByte(int par) // find low and high bytes in 16bits variable and send it 
{
    char h = 0, l = 0; //     
    h = (char) ((par >> 8)); // find high byte 
    WriteUART(h); //send high byte
    l = (char) ((par)); //find low byte
    WriteUART(l); // send low byte !
}
void float2Bytes(float val, char* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    char temp_array[4];
  } u;
  // Overwrite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
void WriteTwoByteFloat(float x) // find low and high bytes in 16bits variable and send it 
{
  // Create union of shared memory space
  union 
  {
    float float_variable;
    char temp_array[4];
  } u;
  // Overwrite bytes of union with float variable
  u.float_variable = x;
  // Assign bytes to input array
  while(U1STAbits.UTXBF==1); //UARTx Transmit Buffer Full Status bit (read-only)  
  //ждем, пока в буфере освободится хотя бы одно место
  U1TXREG = u.temp_array[3]; 
  while(U1STAbits.UTXBF==1); //UARTx Transmit Buffer Full Status bit (read-only)  
  //ждем, пока в буфере освободится хотя бы одно место
  U1TXREG = u.temp_array[2];
  while(U1STAbits.UTXBF==1); //UARTx Transmit Buffer Full Status bit (read-only)  
  //ждем, пока в буфере освободится хотя бы одно место
  U1TXREG = u.temp_array[1];
  while(U1STAbits.UTXBF==1); //UARTx Transmit Buffer Full Status bit (read-only)  
  //ждем, пока в буфере освободится хотя бы одно место
  U1TXREG = u.temp_array[0];        
}

void ClearCycleBuff(void)
{
    int i = 0;
    for ( i=0; i <= BUFF; i++ )
    {
        IC1_arr[i]=0;
    }
}

void WriteCycleBuff( int data)
{
//      if (counter <= NUM)
//    {        
        if ( p_write <= BUFF-1 )
        {
            IC1_arr [p_write] = data;
            p_write++;
            counter++;
        }
        else
        {
            p_write = 0;
            IC1_arr [p_write] = data;
            p_write++;
            flag_w++;
        }    
//    }
}

void ReadCycleBuff(void)
{
//     if ( counter >= NUM )
//    {
//        IEC0bits.T1IE = 0;  // timer 1 interrupt disable
//        //IEC0bits.IC1IE = 0; // input capture channel interrupt disable 
//        //IEC0bits.IC2IE = 0; // input capture channel interrupt disable 
//        IEC1bits.CNIE = 0; // Disable CN interrupts
//        
//        _LATE4 = 0; //switch off motor
//        _LATE5 = 0; //switch off motor   
//        counter = 0;
//    }
    if (p_read > BUFF-1)
    {
        p_read = 0;
        flag_r ++;           
    }
    if (flag_w == flag_r)
    {
        if (p_write > p_read)
        {
            WriteTwoByte(IC1_arr [p_read]);
            p_read++;
        }
        else {
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
    else {
             //_LATG13 =! _LATG13;  // error
    }         
}
