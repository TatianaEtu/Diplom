
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */


#ifndef UART
#define	UART

#include <xc.h> // include processor files - each processor file is guarded


void uart_init(void);
void WriteUART(int data);
void WriteTwoByte(int par);
void float2Bytes(float val, char* bytes_array);
void WriteTwoByteFloat(float x);
void ClearCycleBuff(void);
void WriteCycleBuff( int data);
void ReadCycleBuff(void);

#endif	/* UART */

