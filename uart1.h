/*
 * File:   uart1.h
 * Author: dcl
 *
 * Created on April 8, 2015, 11:53 AM
 */

#ifndef UART1_H
#define UART1_H

#ifdef  __cplusplus
extern "C" {
#endif

void InitUART1( void );
void SoftwareDebounce( unsigned char Data );
void putsUART1(unsigned char *buffer);
unsigned int ReadUART1(void);
char DataRdyUART1(void);
void WriteUART1(unsigned int data);

#ifdef  __cplusplus
}
#endif

#endif  /* UART1_H */
