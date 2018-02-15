/*
 * File:   uart2.h
 * Author: dcl
 *
 * Created on March 2, 2016, 11:12 AM
 */

#ifndef UART2_H
#define UART2_H

#ifdef  __cplusplus
extern "C" {
#endif

void InitUART2( void );
void SoftwareDebounce2( unsigned char Data );
void putsUART2(unsigned char *buffer);
void WriteUART2(unsigned int data);
char DataRdyUART2(void);
unsigned int ReadUART2(void);
#ifdef  __cplusplus
}
#endif

#endif  /* UART2_H */
