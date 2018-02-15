/*
 * File:   libas.h
 * Author: dcl
 *
 * Created on September 9, 2015, 11:39 AM
 */

#ifndef LIBAS_H
#define LIBAS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
typedef union
{
								uint8_t data;
								struct
								{
																uint8_t OCF : 1;
																uint8_t COF : 1;
																uint8_t LIN : 1;
																uint8_t MagInc : 1;
																uint8_t MagDec : 1;
																uint8_t EvenPar : 1;
								};
} ASDataFlags;


// Default constructor, SCLK connects to CLK, CSn to ChipSelect and DI to DO on AS5X45
void InitEncoder(int CLK, int DI, int CSn);
// Read position off sensor, and update
int GetPosition(void);
// Read position off sensor and return flags (wraps ASGetPosition for code size)
ASDataFlags GetDataFlags(void);
// Flags from last read of data
//ASDataFlags Flags;
// Position, for 10bit 0-1023, 12bit 0-4095
//int Position;
// Precision (0 based, 9 is 10bit, 11 is 12bit)
//int DataPrecision;
//int chipSelectPin;
//int clkPin;
//int dInPin;

float GetAngle(void);
float GetKneeAngle();

#ifdef __cplusplus
}
#endif

#endif /* LIBAS_H */
