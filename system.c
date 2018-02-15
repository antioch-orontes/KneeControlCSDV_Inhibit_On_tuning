/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
      #include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
      #include <p33Fxxxx.h>
    #endif
#endif


#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "system.h"          /* variables/params used by system.c             */

/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration funtions, reset source evaluation          */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c.                                          */
/*                                                                            */
/******************************************************************************/

/* Refer to the device Family Reference Manual Oscillator section for
   information about available oscillator configurations.  Typically
   this would involve configuring the oscillator tuning register or clock
   switching useing the compiler's __builtin_write_OSCCON functions.
   Refer to the C Compiler for PIC24 MCUs and dsPIC DSCs User Guide in the
   compiler installation directory /doc folder for documentation on the
   __builtin functions.*/

/* TODO Add clock switching code if appropriate.  An example stub is below.   */
void ConfigureOscillator(void)
{

// Configure PLL prescaler, PLL postscaler, PLL divisor
// (7.37/N1)*(PLLFBD+2)/N2=79.84MHz
// F_OSC = 79.84Mhz


        PLLFBD = 63;
        CLKDIVbits.PLLPOST=0; // N2 = 2
        CLKDIVbits.PLLPRE=1; // N1(1) = 3; N1(0) = 2
// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
        __builtin_write_OSCCONH(0x01);
        __builtin_write_OSCCONL(OSCCON | 0x01);
// Wait for Clock switch to occur
        while(OSCCONbits.COSC != 0b001) {};
// Wait for PLL to lock
        while(OSCCONbits.LOCK!=1) {};

}
