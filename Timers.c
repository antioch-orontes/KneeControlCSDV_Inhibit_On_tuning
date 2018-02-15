/*******************************************************
   Name: Timers.c
   Date: 6/2017
   Created By: Don Truex
   Comments:  Functions related to timers
   Revision History:
   rev number  date     by

 *****************************************************/
#include <xc.h>
#include <stdint.h>
#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

#include "Externals.h"

uint32_t timer1_millis = 0;
uint32_t timer1_overflow_count = 0;
int flag = 0;


void TimersInit(void);

void TimersInit(void)
{
        //Fc = Fosc/2 = 79.84/2 = 39.92
        //Tcy = 25.0495E-9 seconds
        //use timer 3 as a real time interrupt.
        //500 hz
        T3CONbits.TON = 0; // Disable Timer
        T3CONbits.TCS = 0; // Select internal instruction cycle clock
        T3CONbits.TGATE = 0; // disable Gated Timer mode
        T3CONbits.TCKPS = 0b01; // Select 8:1 Prescaler T = 2.004E-7 seconds
        TMR3 = 0x00; // Clear timer register

        // 4990 * 2.004E-7 = .001
        PR3 = 4990; //Load the period value.


        IPC2bits.T3IP = 0x04; // Set Interrupt Priority Level
        //	IPC2bits.T3IP = 0x04; // Set Interrupt Priority Level
        IFS0bits.T3IF = 0; // Clear Interrupt Flag
        IEC0bits.T3IE = 1; // Enable interrupt
        T3CONbits.TON = 1; // Start Timer

}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
        int i;
        //if this interrupt fires at 1khz the next line will output
        //  a square wave at 500 hz.  (.001 low, .001 hi))
        //LATB = PORTB^0x0020;//toggle B5
        T3ISRCount++;

        for (i = 0; i < MAXTIMERS; i++)
        {
                if (gTimers[i] > 0)
                        gTimers[i]--;
        }
        IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
}


/******************************************************************************
 * Function:   uint32_t millis()
 *
 * PreCondition: Timer1 Module must be Initialized with interrupt enabled.
 *
 * Input:        None
 *
 * Output:       Number of milliseconds since the program started (unsigned long)
 *
 * Side Effects: None
 *
 * Overview:
 *****************************************************************************/
uint32_t millis() {
        return timer1_millis;
}

/******************************************************************************
 * Function:   float micros()
 *
 * PreCondition: Timer1 Module must be Initialized with interrupt enabled.
 *
 * Input:        None
 *
 * Output:       Number of microseconds since the program started (float)
 *
 * Side Effects: None
 *
 * Overview:
 *****************************************************************************/

float micros() {
        return ((timer1_overflow_count * PR1) + TMR1)*(0.2004); //us
}

int control_loop(){
        return (flag);
}
