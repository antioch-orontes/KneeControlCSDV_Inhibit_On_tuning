#include <xc.h>
#include <stdint.h>
#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif
#include "timer1.h"

uint32_t timer1_millis = 0;
uint32_t timer1_overflow_count = 0;
int flag = 0;

/******************************************************************************
 * Function:   void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
 *
 * PreCondition: Timer1 Module must be Initialized with interrupt enabled.
 *
 * Input:        None
 *
 * Output:       None
 *
 * Side Effects: None
 *
 * Overview:     Timer1 interrupt service routine called whenever the timer register is overflow.
 *****************************************************************************/

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
        /* Interrupt Service Routine code goes here */
        IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
        timer1_millis += 1;
        timer1_overflow_count += 1;
        flag = !flag;
}

/******************************************************************************
 * Function:        void InitTimer1()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function configures Timer1 module
 *****************************************************************************/

void InitTimer1( void )
{
        // initialization code for 16-bit timer mode
        T1CONbits.TON = 0;// Disable Timer
        T1CONbits.TCS = 0;// Select internal instruction cycle clock
        T1CONbits.TGATE = 0;// Disable Gated Timer mode
        T1CONbits.TCKPS = 0b01;// 1:8 prescale value i tick is 0.2004 us
        TMR1 = 0x00; // Clear timer register
        PR1 = 4990; // Load the period value    one peorid is 1ms (1 ms / 0.2004 us)
        IPC0bits.T1IP = 0x01;// Set Timer1 Interrupt Priority Level
        IFS0bits.T1IF = 0;// Clear Timer1 Interrupt Flag
        //IEC0bits.T1IE = 1;// Enable Timer1 interrupt
        T1CONbits.TON = 1;// Start Timer
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
