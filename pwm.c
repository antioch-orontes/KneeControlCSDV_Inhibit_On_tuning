#include <xc.h>
#include "pwm.h"
#include <stdint.h>        /* Includes uint16_t definition                    */

/* Sets up the PWM module for complementary mode operation.
 * PWM1 H3 output is configured to produce a 10kHz PWM signal.
 * Sets PIN25 as PWM output*/

void init_PWM()
{
        /* ~~~~~~~~~~~~~~~~~~~~~~ PWM1 Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        /* PWM1 I/O Control Register register */
        /* PWM time base operates in a Free Running mode */
        P1TCONbits.PTMOD = 0b00;

        /* PWM time base input clock period is TCY (1:1 prescale) */
        /* PWM time base output post scale is 1:1 */

        P1TCONbits.PTCKPS = 0b00;
        P1TCONbits.PTOPS = 0b00;

        /* Choose PWM time period based on input clock selected */
        /* Refer to Equation 14-1 */
        /* PWM switching frequency is 20 kHz */
        /* FCY is 20 MHz */

        P1TPER = 3991; //FCY = 39.92Mhz FPWM = 10KHz, PxTMR prescaler = 1:1
                       // PxRPER = 39,920,000/(10,000*1)-1 = 3991

        /* PWM I/O pairs 1 to 3 are in complementary mode */
        /* PWM pins H1 (pin 25 PWM1H1 RB14)is enabled for PWM output */
        PWM1CON1bits.PMOD1 = 0;
        PWM1CON1bits.PMOD2 = 0;
        PWM1CON1bits.PMOD3 = 0;
        PWM1CON1bits.PEN1H = 0;
        PWM1CON1bits.PEN2H = 0;
        PWM1CON1bits.PEN3H = 1;

        PWM1CON1bits.PEN1L = 0;
        PWM1CON1bits.PEN2L = 0;
        PWM1CON1bits.PEN3L = 0;

        /* Immediate update of PWM enabled */
        PWM1CON2bits.IUE = 1;

        /* PWM I/O pin controlled by PWM Generator */
        P1OVDCONbits.POVD3H = 1;
        P1OVDCONbits.POVD2H = 1;
        P1OVDCONbits.POVD1H = 1;
        P1OVDCONbits.POVD3L = 1;
        P1OVDCONbits.POVD2L = 1;
        P1OVDCONbits.POVD1L = 1;

        /* Initialize duty cycle values for PWM1, PWM2 and PWM3 signals */
        P1DC1 = 4095;
        P1DC2 = 200;
        P1DC3 = 0;

        P1TCONbits.PTEN = 1;
}
/* set PIN 14 as motor direction output*/
void init_DIR()
{
        //Configure Pin 14 RB5 as open drain output for Direction
        ODCBbits.ODCB5 = 1; // Turn on open drain feature
        LATBbits.LATB5 = 0; // Set value to 0
        TRISBbits.TRISB5 = 1; // Output 0

        //Configure Pin 21 RB10 as open drain output for PWM
        ODCBbits.ODCB10 = 1;
}

void DF (float percent)
{
        uint16_t duty_cycle = 0;
        duty_cycle = percent * 8190;
        TRISBbits.TRISB5 = 1; // Pull up to 5v     "HIGH"
        P1DC3 = duty_cycle;
}

void PF (float percent)
{
        uint16_t duty_cycle = 0;
        duty_cycle = percent * 8190;
        TRISBbits.TRISB5 = 0; // Sink current to GND   "LOW"
        P1DC3 = duty_cycle;
}

void KneeFlexion (float percent)
{
        DF (percent);
}

void KneeExtension (float percent)
{
        PF (percent);
}

void PwmDutyCycle (float percent)
{
        uint16_t duty_cycle = 0;
        duty_cycle = percent * 8190;
        P1DC3 = duty_cycle;
}

void Stop ()
{
        P1DC3 = 0;
}

void Init_Inhibit()
{
        ODCAbits.ODCA1 = 1; // Turn on open drain feature
        TRISAbits.TRISA1 = 1; // Pull up to hv High by setting B5 as input, noting this is the function of Open-drain


}

void Open_Inhibit()
{
        TRISAbits.TRISA1 = 0; //set inhibit pin as low to stop inhibit

}
