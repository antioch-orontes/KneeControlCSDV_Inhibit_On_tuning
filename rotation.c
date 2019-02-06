#include <xc.h>
#include <stdint.h>
#include "rotation.h"

void DF(float percent)
{
        uint16_t duty_cycle = 0;
        duty_cycle = percent * 8190;
        TRISBbits.TRISB5 = 1; // Pull up to 5v     "HIGH"
        P1DC3 = duty_cycle;
}

void PF(float percent)
{
        uint16_t duty_cycle = 0;
        duty_cycle = percent * 8190;
        TRISBbits.TRISB5 = 0; // Sink current to GND   "LOW"
        P1DC3 = duty_cycle;
}

void PwmDutyCycle(float percent)
{
        uint16_t duty_cycle = 0;
        duty_cycle = percent * 8190;
        P1DC3 = duty_cycle;
}

void Stop()
{
        P1DC3 = 0;
}
