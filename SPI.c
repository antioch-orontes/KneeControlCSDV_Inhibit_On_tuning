#include <xc.h>
#include <stdint.h>
#include "SPI.h"

void DAConfigSPI()
{
        /* Set configuration bits for SPI channel 1 and configure PPS for com. with DAC
         * I have set Pins 14, 16, and 15 to SPI; they are all 5 volt tolerant pins
         * 14(SDO), 16(SCK), 15(CS)
         *
         * You should only need to call this once. I would not place it in the main
         * program loop.
         */

        // IFS0bits.SPI1IF = 0; // Clear the Interrupt flag
        // IEC0bits.SPI1IE = 0; // Disable the interrupt

        SPI1STATbits.SPIEN = 0; // Disable SPI module

        // "Interupts might need to disabled for proper unlocking"
        __builtin_write_OSCCONL(OSCCON & ~(1 << 6)); // Unlock Pin Remap capability

        /* Only needed if pins have a possible analog input function shown on the diagram or data sheet
           AD2PCFGLbits.PCFG6=1; // Set pin 15 to digital
           AD2PCFGLbits.PCFG7=1; // Set pin 16 to digital
           AD2PCFGLbits.PCFG8=1; // Set pin 17 to digital
           AD2PCFGLbits.PCFG9=1; // Set pin 18 to digital (not used)
         */

        //TRISBbits.TRISB9=1; // Set pin 18 as input (not used)
        TRISBbits.TRISB5 = 0; // Set pin 14 as output
        TRISBbits.TRISB7 = 0; // Set pin 16 as output
        TRISBbits.TRISB6 = 0; // Set pin 15 as output

        RPOR2bits.RP5R = 7; // Set pin 14/RB5 to SDO1
        RPOR3bits.RP6R = 9; // Set pin 15/RB6 to SS1 (CS)
        RPOR3bits.RP7R = 8; // Set pin 16/RB7 to SCK out

        //RPINR20bits.SDI1R=9; // Set pin 18/RB9 to SDI (not used)
        RPINR20bits.SCK1R = 7; // Set pin 16/RB7 to SCK input (must be set for use in master mode)

        __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Lock Pin Remap

        // SPI1CON1 Register Settings
        SPI1CON1bits.DISSCK = 0; // Internal serial clock is enabled
        SPI1CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
        SPI1CON1bits.MODE16 = 1; // Communication is word-wide (16 bits)
        SPI1CON1bits.MSTEN = 1;  // Master mode enabled
        SPI1CON1bits.SMP = 0;    // Input data is sampled at the middle of data output time
        SPI1CON1bits.CKE = 0;    // Serial output data changes on transition from
                                 // Idle clock state to active clock state
        SPI1CON1bits.CKP = 0;    // Idle state for clock is a low level;
        // active state is a high level
        SPI1CON1bits.SSEN = 0; // Set Pin 15 to standered GPIO output for control in master mode

        /* SPI Clock scaling factor from system clock (15Mhz is MAX SPI clock rate for dsPIC33FJ128MC80)
         *
         * SCK=Fcy/(PPRE*SPRE) => SCK=79.84/(4*2)=9.98Mhz
         *
         *  */

        SPI1CON1bits.PPRE = 2; // Set primary clock prescale factor to 4:1 (10)
        SPI1CON1bits.SPRE = 6; // Set secondary prescale to 2:1 (110)

        // Clock scale factor from system. SCK=Fcy/(PPRE*SPRE) => SCK=79.84/(1*2)=
        // 15Mhz is MAX SCK (SPI Clock) rate for dsPIC33FJ128MC80

        SPI1STATbits.SPIEN = 1; // Enable SPI module

        // Interrupt Controller Settings
        // IFS0bits.SPI1IF=0; // Clear the Interrupt flag
        // IEC0bits.SPI1IE=1; // Enable the interrupt
}

void WriteSPI1(int value)
{
        // Write 16bit output
        int clear;

        clear = SPI1BUF;    // Clear buffer
        LATBbits.LATB6 = 0; // Set Pin 15 (SS1) low
        SPI1BUF = value;    // Write integer "value" to buffer
        while (!SPI1STATbits.SPIRBF)
        {
        };                  // Wait to send data
        LATBbits.LATB6 = 1; // Set Pin 15 (SS1) high
}

void WriteDACstruct(struct DACbitfield command)
{
        int clear;
        clear = SPI1BUF;    // Clear buffer
        LATBbits.LATB6 = 0; // Set Pin 15 (SS1) low
        SPI1BUF = *(unsigned int *)&command;
        while (!SPI1STATbits.SPIRBF)
        {
        };                  // Wait to send data
        LATBbits.LATB6 = 1; // Set Pin 15 (SS1) high
}
