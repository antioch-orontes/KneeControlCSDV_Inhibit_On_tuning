#include <xc.h>
#include "system.h"
#include <libpic30.h>
#include <float.h>
#include "AnalogIO.h"


// add a samp2voltage function

// long Samp_Delay=15;
// unsigned int num_channels=5;    // Number of ADC channels on dsPIC33FJ128MC802
//unsigned int init_ADC_chan[5]={0,0,0,0,0};



void ConfigureAnalog() {
      // Set configuration bits for all Analog input channels
        AD1CON1bits.ADON=0;  // Make sure ADC is off when setting general config settings.

        AD1CON1bits.AD12B=0; // Set to 10bit sampling
        AD1CON2bits.CHPS=0;  // Use internal Channel 0
        AD1CON1bits.FORM=0;  // Set to return an integer between 0-1023 (10bit)
        AD1CON2bits.VCFG=0;  // Use AVDD and AVSS for voltage range (0V-3.3V)
        AD1CON1bits.SIMSAM=0; // Use sequential sampling
        AD1CON2bits.ALTS=0;   // No alternate sample switching



        AD1CON3bits.ADRC=0; // ADC Clock is derived from Systems Clock
       // AD1CON3bits.SAMC=2; // Auto Sample Time = 0 * TAD
       AD1CON3bits.ADCS = 2;

       /* Excerpt from dsPIC33FJ128MC802 ADC datasheet:
        *
        * "ADC Conversion Clock TAD = TCY * (ADCS + 1) = (1/40Mhz) * 3 = 75 ns (13.3 MHz)
        * ADC Conversion Time for 10-bit Tconv = 12 * TAD = 900 ns (1.1 MHz)"
        *
        */

       AD1PCFGL = 0xFFFF; // Fill registers?

       AD1PCFGLbits.PCFG0=0; // AN0 as Analog Input
       AD1PCFGLbits.PCFG1=0; // AN1 as Analog Input
       AD1PCFGLbits.PCFG2=0; // AN2 as Analog Input
       AD1PCFGLbits.PCFG3=0; // AN3 as Analog Input
       AD1PCFGLbits.PCFG4=0; // AN4 as Analog Input
       AD1PCFGLbits.PCFG5=0; // AN5 as Analog Input

       /*

       AD1CON2bits.SMPI = 3; // Select 4 conversions between interrupt
       AD1CON1bits.ASAM=1;   // Enable automatic sampling
       AD1CON2bits.CSCNA = 1; // Enable Channel Scanning
       AD1CSSLbits.CSS0=1; // Enable AN0 for scan
       AD1CSSLbits.CSS1=1; // Enable AN1 for scan
       AD1CSSLbits.CSS4=1; // Enable AN4 for scan
       AD1CSSLbits.CSS5=1; // Enable AN5 for scan
       AD1CON2bits.BUFM=0; // Single 16-word result buffer
       IFS0bits.AD1IF = 0; // Clear the Analog-to-Digital interrupt flag bit
       IEC0bits.AD1IE = 0; // Do Not Enable Analog-to-Digital interrupt

        */

       
       AD1CHS0bits.CH0NA = 0; // Select VREF- for CH0 -ve input
       AD1CON1bits.ADON=1;   // Turn ADC On

       /* IMPORTANT: Microstick II uses pins 1, 4, 5, 21, 22 to perform its development functions.
                     DO NOT make changes to those pins while using the Microstick II
                     interface or it might cause errors while debugging */
        
}


unsigned int ReadAnalogSample(unsigned int input_pin) {
    /* This function returns an integer sample value of an analog voltage which
     * depends on the voltage reference (AVDD) and the bit depth. The returned
     * integer value range is listed below.
     *
     * 8bit:  0-254
     * 10bit: 0-1023
     * 12bit: 0-4095
     *
     * input_pin: a value between 0-4 to select AN0-AN4 pins for input
     * (see ref. image of dsPIC33FJ128MC802 in "reference" folder for more details)
     */
    //__delay_ms(2); // Sample for 2 ms

    AD1CHS0bits.CH0SA=input_pin; // Select ANx for +V input

    AD1CON1bits.SAMP = 1; // Start sampling
    __delay_us(5);     // Sample for 2uS Originally is 2 uS
    AD1CON1bits.SAMP = 0; // Start converting
    while (!AD1CON1bits.DONE) {
         // Conversion done?
    }
    AD1CON1bits.DONE = 0; // Clear conversion done status bit

    return ADC1BUF0; // Return a digital sample value between 0-1024 (10bit)

}

double Samp2Voltage(unsigned int sample) {
    /*This function returns the voltage of the sample signal*/
    //10bit: 1024 samples
    //v_ref=3.3V
    return sample*3.3/1024;
}

