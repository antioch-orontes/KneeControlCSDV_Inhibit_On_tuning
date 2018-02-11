#include <xc.h>
#include "AnalogIO.h"
#include "SPI.h"

const unsigned int _8bit=255;
const unsigned int _10bit=1024;
const unsigned int _12bit=4096;

void DAConfig() {
    // Use this function outside of main loop once to configure DAC
    DAConfigSPI(); // Call config function from SPI library
    LATBbits.LATB6=1; // Set Pin 15 (SS1) high so no random commands will be sent

}

void setVoltage(unsigned int chan, unsigned int voltage_samp)
{
    /* voltage_samp is limited to certain values based on bit depth (10 bit)
     * 8bit: 0-254
     * 10bit: 0-1023
     * 12bit: 0-4095
     */

    struct DACbitfield DAC;
    DAC.voltage=voltage_samp; // Digital Voltage value. Use MATLAB UDF's to calculate int value
    DAC.shutdown=1; // DAC is active
    DAC.gain=1; // 0 = 2x Output voltage, 1 = 1x Ouput voltage
    DAC.buf=1; // 0 = No output buffering, 1 = Buffer output
    DAC.chanAB=chan; // 0 = channel A, 1 = channel B

    WriteDACstruct(DAC);
}

unsigned int Voltage2Samp(double volt) {
    /*This function returns the sample signal of the analog voltage*/
    //10bit: 1024 samples
    //v_ref=5V
    return 1024/5*volt;
}