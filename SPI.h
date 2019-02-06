/*
 * File:   SPI.h
 * Author: Zachery
 *
 * Created on July 18, 2014, 11:23 PM
 */

struct DACbitfield
{
        // 16bit word for MCP4912 DAC
        // LSB --> MSB = Top --> Bottom (MSB left to right convention)
        unsigned int : 2; // Empty fields for 10bit DAC
        unsigned int voltage : 10;
        unsigned int shutdown : 1;
        unsigned int gain : 1;
        unsigned int buf : 1;
        unsigned int chanAB : 1;
};

// DAC SPI functions
void DAConfigSPI();
void WriteSPI1(int value);
void WriteDACstruct(struct DACbitfield command);

// Special SPI functions
