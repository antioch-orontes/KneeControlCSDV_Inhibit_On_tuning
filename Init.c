
/*******************************************************
   Name: Init.c
   Date: 6/2017
   Created By: Don Truex
   Comments:  Init functions
   Revision History:
   rev number  date     by

 *****************************************************/

#include "Externals.h"

void InitOsc(void);
void InitDma3(void);
void InitDma2(void);
void InitCAN1Clock(void);
void InitCAN1(void);

/* Dma Initialization for ECAN1 Transmission */
void InitDma3(void)
{
        //read from ram write to peripheral
        DMA3CONbits.DIR = 1;
        /* DMA Addressing Mode: Peripheral Indirect Addressing mode */
        DMA3CONbits.AMODE = 0x2;
        /* Operating Mode: Continuous, Ping-Pong modes disabled */
        DMA3CONbits.MODE = 0x0;
        /* Assign ECAN1 Transmit event for DMA Channel 0 */
        DMA3REQ = 0x0046;
        /* Set Number of DMA Transfer per ECAN message to 8 words */
        DMA3CNT = 7;
        /* Peripheral Address: ECAN1 Transmit Register */
        DMA3PAD = (int)&C1TXD;
        /* Start Address Offset for ECAN1 Message Buffer 0x0000 */
        DMA3STA = __builtin_dmaoffset(ecan1msgBuf);
        /* Channel Enable: Enable*/
        DMA3CONbits.CHEN = 1;
}

/* Dma Initialization for ECAN1 Reception */
void InitDma2(void)
{

        DMACS0 = 0;
        DMA2CON = 0x0020;
        DMA2PAD = 0x0440; /* ECAN 1 (C1RXD) */
        DMA2CNT = 0x0007;
        DMA2REQ = 0x0022; /* ECAN 1 Receive */
        DMA2STA = __builtin_dmaoffset(ecan1msgBuf);
        DMA2CONbits.CHEN = 1;
}

void InitCAN1Clock(void)
{

        // FCAN is selected to be FCY
        // FCAN = FCY = 40MHz
        //C1CTRL1bits.CANCKS = 0x1;

        /*
           Bit Time = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)=25*TQ
           Phase Segment 1 = 8TQ
           Phase Segment 2 = 8Tq
           Propagation Delay = 8Tq
           Sync Segment = 1TQ
           CiCFG1<BRP> =(FCAN /(2 �N�FBAUD))� 1
         */

        /* Synchronization Jump Width set to 4 TQ */
        C1CFG1bits.SJW = 0x3;
        /* Baud Rate Prescaler */
        C1CFG1bits.BRP = BRP_VAL;
        /* Phase Segment 1 time is 8 TQ */
        C1CFG2bits.SEG1PH = 0x7;
        /* Phase Segment 2 time is set to be programmable */
        C1CFG2bits.SEG2PHTS = 0x1;
        /* Phase Segment 2 time is 6 TQ */
        C1CFG2bits.SEG2PH = 0x5;
        /* Propagation Segment time is 5 TQ */
        C1CFG2bits.PRSEG = 0x4;
        /* Bus line is sampled three times at the sample point */
        C1CFG2bits.SAM = 0x1;
}
void InitCAN1(void)
{
        //B4 is CAN RX
        TRISBbits.TRISB4 = 1;
        RPINR26bits.C1RXR = 4;
        //B15 is CAN TX
        RPOR7bits.RP15R = 0b10000;
        TRISBbits.TRISB15 = 0;
        /* Request Configuration Mode */
        do
        {
                C1CTRL1bits.REQOP = 4;
        } while (C1CTRL1bits.OPMODE != 4);

        InitCAN1Clock();

        C1FCTRLbits.FSA = 0b01000; /* FIFO Starts at Message Buffer 8 */
        C1FCTRLbits.DMABS = 0b110; /* 32 CAN Message Buffers in DMA RAM */

        //There are two steps in configuring a message for receiving.
        //  1.  Setup a filter mask.  The mask determines which bits in the message ID (MID)
        //are compared for receiving.  Set the mask bit to 0 to ignore a bit, 1 to include
        //a bit in the comparison.  Use the mask to allow a filter to accempt a range of
        //messages.
        //  2.  Setup a filter.

        //These drivers were provided by Microchip.
        /*	Mask Configuration.  There are up to 3 filter masks.
         *
                ecan1WriteRxAcptMask(int m, long identifierMask, unsigned int mide, unsigned int exide)

                m = 0 to 2 -> Mask Number

                identifier -> SID <10:0> : EID <17:0>

                mide = 0 -> Match either standard or extended address message if filters match
                mide = 1 -> Match only message types that correpond to 'exide' bit in filter

                exide = 0 -> Match messages with standard identifier addresses
                exide = 1 -> Match messages with extended identifier addresses

         */

        //Mask 0 requires an exact match with the filter MID.
        ecan1WriteRxAcptMask(0, 0x1FFFFFFF, 0, 0);
        //Mask 1 ignores the LSB,  IE if a filter uses this mask and an MID of
        // 0x10 MIDs 0x10 and 0x11 will be received.
        ecan1WriteRxAcptMask(1, 0x1FFFFFFe, 0, 0);

        /*	Filter Configuration

                ecan1WriteRxAcptFilter(int n, long identifier, unsigned int exide,unsigned int bufPnt,unsigned int maskSel)

                n = 0 to 15 -> Filter number

                identifier -> SID <10:0> : EID <17:0>

                exide = 0 -> Match messages with standard identifier addresses
                exide = 1 -> Match messages with extended identifier addresses

                bufPnt = 0 to 14  -> RX Buffer 0 to 14
                bufPnt = 15 -> RX FIFO Buffer

                maskSel = 0	->	Acceptance Mask 0 register contains mask
                maskSel = 1	->	Acceptance Mask 1 register contains mask
                maskSel = 2	->	Acceptance Mask 2 register contains mask
                maskSel = 3	->	No Mask Selection

         */
        //Receive MID 0x24 and place in the FIFO
        ecan1WriteRxAcptFilter(0, 0x00000024, 0, 15, 0);

        /* Enter Normal Mode */
        do
        {
                C1CTRL1bits.REQOP = 0;
        } while (C1CTRL1bits.OPMODE != 0);

        /* ECAN transmit/receive message control */

        C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x0000;
        C1TR01CONbits.TXEN0 = 1; /* ECAN1, Buffer 0 is a Transmit Buffer */
        C1TR01CONbits.TXEN1 = 1; /* ECAN1, Buffer 1 is a Transmit Buffer */
        C1TR23CONbits.TXEN2 = 1; /* ECAN1, Buffer 2 is a Transmit Buffer */
        C1TR23CONbits.TXEN3 = 1; /* ECAN1, Buffer 3 is a Transmit Buffer */
        C1TR45CONbits.TXEN4 = 1; /* ECAN1, Buffer 4 is a Transmit Buffer */
        C1TR45CONbits.TXEN5 = 1; /* ECAN1, Buffer 5 is a Transmit Buffer */
        C1TR67CONbits.TXEN6 = 1; /* ECAN1, Buffer 6 is a Transmit Buffer */
        C1TR67CONbits.TXEN7 = 1; /* ECAN1, Buffer 7 is a Transmit Buffer */

        C1TR01CONbits.TX0PRI = 0b11; /* Message Buffer 0 Priority Level */
        C1TR01CONbits.TX1PRI = 0b11; /* Message Buffer 1 Priority Level */
        C1TR23CONbits.TX2PRI = 0b11; /* Message Buffer 2 Priority Level */
        C1TR23CONbits.TX3PRI = 0b11; /* Message Buffer 3 Priority Level */
        C1TR45CONbits.TX4PRI = 0b11; /* Message Buffer 4 Priority Level */
        C1TR45CONbits.TX5PRI = 0b11; /* Message Buffer 5 Priority Level */
        C1TR67CONbits.TX6PRI = 0b11; /* Message Buffer 6 Priority Level */
        C1TR67CONbits.TX7PRI = 0b11; /* Message Buffer 7 Priority Level */
}

void InitOsc(void)
{
        // Configure Oscillator to operate the device at 40Mhz instruction cycle
        // See section 9 of dsPIC datasheet (DS70591E).
        // Fosc= Fin*M/(N1*N2),
        // Fosc= 7.37M*65/(2*3)=79.84MHz 80Mhz
        // Fcy=Fosc/2 = 39.92

        //7.37/3 = 2.4567, 0.8 < 2.4567 < 8
        CLKDIVbits.PLLPRE = 1; // N1 = PLLPRE + 2 = 3
        //2.4567 * 65 = 159.683, 100 < 159.683 < 200
        PLLFBD = 63; // M  = PLLDIV+2 = 65 9 bits
        //159.683/2 = 79.842
        CLKDIVbits.PLLPOST = 0; // N2 = 2*(PLLPOST + 1) = 2

        // Disable Watch Dog Timer. See section 24 of datasheet (DS70591E).
        RCONbits.SWDTEN = 0;

        // Wait for PLL to stablize.
        // See section 9 of dsPIC datasheet (DS70591E).
        while (OSCCONbits.LOCK != 1)
        {
        };
}
