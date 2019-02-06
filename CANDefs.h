/*******************************************************
   Name: CanDefs.h
   Date: 6/2017
   Created By: Don Truex
   Comments: CAN module #define
   Revision History:

*******************************************************/

#ifndef _CAN_DEFS_H
#define _CAN_DEFS_H

#ifndef uint16
#define uint16 unsigned int
#endif

//The Standard Identifier (SID) Field contains 11 bits and makes up the entire Arbitration Field
//within a standard CAN message, Figure 4-1. This field is typically responsible for identifying the
//message contents and also assigning priority to the message. For this project, the SID has been
//divided into 3 subsections as suggested in the UNB Protocol, and are labeled: Priority Identifier
//(PID) Field, Mode Identifier (MID) Field, and the Node Identifier (NID), Figure 4-2.
#define ACKMSB 0x0100
#define ACKLSB 0x0001

//Priority identifier
#define PRIORITY_HI 0x0    //00  // high priority
#define PRIORITY_MED 0x200 //01 // medium priority
#define PRIORITY_LOW 0x400 //10 // low priority
//Mode Identifier
#define MID_FROM 0   //the NID contains the value of the node that sent the message.
#define MID_TO 0x100 //the NID contains the value of the node that the message is destined

////transmit request register
#define _0x20Buf 0
#define _0x20TXBusy C1TR01CONbits.TXREQ0

#define _0x25Buf 1
#define _0x25TXBusy C1TR01CONbits.TXREQ1

//THese are not used, redefine as needed.
#define NOTUSEDBUF2 2
#define NOTUSEDBUF2TXBusy C1TR23CONbits.TXREQ2
#define NOTUSEDBUF3 3
#define NOTUSEDBUF3TXBusy C1TR23CONbits.TXREQ3
#define NOTUSEDBUF4 4
#define NOTUSEDBUF4TXBusy C1TR45CONbits.TXREQ4
#define NOTUSEDBUF5 5
#define NOTUSEDBUF5TXBusy C1TR45CONbits.TXREQ5
#define NOTUSEDBUF6 6
#define NOTUSEDBUF6TXBusy C1TR67CONbits.TXREQ6
#define NOTUSEDBUF7 7
#define NOTUSEDBUF7TXBusy C1TR67CONbits.TXREQ7

#endif
