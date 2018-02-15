/*******************************************************
   Name: Defs.h
   Date: 6/2017
   Created By: Don Truex
   Comments:
   Revision History:

*******************************************************/

#ifndef __DEFS_H__
#define __DEFS_H__

#define CAN_MSG_DATA  0x01 // message type
#define CAN_MSG_RTR 0x02 // data or RTR
#define CAN_FRAME_EXT 0x03 // Frame type
#define CAN_FRAME_STD 0x04 // extended or standard

/* CAN Baud Rate Configuration    */
#define FCAN    40000000 //FCAN == Fcy == Fosc/2 40Mhz is max
#define BITRATE 1000000
#define NTQ   20    // 20 Time Quanta in a Bit Time
#define BRP_VAL   ((FCAN/(2*NTQ*BITRATE))-1)

/* CAN Message Buffer Configuration */
#define  ECAN1_MSG_BUF_LENGTH   32
typedef unsigned int ECAN1MSGBUF [ECAN1_MSG_BUF_LENGTH][8];

enum {MAINTIMER,MAXTIMERS};

#endif
