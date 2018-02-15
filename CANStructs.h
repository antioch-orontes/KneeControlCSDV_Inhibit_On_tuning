/*******************************************************
   Name: Can.h
   Date: 6/2017
   Created By: Don Truex
   Comments:  Contains data structs and definitions for
   CAN serial bus communication.
   Revision History:

*******************************************************/
#ifndef _CAN_STRUCTS_H_
#define _CAN_STRUCTS_H_
#include "DSPicTypes.h"

//data union will simplify passing data to send function
typedef union {
        UINT8 u8[8];
        //FLOAT32 f32[2];
        INT8 i8[8];
        UINT16 u16[4];
        INT16 i16[4];
        UINT32 u32[2];
        INT32 i32[2];
        // FLOAT32 f32[2];
}CAN_DATA_UNION;

/* message structure in RAM */
typedef struct {
        /* keep track of the buffer status */
        unsigned char buffer_status;
        /* RTR message or data message */
        unsigned char message_type;
        /* frame type extended or standard */
        unsigned char frame_type;
        /* buffer being used to reference the message */
        unsigned char buffer;
        /* 29 bit id max of 0x1FFF FFFF
         *  11 bit id max of 0x7FF */
        unsigned long id;
        /* message data */
        unsigned char data[8];
        /* received message data length */
        unsigned char data_length;
}mID;


/* structure used to queue the FIFO messages in RAM */
typedef struct {
        /* keep track of the FIFO status */
        unsigned char FIFO_status;
        /* the structure of each message */
        mID FIFO[7];
}FIFO;

#endif
