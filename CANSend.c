/*******************************************************
   Name: CanSend.c
   Date: 6/2017
   Created By: Don Truex
   Comments: Build and Transmit CAN messages
   Revision History:

*******************************************************/

#include "Externals.h"

extern void ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);

UINT8 CanSend0x20(CAN_DATA_UNION *pData);

//void CANSendAll(void)
void CANSendAll(float flt_data_1, float flt_data_2, float flt_data_3, int int_data_2)
{
        UINT8 retVal, retryCount;
        static UINT16 packetCount = 0;
        CAN_DATA_UNION data;

        //ang_velocity=-11.11;

        data.u16[0] = (UINT16)((flt_data_1 * 10) + (65535 / 2)); //range of data able to handle, -3276.7 to 3276.7
        data.u16[1] = (UINT16)((flt_data_2 * 10) + (65535 / 2)); //range of data able to handle, -3276.7 to 3276.7
        data.u16[2] = (UINT16)((flt_data_3 * 10) + (65535 / 2)); //range of data able to handle, 0 to 65535
        data.u16[3] = int_data_2;                                //range of data able to handle, 0 to 65535

        //    data.u16[0] =1;
        //    data.u16[1] = 2000;
        //    data.u32[1] = 1;
        //Try to send the message 5 times then give up.
        //The return value only indicates that the message was placed in the transmit queue, not that it was delivered.
        //If there are no other devices on the CAN bus the transmit buffer will fill up and the sending function will return 1;
        //Testing the return value is optional (in general I don't bother).

        retryCount = 0;
        do
        {
                retVal = CanSend0x20(&data);
                retryCount++;
        } while ((retVal == 0) && (retryCount < 5));
}

UINT8 CanSend0x20(CAN_DATA_UNION *pData)
{
        if (_0x20TXBusy == 0)
        {
                //Change the message ID (0x20) and buffer index (_0x20Buf) for other messages
                ecan1WriteTxMsgBufId(_0x20Buf, PRIORITY_HI | MID_FROM | 0x20, 0, 0);

                ecan1msgBuf[_0x20Buf][2] = ((ecan1msgBuf[_0x20Buf][2] & 0xFFF0) + 8); //8 is the data length in bytes

                ecan1msgBuf[_0x20Buf][3] = pData->u16[0];
                ecan1msgBuf[_0x20Buf][4] = pData->u16[1];
                ecan1msgBuf[_0x20Buf][5] = pData->u16[2];
                ecan1msgBuf[_0x20Buf][6] = pData->u16[3];

                _0x20TXBusy = 1;
                return 0; //message successfully queued for sending
        }
        return 1; //buffer is busy, message not sent.  Calling function decides what to do.
}

UINT8 CanSend0x25(CAN_DATA_UNION *pData)
{
        if (_0x25TXBusy == 0)
        {

                ecan1WriteTxMsgBufId(_0x25Buf, PRIORITY_HI | MID_FROM | 0x25, 0, 0);

                ecan1msgBuf[_0x25Buf][2] = ((ecan1msgBuf[_0x25Buf][2] & 0xFFF0) + 8); //8 is the data length in bytes

                ecan1msgBuf[_0x25Buf][3] = pData->u16[0];
                ecan1msgBuf[_0x25Buf][4] = pData->u16[1];
                ecan1msgBuf[_0x25Buf][5] = pData->u16[2];
                ecan1msgBuf[_0x25Buf][6] = pData->u16[3];

                _0x25TXBusy = 1;
                return 0; //message successfully queued for sending
        }
        return 1; //buffer is busy, message not sent.  Calling function decides what to do.
}
