/*******************************************************
   Name: CanParse.c
   Date: 6/2017
   Created By: Don Truex
   Comments: parse routines for CAN messages
   External function: CanParse()
   Revision History:

*******************************************************/


#include "Externals.h"

extern void CanSend(uint16 ID, uint16 PID, uint16* pd);
extern UINT8 CanSend0x25(CAN_DATA_UNION *pData);

void CanParseBuf(UINT16 buf);
UINT16 CanParse(void);
void Parse0x24(unsigned int buf);


#define C1RX1START 8 //first rx buf
#define C1RX1END 15  //last rx buf
#define C1RX1MASK 0xff00

#define C1RX2START 0 //first rx buf
#define C1RX2END 15  //last rx buf
#define C1RX2MASK 0xffff

UINT16 CanParse(void)
{
        UINT16 i, cnt;
        UINT16 m;

        cnt = 0;
        if (C1RXFUL1 & C1RX1MASK)
        {
                for (i = C1RX1START; i <= C1RX1END; i++)
                {
                        m = 1 << i;
                        if (C1RXFUL1 & m)
                        {
                                CanParseBuf(i);
                                C1RXFUL1 &= ~m; //clear the flag
                                cnt++;
                                //return here if you only want to parse
                                //1 message per call
                                //return cnt;
                        }
                }
        }
        if (C1RXFUL2 & C1RX2MASK)
        {
                for (i = C1RX2START; i <= C1RX2END; i++)
                {
                        m = 1 << i;
                        if (C1RXFUL2 & m)
                        {
                                CanParseBuf(i + C1RX2START);
                                C1RXFUL2 &= ~m; //clear the flag
                                //return here if you only want to parse
                                //1 message per call
                                //return cnt;
                        }
                }
        }
        return cnt;
}

void CanParseBuf(UINT16 buf)
{
        UINT16 id;
        id = ecan1msgBuf[buf][0] >> 2;
        switch(id)
        {
        case 0x24:
                Parse0x24(buf);
                break;
        default:
                break;
        }
}

void Parse0x24(UINT16 buf)
{
        CAN_DATA_UNION *pData;
        pData = (CAN_DATA_UNION *)&ecan1msgBuf[buf][3];

        //echo this back in MID 0x25
        CanSend0x25(pData);

}
