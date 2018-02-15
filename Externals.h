/*******************************************************
   Name: Externals.h
   Date: 6/2017
   Created By: Don Truex
   Comments:  Include this file everywhere except Main.c
   Revision History:
   rev number  date     by

 *****************************************************/
#ifndef __EXTERNALS_H__
#define __EXTERNALS_H__

#include "p33fxxxx.h"
#include "Defs.h"
#include "DSPicTypes.h"
#include "CANStructs.h"
#include "CanDefs.h"
#include "CANDrv.h"

extern ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma)));
extern UINT16 gTimers[MAXTIMERS];
extern UINT32 T3ISRCount;
#endif
