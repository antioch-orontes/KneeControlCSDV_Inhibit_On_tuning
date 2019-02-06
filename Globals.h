/*******************************************************
   Name: Globals.h
   Date: 6/2017
   Created By: Don Truex
   Comments: Include this file in Main.c only, externals.h every were else
   Revision History:

*******************************************************/
#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include "p33fxxxx.h"
#include "Defs.h"
#include "DSPicTypes.h"
#include "CANStructs.h"
#include "CanDefs.h"

ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma), aligned(ECAN1_MSG_BUF_LENGTH * 16)));
UINT16 gTimers[MAXTIMERS];
UINT32 T3ISRCount = 0;
#endif
