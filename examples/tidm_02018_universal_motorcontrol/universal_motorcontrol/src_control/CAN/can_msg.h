/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#############################################################################
//
// FILE:  can_msg.h
//
// TITLE: header file for can message
//
//#############################################################################

#ifndef CAN_MSG_H
#define CAN_MSG_H

#include <drivers/mcan.h>
#include "motor_common.h"


extern MCAN_TxBufElement    MCANtxMsg;
extern MCAN_RxBufElement    MCANrxMsg;

extern MCAN_TxBufElement txMsgs[12];
extern uint32_t txMsgIds[12];
extern MCAN_RxBufElement rxMsg;
extern uint32_t gMcanBaseAddr;

extern volatile uint16_t McanMsgRdy;

extern void HAL_initMcan();
extern void HAL_MCanMsgInit();
extern void HAL_CanMsgInit();
extern void HAL_SendCanMsg(uint16_t msgNum);
extern void HAL_CanRxMsgMapping_0xC0();
extern void HAL_CanTxMsgPacking();
extern void HAL_McanMsgInit(MCAN_TxBufElement *txMsg, uint32_t msgId);

#endif
