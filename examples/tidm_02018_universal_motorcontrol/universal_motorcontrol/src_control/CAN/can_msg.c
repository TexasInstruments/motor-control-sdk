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

/*
 *  \file           can_msg.c
 *  \brief          This file contains source file for CAN message initialization, packing and mapping
*/

#include "can_msg.h"

#if defined(CMD_CAN)

// **************************************************************************

MCAN_TxBufElement    MCANtxMsg;
MCAN_RxBufElement    MCANrxMsg;

MCAN_RxBufElement rxMsg;

MCAN_TxBufElement txMsgs[12];
uint32_t txMsgIds[12] = {0xFC, 0xFB, 0xFD, 0xFE,
                         0xFF, 0xF0, 0xF1, 0xF2,
                         0xF3, 0xF4, 0xF5, 0xFA};

RX_1_Type RX_0xC0_Message;
TX_1_Type TX_0xFC_Message;

#define APP_MCAN_BASE_ADDR                       (CONFIG_MCAN0_BASE_ADDR)
#define APP_MCAN_INTR_NUM                        (CONFIG_MCAN0_INTR)
#define APP_MCAN_MSG_LOOP_COUNT                  (10U)
#define APP_MCAN_LOOPBACK_MODE_DISABLE           (FALSE)

/* Allocate Message RAM memory section to filter elements, buffers, FIFO */
/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (3U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (0U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (13U)
#define APP_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (0U)
/* Maximum RX FIFO 1 can be configured is 64 and
 * rest of the memory is allocated to RX buffer which is again of max size 64 */
#define APP_MCAN_FIFO_1_CNT                      (0U)

/* Standard Id configured in this app */
#define APP_MCAN_STD_ID                          (0xC0U)
#define APP_MCAN_STD_ID_MASK                     (0x7FFU)
#define APP_MCAN_STD_ID_SHIFT                    (18U)

#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)

/* In the CAN FD format, the Data length coding differs from the standard CAN.
 * In case of standard CAN it is 8 bytes */

/* Semaphore to indicate transfer completion */
static HwiP_Object       gMcanHwiObject;
uint32_t          gMcanBaseAddr;

volatile uint16_t McanMsgRdy = 0;

static void App_mcanIntrISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus);

    /* If FIFO0/FIFO1 is used, then MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG macro
     * needs to be replaced by MCAN_INTR_SRC_RX_FIFO0_NEW_MSG/
     * MCAN_INTR_SRC_RX_FIFO1_NEW_MSG respectively */
    if (MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG ==
        (intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG))
    {
        McanMsgRdy = 1;
    }

    return;
}

static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                            uint32_t bufNum)
{
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = APP_MCAN_STD_ID + bufNum;
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = bufNum;
    /* Store message in buffer */
    stdFiltElem->sfec  = MCAN_STD_FILT_ELEM_BUFFER;
    /* Below configuration is ignored if message is stored in buffer */
    stdFiltElem->sft   = MCAN_STD_FILT_TYPE_RANGE;

    return;
}

static void App_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams
                                           *msgRAMConfigParams)
{
    int32_t status;

    MCAN_initMsgRamConfigParams(msgRAMConfigParams);

    /* Configure the user required msg ram params */
    msgRAMConfigParams->lss = APP_MCAN_STD_ID_FILTER_CNT;
    msgRAMConfigParams->lse = APP_MCAN_EXT_ID_FILTER_CNT;
    msgRAMConfigParams->txBufCnt = APP_MCAN_TX_BUFF_CNT;
    msgRAMConfigParams->txFIFOCnt = APP_MCAN_TX_FIFO_CNT;
    /* Buffer/FIFO mode is selected */
    msgRAMConfigParams->txBufMode = MCAN_TX_MEM_TYPE_BUF;
    msgRAMConfigParams->txEventFIFOCnt = APP_MCAN_TX_EVENT_FIFO_CNT;
    msgRAMConfigParams->rxFIFO0Cnt = APP_MCAN_FIFO_0_CNT;
    msgRAMConfigParams->rxFIFO1Cnt = APP_MCAN_FIFO_1_CNT;
    /* FIFO blocking mode is selected */
    msgRAMConfigParams->rxFIFO0OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;
    msgRAMConfigParams->rxFIFO1OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;

    status = MCAN_calcMsgRamParamsStartAddr(msgRAMConfigParams);
    DebugP_assert(status == CSL_PASS);

    return;
}

static void App_mcanConfig(Bool enableInternalLpbk)
{
    MCAN_StdMsgIDFilterElement stdFiltElem[APP_MCAN_STD_ID_FILTER_CNT] = {0U};
    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};
    uint32_t                   i;

    /* Initialize MCAN module initParams */
    MCAN_initOperModeParams(&initParams);
    /* CAN FD Mode and Bit Rate Switch Enabled */
    initParams.fdMode          = TRUE;
    initParams.brsEnable       = TRUE;

    /* Initialize MCAN module Global Filter Params */
    MCAN_initGlobalFilterConfigParams(&configParams);

    /* Initialize MCAN module Bit Time Params */
    /* Configuring default 1Mbps and 5Mbps as nominal and data bit-rate resp */
    MCAN_initSetBitTimeParams(&bitTimes);

    /* Initialize MCAN module Message Ram Params */
    App_mcanInitMsgRamConfigParams(&msgRAMConfigParams);

    /* Initialize Filter element to receive msg, should be same as tx msg id */
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT; i++)
    {
        App_mcanInitStdFilterElemParams(&stdFiltElem[i], i);
    }
    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(gMcanBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}

    /* Initialize MCAN module */
    MCAN_init(gMcanBaseAddr, &initParams);
    /* Configure MCAN module Gloabal Filter */
    MCAN_config(gMcanBaseAddr, &configParams);
    /* Configure Bit timings */
    MCAN_setBitTime(gMcanBaseAddr, &bitTimes);
    /* Configure Message RAM Sections */
    MCAN_msgRAMConfig(gMcanBaseAddr, &msgRAMConfigParams);
    /* Set Extended ID Mask */
    MCAN_setExtIDAndMask(gMcanBaseAddr, APP_MCAN_EXT_ID_MASK);

    /* Configure Standard ID filter element */
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT; i++)
    {
        MCAN_addStdMsgIDFilter(gMcanBaseAddr, i, &stdFiltElem[i]);
    }
    if (TRUE == enableInternalLpbk)
    {
        MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, TRUE);
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr))
    {}

    return;
}

static void App_mcanEnableIntr(void)
{
    MCAN_enableIntr(gMcanBaseAddr, MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG, (uint32_t)TRUE);
    /* Select Interrupt Line 0 */
    MCAN_selectIntrLine(gMcanBaseAddr, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_0);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(gMcanBaseAddr, MCAN_INTR_LINE_NUM_0, (uint32_t)TRUE);

    return;
}

void HAL_initMcan()
{
    int32_t                 status = SystemP_SUCCESS;
    HwiP_Params             hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_MCAN_INTR_NUM;
    hwiPrms.callback    = &App_mcanIntrISR;
    status              = HwiP_construct(&gMcanHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Assign MCAN instance address */
    gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(APP_MCAN_BASE_ADDR);

    /* Configure MCAN module, Enable External LoopBack Mode */
    App_mcanConfig(APP_MCAN_LOOPBACK_MODE_DISABLE);

    /* Enable Interrupts */
    App_mcanEnableIntr();
}

void HAL_MCanMsgInit()
{
    //
    // Initialize message to transmit.
    //
    MCANtxMsg.id       = ((uint32_t)(0xE0)) << 18; // Identifier Value.
    MCANtxMsg.rtr      = 0U; // Transmit data frame.
    MCANtxMsg.xtd      = 0U; // 11-bit standard identifier.
    MCANtxMsg.esi      = 0U; // ESI bit in CAN FD format depends only on error
                         // passive flag.
    MCANtxMsg.dlc      = 8U; // CAN + CAN FD: transmit frame has 0-8 data bytes.
    MCANtxMsg.brs      = 1U; // CAN FD frames transmitted with bit rate
                         // switching.
    MCANtxMsg.fdf      = 1U; // Frame transmitted in CAN FD format.
    MCANtxMsg.efc      = 1U; // Store Tx events.
    MCANtxMsg.mm       = 0xAAU; // Message Marker.
}


void HAL_McanMsgInit(MCAN_TxBufElement *txMsg, uint32_t msgId)
{
    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((msgId & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
    txMsg->dlc = MCAN_DATA_SIZE_8BYTES; /* Payload size */
    txMsg->fdf = FALSE; /* CAN FD Frame Format */
    txMsg->brs = FALSE; /* Bit Rate Switch */
    txMsg->xtd = FALSE; /* Extended id not configured */

    return;
}

void HAL_CanMsgInit()
{
    uint32_t i;

    for (i = 0U; i < 12; i++)
    {
        HAL_McanMsgInit(&txMsgs[i], txMsgIds[i]);
    }
}

void HAL_SendCanMsg(uint16_t msgNum)
{
    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, msgNum, &txMsgs[msgNum]);

    /* Add request for transmission, This function will trigger transmission */
    MCAN_txBufAddReq(gMcanBaseAddr, msgNum);
}

void HAL_CanTxMsgPacking(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    uint8_t *data;
    uint16_t i;

    //
    //  CAN ID 0xFC
    //
    TX_0xFC_Message.speedSet_Hz = objMtr->speedRef_Hz;
    TX_0xFC_Message.MotorFault = objMtr->faultMtrUse.all;

    data = (uint8_t *)&TX_0xFC_Message;
    for(i=0;i<8;i++)
    {
        txMsgs[0].data[7-i] = data[i];
    }

}

void HAL_CanRxMsgMapping_0xC0(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    uint8_t *data = (uint8_t *)&RX_0xC0_Message;
    uint16_t i;

    for(i=0;i<8;i++)
    {
        data[i] = rxMsg.data[7-i];
    }

    if((RX_0xC0_Message.flagEnableCmd == TRUE) && (objMtr->faultMtrUse.all == 0))
    {
        objMtr->speedRef_Hz = RX_0xC0_Message.speedSet_Hz;
        objMtr->flagEnableRunAndIdentify = RX_0xC0_Message.flagCmdRun;
    }

}

#endif // CMD_CAN
//
//-- end of this file ----------------------------------------------------------
//
