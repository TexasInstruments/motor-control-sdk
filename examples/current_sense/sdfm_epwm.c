/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 *    distribution and/or other materials provided with the
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

/**
 *  \file   sdfm_epwm.c
 *
 *  \brief  EPWM configuration implementation for SDFM current sense examples.
 *          This implementation follows the same pattern as the universal single
 *          chip servo project, using SDK driver functions directly.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/hw_include/hw_types.h>
#include "sdfm_epwm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* Configuration macros are now defined in sdfm_epwm.h */

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1)
/* EPWM0 IRQ handler forward declaration */
void epwmIrqHandler(void *handle);
HwiP_Object gEpwm0HwiObject;         /* EPWM0 HWI */
/* EPWM global variables */
uint32_t gEpwm0BaseAddr = 0;    /* EPWM0 base address */
volatile uint32_t gEpwmIsrCnt = 0;    /* EPWM0 IRQ count */
SdfmEpwmCfg_t gEpwmCfgPrms = {0};
#endif

#if APP_EPWM1_ENABLE
/* EPWM1 IRQ handler forward declaration */
void epwmIrqHandler1(void *handle);
HwiP_Object gEpwm1HwiObject;         /* EPWM1 HWI */
uint32_t gEpwm1BaseAddr = 0;    /* EPWM1 base address */
SdfmEpwmCfg_t gEpwm1CfgPrms = {0};
volatile uint32_t gEpwmIsrCnt1 = 0;
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Configure EPWM module
 *
 *  This function configures an EPWM module with the specified parameters.
 *  It uses the SDK driver functions to set up timebase, synchronization,
 *  action qualifier, deadband, and event trigger submodules.
 *
 *  \param config [IN] Pointer to EPWM configuration parameters
 */
void sdfmEpwmConfig(SdfmEpwmCfg_t *config)
{
    if (config == NULL)
    {
        return;
    }

    uint32_t baseAddr = config->epwmBaseAddr;
    uint32_t ch = config->epwmCh;
    uint32_t funcClk = config->epwmFuncClk;
    uint32_t tbFreq = config->epwmTbFreq;
    uint32_t outFreq = config->epwmOutFreq;
    uint32_t dutyCycle = config->epwmDutyCycle;
    uint32_t tbCounterDir = config->epwmTbCounterDir;

    /* Configure Time Base submodule */
    EPWM_tbTimebaseClkCfg(baseAddr, tbFreq, funcClk);
    EPWM_tbPwmFreqCfg(baseAddr, tbFreq, outFreq, tbCounterDir,
                      EPWM_SHADOW_REG_CTRL_ENABLE);

    /* Configure TB Sync In Mode */
    if (config->cfgTbSyncIn == FALSE)
    {
        EPWM_tbSyncDisable(baseAddr);
    }
    else
    {
        EPWM_tbSyncEnable(baseAddr, config->tbPhsValue,
                          config->tbSyncInCounterDir);
    }

    /* Configure TB Sync Out Mode */
    if (config->cfgTbSyncOut == FALSE)
    {
        EPWM_tbSetSyncOutMode(baseAddr, EPWM_TB_SYNC_OUT_EVT_DISABLE);
    }
    else
    {
        EPWM_tbSetSyncOutMode(baseAddr, config->tbSyncOutMode);
    }

    /* Set the Time Base Counter Emulation Mode to Free Run */
    EPWM_tbSetEmulationMode(baseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /*
     *  Compute period and duty cycle
     */
    uint32_t period = tbFreq / outFreq;
    if (tbCounterDir == EPWM_TB_COUNTER_DIR_UP_DOWN)
    {
        period /= 2U;
    }

    uint32_t duty = period - ((dutyCycle * period) / 100U);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(baseAddr, EPWM_CC_CMP_A,
                              duty, EPWM_SHADOW_REG_CTRL_ENABLE,
                              EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD,
                              TRUE);
    EPWM_counterComparatorCfg(baseAddr, EPWM_CC_CMP_B,
                              duty, EPWM_SHADOW_REG_CTRL_ENABLE,
                              EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD,
                              TRUE);

    /* Configure Action Qualifier Submodule */
    EPWM_aqActionOnOutputCfg(baseAddr, ch, &config->aqCfg);

    if (config->cfgDb == TRUE)
    {
        /* Configure Dead Band Submodule */
        EPWM_deadbandCfg(baseAddr, &config->dbCfg);
    }
    else
    {
        /* Bypass Dead Band Submodule */
        EPWM_deadbandBypass(baseAddr);
    }

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(baseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(baseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(baseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    if (config->cfgEt == TRUE)
    {
        /* Configure event trigger Submodule */
        EPWM_etIntrCfg(baseAddr, config->intSel, config->intPrd);
        EPWM_etIntrEnable(baseAddr);
    }
}

/**
 *  \brief Initialize EPWM module
 *
 *  This function initializes the EPWM hardware, registers interrupt handler,
 *  and configures the EPWM module based on SysConfig parameters.
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t sdfmEpwmInit(void)
{
#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1)
    int32_t status;
    HwiP_Params hwiPrms;

    /* Initialize EPWM0 base address, perform address translation */
    gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);

    /* Register & enable EPWM0 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &epwmIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gEpwm0HwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure EPWM0 */
    gEpwmCfgPrms.epwmBaseAddr = gEpwm0BaseAddr;
    gEpwmCfgPrms.epwmTbFreq = APP_EPWM0_TB_FREQ;
    gEpwmCfgPrms.epwmOutFreq = APP_EPWM0_OUTPUT_FREQ;
    gEpwmCfgPrms.epwmDutyCycle = APP_EPWM0_DUTY_CYCLE;
    gEpwmCfgPrms.epwmTbCounterDir = APP_EPWM0_TB_COUNTER_DIR;
    gEpwmCfgPrms.cfgTbSyncIn = TRUE;
    gEpwmCfgPrms.tbPhsValue = 0;
    gEpwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    gEpwmCfgPrms.cfgTbSyncOut = TRUE;
    gEpwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
    gEpwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    gEpwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
    gEpwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwmCfgPrms.cfgDb = FALSE;
    gEpwmCfgPrms.cfgEt = TRUE;
    gEpwmCfgPrms.intSel = EPWM_ET_INTR_EVT_CNT_EQ_ZRO;
    gEpwmCfgPrms.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    gEpwmCfgPrms.epwmCh = EPWM_OUTPUT_CH_A;
    gEpwmCfgPrms.epwmFuncClk = APP_EPWM_FCLK;

    sdfmEpwmConfig(&gEpwmCfgPrms);

#if APP_EPWM1_ENABLE
    /* EPWM for SDFM clock generation */
    /* Initialize EPWM base address, perform address translation */
    gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM1_BASE_ADDR);

    HwiP_Params HwiPrms1;

    /* Register & enable EPWM1 interrupt */
    HwiP_Params_init(&hwiPrms1);
    hwiPrms1.intNum      = CONFIG_EPWM1_INTR;
    hwiPrms1.callback    = &epwmIrqHandler1;
    hwiPrms1.args        = 0;
    hwiPrms1.isPulse     = CONFIG_EPWM1_INTR_IS_PULSE;
    hwiPrms1.isFIQ       = FALSE;
    status              = HwiP_construct(&gEpwm1HwiObject, &hwiPrms1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure EPWM1 */
    gEpwm1CfgPrms.epwmBaseAddr = gEpwm1BaseAddr;
    gEpwm1CfgPrms.epwmTbFreq = APP_EPWM1_TB_FREQ;
    gEpwm1CfgPrms.epwmOutFreq = APP_EPWM1_OUTPUT_FREQ;
    gEpwm1CfgPrms.epwmDutyCycle = APP_EPWM1_DUTY_CYCLE;
    gEpwm1CfgPrms.epwmTbCounterDir = APP_EPWM1_TB_COUNTER_DIR;
    gEpwm1CfgPrms.cfgTbSyncIn = FALSE;
    gEpwm1CfgPrms.tbPhsValue = 0;
    gEpwm1CfgPrms.cfgTbSyncOut = FALSE;
    gEpwm1CfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
    gEpwm1CfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwm1CfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwm1CfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    gEpwm1CfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
    gEpwm1CfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwm1CfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    gEpwm1CfgPrms.cfgDb = FALSE;
    gEpwm1CfgPrms.cfgEt = FALSE;
    gEpwm1CfgPrms.intSel = EPWM_ET_INTR_EVT_CNT_EQ_ZRO;
    gEpwm1CfgPrms.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    gEpwm1CfgPrms.epwmCh = EPWM_OUTPUT_CH_A;
    gEpwm1CfgPrms.epwmFuncClk = APP_EPWM_FCLK;

    sdfmEpwmConfig(&gEpwm1CfgPrms);
#endif

    return SystemP_SUCCESS;
#else
    return SystemP_SUCCESS;
#endif
}

/**
 *  \brief De-initialize EPWM module
 *
 *  This function disables EPWM interrupts and destroys the hardware interrupt object.
 */
void sdfmEpwmDeinit(void)
{
#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1)
    /* Disable and clear interrupts for EPWM0 */
    EPWM_etIntrDisable(gEpwm0BaseAddr); /* Disable interrupts */
    EPWM_etIntrClear(gEpwm0BaseAddr);   /* Clear pending interrupts */

    /* Destroy EPWM0 HWI */
    HwiP_destruct(&gEpwm0HwiObject);
#endif
#if APP_EPWM1_ENABLE
    /* Disable and clear interrupts for EPWM1 */
    EPWM_etIntrDisable(gEpwm1BaseAddr); /* Disable interrupts */
    EPWM_etIntrClear(gEpwm1BaseAddr);   /* Clear pending interrupts */

    /* Destroy EPWM1 HWI */
    HwiP_destruct(&gEpwm1HwiObject);
#endif
}

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1)
/**
 *  \brief EPWM0 IRQ handler
 *
 *  This function is called when EPWM0 generates an interrupt.
 *  It clears the interrupt flag and increments the ISR counter.
 *
 *  \param args [IN] User arguments (unused)
 */
void epwmIrqHandler(void *args)
{
    volatile uint16_t status;

    /* debug, increment EPWM0 IRQ count */
    gEpwmIsrCnt++;

    status = EPWM_etIntrStatus(gEpwm0BaseAddr);
    if (status & EPWM_ETFLG_INT_MASK)
    {
        EPWM_etIntrClear(gEpwm0BaseAddr);
    }
}
#endif

#if APP_EPWM1_ENABLE
/**
 *  \brief EPWM1 IRQ handler
 *
 *  This function is called when EPWM1 generates an interrupt.
 *  It clears the interrupt flag and increments the ISR counter.
 *
 *  \param args [IN] User arguments (unused)
 */
void epwmIrqHandler1(void *args)
{
    volatile uint16_t status;

    /* debug, increment EPWM1 IRQ count */
    gEpwmIsrCnt1++;

    status = EPWM_etIntrStatus(gEpwm1BaseAddr);
    if (status & EPWM_ETFLG_INT_MASK)
    {
        EPWM_etIntrClear(gEpwm1BaseAddr);
    }
}
#endif
