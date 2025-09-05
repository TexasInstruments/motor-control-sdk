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
/**
 *  \file   epwm.c
 *
 *  \brief  Configure EPWM for
 *                  EPWM IP version 0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

 #include <math.h>
 #include <drivers/epwm.h>
 #include <drivers/hw_include/hw_types.h>
 #include "app_epwm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
 
/**
 * @brief Configure EPWM
 *
 * @param[in] config   Pointer to configuration parameters
 * @param[out] pPeriod Pointer to store period
 * @param[out] pDuty   Pointer to store duty cycle
 */
void App_epwmConfig(AppEPwmCfg_t *config)
{
    if (config == NULL) {
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
    if (config->cfgTbSyncIn == FALSE) {
        EPWM_tbSyncDisable(baseAddr);
    }
    else {
        EPWM_tbSyncEnable(baseAddr, config->tbPhsValue,
                           config->tbSyncInCounterDir);
    }
    /* Configure TB Sync Out Mode */
    if (config->cfgTbSyncOut == FALSE) {
        EPWM_tbSetSyncOutMode(baseAddr, EPWM_TB_SYNC_OUT_EVT_DISABLE );
    }
    else {

        EPWM_tbSetSyncOutMode(baseAddr, config->tbSyncOutMode);
    }

    /* Set the Time Base Counter Emulation Mode to Free Run */
    EPWM_tbSetEmulationMode(baseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /*
     *  Compute period and duty cycle
     */
    uint32_t period = tbFreq / outFreq;
    if (tbCounterDir == EPWM_TB_COUNTER_DIR_UP_DOWN) {
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

    if (config->cfgDb == TRUE) {
        /* Configure Dead Band Submodule */
        EPWM_deadbandCfg(baseAddr, &config->dbCfg);
    }
    else {
        /* Configure Dead Band Submodule */
        EPWM_deadbandBypass(baseAddr);
    }

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(baseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(baseAddr, EPWM_TZ_EVENT_ONE_SHOT,
                             0U);
    EPWM_tzTripEventDisable(baseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE,
                             0U);

    if (config->cfgEt == TRUE) {
        /* Configure event trigger Submodule */
        EPWM_etIntrCfg(baseAddr, config->intSel, config->intPrd);
        EPWM_etIntrEnable(baseAddr);
    }   
    return;
}
