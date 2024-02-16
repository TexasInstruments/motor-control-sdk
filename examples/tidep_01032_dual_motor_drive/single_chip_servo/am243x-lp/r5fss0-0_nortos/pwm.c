/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <math.h>
#include <drivers/epwm.h>
#include <drivers/hw_include/hw_types.h>
#include "pwm.h"

/* Compute Duty Cycle & CMPx given Vref & EPWM period */
__attribute__((always_inline)) inline void computeCmpx(
    float       Vref,
    uint32_t    epwmPrdVal,
    float       *pEpwmDutyCycle,
    uint16_t    *pEpwmCmpVal
)
{
    float dc_f;
    float cmp_f;
    uint16_t cmp;

    if (Vref > VREF_MAX) {
        /* 100% duty cycle */
        dc_f = 1.0;
        cmp = 0;
    }
    else if (Vref < VREF_MIN) {
        /* 0% duty cycle */
        dc_f = 0.0;
        cmp = epwmPrdVal;
    }
    else {
        /* compute Duty Cycle */
        dc_f = 0.5*(Vref + 1.0);

        /* compute CMPx */
        cmp_f = (1.0 - dc_f)*epwmPrdVal; /* up-down count */
        cmp = (uint16_t)cmp_f;
    }

    *pEpwmDutyCycle = dc_f;
    *pEpwmCmpVal = cmp;
}

void App_epwmConfig(
    AppEPwmCfg_t *pAppEPwmCfg,
    uint32_t *pEpwmPrdVal,
    uint32_t *pEpwmCmpAVal
)
{
    uint32_t epwmBaseAddr;      /* EPWM base address */
    uint32_t epwmCh;            /* EPWM output channel */
    uint32_t epwmFuncClk;       /* EPWM functional clock */
    uint32_t epwmTbFreq;        /* EPWM timebase clock */
    uint32_t epwmOutFreq;       /* EPWM output frequency */
    uint32_t epwmDutyCycle;     /* EPWM duty cycle */
    uint32_t epwmTbCounterDir;  /* EPWM TB counter direction */
    uint32_t epwmPrdVal;
    uint32_t epwmCmpAVal;

    /* Get configuration parameters */
    epwmBaseAddr = pAppEPwmCfg->epwmBaseAddr;
    epwmCh = pAppEPwmCfg->epwmCh;
    epwmFuncClk = pAppEPwmCfg->epwmFuncClk;
    epwmTbFreq = pAppEPwmCfg->epwmTbFreq;
    epwmOutFreq = pAppEPwmCfg->epwmOutFreq;
    epwmDutyCycle = pAppEPwmCfg->epwmDutyCycle;
    epwmTbCounterDir = pAppEPwmCfg->epwmTbCounterDir;

    /* Configure Time Base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, epwmTbFreq, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, epwmTbFreq, epwmOutFreq,
        epwmTbCounterDir, EPWM_SHADOW_REG_CTRL_ENABLE);

    /* Configure TB Sync In Mode */
    if (pAppEPwmCfg->cfgTbSyncIn == FALSE) {
        EPWM_tbSyncDisable(epwmBaseAddr);
    }
    else {
        EPWM_tbSyncEnable(epwmBaseAddr, pAppEPwmCfg->tbPhsValue, pAppEPwmCfg->tbSyncInCounterDir);
    }

    /* Configure TB Sync Out Mode */
    if (pAppEPwmCfg->cfgTbSyncOut == FALSE) {
        EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_DISABLE );
    }
    else {
        EPWM_tbSetSyncOutMode(epwmBaseAddr, pAppEPwmCfg->tbSyncOutMode);
    }

    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /*
     *  PRD value - this determines the period
     */
    /* PRD = (TBCLK/PWM FREQ) */
    epwmPrdVal = epwmTbFreq / epwmOutFreq;
    if (epwmTbCounterDir == EPWM_TB_COUNTER_DIR_UP_DOWN) {
        /*
         *  PRD = (TBCLK/PWM FREQ) / 2
         *  /2 is added because up&down counter is selected. So period is 2 times.
         */
        epwmPrdVal /= 2U;
    }

    /*
     *  COMPA value - this determines the duty cycle
     *  COMPA = (PRD - ((dutycycle * PRD) / 100)
     */
    epwmCmpAVal = (epwmPrdVal - ((epwmDutyCycle * epwmPrdVal) / 100U));

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
        epwmCmpAVal, EPWM_SHADOW_REG_CTRL_ENABLE,
        EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD, TRUE);
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B,
        epwmCmpAVal, EPWM_SHADOW_REG_CTRL_ENABLE,
        EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD, TRUE);

    /* Configure Action Qualifier Submodule */
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, epwmCh, &pAppEPwmCfg->aqCfg);

    if (pAppEPwmCfg->cfgDb == TRUE) {
        /* Configure Dead Band Submodule */
        EPWM_deadbandCfg(epwmBaseAddr, &pAppEPwmCfg->dbCfg);
    }
    else {
        /* Configure Dead Band Submodule */
        EPWM_deadbandBypass(epwmBaseAddr);
    }

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    if (pAppEPwmCfg->cfgEt == TRUE) {
        /* Configure event trigger Submodule */
        EPWM_etIntrCfg(epwmBaseAddr, pAppEPwmCfg->intSel,
            pAppEPwmCfg->intPrd);
        EPWM_etIntrEnable(epwmBaseAddr);
    }

    /* Set return values */
    *pEpwmPrdVal = epwmPrdVal;
    *pEpwmCmpAVal = epwmCmpAVal;
}
