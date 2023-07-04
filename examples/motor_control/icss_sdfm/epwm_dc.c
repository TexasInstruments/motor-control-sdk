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

#include <stdint.h>
#include <math.h>
#include <drivers/epwm.h>
#include "epwm_drv_aux.h"
#include "epwm_mod.h"
#include "epwm_dc.h"

Epwm_Handle epwmInit(
    EPwmCfgPrms_t *pEpwmCfgPrms,
    EPwmObj_t *pEpwmObj
)
{
    Epwm_Handle hEpwm;              /* EPWM handle */
    uint32_t epwmBaseAddr;          /* EPWM base address */
    uint32_t epwmOutChEn;           /* EPWM output channel (A/B) enable bit mask */
    uint32_t epwmTbFreq;            /* EPWM time base clock */
    uint32_t epwmOutFreq;           /* EPWM output frequency */
    uint32_t epwmTbCounterDir;      /* EPWM TB counter direction */
    uint32_t epwmPrdVal;
    uint32_t epwmCmpAVal, epwmCmpBVal;

    /* Get configuration parameters */
    epwmBaseAddr = pEpwmCfgPrms->epwmBaseAddr;
    epwmOutChEn = pEpwmCfgPrms->epwmOutChEn;
    epwmTbFreq = pEpwmCfgPrms->epwmTbFreq;
    epwmOutFreq = pEpwmCfgPrms->epwmOutFreq;
    epwmTbCounterDir = pEpwmCfgPrms->epwmTbCounterDir;

    /* Configure Time Base submodule */
    writeTbClkDiv(epwmBaseAddr, pEpwmCfgPrms->hspClkDiv, pEpwmCfgPrms->clkDiv);
    tbPwmFreqCfg(epwmBaseAddr, epwmTbFreq, epwmOutFreq,
        epwmTbCounterDir, EPWM_SHADOW_REG_CTRL_ENABLE, &epwmPrdVal);

    /* Configure TB Sync In Mode */
    if (pEpwmCfgPrms->cfgTbSyncIn == FALSE) {
        EPWM_tbSyncDisable(epwmBaseAddr);
    }
    else {
        EPWM_tbSyncEnable(epwmBaseAddr, pEpwmCfgPrms->tbPhsValue, pEpwmCfgPrms->tbSyncInCounterDir);
    }

    /* Configure TB Sync Out Mode */
    if (pEpwmCfgPrms->cfgTbSyncOut == FALSE) {
        EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_DISABLE );
    }
    else {
        EPWM_tbSetSyncOutMode(epwmBaseAddr, pEpwmCfgPrms->tbSyncOutMode);
    }

    /* Configure emulation mode */
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    if ((epwmOutChEn >> 0) & 0x1) {
        /*
         *  COMPA value - this determines the duty cycle
         *  COMPA = (PRD - ((dutycycle * PRD) / 100)
         */
        epwmCmpAVal = (epwmPrdVal - ((pEpwmCfgPrms->epwmDutyCycle[EPWM_OUTPUT_CH_A] * epwmPrdVal) / 100U));
        //epwmCmpAVal = 1; // FL: force max duty cycle just before 100% DC to see where EPWM period occurs

        /* Configure counter compare submodule */
        EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
            epwmCmpAVal, EPWM_SHADOW_REG_CTRL_ENABLE,
                EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
        /* Configure Action Qualifier Submodule */
        EPWM_aqActionOnOutputCfg(epwmBaseAddr, EPWM_OUTPUT_CH_A,
            &pEpwmCfgPrms->aqCfg[EPWM_OUTPUT_CH_A]);
    }

    if ((epwmOutChEn >> 1) & 0x1) {
        /*
         *  COMPB value - this determines the duty cycle
         *  COMPB = (PRD - ((dutycycle * PRD) / 100)
         */
        epwmCmpBVal = (epwmPrdVal - ((pEpwmCfgPrms->epwmDutyCycle[EPWM_OUTPUT_CH_B] * epwmPrdVal) / 100U));

        /* Configure counter compare submodule */
        EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B,
            epwmCmpBVal, EPWM_SHADOW_REG_CTRL_ENABLE,
                EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
        /* Configure Action Qualifier Submodule */
        EPWM_aqActionOnOutputCfg(epwmBaseAddr, EPWM_OUTPUT_CH_B,
            &pEpwmCfgPrms->aqCfg[EPWM_OUTPUT_CH_B]);
    }

    if (pEpwmCfgPrms->cfgDb == TRUE) {
        /* Configure Dead Band Submodule */
        EPWM_deadbandCfg(epwmBaseAddr, &pEpwmCfgPrms->dbCfg);
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

    if (pEpwmCfgPrms->cfgEt == TRUE) {
        /* Configure event trigger Submodule */
        EPWM_etIntrCfg(epwmBaseAddr, pEpwmCfgPrms->intSel,
            pEpwmCfgPrms->intPrd);
        EPWM_etIntrEnable(epwmBaseAddr);
    }

    /* Init PWM object */
    hEpwm = (Epwm_Handle)pEpwmObj;
    hEpwm->epwmId = pEpwmCfgPrms->epwmId;
    hEpwm->epwmBaseAddr = pEpwmCfgPrms->epwmBaseAddr;
    hEpwm->epwmOutChEn = pEpwmCfgPrms->epwmOutChEn;
    hEpwm->hspClkDiv = pEpwmCfgPrms->hspClkDiv;
    hEpwm->clkDiv = pEpwmCfgPrms->clkDiv;
    hEpwm->epwmTbFreq = pEpwmCfgPrms->epwmTbFreq;
    hEpwm->epwmOutFreq = pEpwmCfgPrms->epwmOutFreq;
    hEpwm->epwmPrdVal = epwmPrdVal;
    hEpwm->toggleEpwmPrd = FALSE;
    hEpwm->toggleEpwmPrdState = 0;
    hEpwm->epwmPrdValL = 0;
    hEpwm->epwmPrdValU = 0;
    if ((epwmOutChEn >> 0) & 0x1) {
        hEpwm->epwmDutyCycle[EPWM_OUTPUT_CH_A] = pEpwmCfgPrms->epwmDutyCycle[EPWM_OUTPUT_CH_A];
        hEpwm->cmpAVal = epwmCmpAVal;
        hEpwm->cmpANzToZ = FALSE;
        hEpwm->cmpAZToNz = FALSE;
    }
    if ((epwmOutChEn >> 1) & 0x1) {
        hEpwm->epwmDutyCycle[EPWM_OUTPUT_CH_B] = pEpwmCfgPrms->epwmDutyCycle[EPWM_OUTPUT_CH_B];
        hEpwm->cmpBVal = epwmCmpBVal;
        hEpwm->cmpBNzToZ = FALSE;
        hEpwm->cmpBZToNz = FALSE;
    }

    return hEpwm;
}

/* Update EPWM period */
int32_t epwmUpdatePrd(
    Epwm_Handle hEpwm,
    uint32_t epwmOutFreqSet
)
{
    float epwmPrdVal_f;
    uint32_t epwmPrdVal;
    uint32_t rem;

    /* Check for EPWM period toggle */
    if (hEpwm->toggleEpwmPrd == TRUE) {
        hEpwm->epwmPrdVal = (hEpwm->toggleEpwmPrdState == 0) ? hEpwm->epwmPrdValL : hEpwm->epwmPrdValU;
        hEpwm->toggleEpwmPrdState ^= 0x1;

        /* Write next period count */
        writeTbPrd(hEpwm->epwmBaseAddr, hEpwm->epwmPrdVal);
    }

    /* Check for PWM frequency change */
    if (hEpwm->epwmOutFreq != epwmOutFreqSet) {
        epwmPrdVal_f = (float)hEpwm->epwmTbFreq / epwmOutFreqSet;
        epwmPrdVal_f = roundf(epwmPrdVal_f);

        epwmPrdVal = (uint32_t)epwmPrdVal_f;
        rem = epwmPrdVal - epwmPrdVal/2*2;
        if (rem == 0) {
            /* Period is divisible by 2,
               alternating period not employed */
            hEpwm->toggleEpwmPrd = FALSE;
            hEpwm->toggleEpwmPrdState = 0;
            hEpwm->epwmPrdValL = 0;
            hEpwm->epwmPrdValU = 0;
            hEpwm->epwmPrdVal = epwmPrdVal/2;
        } else {
            /* Period is not divisible by 2,
               alternating period employed to provide correct average EPWM frequency:
                EPWM period 2*n     : TBPRD <- 'Lower' period
                EPWM period 2*n+1   : TBPRD <- 'Upper' period
            */
            hEpwm->toggleEpwmPrd = TRUE;
            hEpwm->toggleEpwmPrdState = 1;
            hEpwm->epwmPrdValL = epwmPrdVal/2;
            hEpwm->epwmPrdValU = epwmPrdVal/2+1;
            hEpwm->epwmPrdVal = hEpwm->epwmPrdValL;
        }

        /* Write next period count */
        writeTbPrd(hEpwm->epwmBaseAddr, hEpwm->epwmPrdVal);

        hEpwm->epwmOutFreq = epwmOutFreqSet;
    }

    return EPWM_DC_SOK;
}

/* Update EPWM A/B outputs */
int32_t epwmUpdateOut(
    Epwm_Handle hEpwm,
    float VrefA,
    float VrefB
)
{
    float dcVal;        /* EPWM duty cycle value */
    uint16_t cmpVal;    /* EPWM CMP value */

    if ((hEpwm->epwmOutChEn >> 0) & 0x1) {
        /* Compute next Duty Cycle and CMP values */
        computeCmpx(VrefA, hEpwm->epwmPrdVal, &dcVal, &cmpVal);

        /* Write next CMPA value */
        writeCmpA(hEpwm->epwmBaseAddr, cmpVal);

        /* EPWM 100% Duty Cycle */
        /* Handle transition to 100% Duty Cycle */
        if (hEpwm->cmpANzToZ == TRUE) {
            /* restore original AQ */
            cfgOutChAAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_DONOTHING);
            hEpwm->cmpANzToZ = FALSE;
        }
        if ((hEpwm->cmpAVal != 0) && (cmpVal == 0)) {
            /* set AQ to set for next period */
            cfgOutChAAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_HIGH);
            hEpwm->cmpANzToZ = TRUE;
        }

        /* Handle transition from 100% Duty Cycle */
        if (hEpwm->cmpAZToNz == TRUE) {
            /* restore original AQ */
            cfgOutChAAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_DONOTHING);
            hEpwm->cmpAZToNz = FALSE;
        }
        if ((hEpwm->cmpAVal == 0) && (cmpVal != 0)) {
            /* set AQ to clear for next period */
            cfgOutChAAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_LOW);
            hEpwm->cmpAZToNz = TRUE;
        }
        hEpwm->cmpAVal = cmpVal;
    }

    if ((hEpwm->epwmOutChEn >> 1) & 0x1) {
        /* Compute next Duty Cycle and CMP values */
        computeCmpx(VrefB, hEpwm->epwmPrdVal, &dcVal, &cmpVal);

        /* Write next CMPB value */
        writeCmpB(hEpwm->epwmBaseAddr, cmpVal);

        /* EPWM 100% Duty Cycle */
        /* Handle transition to 100% Duty Cycle */
        if (hEpwm->cmpBNzToZ == TRUE) {
            /* restore original AQ */
            cfgOutChBAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_DONOTHING);
            hEpwm->cmpBNzToZ = FALSE;
        }
        if ((hEpwm->cmpBVal != 0) && (cmpVal == 0)) {
            /* set AQ to set for next period */
            cfgOutChBAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_HIGH);
            hEpwm->cmpBNzToZ = TRUE;
        }

        /* Handle transition from 100% Duty Cycle */
        if (hEpwm->cmpBZToNz == TRUE) {
            /* restore original AQ */
            cfgOutChBAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_DONOTHING);
            hEpwm->cmpBZToNz = FALSE;
        }
        if ((hEpwm->cmpBVal == 0) && (cmpVal != 0)) {
            /* set AQ to clear for next period */
            cfgOutChBAqZero(hEpwm->epwmBaseAddr, EPWM_AQ_ACTION_LOW);
            hEpwm->cmpBZToNz = TRUE;
        }
        hEpwm->cmpBVal = cmpVal;
    }

    return EPWM_DC_SOK;
}
