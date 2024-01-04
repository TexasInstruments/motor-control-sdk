/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef _PWM_H_
#define _PWM_H_

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/epwm.h>
#include "epwm_drv_aux.h"

/* Switching control for EPWM output frequency */
#define SET_EPWM_OUT_FREQ8K_THR     ( 4000 )
#define SET_EPWM_OUT_FREQ16K_THR    ( SET_EPWM_OUT_FREQ8K_THR + 8000 )
#define SET_EPWM_OUT_FREQ4K_THR     ( SET_EPWM_OUT_FREQ16K_THR + 16000 )

/* Frequency of PWM output signal in Hz */
#define APP_EPWM_OUTPUT_FREQ_4K         ( 1U * 4000U )
#define APP_EPWM_OUTPUT_FREQ_8K         ( 1U * 8000U )
#define APP_EPWM_OUTPUT_FREQ_20K        ( 1U * 20000U )
#define APP_EPWM_OUTPUT_FREQ_50K        ( 1U * 50000U )
#define APP_EPWM_OUTPUT_FREQ            ( APP_EPWM_OUTPUT_FREQ_50K ) /* init freq */
/* Deadband RED/FED timer counts */
#define APP_EPWM_DB_RED_COUNT           ( 5 ) /* 20 nsec @ 250 MHz  */
#define APP_EPWM_DB_FED_COUNT           ( 5 ) /* 20 nsec @ 250 MHz */

/* Min / max output amplitude.
   Waveform amplitude values beyond these thresholds are saturated. */
#define VREF_MAX                        (  1.0f )
#define VREF_MIN                        ( -1.0f )

/* Sinusoid parameters */
#define SIN_FREQ                        ( 6 )      /* sinusoid frequency */
#define SIN_AMP                         ( 0.0 )     /* sinusoid amplitude */

/* EPWM configuration */
typedef struct _AppEPwmCfg_t
{
    uint32_t    epwmBaseAddr;       /* EPWM base address */
    uint32_t    epwmCh;             /* EPWM output channel */
    uint32_t    epwmFuncClk;        /* EPWM functional clock */
    uint32_t    epwmTbFreq;         /* EPWM timebase clock */
    uint32_t    epwmOutFreq;        /* EPWM output frequency */
    uint32_t    epwmDutyCycle;      /* EPWM duty cycle */
    uint32_t    epwmTbCounterDir;   /* EPWM counter direction (Up, Down, Up/Down) */
    /* TB sync in config */
    Bool        cfgTbSyncIn;        /* config TB sync in flag (true/false) */
    uint32_t    tbPhsValue;         /* cfgTbSyncIn==TRUE: timer phase value to load on Sync In event */
    uint32_t    tbSyncInCounterDir; /* cfgTbSyncIn==TRUE: counter direction on Sync In event */
    /* TB sync out config */
    Bool        cfgTbSyncOut;       /* config TB sync output flag (true/false) */
    uint32_t    tbSyncOutMode;      /* cfgTbSyncOut==TRUE: Sync Out mode */
    /* AQ config */
    EPWM_AqActionCfg aqCfg;       /* Action Qualifier config */
    /* DB config */
    Bool        cfgDb;              /* config DB flag (true/false) */
    EPWM_DeadbandCfg dbCfg;       /* Deadband config */
    /* ET config */
    Bool        cfgEt;              /* config ET module */
    uint32_t    intSel;             /* ET interrupt select */
    uint32_t    intPrd;             /* ET interrupt period */
} AppEPwmCfg_t;

/* Application specific initial PWM configuration */
void App_epwmConfig(
    AppEPwmCfg_t *pAppEPwmCfg,
    uint32_t *pEpwmPrdVal,
    uint32_t *pEpwmCmpAVal
);

/* Compute Duty Cycle & CMPx given Vref & EPWM period */
void computeCmpx(
    float       Vref,
    uint32_t    epwmPrdVal,
    float       *pEpwmDutyCycle,
    uint16_t    *pEpwmCmpVal
);

#endif /* _PWM_H_ */
