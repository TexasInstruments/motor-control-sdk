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

#ifndef _EPWM_DC_H_
#define _EPWM_DC_H_

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/epwm.h>

/* Status return values */
#define EPWM_DC_SOK         ( 0 )

#define EPWM_ID_0           ( 0 )
#define EPWM_ID_1           ( 1 )
#define EPWM_ID_2           ( 2 )

#define EPWM_NUM_OUT_CH     ( EPWM_OUTPUT_CH_MAX + 1 )

/* EPWM configuration parameters */
typedef struct _EPwmCfgPrms_t
{
    uint32_t    epwmId;             /* EPWM ID */
    uint32_t    epwmBaseAddr;       /* EPWM base address */
    uint32_t    epwmOutChEn;        /* EPWM output channel (A/B) enable bit mask */
    uint32_t    hspClkDiv;          /* EPWM High-Speed Time-base Clock Prescale Bits */
    uint32_t    clkDiv;             /* EPWM Time-base Clock Prescale Bits */
    uint32_t    epwmTbFreq;         /* EPWM timebase clock */
    uint32_t    epwmOutFreq;        /* EPWM output frequency */
    /* EPWM duty cycle */
    uint32_t    epwmDutyCycle[EPWM_NUM_OUT_CH];
    uint32_t    epwmTbCounterDir;   /* EPWM counter direction (Up, Down, Up/Down) */

    /* TB sync in config */
    Bool        cfgTbSyncIn;        /* config TB sync in flag (true/false) */
    uint32_t 	tbPhsValue;         /* cfgTbSyncIn==TRUE: timer phase value to load on Sync In event */
    uint32_t 	tbSyncInCounterDir; /* cfgTbSyncIn==TRUE: counter direction on Sync In event */

    /* TB sync out config */
    Bool        cfgTbSyncOut;       /* config TB sync output flag (true/false) */
    uint32_t 	tbSyncOutMode;      /* cfgTbSyncOut==TRUE: Sync Out mode */

    /* AQ config */
    EPWM_AqActionCfg aqCfg[EPWM_NUM_OUT_CH];

    /* DB config */
    Bool        cfgDb;              /* config DB flag (true/false) */
    EPWM_DeadbandCfg dbCfg;       /* Deadband config */

    /* ET config */
    Bool        cfgEt;              /* config ET module */
    uint32_t    intSel;             /* ET interrupt select */
    uint32_t    intPrd;             /* ET interrupt period */
} EPwmCfgPrms_t;

/* EPWM object */
typedef struct _EPwmObj_t
{
    uint32_t    epwmId;             /* EPWM ID */
    uint32_t    epwmBaseAddr;       /* EPWM base address */
    uint32_t    epwmOutChEn;        /* EPWM output channel (A/B) enable bit mask */
    uint32_t    hspClkDiv;          /* EPWM High-Speed Time-base Clock Prescale Bits */
    uint32_t    clkDiv;             /* EPWM Time-base Clock Prescale Bits */
    uint32_t    epwmTbFreq;         /* EPWM timebase clock */
    uint32_t    epwmOutFreq;        /* EPWM output frequency */
    /* EPWM duty cycle */
    uint32_t    epwmDutyCycle[EPWM_NUM_OUT_CH];

    uint32_t    epwmPrdVal;         /* EPWM period value */
    
    /* For handling up-down count alternating period
       when period isn't divisible by 2 */
    Bool        toggleEpwmPrd;      /* Flag for EPWM in alternating period mode */
    uint8_t     toggleEpwmPrdState; /* Alternating period state:
                                       'Lower' or 'Upper' period written on alternate ISRs */
    uint32_t    epwmPrdValL;        /* 'Lower' EPWM period value written in 'Lower' state */
    uint32_t    epwmPrdValU;        /* 'Upper' EPWM period value written in 'Upper' state */

    /* For handling ChA 100% Duty Cycle */
    uint32_t    cmpAVal;            /* Current CMPA value */
    Bool        cmpANzToZ;          /* Flag for EPWM transition CMPA!=0 to CMPA=0 */
    Bool        cmpAZToNz;          /* Flag for EPWM transition CMPA=0 to CMPA!=0 */

    /* For handling ChB 100% Duty Cycle */
    uint32_t    cmpBVal;            /* Current CMPB value */
    Bool        cmpBNzToZ;          /* Flag for EPWM transition CMPB!=0 to CMPB=0 */
    Bool        cmpBZToNz;          /* Flag for EPWM transition CMPB=0 to CMPB!=0 */
} EPwmObj_t;

/* EPWM Handle */
typedef EPwmObj_t * Epwm_Handle;

/* Initialize EPWM */
Epwm_Handle epwmInit(
    EPwmCfgPrms_t *pEpwmCfgPrms,
    EPwmObj_t *pEpwmObj
);

/* Update EPWM period */
int32_t epwmUpdatePrd(
    Epwm_Handle hEpwm,
    uint32_t epwmOutFreqSet
);

/* Update EPWM A/B outputs */
int32_t epwmUpdateOut(
    Epwm_Handle hEpwm,
    float VrefA,
    float VrefB
);

#endif /* _EPWM_DC_H_ */
