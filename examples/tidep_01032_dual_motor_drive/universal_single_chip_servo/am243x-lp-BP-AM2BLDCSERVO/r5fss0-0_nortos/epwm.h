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

/* 
*
* \brief  header file to be included in all labs
*
*/
#ifndef EPWM_MAIN_H
#define EPWM_MAIN_H


/******************************************************************************
*
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
*
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
*
* \defgroup EPWM MAIN
* @{
*
******************************************************************************/

#include "math_types.h"
#include <math.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/epwm.h>

/*
 *  This header file defines the constants and data structures used
 *  in the EPWM file.
 */

/*
 *  Frequency of PWM output signal in Hz
 *
 *  4kHz
 *  8kHz
 *  15kHz
 *  20kHz
 *  30kHz
 *  50kHz
 *
 *  Initial frequency
 */
#define APP_EPWM_OUTPUT_FREQ_4K       (1U * 4000U)
#define APP_EPWM_OUTPUT_FREQ_8K       (1U * 8000U)
#define APP_EPWM_OUTPUT_FREQ_15K      (1U * 15000U)
#define APP_EPWM_OUTPUT_FREQ_20K      (1U * 20000U)
#define APP_EPWM_OUTPUT_FREQ_30K      (1U * 30000U)
#define APP_EPWM_OUTPUT_FREQ_50K      (1U * 50000U)

#define APP_EPWM_OUTPUT_FREQ          (APP_EPWM_OUTPUT_FREQ_15K)

/*
 *  EPWM configuration
 *
 *  This structure contains the configuration parameters
 *  for the EPWM module.
 */
typedef struct _AppEPwmCfg_t
{
    uint32_t    epwmBaseAddr;       /* EPWM base address */
    uint32_t    epwmCh;             /* EPWM output channel */
    uint32_t    epwmFuncClk;        /* EPWM functional clock */
    uint32_t    epwmTbFreq;         /* EPWM timebase clock */
    uint32_t    epwmOutFreq;        /* EPWM output frequency */
    uint32_t    epwmDutyCycle;      /* EPWM duty cycle */
    uint32_t    epwmTbCounterDir;   /* EPWM counter direction (Up, Down, Up/Down) */
    Bool        cfgTbSyncIn;        /* config TB sync in flag (true/false) */
    uint32_t    tbPhsValue;         /* cfgTbSyncIn==TRUE: timer phase value to load on Sync In event */
    uint32_t    tbSyncInCounterDir; /* cfgTbSyncIn==TRUE: counter direction on Sync In event */
    Bool        cfgTbSyncOut;       /* config TB sync output flag (true/false) */
    uint32_t    tbSyncOutMode;      /* cfgTbSyncOut==TRUE: Sync Out mode */
    EPWM_AqActionCfg aqCfg;       /* Action Qualifier config */
    Bool        cfgDb;              /* config DB flag (true/false) */
    EPWM_DeadbandCfg dbCfg;       /* Deadband config */
    Bool        cfgEt;              /* config ET module */
    uint32_t    intSel;             /* ET interrupt select */
    uint32_t    intPrd;             /* ET interrupt period */
} AppEPwmCfg_t;


/**
 * \brief Counter Compare Module
 *
 * This enum defines the valid values for the counter compare module.
 *
 * \sa EPWM_getCounterCompareValue()
 */
typedef enum
{
    EPWM_COUNTER_COMPARE_A = 0, /**< counter compare A */
    EPWM_COUNTER_COMPARE_B = 2, /**< counter compare B */
} EPWM_CounterCompareModule;

/**
 * @brief Configure EPWM
 *
 * @param[in] config   Pointer to configuration parameters
 * @param[out] pPeriod Pointer to store period
 * @param[out] pDuty   Pointer to store duty cycle
 */
void App_epwmConfig(AppEPwmCfg_t *config);

/**
 * \brief Get the PWM period count.
 *
 * This function gets the period of the PWM count.
 *
 * \param base is the base address of the EPWM module.
 *
 * \return The period count value.
 */
static inline uint16_t
EPWM_getTimeBasePeriod(uint32_t base)
{
    // Read from TBPRD bit
    return(HW_RD_REG16(base + PWMSS_EPWM_TBPRD));
}

/**
 * \brief Get counter compare values.
 *
 * This function gets the counter compare value for counter compare registers.
 *
 * \param base is the base address of the EPWM module.
 * \param compModule is the Counter Compare value module.
 *
 * \return The counter compare count value.
 */
static inline uint16_t
EPWM_getCounterCompareValue(uint32_t base, EPWM_CounterCompareModule compModule)
{
    uint32_t registerOffset;
    uint16_t compCount;

    // Get the register offset for the Counter compare
    registerOffset = base + PWMSS_EPWM_CMPA + (uint16_t)compModule;

    // Read from the counter compare registers.
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
    {
        // Read COMPA or COMPB bits
        compCount = (uint16_t)((HW_RD_REG32(registerOffset) &
                    (uint32_t)PWMSS_EPWM_CMPA_MASK) >>
                    PWMSS_EPWM_CMPA_SHIFT);
    }
    else
    {
        // Read COMPC or COMPD bits
        compCount = HW_RD_REG16(registerOffset);
    }
    return(compCount);
}

/**
 * \brief Set counter compare values.
 *
 * This function sets the counter compare value for counter compare registers.
 *
 * \param base is the base address of the EPWM module.
 * \param compModule is the Counter Compare value module.
 * \param compCount is the counter compare count value.
 *
 * \return None.
 */
static inline void
EPWM_setCounterCompareValue(uint32_t base, EPWM_CounterCompareModule compModule,
                            uint16_t compCount)
{
    uint32_t registerOffset;

    // Get the register offset for the Counter compare
    registerOffset = base + PWMSS_EPWM_CMPA + (uint16_t)compModule;

    // Write to the counter compare registers.
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
    {
        // Write to COMPA or COMPB bits
        HW_WR_REG16(registerOffset, compCount);
    }
    else
    {
        // Write to COMPC or COMPD bits
        HW_WR_REG16(registerOffset, compCount);
    }
}

#ifdef __cplusplus
}
#endif

#endif // end of EPWM_MAIN_H definition
