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
 *  \file   sdfm_epwm.h
 *
 *  \brief  EPWM configuration API for SDFM current sense examples.
 *          This module provides EPWM configuration following the same pattern
 *          as the universal single chip servo project.
 */

#ifndef SDFM_EPWM_H
#define SDFM_EPWM_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/epwm.h>
#include <drivers/epwm/v0/hw_pwmss_epwm.h>
#include "ti_drivers_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief EPWM0 configuration macros
 *
 *  These macros define the default configuration for EPWM0 used in SDFM examples.
 *  EPWM0 is synchronized with IEP for precise timing control.
 */
#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1)
/* EPWM functional clock (same for all EPWMs) */
#define APP_EPWM_FCLK                   ( CONFIG_EPWM0_FCLK )

/* EPWM Time Base clock */
#define APP_EPWM0_TB_FREQ               ( CONFIG_EPWM0_FCLK )

/* Initial Duty Cycle of PWM output signal in %, 0 to 100 */
#define APP_EPWM0_DUTY_CYCLE            ( 50U )

/* EPWM output frequency - synchronized with IEP reset frequency */
#define APP_EPWM0_OUTPUT_FREQ           ( CONFIG_SDFM0_IEP_RESET_FREQ )

/* PWM count direction (Up, Down, Up/Down) */
#define APP_EPWM0_TB_COUNTER_DIR        ( EPWM_TB_COUNTER_DIR_UP_DOWN )
#endif

/**
 *  \brief EPWM1 configuration for sigma delta clock generation
 *
 *  EPWM1 can be used to generate the sigma delta modulator clock.
 *  Set APP_EPWM1_ENABLE to 1 to enable EPWM1.
 *  Make sure EPWM1 is added in SysConfig before enabling this macro.
 */
#define APP_EPWM1_ENABLE                ( 0 )

#if APP_EPWM1_ENABLE
/* EPWM1 output frequency - sigma delta clock (20MHz) */
#define APP_EPWM1_OUTPUT_FREQ           ( 1U * 20000000U )

/* EPWM1 Time Base clock */
#define APP_EPWM1_TB_FREQ               ( CONFIG_EPWM1_FCLK )

/* EPWM1 Initial Duty Cycle of PWM output signal in %, 0 to 100 */
#define APP_EPWM1_DUTY_CYCLE            ( 50U )

/* EPWM1 PWM count direction (Up, Down, Up/Down) */
#define APP_EPWM1_TB_COUNTER_DIR        ( EPWM_TB_COUNTER_DIR_UP_DOWN )
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief EPWM configuration structure
 *
 *  This structure contains the configuration parameters for the EPWM module.
 *  It follows the same pattern as the universal single chip servo project.
 */
typedef struct
{
    uint32_t    epwmBaseAddr;       /**< EPWM base address */
    uint32_t    epwmCh;             /**< EPWM output channel */
    uint32_t    epwmFuncClk;        /**< EPWM functional clock */
    uint32_t    epwmTbFreq;         /**< EPWM timebase clock */
    uint32_t    epwmOutFreq;        /**< EPWM output frequency */
    uint32_t    epwmDutyCycle;      /**< EPWM duty cycle (0-100%) */
    uint32_t    epwmTbCounterDir;   /**< EPWM counter direction (Up, Down, Up/Down) */
    Bool        cfgTbSyncIn;        /**< Config TB sync in flag (TRUE/FALSE) */
    uint32_t    tbPhsValue;         /**< cfgTbSyncIn==TRUE: timer phase value to load on Sync In event */
    uint32_t    tbSyncInCounterDir; /**< cfgTbSyncIn==TRUE: counter direction on Sync In event */
    Bool        cfgTbSyncOut;       /**< Config TB sync output flag (TRUE/FALSE) */
    uint32_t    tbSyncOutMode;      /**< cfgTbSyncOut==TRUE: Sync Out mode */
    EPWM_AqActionCfg aqCfg;         /**< Action Qualifier config */
    Bool        cfgDb;              /**< Config DB flag (TRUE/FALSE) */
    EPWM_DeadbandCfg dbCfg;         /**< Deadband config */
    Bool        cfgEt;              /**< Config ET module */
    uint32_t    intSel;             /**< ET interrupt select */
    uint32_t    intPrd;             /**< ET interrupt period */
} SdfmEpwmCfg_t;

/**
 *  \brief Counter Compare Module enum
 *
 *  This enum defines the valid values for the counter compare module.
 */
typedef enum
{
    SDFM_EPWM_COUNTER_COMPARE_A = 0, /**< Counter compare A */
    SDFM_EPWM_COUNTER_COMPARE_B = 2, /**< Counter compare B */
} SdfmEpwmCounterCompareModule;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Configure EPWM module
 *
 *  This function configures an EPWM module with the specified parameters.
 *  It sets up the timebase, synchronization, action qualifier, deadband,
 *  and event trigger submodules using the SDK driver functions.
 *
 *  \param config [IN] Pointer to EPWM configuration parameters
 */
void sdfmEpwmConfig(SdfmEpwmCfg_t *config);

/**
 *  \brief Get EPWM timebase period value
 *
 *  This inline function reads the current timebase period register value.
 *
 *  \param base [IN] EPWM base address
 *
 *  \return Timebase period value
 */
static inline uint16_t sdfmEpwmGetTimeBasePeriod(uint32_t base)
{
    return (HW_RD_REG16(base + PWMSS_EPWM_TBPRD));
}

/**
 *  \brief Get EPWM counter compare value
 *
 *  This inline function reads the counter compare value for CMPA or CMPB.
 *
 *  \param base       [IN] EPWM base address
 *  \param compModule [IN] Counter Compare module (SDFM_EPWM_COUNTER_COMPARE_A or SDFM_EPWM_COUNTER_COMPARE_B)
 *
 *  \return Counter compare value
 */
static inline uint16_t sdfmEpwmGetCounterCompareValue(
    uint32_t base,
    SdfmEpwmCounterCompareModule compModule
)
{
    uint32_t registerOffset;
    uint16_t compCount;

    registerOffset = base + PWMSS_EPWM_CMPA + (uint16_t)compModule;

    if ((compModule == SDFM_EPWM_COUNTER_COMPARE_A) || (compModule == SDFM_EPWM_COUNTER_COMPARE_B))
    {
        compCount = (uint16_t)((HW_RD_REG32(registerOffset) &
                    (uint32_t)PWMSS_EPWM_CMPA_MASK) >>
                    PWMSS_EPWM_CMPA_SHIFT);
    }
    else
    {
        compCount = HW_RD_REG16(registerOffset);
    }

    return compCount;
}

/**
 *  \brief Set EPWM counter compare value
 *
 *  This inline function writes a counter compare value to CMPA or CMPB.
 *
 *  \param base       [IN] EPWM base address
 *  \param compModule [IN] Counter Compare module (SDFM_EPWM_COUNTER_COMPARE_A or SDFM_EPWM_COUNTER_COMPARE_B)
 *  \param compCount  [IN] Counter compare value to write
 */
static inline void sdfmEpwmSetCounterCompareValue(
    uint32_t base,
    SdfmEpwmCounterCompareModule compModule,
    uint16_t compCount
)
{
    uint32_t registerOffset;

    registerOffset = base + PWMSS_EPWM_CMPA + (uint16_t)compModule;

    HW_WR_REG16(registerOffset, compCount);
}

/**
 *  \brief Initialize EPWM module
 *
 *  This function initializes the EPWM hardware, registers interrupt handler,
 *  and configures the EPWM module based on provided configuration parameters.
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t sdfmEpwmInit(void);

/**
 *  \brief De-initialize EPWM module
 *
 *  This function disables EPWM interrupts and destroys the hardware interrupt object.
 */
void sdfmEpwmDeinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SDFM_EPWM_H */
