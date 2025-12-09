/*
 *  Copyright (C) 2023-25 Texas Instruments Incorporated
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
 *  \file   sdfm_example.h
 *
 *  \brief  SDFM example helper functions and definitions.
 *
 *  This file provides initialization and configuration functions for ICSSG SDFM
 *  (Sigma-Delta Filter Module) current sensing on TI AM243x devices.
 *
 *  Key Functions:
 *  - SDFM_pruIcssInit()  : Initialize ICSSG PRU subsystem for SDFM
 *  - appSdfmPruInit()       : Load PRU firmware and configure SDFM parameters
 */

#ifndef _SDFM_H_
#define _SDFM_H_

#include <stdint.h>
#include <drivers/pruicss.h>
#include "current_sense/sdfm/include/sdfm_api.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Status codes */
#define SDFM_ERR_NERR               (  0 )  /* no error */
#define SDFM_ERR_CFG_PIN_MUX        ( -1 )  /* pin mux configuration error */
#define SDFM_ERR_CFG_ICSSG_CLKCFG   ( -2 )  /* ICSSG clock configuration error */
#define SDFM_ERR_INIT_ICSSG         ( -3 )  /* initialize ICSSG error */
#define SDFM_ERR_CFG_MCU_INTR       ( -4 )  /* interrupt configuration error */
#define SDFM_ERR_INIT_PRU_SDFM      ( -5 )  /* initialize PRU for SDFM error */
#define SDFM_ERR_INIT_SDFM          ( -6 )  /* initialize SDFM error */

/**
 * \brief Address translation macros for TCM to SoC view
 *
 * These macros translate R5F core-local TCM (Tightly Coupled Memory) addresses
 * to SoC global view addresses. This translation is required when the PRU firmware
 * needs to access data structures allocated in R5F TCM memory, since PRU uses
 * the SoC address space rather than the core-local address space.
 *
 */
#define CPU0_ATCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_ATCM_BASE+(x))  /**< CPU0 ATCM: core local to SoC global address */
#define CPU1_ATCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_ATCM_BASE+(x))  /**< CPU1 ATCM: core local to SoC global address */
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))  /**< CPU0 BTCM: core local to SoC global address */
#define CPU1_BTCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_BTCM_BASE+(x - CSL_R5FSS1_BTCM_BASE))  /**< CPU1 BTCM: core local to SoC global address */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Initialize ICSSG PRU subsystem for SDFM
 *
 *  Disables PRU cores, resets memories, sets pin mux, and initializes INTC.
 *
 *  \param icssInstId     [in] ICSSG instance ID (CONFIG_PRU_ICSS0, etc.)
 *  \param sliceId        [in] PRU slice ID (ICSSG_SLICE_ID_0 or ICSSG_SLICE_ID_1)
 *  \param saMuxMode      [in] SA MUX mode for pin configuration
 *  \param loadShareMode  [in] Load share mode enable (0=single PRU, 1=multi-PRU)
 *  \param pPruIcssHandle [out] Pointer to store PRUICSS handle
 *
 *  \return SDFM_ERR_NERR on success, error code otherwise
 */
int32_t SDFM_pruIcssInit(
    uint8_t icssInstId,
    uint8_t sliceId,
    uint8_t saMuxMode,
    uint8_t loadShareMode,
    PRUICSS_Handle *pPruIcssHandle
);

/**
 *  \brief Initialize PRU cores for SDFM
 *
 *  Loads PRU firmware, configures SDFM parameters, enables PRU cores,
 *  and initializes SDFM driver.
 *
 *  \param pruIcssHandle [in] PRUICSS handle from SDFM_pruIcssInit()
 *  \param pSdfmPrms     [in] SDFM parameters structure (from SysConfig)
 *  \param pHSdfm        [out] Pointer to store SDFM handle
 *
 *  \return SDFM_ERR_NERR on success, error code otherwise
 */
int32_t appSdfmPruInit(
    PRUICSS_Handle pruIcssHandle,
    SDFM_Params pSdfmPrms,
    SDFM_Handle *pHSdfm
);

#endif /* _SDFM_H_ */
