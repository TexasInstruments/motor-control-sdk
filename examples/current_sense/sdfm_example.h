/*
 *  Copyright (C) 2023-2025 Texas Instruments Incorporated
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
 *  - sdfm_pruicss_init()      : Initialize ICSSG PRU subsystem for SDFM
 *  - sdfm_load_firmware()     : Load PRU firmware
 *  - sdfm_configure_and_enable() : Configure SDFM parameters and enable
 */

#ifndef _SDFM_H_
#define _SDFM_H_

#include <stdint.h>
#include <drivers/pruicss.h>
#include "current_sense/sdfm/include/sdfm_api.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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
 *  \param pruicss_handle     [out] Pointer to store PRUICSS handle (output parameter)
 *  \param pruicss_instance   [in] ICSSG instance ID (CONFIG_PRU_ICSS0, etc.)
 *  \param pruicss_slice      [in] PRU slice ID (ICSSG_SLICE_ID_0 or ICSSG_SLICE_ID_1)
 *  \param load_share_enabled [in] Load share mode enable (0=single PRU, 1=multi-PRU)
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t sdfmPruicssInit(PRUICSS_Handle *pruicss_handle, uint8_t pruicss_instance, uint8_t pruicss_slice, uint8_t load_share_enabled);

/**
 *  \brief Load PRU firmware for SDFM
 *
 *  Loads PRU firmware into PRU cores and enables them.
 *
 *  \param pruIcssHandle [in] PRUICSS handle from sdfmPruicssInit()
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t sdfmLoadFirmware(PRUICSS_Handle pruIcssHandle);

/**
 *  \brief Configure SDFM settings and enable SDFM firmware
 *
 *  Enables channels, configures sample buffers, sets up snoop/trigger modes,
 *  configures comparators and overcurrent detection, and enables SDFM to start sampling.
 *
 *  \param handle [in] SDFM handle
 *
 *  \return SystemP_SUCCESS on success, error code otherwise
 */
int32_t sdfmConfigureAndEnable(SDFM_Handle handle);

#endif /* _SDFM_H_ */
