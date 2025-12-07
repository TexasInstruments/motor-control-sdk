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

#ifndef _BISSC_PERIODIC_TRIGGER_H_
#define _BISSC_PERIODIC_TRIGGER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include<stdint.h>
#include <position_sense/bissc/include/bissc_drv.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief IEP counter default increment value (1 per clock cycle) */
#define IEP_DEFAULT_INC     0x1

/** \brief IEP counter enable bit in Global Config register (start counter) */
#define IEP_COUNTER_EN      0x1

/** \brief IEP reset counter on CMP0 event enable bit */
#define IEP_RST_CNT_EN      0x1

/** \brief IEP Compare 0 (CMP0) event enable bit (bit 1 in CMP_CFG_REG) */
#define IEP_CMP0_ENABLE     (0x1 << 1)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief   Structure defining BiSS-C periodic trigger interface configuration
 *
 * \details Contains BiSS-C driver handle, trigger count values and IEP reset
 *          count for periodic mode operation, in which automatic BiSS-C transaction
 *          is triggered at configured intervals.
 */
typedef struct bissc_periodic_interface_s
{
  bissc_handle handle[CONFIG_BISSC_NUM_INSTANCES];
  /**< BiSS-C driver handle obtained from bissc_init().
   *   Used to access driver configuration and PRU-ICSS resources */

  uint64_t periodic_trigger_count[CONFIG_BISSC_NUM_INSTANCES][BISSC_NUM_CH_PER_SLICE_MAX];
  /**< IEP counter value for periodic trigger (in IEP clock cycles) per instance and channel. */

  uint64_t iep_reset_count;
  /**< IEP counter reset value (in IEP clock cycles) for CMP0 event.
   *   When IEP counter reaches this value, it resets to 0, creating periodic cycles */
} bissc_periodic_interface;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * \brief   Configure BiSS-C encoder for periodic trigger mode
 *
 * \details This function configures the BiSS-C encoder interface to operate in periodic
 *          trigger mode, where encoder position data is automatically sampled at regular
 *          intervals using the PRU-ICSS IEP (Industrial Ethernet Peripheral) timer.
 *
 *          The function performs the following operations:
 *          1. Configures IEP timer with specified periodic trigger count and reset count
 *          2. Enables IEP Compare 0 (CMP0) event for periodic triggering
 *          3. Registers interrupt handler for processing periodic samples
 *          4. Enables PRU interrupt handling
 *
 *          In periodic mode:
 *          - IEP counter increments at IEP clock rate (default: 200 MHz)
 *          - When counter reaches periodic_trigger_count, encoder transaction is triggered for that channel
 *          - When counter reaches iep_reset_count, counter resets to 0 (defines period)
 *          - Interrupt handler is called on each BiSS-C transaction completion
 *
 *          Requirements:
 *          - BiSS-C driver must be initialized with bissc_init() before calling this function
 *          - periodic_trigger_count must be less than iep_reset_count
 *          - IEP clock must be configured via SysConfig
 *
 * \param[in]   bissc_periodic_interface  Pointer to periodic interface structure containing:
 *                                           - handle: BiSS-C driver handle from bissc_init()
 *                                           - periodic_trigger_count[]: IEP count value for trigger
 *                                             per channel
 *                                           - iep_reset_count: IEP count value for counter reset
 *
 * \retval      SystemP_SUCCESS    Configuration successful, periodic mode active
 * \retval      SystemP_FAILURE    Configuration failed (NULL interface pointer, invalid handle,
 *                                 or configuration error)
 *
 * \note        Call bissc_stop_periodic_mode() before returning to host trigger mode
 *
 */

int32_t bissc_config_periodic_mode(bissc_periodic_interface *bissc_periodic_interface);

/**
 * \brief   Stop BiSS-C periodic trigger mode
 *
 * \details This function disables periodic trigger mode for the BiSS-C encoder interface.
 *
 *          The function performs the following operations:
 *          1. Disables PRU interrupts for periodic trigger events
 *          2. Disables IEP Compare 0 (CMP0) event
 *          3. Stops IEP counter
 *          4. Unregisters interrupt handler
 *
 *          After calling this function:
 *          - IEP timer is stopped
 *          - No automatic encoder transactions occur
 *          - Application must enable host trigger mode and call
 *            bissc_command_process() explicitly for each transaction
 *
 * \param[in]   bissc_periodic_interface  Pointer to periodic interface structure containing
 *                                        the BiSS-C driver handle(s) to stop
 *
 * \retval      SystemP_SUCCESS    Periodic mode stopped successfully
 * \retval      SystemP_FAILURE    Failed to stop periodic mode (NULL interface pointer or invalid handle)
 *
 */
int32_t bissc_stop_periodic_mode(bissc_periodic_interface *bissc_periodic_interface);

#endif /* _BISSC_PERIODIC_TRIGGER_H_ */
