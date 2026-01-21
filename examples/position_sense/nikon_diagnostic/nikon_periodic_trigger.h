/*
 *  Copyright (C) 2024-2026 Texas Instruments Incorporated
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

#ifndef _NIKON_PERIODIC_TRIGGER_H_
#define _NIKON_PERIODIC_TRIGGER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include<stdint.h>
#include <position_sense/nikon/include/nikon_drv.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* TIMESYNC router configuration register offsets and values */
#define NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE            (4U)
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET         (8U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*ICSSG0 PRG0_IEP0_LATCH0_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT9_OFFSET         (9U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*ICSSG0 PRG0_IEP0_LATCH1_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET        (10U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG0 PRG0_IEP1_LATCH0_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT11_OFFSET        (11U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG0 PRG0_IEP1_LATCH1_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET        (12U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP0_LATCH0_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT13_OFFSET        (13U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP0_LATCH1_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET        (14U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP1_LATCH0_IN0*/
#define NIKON_TIMESYNC_EVENT_ROUTER_OUT15_OFFSET        (15U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP1_LATCH1_IN0*/

/*CAP0 is used for Channel 1 for CAP mode. To use a different CAP event, check AM243x TRM section 9.3.2.2 GPIOMUX_INTRTR0 Integration */
#define NIKON_GPIOMUX_INTROUTER0_IEP0_CAP_OFFSET        (18U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*GPIOMUX0 IEP0_CAP_IN*/
#define NIKON_GPIOMUX_INTROUTER0_IEP1_CAP_OFFSET        (24U * NIKON_TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*GPIOMUX0 IEP1_CAP_IN*/

#define NIKON_TIMESYNC_EVENT_ROUTER_IN25                (0x00010019U)  /* PRU_ICSSG0_PR1_EDC0_SYNC0_OUT_0 */
#define NIKON_TIMESYNC_EVENT_ROUTER_IN27                (0x0001001BU)  /* PRU_ICSSG0_PR1_EDC1_SYNC0_OUT_0 */
#define NIKON_TIMESYNC_EVENT_ROUTER_IN29                (0x0001001DU)  /* PRU_ICSSG1_PR1_EDC0_SYNC0_OUT_0 */
#define NIKON_TIMESYNC_EVENT_ROUTER_IN31                (0x0001001FU)  /* PRU_ICSSG1_PR1_EDC1_SYNC0_OUT_0 */

/* GPIO number need to be configured based on the input source GPIO pin number.
 * GPIO0_GPIO_4 is used in this example.
 * Refer to the GPIOMUX_INTRTR0 Interrupt Map in 9.4.1.8 section of the AM243x TRM for more details. */
#define NIKON_GPIOMUX_INTROUTER0_CAP_GPIO_IN            (0x00010004U)

/* IEP SYNC control register bit definitions */
#define NIKON_IEP_SYNC_CTRL_SYNC01_EN_SHIFT             (0U)           /* SYNC01 enable bit position */
#define NIKON_IEP_SYNC_CTRL_SYNC01_EN_MASK              (0x00000001U)  /* SYNC01 enable bit mask */
#define NIKON_IEP_SYNC_CTRL_SYNC0_EN_SHIFT              (1U)           /* SYNC1 enable bit position */
#define NIKON_IEP_SYNC_CTRL_SYNC0_EN_MASK               (0x00000002U)  /* SYNC1 enable bit mask */
#define NIKON_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_SHIFT       (5U)           /* SYNC0 cyclic generation bit position */
#define NIKON_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK        (0x00000020U)  /* SYNC0 cyclic generation bit mask */

/* IEP SYNC configuration values */
#define NIKON_IEP_CMP1_START_DELAY                      (100U)         /* IEP CMP1 start delay in cycles */
#define NIKON_IEP_SYNC0_PULSE_WIDTH                     (10U)          /* SYNC0 high pulse time in IEP clock cycles */
#define NIKON_IEP_CMP_EVENT_FOR_RESET                   (0U)           /* CMP event number used for IEP Reset */
#define NIKON_IEP_CMP_EVENT_FOR_SYNC0                   (1U)           /* CMP event number used for SYNC0 generation */

/*IEP Counter configuration*/
#define NIKON_IEP_COUNTER_ENABLE                        (1U)          /* IEP counter enable value */
#define NIKON_IEP_COUNTER_DISABLE                       (0U)          /* IEP counter disable value */
#define NIKON_IEP_COUNTER_INCREMENT                     (1U)          /* IEP counter increment value */

/* IEP CMP configuration register bit shifts and masks */
#define NIKON_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT          (0x1U)        /* CMP enable bit shift */
#define NIKON_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT (0x0U)        /* CMP0 reset counter enable bit shift */

/* Macros to extract lower and upper 32 bits from 64-bit values */
#define NIKON_GET_LOWER_32BITS(x)                       ((uint32_t)((x) & 0xFFFFFFFFU))           /* Extract lower 32 bits */
#define NIKON_GET_UPPER_32BITS(x)                       ((uint32_t)(((x) >> 32) & 0xFFFFFFFFU))  /* Extract upper 32 bits */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief   Structure defining Nikon periodic trigger interface configuration
 *
 * \details Contains Nikon driver handle, trigger count values and IEP reset
 *          count for periodic mode operation, in which automatic encoder transaction
 *          is triggered at configured intervals.
 */
typedef struct nikon_periodic_interface_s
{
  nikon_handle handle[CONFIG_NIKON_NUM_INSTANCES];
  /**< Nikon driver handle obtained from nikon_init().
   *   Used to access driver configuration and PRU-ICSS resources */

  uint64_t periodic_trigger_count[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX];
  /**< IEP counter value for periodic trigger (in IEP clock cycles) per instance and channel. */

  uint64_t iep_reset_count;
  /**< IEP counter reset value (in IEP clock cycles) for CMP0 event.
   *   When IEP counter reaches this value, it resets to 0, creating periodic cycles */

  uint8_t is_cap_mode;
  /**< Flag indicating periodic trigger mode: 0 = CMP mode, 1 = CAP mode */
} nikon_periodic_interface;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * \brief   Configure Nikon encoder for periodic trigger mode
 *
 * \details This function configures the Nikon encoder interface to operate in periodic
 *          trigger mode, where encoder position data is automatically sampled at regular
 *          intervals using the PRU-ICSS IEP (Industrial Ethernet Peripheral) timer.
 *
 *          The function performs the following operations:
 *          1. Configures IEP timer with specified periodic trigger count and reset count
 *          2. Enables IEP Compare or Capture events based on is_cap_mode setting
 *          3. Registers interrupt handler for processing periodic samples
 *          4. Enables PRU interrupt handling
 *
 *          **CMP Mode (is_cap_mode = 0):**
 *          - IEP counter increments at IEP clock rate
 *          - When counter reaches periodic_trigger_count, encoder transaction is triggered
 *          - When counter reaches iep_reset_count, counter resets to 0 (defines period)
 *          - Interrupt handler is called on each encoder transaction completion
 *
 *          **CAP Mode (is_cap_mode = 1):**
 *          - IEP captures counter value when external signal triggers CAP event
 *          - Encoder transaction is triggered on each external event
 *          - Requires TIMESYNC/GPIOMUX router configuration (AM243x), or
 *            XBAR configuration (AM26x)
 *          - Suitable for event-driven sampling synchronized with external signals
 *          - Interrupt handler is called on each encoder transaction completion
 *
 *          Requirements:
 *          - Nikon driver must be initialized with nikon_init() before calling this function
 *          - CMP mode: periodic_trigger_count must be less than iep_reset_count
 *          - CAP mode: TIMESYNC/GPIOMUX router (AM243x) or XBAR (AM26x) must be configured
 *
 * \param[in]   nikon_periodic_interface  Pointer to periodic interface structure containing:
 *                                           - handle: Nikon driver handle from nikon_init()
 *                                           - periodic_trigger_count[]: IEP count for trigger (CMP mode)
 *                                           - iep_reset_count: IEP count for counter reset
 *                                           - is_cap_mode: 0 = CMP mode, 1 = CAP mode
 *
 * \retval      SystemP_SUCCESS    Configuration successful, periodic mode active
 * \retval      SystemP_FAILURE    Configuration failed (NULL interface pointer, invalid handle,
 *                                 or configuration error)
 *
 * \note        Call nikon_stop_periodic_mode() before returning to host trigger mode
 *
 */

int32_t nikon_config_periodic_mode(nikon_periodic_interface *nikon_periodic_interface);

/**
 * \brief   Stop Nikon periodic trigger mode
 *
 * \details This function disables periodic trigger mode for the Nikon encoder interface.
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
 *          - Application must enable host trigger mode
 *
 * \param[in]   nikon_periodic_interface  Pointer to periodic interface structure containing
 *                                        the Nikon driver handle(s) to stop
 *
 * \retval      SystemP_SUCCESS    Periodic mode stopped successfully
 * \retval      SystemP_FAILURE    Failed to stop periodic mode (NULL interface pointer or invalid handle)
 *
 */
int32_t nikon_stop_periodic_mode(nikon_periodic_interface *nikon_periodic_interface);

#endif /* _NIKON_PERIODIC_TRIGGER_H_ */
