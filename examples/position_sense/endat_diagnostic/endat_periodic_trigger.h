/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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

#ifndef _ENDAT_H_
#define _ENDAT_H_

#include<stdint.h>
#include <position_sense/endat/include/endat_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

struct endat_periodic_interface
{
  Endat_Handle endat_handle;
  uint64_t iep_reset_count;
  uint64_t ch_trigger_count[3];
  uint8_t  is_cap_mode;
  uint64_t iep_sync0_period;
};
/* IEP trigger host event numbers */
#if CONFIG_ENDAT0_PRUICSS_PRUx == 1
#define PRU_TRIGGER_HOST_ENDAT_EVT0   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#else
#define PRU_TRIGGER_HOST_ENDAT_EVT0   ( 5+16 )    /* pr0_pru_mst_intr[5]_intr_req */
#endif

#define PRU_TRIGGER_HOST_ENDAT_EVT1   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
#define PRU_TRIGGER_HOST_ENDAT_EVT2   ( 4+16 )    /* pr0_pru_mst_intr[4]_intr_req */

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
#if CONFIG_ENDAT1_PRUICSS_PRUx == 1
#define PRU_TRIGGER_HOST_ENDAT1_EVT   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#else
#define PRU_TRIGGER_HOST_ENDAT1_EVT   ( 5+16 )    /* pr0_pru_mst_intr[5]_intr_req */
#endif
#endif

uint32_t endat_config_periodic_mode(struct endat_periodic_interface *endat_periodic_interface);

void endat_stop_periodic_continuous_mode(struct endat_periodic_interface *endat_periodic_interface);

/* TIMESYNC router configuration register offsets and values */
#define TIMESYNC_EVENT_ROUTER_REG_SIZE         (4U)
#define TIMESYNC_EVENT_ROUTER_OUT8_OFFSET      (8U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*ICSSG0 PRG0_IEP0_LATCH0_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT9_OFFSET      (9U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*ICSSG0 PRG0_IEP0_LATCH1_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT10_OFFSET     (10U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG0 PRG0_IEP1_LATCH0_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT11_OFFSET     (11U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG0 PRG0_IEP1_LATCH1_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT12_OFFSET     (12U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP0_LATCH0_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT13_OFFSET     (13U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP0_LATCH1_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT14_OFFSET     (14U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP1_LATCH0_IN0*/
#define TIMESYNC_EVENT_ROUTER_OUT15_OFFSET     (15U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U) /*ICSSG1 PRG1_IEP1_LATCH1_IN0*/

/*CAP0 is used for Channel1 for periodic contimuos cap mode, to changes cap event different cap number offset can be found at TRM section 9.3.2.2 GPIOMUX_INTRTR0 Integration.*/
#define GPIOMUX_INTROUTER0_IEP0_CAP_OFFSET     (18U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*GPIOMUX0 IEP0_CAP_IN*/
#define GPIOMUX_INTROUTER0_IEP1_CAP_OFFSET     (24U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)  /*GPIOMUX0 IEP1_CAP_IN*/


#define TIMESYNC_EVENT_ROUTER_IN25              (0x00010019U)  /* PRU_ICSSG0_PR1_EDC0_SYNC0_OUT_0 */
#define TIMESYNC_EVENT_ROUTER_IN27              (0x0001001BU)  /* PRU_ICSSG0_PR1_EDC1_SYNC0_OUT_0 */
#define TIMESYNC_EVENT_ROUTER_IN29              (0x0001001DU)  /* PRU_ICSSG1_PR1_EDC0_SYNC0_OUT_0 */
#define TIMESYNC_EVENT_ROUTER_IN31              (0x0001001FU)  /* PRU_ICSSG1_PR1_EDC1_SYNC0_OUT_0 */

/* GPIO number need to be configured based on the input source GPIO pin number.
 * Refer to the GPIOMUX_INTRTR0 Interrupt Map section in the TRM (9.4.1.8) for details. */
#define GPIOMUX_INTROUTER0_CAP_GPIO_IN          (0x00010004U)  /* PINFUNCTION_PRG0_IEP_CAP_IN */

/* IEP SYNC control register bit definitions */
#define IEP_SYNC_CTRL_SYNC01_EN_SHIFT            (0U)           /* SYNC01 enable bit position */
#define IEP_SYNC_CTRL_SYNC01_EN_MASK             (0x00000001U)  /* SYNC01 enable bit mask */
#define IEP_SYNC_CTRL_SYNC0_EN_SHIFT            (1U)           /* SYNC1 enable bit position */
#define IEP_SYNC_CTRL_SYNC0_EN_MASK             (0x00000002U)  /* SYNC1 enable bit mask */
#define IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_SHIFT     (5U)           /* SYNC0 cyclic generation bit position */
#define IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK      (0x00000020U)  /* SYNC0 cyclic generation bit mask */

/* IEP SYNC configuration values */
#define IEP_CMP1_START_DELAY             (100U)         /* IEP CMP1 start delay in cycles */
#define IEP_SYNC0_PULSE_WIDTH            (10U)          /* SYNC0 high pulse time in IEP clock cycles */
#define IEP_CMP_EVENT_FOR_SYNC0          (1U)           /* IEP CMP event number used for SYNC0 generation */
#define IEP_CMP_EVENT_FOR_IEP_RESET      (0U)           /* IEP CMP event number used for IEP counter reset */


static void pruEnDatIrqHandler(void *handle);
static void rtuEnDatIrqHandler(void *handle);
static void txpruEnDatIrqHandler(void *handle);

#endif /* _ENDAT_H_ */
