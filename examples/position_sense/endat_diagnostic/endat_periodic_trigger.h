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
  void *pruicss_iep;
  void *pruicss_dmem;
  uint8_t load_share;
  uint64_t cmp0_count;
  uint64_t ch0_trigger_count;
  uint64_t ch1_trigger_count;
  uint64_t ch2_trigger_count;
};
#define IEP_DEFAULT_INC    0x1;
#define IEP_DEFAULT_INC_EN  0x4;
#define IEP_COUNTER_EN      0x1;
#define IEP_RST_CNT_EN      0x1;
#define IEP_CMP0_ENABLE     0x1 << 1;

#if CONFIG_ENDAT0_PRUICSS_PRUx == 1
#define PRU_TRIGGER_HOST_ENDAT_EVT0   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define IEP_CH0_CMP_EVNT ( 3 )
#define IEP_CH1_CMP_EVNT ( 5 )
#define IEP_CH2_CMP_EVNT ( 6 )
#else
#define PRU_TRIGGER_HOST_ENDAT_EVT0   ( 5+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define IEP_CH0_CMP_EVNT ( 7 )
#define IEP_CH1_CMP_EVNT ( 8 )
#define IEP_CH2_CMP_EVNT ( 9 )
#endif

#define PRU_TRIGGER_HOST_ENDAT_EVT1   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
#define PRU_TRIGGER_HOST_ENDAT_EVT2   ( 4+16 )   /* pr0_pru_mst_intr[4]_intr_req */

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
#if CONFIG_ENDAT1_PRUICSS_PRUx == 1
#define PRU_TRIGGER_HOST_ENDAT1_EVT   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define ENDAT1_IEP_CH0_CMP_EVNT ( 3 )
#define ENDAT1_IEP_CH1_CMP_EVNT ( 5 )
#define ENDAT1_IEP_CH2_CMP_EVNT ( 6 )
#else
#define PRU_TRIGGER_HOST_ENDAT1_EVT   ( 5+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define ENDAT1_IEP_CH0_CMP_EVNT ( 7 )
#define ENDAT1_IEP_CH1_CMP_EVNT ( 8 )
#define ENDAT1_IEP_CH2_CMP_EVNT ( 9 )
#endif
#endif

uint32_t endat_config_periodic_mode(struct endat_periodic_interface *endat_periodic_interface, PRUICSS_Handle handle, Endat_Handle handle);

void endat_stop_periodic_continuous_mode(struct endat_periodic_interface *endat_periodic_interface);

#define ENDAT_PERIODIC_MODE_IEP_INSTANCE  0

static void pruEnDatIrqHandler(void *handle);
static void rtuEnDatIrqHandler(void *handle);
static void txpruEnDatIrqHandler(void *handle);

#endif /* _ENDAT_H_ */
