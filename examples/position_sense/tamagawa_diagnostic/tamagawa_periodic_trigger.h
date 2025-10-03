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

#ifndef _TAMAGAWA_H_
#define _TAMAGAWA_H_

#include<stdint.h>
#include <position_sense/tamagawa/include/tamagawa_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

struct tamagawa_periodic_interface
{
  uint64_t periodic_trigger_count;
  uint64_t iep_reset_count;
};

#define IEP_DEFAULT_INC    0x1;
#define IEP_DEFAULT_INC_EN  0x4;
#define IEP_COUNTER_EN      0x1;
#define IEP_RST_CNT_EN      0x1;
#define IEP_CMP0_ENABLE     0x1 << 1;

/*
 * This is also defined in firmware, both macros need to be updated in both places
 */
#if (CONFIG_TAMAGAWA0_PRUICSS_PRUx == 1)
#define IEP_CMP_EVENT       ( 3 )
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT0   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#else
#define IEP_CMP_EVENT       ( 4 )
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT0   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
#endif

#define TAMAGAWA_PERIODIC_MODE_IEP_INSTANCE   0
uint32_t tamagawa_config_periodic_mode(struct tamagawa_periodic_interface *tamagawa_periodic_interface, PRUICSS_Handle handle, uint8_t tamagawa_instnace);

void tamagawa_stop_periodic_continuous_mode(struct tamagawa_periodic_interface *tamagawa_periodic_interface);

static void pruTamagawaIrqHandler0(void *args);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
void pruTamagawaDualChannelIrqHandler0(void *args);
#if (CONFIG_TAMAGAWA1_PRUICSS_PRUx == 1)
#define DUAL_CH_IEP_CMP_EVENT       ( 3 )
#define PRU_TRIGGER_HOST_TAMAGAWA_DUAL_CH_EVT0   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#else
#define DUAL_CH_IEP_CMP_EVENT       ( 4 )
#define PRU_TRIGGER_HOST_TAMAGAWA_DUAL_CH_EVT0   ( 3+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#endif
#endif



#endif /* _TAMAGAWA_H_ */
