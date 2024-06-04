/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef _NIKON_H_
#define _NIKON_H_

#include<stdint.h>
#include <position_sense/nikon/include/nikon_drv.h>

struct nikon_periodic_interface
{
  void *pruicss_iep;
  uint64_t cmp3;
  uint64_t cmp0;
};

#define IEP_DEFAULT_INC     0x1
#define IEP_DEFAULT_INC_EN  0x4
#define IEP_COUNTER_EN      0x1
#define IEP_RST_CNT_EN      0x1
#define IEP_CMP0_ENABLE     (0x1 << 1)
#define IEP_CMP3_EVNT       (0x1 << 3)

#define PRU_TRIGGER_HOST_NIKON_EVT0   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */

uint32_t nikon_config_periodic_mode(struct nikon_periodic_interface *nikon_periodic_interface, PRUICSS_Handle handle);

void nikon_stop_periodic_mode(struct nikon_periodic_interface *nikon_periodic_interface);

static void pru_nikon_irq_handler0(void *handle);

void nikon_periodic_interface_init(struct nikon_priv *priv, struct nikon_periodic_interface *nikon_periodic_interface,int64_t cmp0, int64_t cmp3);

#endif /* _NIKON_H_ */