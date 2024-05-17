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


#include<stdio.h>
#include<stdint.h>
#include<math.h>

#include <drivers/pruicss.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/tistdtypes.h>
#include <kernel/dpl/ClockP.h>
#include "nikon_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/nikon/include/nikon_drv.h>

static HwiP_Object gIcssgEncoderHwiObject0;  /* ICSSG NIKON PRU FW HWI */
struct nikon_priv *priv;
/* ICSSG Interrupt settings */
#define ICSSG_PRU_NIKON_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
uint32_t gPrunikonIrqCnt0;

/*global variable */
void *gPruIcss_iep;

PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
extern PRUICSS_IntcInitData icss0_intc_initdata;

void nikon_config_iep(struct nikon_periodic_interface *nikon_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = nikon_periodic_interface->pruicss_iep;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;
    uint64_t cmp0 = 0;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);

    /*enable IEP reset by cmp0 event*/
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /*set IEP counter to ZERO*/
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG1, 0);

    /*configure cmp3 registers*/
    event |= (0x1 << 4 );
    event_clear |= (0x1 << 3);
    cmp_reg0 = (nikon_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (nikon_periodic_interface->cmp3>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG1,  cmp_reg1);

    cmp0 = nikon_periodic_interface->cmp0;

    /*clear event*/
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (cmp0 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (cmp0>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1,  cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG );
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, temp);
}


void nikon_interrupt_config(struct nikon_periodic_interface *nikon_periodic_interface)
{
    int32_t status;
    HwiP_Params hwiPrms;
    /* Register & enable ICSSG nikon PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_NIKON_INT_NUM;
    hwiPrms.callback    = &pru_nikon_irq_handler0;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

}
uint32_t nikon_config_periodic_mode(struct nikon_periodic_interface *nikon_periodic_interface, PRUICSS_Handle handle)
{
    int32_t  status;
    gPruIcssXHandle = handle;
    gPruIcss_iep = nikon_periodic_interface->pruicss_iep;
    /*configure IEP*/
    nikon_config_iep(nikon_periodic_interface);
    /* Initialize ICSS INTC */
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
        if (status != SystemP_SUCCESS)
        {
            return 0;
        }
    /*config Interrupt*/
    nikon_interrupt_config(nikon_periodic_interface);
    return 1;

}

void nikon_stop_periodic_mode(struct nikon_periodic_interface *nikon_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = nikon_periodic_interface->pruicss_iep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, temp);
}

/* PRU nikon FW IRQ handler */
void pru_nikon_irq_handler0(void *args)
{

    /* debug, inncrement PRU SDFM IRQ count */
    gPrunikonIrqCnt0++;

    /* clear Cmp3 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP3_EVNT;
    HW_WR_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDFM_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_NIKON_EVT0);

}

void nikon_periodic_interface_init(struct nikon_priv *priv, struct nikon_periodic_interface *nikon_periodic_interface, int64_t cmp0, int64_t cmp3)
{
    cmp0 = (cmp0 * priv->core_clk_freq)/1000000000; /* convert nano sec to PRU cycles */
    cmp3 = (cmp3 * priv->core_clk_freq)/1000000000; /* convert nano sec to PRU cycles */
    nikon_periodic_interface->pruicss_iep = priv->pruicss_iep;
    nikon_periodic_interface->cmp3 = cmp3;
    nikon_periodic_interface->cmp0 = cmp0;
}
