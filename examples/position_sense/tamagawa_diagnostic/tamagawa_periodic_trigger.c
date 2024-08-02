/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include "tamagawa_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/tamagawa/include/tamagawa_drv.h>


static HwiP_Object gIcssgEncoderHwiObject0;  /* ICSSG Tamagawa PRU FW HWI */

/* ICSSG Interrupt settings */
#if (SOC_AM261X || SOC_AM263X)
#define ICSSG_PRU_TAMAGAWA_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0 )
#else
#define ICSSG_PRU_TAMAGAWA_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
#endif
uint32_t gPrutamagawaIrqCnt0;

/*global variable */
void *gPruss_iep;

PRUICSS_Handle gPruIcssXHandle;

/*am261x does not support periodic mode*/
#if(!SOC_AM261X) 
/* ICSS INTC configuration */
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

void tamagawa_config_iep(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /*reset iep timer*/
    void *pruss_iep = tamagawa_periodic_interface->pruss_iep;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /*enable IEP reset by cmp0 event*/
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /*set IEP counter to ZERO*/
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /*configure cmp3 registers*/
    event |= (0x1 << 4 );
    event_clear |= (0x1 << 3);
    cmp_reg0 = (tamagawa_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->cmp3>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG1,  cmp_reg1);


    /*clear event*/
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (tamagawa_periodic_interface->cmp0 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->cmp0>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1,  cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}


void tamagawa_interrupt_config(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    int32_t status;
    HwiP_Params hwiPrms;
    /* Register & enable ICSSG tamagawa PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_TAMAGAWA_INT_NUM;
    hwiPrms.callback    = &prutamagawaIrqHandler0;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

}
uint32_t  tamagawa_config_periodic_mode(struct tamagawa_periodic_interface *tamagawa_periodic_interface, PRUICSS_Handle handle)
{

    gPruIcssXHandle = handle;
    gPruss_iep = tamagawa_periodic_interface->pruss_iep;
    /*configure IEP*/
    tamagawa_config_iep(tamagawa_periodic_interface);
    /* Initialize ICSS INTC */
    /*am261x does not support periodic mode*/
#if(!SOC_AM261X) 
    int32_t  status;
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return 0;
    }
#endif
    /*config Interrupt*/
    tamagawa_interrupt_config(tamagawa_periodic_interface);
    return 1;

}

void tamagawa_stop_periodic_continuous_mode(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /*reset iep timer*/
    void *pruss_iep = tamagawa_periodic_interface->pruss_iep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

/* PRU tamagawa FW IRQ handler */
void prutamagawaIrqHandler0(void *args)
{

    /* debug, inncrement PRU SDFM IRQ count */
    gPrutamagawaIrqCnt0++;

    /* clear Cmp3 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP3_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDFM_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_TAMAGAWA_EVT0);

}
