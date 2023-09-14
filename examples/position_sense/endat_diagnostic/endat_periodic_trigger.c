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
#include "endat_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/endat/include/endat_drv.h>
#include <position_sense/endat/include/endat_interface.h>


HwiP_Params hwiPrms;
static HwiP_Object gIcssgEncoderHwiObject0;  /* ICSSG EnDat PRU FW HWI */
static HwiP_Object gIcssgEncoderHwiObject1;  /* ICSSG EnDat PRU FW HWI */
static HwiP_Object gIcssgEncoderHwiObject2;  /* ICSSG EnDat PRU FW HWI */

/* ICSSG Interrupt settings */
#define ICSSG_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
uint32_t gPruEnDatIrqCnt0;
uint32_t gPruEnDatIrqCnt1;
uint32_t gPruEnDatIrqCnt2;

/*global variable */
void *gPruss_iep; 



PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
static const PRUICSS_IntcInitData gPruicssIntcInitdata = PRUICSS_INTC_INITDATA;

void endat_config_iep(struct endat_periodic_interface *endat_periodic_interface)
{
    /*reset iep timer*/
    void *pruss_iep = endat_periodic_interface->pruss_iep;
    struct endat_pruss_xchg *pruss_xchg = endat_periodic_interface->pruss_dmem;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;
    uint64_t cmp0 = 0;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);

    /*enable IEP reset by cmp0 event*/
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /*set IEP counter to ZERO*/
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG1, 0);

    /*Clear all event & configure*/
    if(endat_periodic_interface->load_share)
    {
        event |= pruss_xchg->config[0].channel==1?(0x1 << 4 ):0; //CMP3 for ch0
        event |= pruss_xchg->config[1].channel==2?(0x1 << 6 ):0;  //CMP5 for Ch1
        event |= pruss_xchg->config[2].channel==4?(0x1 << 7 ):0;  //CMP6 for CH2

        /*clear event*/
        event_clear |= pruss_xchg->config[0].channel==1?(0x1 << 3 ):0; //CMP3 for ch0
        event_clear |= pruss_xchg->config[1].channel==2?(0x1 << 5 ):0;  //CMP5 for Ch1
        event_clear |= pruss_xchg->config[2].channel==4?(0x1 << 6 ):0;  //CMP6 for CH2

        if(pruss_xchg->config[0].channel)
        {
            cmp_reg0 = (endat_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (endat_periodic_interface->cmp3>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG0,  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG1,  cmp_reg1);

            cmp0 = cmp0 > endat_periodic_interface->cmp3 ? cmp0: endat_periodic_interface->cmp3;
        }

        if(pruss_xchg->config[1].channel)
        {
            cmp_reg0 = (endat_periodic_interface->cmp5 & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (endat_periodic_interface->cmp5>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP5_REG0,  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP5_REG1,  cmp_reg1);

            cmp0 = cmp0 > endat_periodic_interface->cmp5 ? cmp0: endat_periodic_interface->cmp5;
        }

        if(pruss_xchg->config[2].channel)
        {
            cmp_reg0 = (endat_periodic_interface->cmp6 & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (endat_periodic_interface->cmp6>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP6_REG0,  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP6_REG1,  cmp_reg1);

            cmp0 = cmp0 > endat_periodic_interface->cmp6 ? cmp0: endat_periodic_interface->cmp6;
        }


    }
    else
    {
        event |= (0x1 << 4 );
        event_clear |= (0x1 << 3);
        cmp_reg0 = (endat_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (endat_periodic_interface->cmp3>>32 & 0xffffffff);

        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG0,  cmp_reg0);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG1,  cmp_reg1);

        cmp0 = endat_periodic_interface->cmp3;

    }
    /*clear event*/
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp0 = 2*cmp0;
    cmp_reg0 = (cmp0 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (cmp0>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1,  cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG );
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, temp);
}


void endat_interrupt_config(struct endat_periodic_interface *endat_periodic_interface)
{
    struct endat_pruss_xchg *pruss_xchg = endat_periodic_interface->pruss_dmem;
    int32_t status;
    if(endat_periodic_interface->load_share)
    {
        if(pruss_xchg->config[0].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM;
            hwiPrms.callback    = &pruEnDatIrqHandler0;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
        if(pruss_xchg->config[1].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM + 1;
            hwiPrms.callback    = &pruEnDatIrqHandler1;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoderHwiObject1, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
        if(pruss_xchg->config[2].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM + 2;
            hwiPrms.callback    = &pruEnDatIrqHandler2;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoderHwiObject2, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
    }
    else
    {
        /* Register & enable ICSSG EnDat PRU FW interrupt */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM;
        hwiPrms.callback    = &pruEnDatIrqHandler0;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);

    }

}    
uint32_t  endat_config_periodic_mode(struct endat_periodic_interface *endat_periodic_interface, PRUICSS_Handle handle)
{
    int32_t  status;
    gPruIcssXHandle = handle;
    gPruss_iep = endat_periodic_interface->pruss_iep;
    /*configure IEP*/
    endat_config_iep(endat_periodic_interface);
    /* Initialize ICSS INTC */
    status = PRUICSS_intcInit(gPruIcssXHandle, &gPruicssIntcInitdata);
        if (status != SystemP_SUCCESS)
        {
            return 0;
        }
    /*config Interrupt*/
    endat_interrupt_config(endat_periodic_interface);
    return 1;

}

void endat_stop_periodic_continuous_mode(struct endat_periodic_interface *endat_periodic_interface)
{
    /*reset iep timer*/
    void *pruss_iep = endat_periodic_interface->pruss_iep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, temp);
}

/* PRU EnDat FW IRQ handler */
void pruEnDatIrqHandler0(void *args)
{
    
    /* debug, inncrement PRU SDFM IRQ count */
    gPruEnDatIrqCnt0++;

    /* clear Cmp3 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP3_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDFM_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_SDFM_EVT0);
    
}
/* PRU EnDat FW IRQ handler */
void pruEnDatIrqHandler1(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPruEnDatIrqCnt1++;

    /* clear Cmp5 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP5_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDFM_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */

    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_SDFM_EVT1);


}
/* PRU EnDat FW IRQ handler */
void pruEnDatIrqHandler2(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPruEnDatIrqCnt2++;

    /* clear Cmp6 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP6_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDFM_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_SDFM_EVT2);


}
