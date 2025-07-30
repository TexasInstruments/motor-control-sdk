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


#include<stdio.h>
#include<stdint.h>
#include<math.h>

#include <drivers/pruicss.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/tistdtypes.h>
#include <kernel/dpl/ClockP.h>
#include "bissc_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/bissc/include/bissc_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

HwiP_Params hwiPrms;
static HwiP_Object gIcssgEncoderHwiObject0;  /* ICSSG BiSS-C PRU FW HWI */
static HwiP_Object gIcssgEncoderHwiObject1;  /* ICSSG BiSS-C PRU FW HWI */
static HwiP_Object gIcssgEncoderHwiObject2;  /* ICSSG BiSS-C PRU FW HWI */

/* ICSSG Interrupt settings */
#ifdef PRUICSSM
#if (CONFIG_BISSC0_PRUICSSx == 1)
#define ICSS_PRU_BISSC_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0 )
#else
#define ICSS_PRU_BISSC_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0 )
#endif
#else
#if (CONFIG_BISSC0_PRUICSSx == 1)
#define ICSS_PRU_BISSC_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0 )
#else
#define ICSS_PRU_BISSC_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
#endif
#endif
uint32_t gPrubisscIrqCnt0, gPrubisscIrqCnt1, gPrubisscIrqCnt2;

/*global variable */
void *gPruIcss_iep;

PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
#if (CONFIG_BISSC0_PRUICSSx == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

void bissc_config_iep(struct bissc_periodic_interface *bissc_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = bissc_periodic_interface->pruicss_iep;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;
    uint64_t cmp0 = 0;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /*enable IEP reset by cmp0 event*/
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /*set IEP counter to ZERO*/
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /*Clear all event & configure*/
    if(CONFIG_BISSC0_LOAD_SHARE_MODE)
    {
        event |= CONFIG_BISSC0_CHANNEL0==1?(0x1 << 4 ):0; //CMP3 for ch0
        event |= CONFIG_BISSC0_CHANNEL1==1?(0x1 << 6 ):0;  //CMP5 for Ch1
        event |= CONFIG_BISSC0_CHANNEL2==1?(0x1 << 7 ):0;  //CMP6 for CH2

        /*clear event*/
        event_clear |= CONFIG_BISSC0_CHANNEL0==1?(0x1 << 3 ):0; //CMP3 for ch0
        event_clear |= CONFIG_BISSC0_CHANNEL1==1?(0x1 << 5 ):0;  //CMP5 for Ch1
        event_clear |= CONFIG_BISSC0_CHANNEL2==1?(0x1 << 6 ):0;  //CMP6 for CH2

        if(CONFIG_BISSC0_CHANNEL0)
        {
            cmp_reg0 = (bissc_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->cmp3>>32 & 0xffffffff);


            HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG0,  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG1,  cmp_reg1);


        }

        if(CONFIG_BISSC0_CHANNEL1)
        {
            cmp_reg0 = (bissc_periodic_interface->cmp5 & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->cmp5>>32 & 0xffffffff);


            HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP5_REG0,  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP5_REG1,  cmp_reg1);


        }

        if(CONFIG_BISSC0_CHANNEL2)
        {
            cmp_reg0 = (bissc_periodic_interface->cmp6 & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->cmp6>>32 & 0xffffffff);


            HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP6_REG0,  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP6_REG1,  cmp_reg1);


        }
    }
    else
    {
        event |= (0x1 << 4 );
        event_clear |= (0x1 << 3);
        cmp_reg0 = (bissc_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (bissc_periodic_interface->cmp3>>32 & 0xffffffff);


        HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG0,  cmp_reg0);
        HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG1,  cmp_reg1);

    }
    cmp0 = bissc_periodic_interface->cmp0;

    /*clear event*/
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (cmp0 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (cmp0>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0, cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1, cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}


void bissc_interrupt_config(struct bissc_periodic_interface *bissc_periodic_interface)
{
    int32_t status;
    if(CONFIG_BISSC0_LOAD_SHARE_MODE)
    {
        if(CONFIG_BISSC0_CHANNEL0)
        {
            /* Register & enable ICSSG bissc PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_PRU_BISSC_INT_NUM;
            hwiPrms.callback    = &prubisscIrqHandler0;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
        if(CONFIG_BISSC0_CHANNEL1)
        {
            /* Register & enable ICSSG bissc PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_PRU_BISSC_INT_NUM + 1;
            hwiPrms.callback    = &prubisscIrqHandler1;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoderHwiObject1, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
        if(CONFIG_BISSC0_CHANNEL2)
        {
            /* Register & enable ICSSG bissc PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_PRU_BISSC_INT_NUM + 2;
            hwiPrms.callback    = &prubisscIrqHandler2;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoderHwiObject2, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
    }
    else
    {
        /* Register & enable ICSSG bissc PRU FW interrupt */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_PRU_BISSC_INT_NUM;
        hwiPrms.callback    = &prubisscIrqHandler0;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);

    }

}
uint32_t bissc_config_periodic_mode(struct bissc_periodic_interface *bissc_periodic_interface, PRUICSS_Handle handle)
{
    int32_t  status;
    gPruIcssXHandle = handle;
    gPruIcss_iep = bissc_periodic_interface->pruicss_iep;
    /*configure IEP*/
    bissc_config_iep(bissc_periodic_interface);
    /* Initialize ICSS INTC */
#if (CONFIG_BISSC0_PRUICSSx == 1)
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss1_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return 0;
    }
#else
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return 0;
    }
#endif
    /*config Interrupt*/
    bissc_interrupt_config(bissc_periodic_interface);
    return 1;

}

void bissc_stop_periodic_mode(struct bissc_periodic_interface *bissc_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = bissc_periodic_interface->pruicss_iep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

/* PRU bissc FW IRQ handler */
void prubisscIrqHandler0(void *args)
{

    /* debug, inncrement PRU SDFM IRQ count */
    gPrubisscIrqCnt0++;

    /* clear Cmp3 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP3_EVNT;
    HW_WR_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        18 = 16+2, 2 is Host Interrupt Number. See AM243x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_BISSC_EVT0);

}

void prubisscIrqHandler1(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPrubisscIrqCnt1++;

    /* clear Cmp3 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP5_EVNT;
    HW_WR_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 19 to ICSSG_STATUS_CLR_INDEX_REG
        19 = 16+3, 3 is Host Interrupt Number. See AM243x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_BISSC_EVT1);
}

void prubisscIrqHandler2(void *args)
{

    /* debug, inncrement PRU SDFM IRQ count */
    gPrubisscIrqCnt2++;

    /* clear Cmp3 event*/
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP6_EVNT;
    HW_WR_REG8((uint8_t*)gPruIcss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    /* Write 20 to ICSSG_STATUS_CLR_INDEX_REG
        20 = 16+4, 4 is Host Interrupt Number. See AM243x TRM.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_BISSC_EVT2);

}

void bissc_periodic_interface_init(struct bissc_priv *priv, struct bissc_periodic_interface *bissc_periodic_interface, int64_t cmp3,
    int64_t cmp5, int64_t cmp6, int64_t cmp0)
{
    bissc_periodic_interface->pruicss_iep = priv->pruicss_iep;
    bissc_periodic_interface->cmp3 = cmp3;
    bissc_periodic_interface->cmp5 = cmp5;
    bissc_periodic_interface->cmp6 = cmp6;
    bissc_periodic_interface->cmp0 = cmp0;
}
