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
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

HwiP_Params hwiPrms;
static HwiP_Object gIcssgEncoderHwiObject0;  /* ICSSG EnDat PRU FW HWI */
static HwiP_Object gIcssgEncoderHwiObject1;  /* ICSSG EnDat PRU FW HWI */
static HwiP_Object gIcssgEncoderHwiObject2;  /* ICSSG EnDat PRU FW HWI */

/* ICSS Interrupt settings */
#ifdef PRUICSSM
#if (CONFIG_ENDAT0_PRUICSSx == 1)
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2 )
#else
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2 )
#endif
#else
#if (CONFIG_ENDAT0_PRUICSSx == 1)
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2 )
#else
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2 )
#endif
#endif
uint32_t gPruEnDatIrqCnt0;
uint32_t gPruEnDatIrqCnt1;
uint32_t gPruEnDatIrqCnt2;

/*global variable */
void *gPruss_iep; 



PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
#if (CONFIG_ENDAT0_PRUICSSx == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

void endat_config_iep(struct endat_periodic_interface *endat_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = endat_periodic_interface->pruicss_iep;
    struct endat_pruss_xchg *pruss_xchg = endat_periodic_interface->pruicss_dmem;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
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
    if(endat_periodic_interface->load_share)
    {
        /*enable event*/
        event |= pruss_xchg->config[0].channel==1?(0x1 << (IEP_CH0_CMP_EVNT + 1)):0; 
        event |= pruss_xchg->config[1].channel==2?(0x1 << (IEP_CH1_CMP_EVNT + 1)):0;  
        event |= pruss_xchg->config[2].channel==4?(0x1 << (IEP_CH2_CMP_EVNT + 1)):0;  

        /*clear event*/
        event_clear |= pruss_xchg->config[0].channel==1?(0x1 << IEP_CH0_CMP_EVNT):0; 
        event_clear |= pruss_xchg->config[1].channel==2?(0x1 << IEP_CH1_CMP_EVNT):0; 
        event_clear |= pruss_xchg->config[2].channel==4?(0x1 << IEP_CH2_CMP_EVNT):0;  

        if(pruss_xchg->config[0].channel)
        {
            cmp_reg0 = (endat_periodic_interface->ch0_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (endat_periodic_interface->ch0_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8),  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8),  cmp_reg1);
        }

        if(pruss_xchg->config[1].channel)
        {
            cmp_reg0 = (endat_periodic_interface->ch1_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (endat_periodic_interface->ch1_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH1_CMP_EVNT*8),  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH1_CMP_EVNT*8),  cmp_reg1);

        }

        if(pruss_xchg->config[2].channel)
        {
            cmp_reg0 = (endat_periodic_interface->ch2_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (endat_periodic_interface->ch2_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH2_CMP_EVNT*8),  cmp_reg0);
            HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH2_CMP_EVNT*8),  cmp_reg1);

        }

    }
    else
    {
        event |= (0x1 << (IEP_CH0_CMP_EVNT + 1));
        event_clear |= (0x1 << IEP_CH0_CMP_EVNT);
        cmp_reg0 = (endat_periodic_interface->ch0_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (endat_periodic_interface->ch0_trigger_count>>32 & 0xffffffff);

        HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8),  cmp_reg0);
        HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8),  cmp_reg1);

    }
    /*clear event*/
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (endat_periodic_interface->cmp0_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (endat_periodic_interface->cmp0_count>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1,  cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}


void endat_interrupt_config(struct endat_periodic_interface *endat_periodic_interface)
{
    struct endat_pruss_xchg *pruss_xchg = endat_periodic_interface->pruicss_dmem;
    int32_t status;
    if(endat_periodic_interface->load_share)
    {
        if(pruss_xchg->config[0].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_PRU_ENDAT_INT_NUM;
            hwiPrms.callback    = &pruEnDatIrqHandler;
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
            hwiPrms.intNum      = ICSS_RTU_ENDAT_INT_NUM;
            hwiPrms.callback    = &rtuEnDatIrqHandler;
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
            hwiPrms.intNum      = ICSS_TXPRU_ENDAT_INT_NUM;
            hwiPrms.callback    = &txpruEnDatIrqHandler;
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
        hwiPrms.intNum      = ICSS_PRU_ENDAT_INT_NUM;
        hwiPrms.callback    = &pruEnDatIrqHandler;
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
    gPruss_iep = endat_periodic_interface->pruicss_iep;
    /*configure IEP*/
    endat_config_iep(endat_periodic_interface);
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
    endat_interrupt_config(endat_periodic_interface);
    return 1;

}

void endat_stop_periodic_continuous_mode(struct endat_periodic_interface *endat_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = endat_periodic_interface->pruicss_iep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

/* PRU EnDat FW IRQ handler */
void pruEnDatIrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt0++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT0);
}
/* PRU EnDat FW IRQ handler */
void rtuEnDatIrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt1++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT1);
}
/* PRU EnDat FW IRQ handler */
void txpruEnDatIrqHandler(void *args)
{
    /*Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt2++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT2);
}
