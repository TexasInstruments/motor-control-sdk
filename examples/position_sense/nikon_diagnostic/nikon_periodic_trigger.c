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
#include "nikon_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/nikon/include/nikon_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

HwiP_Params hwiPrms;
static HwiP_Object gIcssEncoderHwiObject0;  /* ICSS NIKON PRU FW HWI */
static HwiP_Object gIcssEncoderHwiObject1;  /* ICSS NIKON PRU FW HWI */
static HwiP_Object gIcssEncoderHwiObject2;  /* ICSS NIKON PRU FW HWI */
struct nikon_priv *priv;
/* ICSS Interrupt settings */
#ifdef PRUICSSM
#if (CONFIG_NIKON0_PRUICSSx == 1)
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_NIKON0_PRUICSSx == 1)
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#endif
#endif
uint32_t gRtuNikonIrqCnt, gPruNikonIrqCnt, gTxpruNikonIrqCnt;

/*global variable */
void *gPruIcss_iep;

PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
#if(CONFIG_NIKON0_PRUICSSx == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

void nikon_config_iep(struct nikon_periodic_interface *nikon_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = nikon_periodic_interface->pruicss_iep;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;
    uint64_t iep_reset_count = 0;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /*enable IEP reset by cmp0 event*/
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /*set IEP counter to ZERO*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /*Clear all event & configure*/
    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
    {
        event |= CONFIG_NIKON0_CHANNEL0 == 1 ? (0x1 << (IEP_CH0_CMP_EVNT + 1)):0;
        event |= CONFIG_NIKON0_CHANNEL1 == 1 ? (0x1 << (IEP_CH1_CMP_EVNT + 1)):0;
        event |= CONFIG_NIKON0_CHANNEL2 == 1 ? (0x1 << (IEP_CH2_CMP_EVNT + 1)):0;

        /*clear event*/
        event_clear |= CONFIG_NIKON0_CHANNEL0 == 1 ? (0x1 << (IEP_CH0_CMP_EVNT)):0;
        event_clear |= CONFIG_NIKON0_CHANNEL1 == 1 ? (0x1 << (IEP_CH1_CMP_EVNT)):0;
        event_clear |= CONFIG_NIKON0_CHANNEL2 == 1 ? (0x1 << (IEP_CH2_CMP_EVNT)):0;

        if(CONFIG_NIKON0_CHANNEL0)
        {
            cmp_reg0 = (nikon_periodic_interface->ch0_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (nikon_periodic_interface->ch0_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8,  cmp_reg1);
        }

        if(CONFIG_NIKON0_CHANNEL1)
        {
            cmp_reg0 = (nikon_periodic_interface->ch1_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (nikon_periodic_interface->ch1_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH1_CMP_EVNT*8, cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH1_CMP_EVNT*8, cmp_reg1);

        }

        if(CONFIG_NIKON0_CHANNEL2)
        {
            cmp_reg0 = (nikon_periodic_interface->ch2_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (nikon_periodic_interface->ch2_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH2_CMP_EVNT*8, cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH2_CMP_EVNT*8,  cmp_reg1);

        }
    }
    else
    {
        event |= (0x1 << (IEP_CH0_CMP_EVNT + 1));
        event_clear |= (0x1 << (IEP_CH0_CMP_EVNT));
        cmp_reg0 = (nikon_periodic_interface->ch0_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (nikon_periodic_interface->ch0_trigger_count>>32 & 0xffffffff);

        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8,  cmp_reg0);
        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8,  cmp_reg1);

    }
    iep_reset_count = nikon_periodic_interface->iep_reset_count;

    /*clear event*/
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (iep_reset_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (iep_reset_count>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1,  cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}


void nikon_interrupt_config(struct nikon_periodic_interface *nikon_periodic_interface)
{
    int32_t status;
    /* Register & enable ICSS nikon PRU FW interrupt */
    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
    {
        if(CONFIG_NIKON0_CHANNEL0)
        {
            /* Register & enable ICSSG bissc PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_RTU_NIKON_INT_NUM;
            hwiPrms.callback    = &rtu_nikon_irq_handler;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssEncoderHwiObject0, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
        }
        if(CONFIG_NIKON0_CHANNEL1)
        {
            /* Register & enable ICSSG bissc PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_PRU_NIKON_INT_NUM;
            hwiPrms.callback    = &pru_nikon_irq_handler;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssEncoderHwiObject1, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
        }
        if(CONFIG_NIKON0_CHANNEL2)
        {
            /* Register & enable ICSSG bissc PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_TXPRU_NIKON_INT_NUM;
            hwiPrms.callback    = &txpru_nikon_irq_handler;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssEncoderHwiObject2, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
        }
    }
    else
    {
        /* Register & enable ICSSG bissc PRU FW interrupt */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_RTU_NIKON_INT_NUM;
        hwiPrms.callback    = &rtu_nikon_irq_handler;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssEncoderHwiObject0, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);
    }

}
uint32_t nikon_config_periodic_mode(struct nikon_periodic_interface *nikon_periodic_interface, PRUICSS_Handle handle)
{
    int32_t status;
    gPruIcssXHandle = handle;
    gPruIcss_iep = nikon_periodic_interface->pruicss_iep;
    /*configure IEP*/
    nikon_config_iep(nikon_periodic_interface);
    /* Initialize ICSS INTC */
#if(CONFIG_NIKON0_PRUICSSx == 1)
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
    nikon_interrupt_config(nikon_periodic_interface);
    return 1;

}

void nikon_stop_periodic_mode(struct nikon_periodic_interface *nikon_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = nikon_periodic_interface->pruicss_iep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

/* RTU Nikon FW IRQ handler */
void rtu_nikon_irq_handler(void *args)
{
    /* Increment RTU NIKON IRQ count */
    gRtuNikonIrqCnt++;

    /* Clear interrupt at source */
    /* Write 18 to ICSS STATUS CLR INDEX Register
        18 = 16+2, 2 is Host Interrupt Number. See TRM for more details.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, RTU_TRIGGER_HOST_EVT);
}

/* PRU Nikon FW IRQ handler */
void pru_nikon_irq_handler(void *args)
{
    /* Increment PRU NIKON IRQ count */
    gPruNikonIrqCnt++;

    /* Clear interrupt at source */
    /* Write 19 to ICSS STATUS CLR INDEX Register
        19 = 16+3, 3 is Host Interrupt Number. See TRM for more details.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_EVT);
}

/* TXPRU Nikon FW IRQ handler */
void txpru_nikon_irq_handler(void *args)
{
    /* Increment TXPRU NIKON IRQ count */
    gTxpruNikonIrqCnt++;

    /* Clear interrupt at source */
    /* Write 20 to ICSS STATUS CLR INDEX Register
        20 = 16+4, 4 is Host Interrupt Number. See TRM for more details.
    */
    PRUICSS_clearEvent(gPruIcssXHandle, TXPRU_TRIGGER_HOST_EVT);
}

void nikon_periodic_interface_init(struct nikon_priv *priv, struct nikon_periodic_interface *nikon_periodic_interface, int64_t iep_reset_count, int64_t ch0_trigger_count, int64_t ch1_trigger_count, int64_t ch2_trigger_count)
{
    nikon_periodic_interface->pruicss_iep = priv->pruicss_iep;
    nikon_periodic_interface->ch0_trigger_count = ch0_trigger_count;
    nikon_periodic_interface->iep_reset_count = iep_reset_count;
    nikon_periodic_interface->ch1_trigger_count= ch1_trigger_count;
    nikon_periodic_interface->ch2_trigger_count = ch2_trigger_count;
}
