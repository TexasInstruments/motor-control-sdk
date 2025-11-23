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


static HwiP_Object gIcssgEncoderHwiObject0;  /* ICSSG Tamagawa PRU FW HWI */

/* ICSSG Interrupt settings */
#if (SOC_AM261X || SOC_AM263PX || SOC_AM263X)
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
#define ICSS_PRU_TAMAGAWA_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0 )
#else
#define ICSS_PRU_TAMAGAWA_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0 )
#endif
#else
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
#define ICSS_PRU_TAMAGAWA_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0 )
#else
#define ICSS_PRU_TAMAGAWA_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
#endif
#endif
uint32_t gPrutamagawaIrqCnt0;

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if (SOC_AM261X || SOC_AM263PX || SOC_AM263X)
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
#define ICSS_PRU_TAMAGAWA_DUAL_CHANNEL_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1 )
#else
#define ICSS_PRU_TAMAGAWA_DUAL_CHANNEL_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1 )
#endif
#else
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
#define ICSS_PRU_TAMAGAWA_DUAL_CHANNEL_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1 )
#else
#define ICSS_PRU_TAMAGAWA_DUAL_CHANNEL_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1 )
#endif
#endif
uint32_t gPrutamagawaDualChannelIrqCnt0;

static HwiP_Object gIcssgEncoder1HwiObject0; 
#endif 

/*global variable */
void *gPruIcssIep;

PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

void tamagawa_config_iep(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = gPruIcssIep;
    uint8_t temp;
    uint16_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /*enable IEP reset by cmp0 event*/
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /*set IEP counter to ZERO*/
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /*configure cmp registers*/
    event |= (0x1 << (IEP_CMP_EVENT + 1));
    event_clear |= (0x1 << IEP_CMP_EVENT);

    cmp_reg0 = (tamagawa_periodic_interface->periodic_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->periodic_trigger_count>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + 8*IEP_CMP_EVENT),  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + 8*IEP_CMP_EVENT),  cmp_reg1);

    /*clear event*/
    HW_WR_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (tamagawa_periodic_interface->iep_reset_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->iep_reset_count>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1,  cmp_reg1);

    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
void tamagawa1_config_iep(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = gPruIcssIep;
    uint16_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;

    /* cmp cfg reg */
    event = HW_RD_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    event |= (0x1 << (DUAL_CH_IEP_CMP_EVENT + 1));
    event_clear |= (0x1 << DUAL_CH_IEP_CMP_EVENT);


    cmp_reg0 = (tamagawa_periodic_interface->periodic_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->periodic_trigger_count>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + 8*DUAL_CH_IEP_CMP_EVENT),  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + 8*DUAL_CH_IEP_CMP_EVENT),  cmp_reg1);

    /*clear event*/
    HW_WR_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG16((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}
#endif

void tamagawa_interrupt_config(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    int32_t status;
    HwiP_Params hwiPrms;
    /* Register & enable ICSSG tamagawa PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSS_PRU_TAMAGAWA_INT_NUM;
    hwiPrms.callback    = &pruTamagawaIrqHandler0;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
}

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
void tamagawa1_interrupt_config(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    int32_t status;
    HwiP_Params hwiPrms;
    /* Register & enable ICSSG tamagawa PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSS_PRU_TAMAGAWA_DUAL_CHANNEL_INT_NUM;
    hwiPrms.callback    = &pruTamagawaDualChannelIrqHandler0;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoder1HwiObject0, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
}
  
#endif

uint32_t tamagawa_config_periodic_mode(struct tamagawa_periodic_interface *tamagawa_periodic_interface, PRUICSS_Handle handle, uint8_t tamagawa_instnace)
{
    int32_t  status;
    gPruIcssXHandle = handle;
#if TAMAGAWA_PERIODIC_MODE_IEP_INSTANCE == 0
    gPruIcssIep =  (void *)(((PRUICSS_HwAttrs *)(handle->hwAttrs))->iep0RegBase);
#else
    gPruIcssIep =  (void *)(((PRUICSS_HwAttrs *)(handle->hwAttrs))->iep1RegBase);
#endif
    /*configure IEP*/
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    if(tamagawa_instnace == 1)
    {
        tamagawa1_config_iep(tamagawa_periodic_interface);
        /*config Interrupt*/
        tamagawa1_interrupt_config(tamagawa_periodic_interface);
    }
    else
    {
        tamagawa_config_iep(tamagawa_periodic_interface);
        /*config Interrupt*/
        tamagawa_interrupt_config(tamagawa_periodic_interface);
        /* Initialize ICSS INTC */
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
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
    }
#else
    tamagawa_config_iep(tamagawa_periodic_interface);
    /*config Interrupt*/
    tamagawa_interrupt_config(tamagawa_periodic_interface);
    /* Initialize ICSS INTC */
#if (CONFIG_TAMAGAWA0_PRUICSSx == 1)
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
#endif
   
    return 1;
}

void tamagawa_stop_periodic_continuous_mode(struct tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /*reset iep timer*/
    void *pruicss_iep = gPruIcssIep;
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG );
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    HwiP_destruct(&gIcssgEncoderHwiObject0);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    HwiP_destruct(&gIcssgEncoder1HwiObject0);
#endif
}

/* PRU tamagawa FW IRQ handler */
void pruTamagawaIrqHandler0(void *args)
{

    /* inncrement PRU SDFM IRQ count */
    gPrutamagawaIrqCnt0++;

    /* clear Cmp event*/
    HW_WR_REG8((uint8_t*)gPruIcssIep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, 1 << IEP_CMP_EVENT);
    /* Clear interrupt at source */
    
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_TAMAGAWA_EVT0);

}

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
void pruTamagawaDualChannelIrqHandler0(void *args)
{

    /* inncrement PRU IRQ count */
    gPrutamagawaDualChannelIrqCnt0++;
    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_TAMAGAWA_DUAL_CH_EVT0);

}
#endif
