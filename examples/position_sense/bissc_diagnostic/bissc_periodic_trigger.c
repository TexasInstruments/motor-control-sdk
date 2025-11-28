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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_BISSC_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_BISSC_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_BISSC0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#else
/* ICSSG Interrupt Numbers */
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_BISSC_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_BISSC_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_BISSC0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_BISSC_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
/** \brief RTU-PRU BiSS-C interrupt event number (18 = 2 + 16) */
#define RTU_TRIGGER_HOST_BISSC_EVT      ( 2+16 )
/** \brief PRU BiSS-C interrupt event number (19 = 3 + 16) */
#define PRU_TRIGGER_HOST_BISSC_EVT      ( 3+16 )
/** \brief TX-PRU BiSS-C interrupt event number (20 = 4 + 16) */
#define TXPRU_TRIGGER_HOST_BISSC_EVT    ( 4+16 )
#else
/** \brief PRU BiSS-C interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_BISSC_EVT      ( 2+16 )
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gBisscHwiObject[BISSC_NUM_CH_PER_SLICE_MAX];

uint32_t gRtuBisscIrqCnt = 0;
uint32_t gPruBisscIrqCnt = 0;
uint32_t gTxpruBisscIrqCnt = 0;

/* ICSS INTC configuration */
#if (CONFIG_BISSC0_PRUICSS_INSTANCE == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void bissc_config_iep(bissc_periodic_interface *bissc_periodic_interface);

static void bissc_interrupt_config(bissc_periodic_interface *bissc_periodic_interface);

void bissc_pru_irq_handler(void *pruicss_handle);

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
void bissc_rtupru_irq_handler(void *pruicss_handle);
void bissc_txpru_irq_handler(void *pruicss_handle);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void bissc_config_iep(bissc_periodic_interface *bissc_periodic_interface)
{
    const bissc_attrs *attrs = bissc_get_attrs(bissc_periodic_interface->handle);
    bissc_priv *priv = bissc_get_priv(bissc_periodic_interface->handle);
    void *pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
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
    if(attrs->load_share_enabled)
    {
        event |= attrs->channel0_enabled ? (0x1 << (IEP_CH0_CMP_EVNT + 1)):0;
        event |= attrs->channel1_enabled ? (0x1 << (IEP_CH1_CMP_EVNT + 1)):0;
        event |= attrs->channel2_enabled ? (0x1 << (IEP_CH2_CMP_EVNT + 1)):0;

        /*clear event*/
        event_clear |= attrs->channel0_enabled ? (0x1 << (IEP_CH0_CMP_EVNT)):0;
        event_clear |= attrs->channel1_enabled ? (0x1 << (IEP_CH1_CMP_EVNT)):0;
        event_clear |= attrs->channel2_enabled ? (0x1 << (IEP_CH2_CMP_EVNT)):0;

        if(attrs->channel0_enabled)
        {
            cmp_reg0 = (bissc_periodic_interface->ch0_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->ch0_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8,  cmp_reg1);

        }

        if(attrs->channel1_enabled)
        {
            cmp_reg0 = (bissc_periodic_interface->ch1_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->ch1_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH1_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH1_CMP_EVNT*8,  cmp_reg1);

        }

        if(attrs->channel2_enabled)
        {
            cmp_reg0 = (bissc_periodic_interface->ch2_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->ch2_trigger_count>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH2_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH2_CMP_EVNT*8,  cmp_reg1);

        }
    }
    else
    {
        event |= (0x1 << (IEP_CH0_CMP_EVNT + 1));
        event_clear |= (0x1 << (IEP_CH0_CMP_EVNT));
        cmp_reg0 = (bissc_periodic_interface->ch0_trigger_count & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (bissc_periodic_interface->ch0_trigger_count>>32 & 0xffffffff);

        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8,  cmp_reg0);
        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8,  cmp_reg1);

    }
    iep_reset_count = bissc_periodic_interface->iep_reset_count;

    /*clear event*/
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /*configure cmp0 registers*/
    cmp_reg0 = (iep_reset_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (iep_reset_count>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0, cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1, cmp_reg1);


    /*write IEP default increment & IEP start*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}


static void bissc_interrupt_config(bissc_periodic_interface *bissc_periodic_interface)
{
    bissc_priv *priv = bissc_get_priv(bissc_periodic_interface->handle);
    void *pruicss_handle = (void *)(priv->pruicss_handle);
    int32_t status;
    HwiP_Params hwi_params;

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    /* Register and enable RTU-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_RTU_BISSC_INT_NUM;
    hwi_params.callback = &bissc_rtupru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gBisscHwiObject[0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_BISSC_INT_NUM;
    hwi_params.callback = &bissc_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gBisscHwiObject[1], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)

    /* Register and enable TX-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_TXPRU_BISSC_INT_NUM;
    hwi_params.callback = &bissc_txpru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gBisscHwiObject[2], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#else
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_BISSC_INT_NUM;
    hwi_params.callback = &bissc_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gBisscHwiObject[0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
}

uint32_t bissc_config_periodic_mode(bissc_periodic_interface *bissc_periodic_interface)
{
    int32_t  status;
    bissc_priv *priv = bissc_get_priv(bissc_periodic_interface->handle);
    void *pruicss_handle = priv->pruicss_handle;
    
    /* Configure IEP*/
    bissc_config_iep(bissc_periodic_interface);
    
    /* Initialize PRU-ICSS Interrupt Controller */
#if (CONFIG_BISSC0_PRUICSS_INSTANCE == 1)
    status = PRUICSS_intcInit(pruicss_handle, &icss1_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return status;
    }
#else
    status = PRUICSS_intcInit(pruicss_handle, &icss0_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return status;
    }
#endif
    /*config Interrupt*/
    bissc_interrupt_config(bissc_periodic_interface);
    return SystemP_SUCCESS;

}

void bissc_stop_periodic_mode(bissc_periodic_interface *bissc_periodic_interface)
{
    bissc_priv *priv = bissc_get_priv(bissc_periodic_interface->handle);
    void *pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    uint8_t temp;
    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

/* PRU FW IRQ handler */
void bissc_pru_irq_handler(void *pruicss_handle)
{
    /* Increment PRU NIKON IRQ count */
    gPruBisscIrqCnt++;

    /* Clear interrupt at source */
    /* Write 19 to ICSS_STATUS_CLR_INDEX_REG
        19 = 16+3, 3 is Host Interrupt Number. See TRM for more details.
    */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_BISSC_EVT);
}

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
/* RTU-PRU FW IRQ handler */
void bissc_rtupru_irq_handler(void *pruicss_handle)
{
    /* Increment RTU NIKON IRQ count */
    gRtuBisscIrqCnt++;

    /* Clear interrupt at source */
    /* Write 18 to ICSS_STATUS_CLR_INDEX_REG
        18 = 16+2, 2 is Host Interrupt Number. See TRM for more details.
    */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, RTU_TRIGGER_HOST_BISSC_EVT);

}

/* TX-PRU FW IRQ handler */
void bissc_txpru_irq_handler(void *pruicss_handle)
{
    /* Increment TXPRU NIKON IRQ count */
    gTxpruBisscIrqCnt++;

    /* Clear interrupt at source */
    /* Write 20 to ICSS_STATUS_CLR_INDEX_REG
        20 = 16+4, 4 is Host Interrupt Number. See TRM for more details.
    */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, TXPRU_TRIGGER_HOST_BISSC_EVT);

}
#endif

void bissc_periodic_interface_init(bissc_handle handle, bissc_periodic_interface *bissc_periodic_interface_instance, int64_t ch0_trigger_count,
    int64_t ch1_trigger_count, int64_t ch2_trigger_count, int64_t iep_reset_count)
{
    bissc_periodic_interface_instance->handle = handle;
    bissc_periodic_interface_instance->ch0_trigger_count = ch0_trigger_count;
    bissc_periodic_interface_instance->ch1_trigger_count = ch1_trigger_count;
    bissc_periodic_interface_instance->ch2_trigger_count = ch2_trigger_count;
    bissc_periodic_interface_instance->iep_reset_count = iep_reset_count;
}
