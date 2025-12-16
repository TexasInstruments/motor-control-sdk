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

/**
 * \file  bissc_periodic_trigger.c
 *
 * \brief BiSS-C periodic trigger mode implementation using IEP timer
 *
 * This file implements periodic trigger mode for BiSS-C encoder interface.
 * In periodic mode, encoder position command is automatically triggered at regular
 * intervals by PRU using the PRU-ICSS Industrial Ethernet Peripheral (IEP) timer,
 * eliminating the need for host (R5F) intervention to trigger a command. After the
 * response is received, PRU triggers host (R5F) interrupt.
 *
 * \par IEP Timer Configuration:
 * The IEP timer is a PRU-ICSS instance-level resource shared between slices.
 * Therefore, IEP configuration uses the first handle (CONFIG_BISSC0) to access
 * the PRU-ICSS hardware attributes, regardless of how many slices are active.
 * Each slice/instance can have different trigger counts per channel, but they
 * share the same IEP reset count (period).
 * - Trigger Count: IEP counter value when BiSS-C transaction is initiated
 * - Reset Count: IEP counter value when counter resets to 0 (defines period)
 *
 * \par First instance (CONFIG_BISSC0) is used for shared resources:
 * Several operations use gAppBisscHandle[CONFIG_BISSC0] to access shared PRU-ICSS
 * resources:
 * 1. IEP timer configuration (bissc_config_iep()): IEP is PRU-ICSS instance-level,
 *    not slice-specific. Using first handle ensures consistent access.
 * 2. INTC initialization (bissc_config_periodic_mode()): INTC is initialized once
 *    per PRU-ICSS instance, not per slice.
 * 3. This approach works correctly because validation in bissc_pruicss_init()
 *    ensures both instances use the same PRU-ICSS instance.
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
#define RTU_TRIGGER_HOST_BISSC_EVT      (2+16)
/** \brief PRU BiSS-C interrupt event number (19 = 3 + 16) */
#define PRU_TRIGGER_HOST_BISSC_EVT      (3+16)
/** \brief TX-PRU BiSS-C interrupt event number (20 = 4 + 16) */
#define TXPRU_TRIGGER_HOST_BISSC_EVT    (4+16)
#else
/** \brief PRU BiSS-C interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_BISSC_EVT      (2+16)
#endif

/** \brief IEP Compare event number for Channel 0 trigger */
#define IEP_CH0_CMP_EVENT               (3)

/** \brief IEP Compare event number for Channel 1 trigger */
#define IEP_CH1_CMP_EVENT               (5)

/** \brief IEP Compare event number for Channel 2 trigger */
#define IEP_CH2_CMP_EVENT               (6)

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other
 * combinations, update code and remove this line.
 */

#if (CONFIG_BISSC1_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_BISSC_INT_NUM_SECOND_SLICE         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#else
#define ICSS_PRU_BISSC_INT_NUM_SECOND_SLICE         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#endif

#if (CONFIG_BISSC1_PRUICSS_SLICE == 1)
#define IEP_CH0_CMP_EVENT_SECOND_SLICE              (3)
#define PRU_TRIGGER_HOST_BISSC_EVT_SECOND_SLICE     (2+16)
#else
#define IEP_CH0_CMP_EVENT_SECOND_SLICE              (4)
#define PRU_TRIGGER_HOST_BISSC_EVT_SECOND_SLICE     (3+16)
#endif /* CONFIG_BISSC1_PRUICSS_SLICE */

#endif /* BISSC_DUAL_PRU_SLICE_ENABLE */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gBisscHwiObject[CONFIG_BISSC_NUM_INSTANCES][BISSC_NUM_CH_PER_SLICE_MAX];
uint32_t gPruBisscIrqCnt[CONFIG_BISSC_NUM_INSTANCES][BISSC_NUM_CH_PER_SLICE_MAX] = {0};

/* PRU-ICSS INTC Configuration uses first BiSS-C instance */
/* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
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

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Only PRU IRQ
 * handler is defined.
 */
void bissc_pru_irq_handler_second_slice(void *pruicss_handle);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void bissc_config_iep(bissc_periodic_interface *bissc_periodic_interface)
{
    /* PRU-ICSS Level Global Configuration uses first BiSS-C handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
    const bissc_attrs *attrs = bissc_get_attrs(bissc_periodic_interface->handle[CONFIG_BISSC0]);
    bissc_priv *priv = bissc_get_priv(bissc_periodic_interface->handle[CONFIG_BISSC0]);
    void *pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    uint8_t temp;
    uint32_t event;
    uint32_t event_clear;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint64_t iep_reset_count = 0;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg */
    event = HW_RD_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

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
        if(attrs->channel0_enabled)
        {
            event |= (0x1 << (IEP_CH0_CMP_EVENT + 1));
            event_clear |= (0x1 << (IEP_CH0_CMP_EVENT));

            cmp_reg0 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][0] & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][0]>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVENT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVENT*8,  cmp_reg1);
        }
        if(attrs->channel1_enabled)
        {
            event |= (0x1 << (IEP_CH1_CMP_EVENT + 1));
            event_clear |= (0x1 << (IEP_CH1_CMP_EVENT));

            cmp_reg0 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][1] & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][1]>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH1_CMP_EVENT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH1_CMP_EVENT*8,  cmp_reg1);
        }
        if(attrs->channel2_enabled)
        {
            event |= (0x1 << (IEP_CH2_CMP_EVENT + 1));
            event_clear |= (0x1 << (IEP_CH2_CMP_EVENT));

            cmp_reg0 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][2] & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][2]>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH2_CMP_EVENT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH2_CMP_EVENT*8,  cmp_reg1);
        }
    }
    else
    {
        event |= (0x1 << (IEP_CH0_CMP_EVENT + 1));
        event_clear |= (0x1 << (IEP_CH0_CMP_EVENT));
        cmp_reg0 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][0] & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC0][0]>>32 & 0xffffffff);

        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVENT*8,  cmp_reg0);
        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVENT*8,  cmp_reg1);

    }

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    event |= (0x1 << (IEP_CH0_CMP_EVENT_SECOND_SLICE + 1));
    event_clear |= (0x1 << (IEP_CH0_CMP_EVENT_SECOND_SLICE));
    cmp_reg0 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC1][0] & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (bissc_periodic_interface->periodic_trigger_count[CONFIG_BISSC1][0]>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVENT_SECOND_SLICE*8,  cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVENT_SECOND_SLICE*8,  cmp_reg1);
#endif

    /*clear event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    iep_reset_count = bissc_periodic_interface->iep_reset_count;

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
    /* PRU-ICSS Level Global Configuration uses first BiSS-C handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
    bissc_priv *priv = bissc_get_priv(bissc_periodic_interface->handle[CONFIG_BISSC0]);
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
    status              = HwiP_construct(&gBisscHwiObject[CONFIG_BISSC0][0], &hwi_params);
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
    status              = HwiP_construct(&gBisscHwiObject[CONFIG_BISSC0][1], &hwi_params);
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
    status              = HwiP_construct(&gBisscHwiObject[CONFIG_BISSC0][2], &hwi_params);
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
    status              = HwiP_construct(&gBisscHwiObject[CONFIG_BISSC0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_BISSC_INT_NUM_SECOND_SLICE;
    hwi_params.callback = &bissc_pru_irq_handler_second_slice;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gBisscHwiObject[CONFIG_BISSC1][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
}

int32_t bissc_config_periodic_mode(bissc_periodic_interface *bissc_periodic_interface)
{
    int32_t     status;
    uint32_t    i;
    bissc_priv  *priv;
    void        *pruicss_handle;

    /* NULL check on interface pointer and handle(s) */
    if(bissc_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {
        if(bissc_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first BiSS-C handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
    priv = bissc_get_priv(bissc_periodic_interface->handle[CONFIG_BISSC0]);
    pruicss_handle = (void *)(priv->pruicss_handle);

    /* Configure IEP */
    bissc_config_iep(bissc_periodic_interface);

    /* Initialize PRU-ICSS Interrupt Controller */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
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
    /* Configure Interrupts */
    bissc_interrupt_config(bissc_periodic_interface);
    return SystemP_SUCCESS;

}

int32_t bissc_stop_periodic_mode(bissc_periodic_interface *bissc_periodic_interface)
{
    bissc_priv *priv;
    void *pruicss_iep;
    uint8_t temp;
    uint32_t i;

    /* NULL check on interface pointer and handle(s) */
    if(bissc_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {
        if(bissc_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first BiSS-C handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
    priv = bissc_get_priv(bissc_periodic_interface->handle[CONFIG_BISSC0]);
    pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);

    /*Stop IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    HwiP_destruct(&gBisscHwiObject[CONFIG_BISSC0][0]);
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    HwiP_destruct(&gBisscHwiObject[CONFIG_BISSC0][1]);
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    HwiP_destruct(&gBisscHwiObject[CONFIG_BISSC0][2]);
#endif
#else
    HwiP_destruct(&gBisscHwiObject[CONFIG_BISSC0][0]);
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    HwiP_destruct(&gBisscHwiObject[CONFIG_BISSC1][0]);
#endif
    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void bissc_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* In load share mode, index 1 is used for channel 1 connected to PRU */
    gPruBisscIrqCnt[CONFIG_BISSC0][1]++;
#else
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruBisscIrqCnt[CONFIG_BISSC0][0]++;
#endif
    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_BISSC_EVT);
}

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
/* RTU-PRU FW IRQ handler */
void bissc_rtupru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruBisscIrqCnt[CONFIG_BISSC0][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, RTU_TRIGGER_HOST_BISSC_EVT);

}

/* TX-PRU FW IRQ handler */
void bissc_txpru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruBisscIrqCnt[CONFIG_BISSC0][2]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, TXPRU_TRIGGER_HOST_BISSC_EVT);

}
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Only PRU IRQ
 * handler is defined.
 */

/* PRU FW IRQ handler */
void bissc_pru_irq_handler_second_slice(void *pruicss_handle)
{
    /* Increment IRQ count */
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruBisscIrqCnt[CONFIG_BISSC1][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_BISSC_EVT);
}
#endif