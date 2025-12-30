/*
 *  Copyright (C) 2023-2025 Texas Instruments Incorporated
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
 * \file  tamagawa_periodic_trigger.c
 *
 * \brief Tamagawa periodic trigger mode implementation using IEP timer
 *
 * This file implements periodic trigger mode for Tamagawa encoder interface.
 * In periodic mode, encoder position command is automatically triggered at regular
 * intervals by PRU using the PRU-ICSS Industrial Ethernet Peripheral (IEP) timer,
 * eliminating the need for  host (R5F) intervention to trigger a command. After the
 * response is received, PRU triggers  host (R5F) interrupt.
 *
 * \par IEP Timer Configuration:
 * The IEP timer is a PRU-ICSS instance-level resource shared between slices.
 * Therefore, IEP configuration uses the first handle (CONFIG_TAMAGAWA0) to access
 * the PRU-ICSS hardware attributes, regardless of how many slices are active.
 * Each slice/instance can have different trigger counts per channel, but they
 * share the same IEP reset count (period).
 * - Trigger Count: IEP counter value when Tamagawa command is initiated
 * - Reset Count: IEP counter value when counter resets to 0 (defines period)
 *
 * \par First instance (CONFIG_TAMAGAWA0) is used for shared resources:
 * Several operations use gAppTamagawaHandle[CONFIG_TAMAGAWA0] to access shared PRU-ICSS
 * resources:
 * 1. IEP timer configuration (tamagawa_config_iep()): IEP is PRU-ICSS instance-level,
 *    not slice-specific. Using first handle ensures consistent access.
 * 2. INTC initialization (tamagawa_config_periodic_mode()): INTC is initialized once
 *    per PRU-ICSS instance, not per slice.
 * 3. This approach works correctly because validation in tamagawa_pruicss_init()
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
#include "tamagawa_periodic_trigger.h"
#include <drivers/soc.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if (CONFIG_TAMAGAWA0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_TAMAGAWA_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_TAMAGAWA_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif /* CONFIG_TAMAGAWA0_PRUICSS_INSTANCE */
#else
/* ICSSG Interrupt Numbers */
#if (CONFIG_TAMAGAWA0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_TAMAGAWA_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#define ICSS_RTU_TAMAGAWA_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1)
#define ICSS_TX_TAMAGAWA_INT_NUM          (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_PRU_TAMAGAWA_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSS_RTU_TAMAGAWA_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSS_TX_TAMAGAWA_INT_NUM          (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#endif /* CONFIG_TAMAGAWA0_PRUICSS_INSTANCE */
#endif /* SOC_AM243X */

#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
#define IEP_CMP_EVENT       ( 3 )
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
/* Load-share mode events for SLICE1 */
#define RTU_TRIGGER_HOST_TAMAGAWA_EVT   ( 4+16 )    /* pr0_pru_mst_intr[4]_intr_req (RTU-PRU) */
#define TX_TRIGGER_HOST_TAMAGAWA_EVT    ( 6+16 )    /* pr0_pru_mst_intr[6]_intr_req (TX-PRU) */
#else
#define IEP_CMP_EVENT       ( 4 )
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
/* Load-share mode events for SLICE0 */
#define RTU_TRIGGER_HOST_TAMAGAWA_EVT   ( 5+16 )    /* pr0_pru_mst_intr[5]_intr_req (RTU-PRU) */
#define TX_TRIGGER_HOST_TAMAGAWA_EVT    ( 7+16 )    /* pr0_pru_mst_intr[7]_intr_req (TX-PRU) */
#endif /* CONFIG_TAMAGAWA0_PRUICSS_SLICE */

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other
 * combinations, update code and remove this line.
 */
#if (CONFIG_TAMAGAWA1_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_TAMAGAWA_INT_NUM_SECOND_SLICE  (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_3)
#else
#define ICSS_PRU_TAMAGAWA_INT_NUM_SECOND_SLICE  (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_3)
#endif /* CONFIG_TAMAGAWA1_PRUICSS_INSTANCE */

#if (CONFIG_TAMAGAWA1_PRUICSS_PRUx == 1)
#define IEP_CMP_EVENT_SECOND_SLICE       ( 3 )
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT_SECOND_SLICE   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#else
#define IEP_CMP_EVENT_SECOND_SLICE       ( 4 )
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT_SECOND_SLICE   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
#endif /* CONFIG_TAMAGAWA1_PRUICSS_PRUx */
#endif /* TAMAGAWA_DUAL_PRU_SLICE_ENABLE */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static HwiP_Object gTamagawaHwiObject[CONFIG_TAMAGAWA_NUM_INSTANCES][TAMAGAWA_MAX_CHANNELS_PER_SLICE];
uint32_t gPruTamagawaIrqCnt[CONFIG_TAMAGAWA_NUM_INSTANCES][TAMAGAWA_MAX_CHANNELS_PER_SLICE] = {{0}};

/* PRU-ICSS INTC Configuration uses first Tamagawa instance */
/* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
#if (CONFIG_TAMAGAWA0_PRUICSS_INSTANCE == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void tamagawa_config_iep(tamagawa_periodic_interface *tamagawa_periodic_interface);

static void tamagawa_interrupt_config(tamagawa_periodic_interface *tamagawa_periodic_interface);

void tamagawa_pru_irq_handler(void *pruicss_handle);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x.
 */
void tamagawa_pru_irq_handler_second_slice(void *pruicss_handle);
#endif

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
/* Load-share mode interrupt handlers */
void tamagawa_rtu_pru_irq_handler(void *pruicss_handle);
void tamagawa_tx_pru_irq_handler(void *pruicss_handle);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void tamagawa_config_iep(tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
    tamagawa_priv *priv = tamagawa_get_priv(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
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

    /* Configure CMP based on periodic_trigger_count of first handle */
    event |= (0x1 << (IEP_CMP_EVENT + 1));
    event_clear |= (0x1 << (IEP_CMP_EVENT));
    cmp_reg0 = (tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA0] & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA0]>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CMP_EVENT*8,  cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CMP_EVENT*8,  cmp_reg1);

    /*clear event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* Configure CMP based on periodic_trigger_count of second handle */

    event = HW_RD_REG32((uint32_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG32((uint32_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    event |= (0x1 << (IEP_CMP_EVENT_SECOND_SLICE + 1));
    event_clear |= (0x1 << (IEP_CMP_EVENT_SECOND_SLICE));
    cmp_reg0 = (tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA1] & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA1]>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CMP_EVENT_SECOND_SLICE*8,  cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CMP_EVENT_SECOND_SLICE*8,  cmp_reg1);

    /*clear event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
#endif

    iep_reset_count = tamagawa_periodic_interface->iep_reset_count;

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


static void tamagawa_interrupt_config(tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
    tamagawa_priv *priv = tamagawa_get_priv(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
    void *pruicss_handle = (void *)(priv->pruicss_handle);
    int32_t status;
    HwiP_Params hwi_params;

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* Load-share mode: Register interrupts for RTU-PRU, PRU, and TX-PRU */

    /* Register and enable RTU-PRU FW interrupt for Channel 0 - only if channel 0 is enabled */
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_RTU_TAMAGAWA_INT_NUM;
    hwi_params.callback = &tamagawa_rtu_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /* Register and enable PRU FW interrupt for Channel 1 - only if channel 1 is enabled */
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_TAMAGAWA_INT_NUM;
    hwi_params.callback = &tamagawa_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][1], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /* Register and enable TX-PRU FW interrupt for Channel 2 - only if channel 2 is enabled */
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_TX_TAMAGAWA_INT_NUM;
    hwi_params.callback = &tamagawa_tx_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][2], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

#else
    /* Single/dual PRU modes: Register single PRU interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_TAMAGAWA_INT_NUM;
    hwi_params.callback = &tamagawa_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_TAMAGAWA_INT_NUM_SECOND_SLICE;
    hwi_params.callback = &tamagawa_pru_irq_handler_second_slice;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gTamagawaHwiObject[CONFIG_TAMAGAWA1][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#endif
}

int32_t tamagawa_config_periodic_mode(tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    int32_t         status;
    tamagawa_priv   *priv;
    void            *pruicss_handle;
    uint32_t        i;

    /* NULL check on interface pointer and handle(s) */
    if(tamagawa_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        if(tamagawa_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
    priv = tamagawa_get_priv(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
    pruicss_handle = (void *)(priv->pruicss_handle);

    /* Configure IEP */
    tamagawa_config_iep(tamagawa_periodic_interface);

    /* Initialize PRU-ICSS Interrupt Controller */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
#if (CONFIG_TAMAGAWA0_PRUICSS_INSTANCE == 1)
    status = PRUICSS_intcInit(pruicss_handle, &icss1_intc_initdata);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }
#else
    status = PRUICSS_intcInit(pruicss_handle, &icss0_intc_initdata);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }
#endif
    /* Configure Interrupts */
    tamagawa_interrupt_config(tamagawa_periodic_interface);
    return SystemP_SUCCESS;

}

int32_t tamagawa_stop_periodic_mode(tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    tamagawa_priv *priv;
    void *pruicss_iep;
    uint8_t temp;
    uint32_t i;

    /* NULL check on interface pointer and handle(s) */
    if(tamagawa_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        if(tamagawa_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
    priv = tamagawa_get_priv(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
    pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);

    /*Stop IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* Load-share mode: Destruct PRU interrupts */
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][0]);  /* RTU-PRU */
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][1]);  /* PRU */
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][2]);  /* TX-PRU */
#endif
#else
    /* Single/dual PRU modes */
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][0]);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA1][0]);
#endif
#endif

    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void tamagawa_pru_irq_handler(void *pruicss_handle)
{
#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* Load-share mode: PRU handles Channel 1 */
    gPruTamagawaIrqCnt[CONFIG_TAMAGAWA0][1]++;
#else
    /* Single PRU mode*/
    gPruTamagawaIrqCnt[CONFIG_TAMAGAWA0][0]++;
#endif

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_TAMAGAWA_EVT);
}

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
void tamagawa_pru_irq_handler_second_slice(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruTamagawaIrqCnt[CONFIG_TAMAGAWA1][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_TAMAGAWA_EVT_SECOND_SLICE);
}
#endif

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
/* RTU-PRU FW IRQ handler for Channel 0 */
void tamagawa_rtu_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count for Channel 0 (RTU-PRU) */
    gPruTamagawaIrqCnt[CONFIG_TAMAGAWA0][0]++;

    /* Clear interrupt at source - use RTU-PRU event */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, RTU_TRIGGER_HOST_TAMAGAWA_EVT);
}

/* TX-PRU FW IRQ handler for Channel 2 */
void tamagawa_tx_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count for Channel 2 (TX-PRU) */
    gPruTamagawaIrqCnt[CONFIG_TAMAGAWA0][2]++;

    /* Clear interrupt at source - use TX-PRU event */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, TX_TRIGGER_HOST_TAMAGAWA_EVT);
}
#endif