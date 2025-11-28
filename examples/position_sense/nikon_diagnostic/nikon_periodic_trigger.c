/*
 *  Copyright (C) 2024-2025 Texas Instruments Incorporated
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
 * \file  nikon_periodic_trigger.c
 *
 * \brief Nikon periodic trigger mode implementation using IEP timer
 *
 * This file implements periodic trigger mode for Nikon encoder interface.
 * In periodic mode, encoder position data is automatically sampled at regular
 * intervals using the PRU-ICSS Industrial Ethernet Peripheral (IEP) timer,
 * eliminating the need for host CPU intervention for each transaction.
 *
 * \par Dual Slice Support:
 * This implementation supports dual PRU slice operation where two independent
 * Nikon instances run simultaneously on different slices (PRU0/PRU1) of the
 * same PRU-ICSS instance.
 *
 * \par IEP Timer Configuration:
 * The IEP timer is a PRU-ICSS instance-level resource shared between slices.
 * Therefore, IEP configuration uses the first handle (CONFIG_NIKON0) to access
 * the PRU-ICSS hardware attributes, regardless of how many slices are active.
 * Each slice/instance can have different trigger counts per channel, but they
 * share the same IEP reset count (period).
 *
 * \par Key Concepts:
 * - Instance: Software driver instance (CONFIG_NIKON0, CONFIG_NIKON1) representing
 *   independent Nikon configuration and state
 * - Slice: Hardware PRU slice (0 or 1) within PRU-ICSS where firmware executes
 * - IEP Timer: Shared hardware timer resource at PRU-ICSS instance level
 * - Trigger Count: IEP counter value when encoder transaction is initiated
 * - Reset Count: IEP counter value when counter resets to 0 (defines period)
 *
 * \par Why First Handle (CONFIG_NIKON0) is Used:
 * The IEP timer hardware is shared across all PRU slices in a PRU-ICSS instance.
 * While each Nikon instance has its own driver handle, they all access the same
 * IEP registers through the PRU-ICSS hardware attributes. Using the first handle
 * (CONFIG_NIKON0) to access IEP is a design choice that:
 * 1. Ensures consistent access to the shared IEP resource
 * 2. Simplifies the code by having a single reference point
 * 3. Works correctly because both instances point to the same PRU-ICSS instance
 *    (enforced by validation checks in nikon_pruicss_init())
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
#include "nikon_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/nikon/include/nikon_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#else
/* ICSSG Interrupt Numbers */
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
/** \brief RTU-PRU Nikon interrupt event number (18 = 2 + 16) */
#define RTU_TRIGGER_HOST_NIKON_EVT      ( 2+16 )
/** \brief PRU Nikon interrupt event number (19 = 3 + 16) */
#define PRU_TRIGGER_HOST_NIKON_EVT      ( 3+16 )
/** \brief TX-PRU Nikon interrupt event number (20 = 4 + 16) */
#define TXPRU_TRIGGER_HOST_NIKON_EVT    ( 4+16 )
#else
/** \brief PRU Nikon interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_NIKON_EVT      ( 2+16 )
#endif

/** \brief IEP Compare event number for Channel 0 trigger */
#define IEP_CH0_CMP_EVNT ( 3 )

/** \brief IEP Compare event number for Channel 1 trigger */
#define IEP_CH1_CMP_EVNT ( 5 )

/** \brief IEP Compare event number for Channel 2 trigger */
#define IEP_CH2_CMP_EVNT ( 6 )

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gNikonHwiObject[NIKON_NUM_CH_PER_SLICE_MAX];
uint32_t gPruNikonIrqCnt[NIKON_NUM_CH_PER_SLICE_MAX] = {0};

/* ICSS INTC configuration */
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void nikon_config_iep(nikon_periodic_interface *nikon_periodic_interface);

static void nikon_interrupt_config(nikon_periodic_interface *nikon_periodic_interface);

void nikon_pru_irq_handler(void *pruicss_handle);

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
void nikon_rtupru_irq_handler(void *pruicss_handle);
void nikon_txpru_irq_handler(void *pruicss_handle);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Configure IEP timer for periodic Nikon transaction triggering
 *
 * \details This function configures the Industrial Ethernet Peripheral (IEP) timer
 *          to automatically trigger Nikon encoder transactions at specified intervals.
 *
 * \par Why CONFIG_NIKON0 is Used:
 * The IEP timer is a hardware resource shared across all PRU slices within the same
 * PRU-ICSS instance. Even in dual slice mode where CONFIG_NIKON0 and CONFIG_NIKON1
 * run on different slices, they both use the same PRU-ICSS instance (validated in
 * nikon_pruicss_init()). Therefore:
 * - We use handle[CONFIG_NIKON0] to access the PRU-ICSS hardware attributes
 * - The IEP base address obtained is the same regardless of which handle is used
 * - This provides a consistent reference point for IEP configuration
 * - Each instance can still have different trigger counts per channel via the
 *   periodic_trigger_count[instance][channel] array
 *
 * \par IEP Timer Operation:
 * - IEP counter increments at IEP clock rate (typically 200 MHz)
 * - Compare events (CMP3, CMP5, CMP6) are configured for channel triggers
 * - CMP0 event resets the counter, creating periodic cycles
 * - Each channel can trigger at different IEP count values within the cycle
 *
 * \param[in] nikon_periodic_interface  Pointer to periodic interface structure
 *                                      containing handles and trigger counts for
 *                                      all configured instances
 */
static void nikon_config_iep(nikon_periodic_interface *nikon_periodic_interface)
{
    /* Use first handle to access PRU-ICSS hardware resources (IEP is shared across slices) */
    const nikon_attrs *attrs = nikon_get_attrs(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    nikon_priv *priv = nikon_get_priv(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    void *pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    uint8_t temp;
    uint32_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;
    uint64_t iep_reset_count = 0;

    /*clear IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* cmp cfg reg - use uint8_t* for byte-addressed CSL register offsets */
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
            event |= (0x1 << (IEP_CH0_CMP_EVNT + 1));
            event_clear |= (0x1 << (IEP_CH0_CMP_EVNT));

            cmp_reg0 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][0] & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][0]>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8,  cmp_reg1);
        }
        if(attrs->channel1_enabled)
        {
            event |= (0x1 << (IEP_CH1_CMP_EVNT + 1));
            event_clear |= (0x1 << (IEP_CH1_CMP_EVNT));

            cmp_reg0 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][1] & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][1]>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH1_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH1_CMP_EVNT*8,  cmp_reg1);
        }
        if(attrs->channel2_enabled)
        {
            event |= (0x1 << (IEP_CH2_CMP_EVNT + 1));
            event_clear |= (0x1 << (IEP_CH2_CMP_EVNT));

            cmp_reg0 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][2] & 0xffffffff) - IEP_DEFAULT_INC;
            cmp_reg1 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][2]>>32 & 0xffffffff);

            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH2_CMP_EVNT*8,  cmp_reg0);
            HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH2_CMP_EVNT*8,  cmp_reg1);
        }
    }
    else
    {
        event |= (0x1 << (IEP_CH0_CMP_EVNT + 1));
        event_clear |= (0x1 << (IEP_CH0_CMP_EVNT));
        cmp_reg0 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][0] & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][0]>>32 & 0xffffffff);

        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CH0_CMP_EVNT*8,  cmp_reg0);
        HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CH0_CMP_EVNT*8,  cmp_reg1);

    }

    /*clear event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /*enable event*/
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    iep_reset_count = nikon_periodic_interface->iep_reset_count;

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


static void nikon_interrupt_config(nikon_periodic_interface *nikon_periodic_interface)
{
    nikon_priv *priv = nikon_get_priv(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    void *pruicss_handle = (void *)(priv->pruicss_handle);
    int32_t status;
    HwiP_Params hwi_params;

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    /* Register and enable RTU-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_RTU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_rtupru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[1], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)

    /* Register and enable TX-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_TXPRU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_txpru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[2], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#else
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
}

int32_t nikon_config_periodic_mode(nikon_periodic_interface *nikon_periodic_interface)
{
    int32_t         status;
    nikon_priv      *priv = nikon_get_priv(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    void            *pruicss_handle = (void *)(priv->pruicss_handle);

    /* Configure IEP */
    nikon_config_iep(nikon_periodic_interface);

    /* Initialize PRU-ICSS Interrupt Controller */
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
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
    nikon_interrupt_config(nikon_periodic_interface);
    return SystemP_SUCCESS;

}

int32_t nikon_stop_periodic_mode(nikon_periodic_interface *nikon_periodic_interface)
{
    nikon_priv *priv;
    void *pruicss_iep;
    uint8_t temp;

    /* NULL check on interface pointer and handle */
    if(nikon_periodic_interface == NULL || nikon_periodic_interface->handle[CONFIG_NIKON0] == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = nikon_get_priv(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);

    /*Stop IEP*/
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    HwiP_destruct(&gNikonHwiObject[0]);
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    HwiP_destruct(&gNikonHwiObject[1]);
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    HwiP_destruct(&gNikonHwiObject[2]);
#endif
#else
    HwiP_destruct(&gNikonHwiObject[0]);
#endif
    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void nikon_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* In load share mode, index 1 is used for channel 1 connected to PRU */
    gPruNikonIrqCnt[1]++;
#else
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruNikonIrqCnt[0]++;
#endif
    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_NIKON_EVT);
}

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
/* RTU-PRU FW IRQ handler */
void nikon_rtupru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruNikonIrqCnt[0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, RTU_TRIGGER_HOST_NIKON_EVT);

}

/* TX-PRU FW IRQ handler */
void nikon_txpru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruNikonIrqCnt[2]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, TXPRU_TRIGGER_HOST_NIKON_EVT);

}
#endif
