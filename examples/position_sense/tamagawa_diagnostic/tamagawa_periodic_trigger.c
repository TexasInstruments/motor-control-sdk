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
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
/* Load-share mode events for SLICE1 */
#define RTU_TRIGGER_HOST_TAMAGAWA_EVT   ( 4+16 )    /* pr0_pru_mst_intr[4]_intr_req (RTU-PRU) */
#define TX_TRIGGER_HOST_TAMAGAWA_EVT    ( 6+16 )    /* pr0_pru_mst_intr[6]_intr_req (TX-PRU) */
#else
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

#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT_SECOND_SLICE   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#else
#define PRU_TRIGGER_HOST_TAMAGAWA_EVT_SECOND_SLICE   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
#endif /* CONFIG_TAMAGAWA1_PRUICSS_SLICE */
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

/* IEP Configuration Functions */
#if defined(SOC_AM243X) 
static void tamagawa_config_iep_cap_for_sync(tamagawa_handle handle, uint64_t iep_sync0_period);
static void tamagawa_disable_iep_sync(tamagawa_handle handle);
#endif /* SOC_AM243X */
static void tamagawa_config_iep(tamagawa_periodic_interface *tamagawa_periodic_interface);

static void tamagawa_interrupt_config(tamagawa_periodic_interface *tamagawa_periodic_interface);

/* IEP Counter Control */
static void tamagawa_enable_iep_counter(tamagawa_handle handle);
static void tamagawa_disable_iep_counter(tamagawa_handle handle);

/* IEP Reset Control */
static void tamagawa_enable_iep_reset_on_cmp0(tamagawa_handle handle, uint64_t iep_reset_count);
static void tamagawa_disable_iep_reset_on_cmp0(tamagawa_handle handle);

/* IEP CMP Event Functions */
static void tamagawa_enable_iep_cmp_event(tamagawa_handle handle, uint64_t trigger_point, uint8_t event_num);
static void tamagawa_disable_iep_cmp_event(tamagawa_handle handle, uint8_t event_num);

/* IEP CAP Event Functions */
static void tamagawa_enable_iep_cap_event(tamagawa_handle handle, uint8_t event_num);
static void tamagawa_disable_iep_cap_event(tamagawa_handle handle, uint8_t event_num);

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

#if defined(SOC_AM243X) 
/**
 * \brief Configure IEP CAP mode for periodic trigger using SYNC signal
 *
 * \details This function configures IEP SYNC OUT0 generation and routes it to CAP inputs:
 * - AM243x: IEP SYNC OUT0 routed to LATCH inputs via Time Sync Router
 * - AM26x: Uses EPWM crossbar configuration (done in SysConfig)
 *
 * Channel Configuration:
 * - Non-Load Share: CAP6 (LATCH0) for all channels
 * - Load Share Ch0: CAP6 via LATCH0_IN0
 * - Load Share Ch1: GPIO Mux to CAP0 (requires external GPIO connection)
 * - Load Share Ch2: CAP7 via LATCH1_IN0
 *
 *
 * \param handle Tamagawa driver handle
 * \param iep_sync0_period IEP SYNC OUT0 period in IEP clock cycles
 *
 */
static void tamagawa_config_iep_cap_for_sync(tamagawa_handle handle, uint64_t iep_sync0_period)
{
    const tamagawa_attrs *attrs = tamagawa_get_attrs(handle);
    void *pru_iep = attrs->iep_base_addr;
    uint32_t reg_value;

    /* Configure IEP CMP1 to start SYNC OUT0 after 100 cycles */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG0, TAMAGAWA_IEP_CMP1_START_DELAY);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG1, 0);

    /* Enable CMP1 event */
    reg_value = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    reg_value |= ((uint32_t)1U <<TAMAGAWA_IEP_CMP_EVENT_FOR_SYNC0) << TAMAGAWA_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT;  /* CMP1 enable bit */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, reg_value);

    /* Enable SYNC OUT0 cyclic generation */
    reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value |= (TAMAGAWA_IEP_SYNC_CTRL_SYNC01_EN_MASK | TAMAGAWA_IEP_SYNC_CTRL_SYNC0_EN_MASK);
    reg_value |= TAMAGAWA_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK;
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Configure SYNC OUT0 pulse width and period */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_PWIDTH_REG, TAMAGAWA_IEP_SYNC0_PULSE_WIDTH);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC0_PERIOD_REG, iep_sync0_period);

    /* Route SYNC OUT0 to LATCH inputs via Time Sync Event Router */

    if(attrs->load_share_enabled)
    {
        if(attrs->iep_instance == 0)
        {
            if(attrs->channel0_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN29);
                }
                else
                {
                    /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN25);
                }
            }
            if(attrs->channel2_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT13_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN29);
                }
                else
                {
                    /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT9_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN25);
                }
            }
        }
        else
        {
            if(attrs->channel0_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN31);
                }
                else
                {
                    /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN27);
                }
            }
            if(attrs->channel2_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT15_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN31);
                }
                else
                {
                    /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT11_OFFSET),
                                TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN27);
                }
            }
        }

        /* Channel 1 configuration for ICSSG0 and ICSSG1 */
        if(attrs->channel1_enabled)
        {
            /* Route GPIO to IEP CAP0 input*/
            if(attrs->iep_instance == 0)
            {
                /* Connect GPIO to IEP0 CAP_IN */
                HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + TAMAGAWA_GPIOMUX_INTROUTER0_IEP0_CAP_OFFSET),
                            TAMAGAWA_GPIOMUX_INTROUTER0_CAP_GPIO_IN);
            }
            else
            {
                /* Connect GPIO to IEP1 CAP_IN */
                HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + TAMAGAWA_GPIOMUX_INTROUTER0_IEP1_CAP_OFFSET),
                            TAMAGAWA_GPIOMUX_INTROUTER0_CAP_GPIO_IN);
            }
        }
    }
    else
    {
        /* Non-load share mode: Route SYNC OUT0 to LATCH0_IN0 for single channel */
        if(attrs->iep_instance == 0)
        {
            if(attrs->pruicss_instance == 1)
            {
                /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                            TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN29);
            }
            else
            {
                /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                            TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN25);
            }
        }
        else
        {
            if(attrs->pruicss_instance == 1)
            {
                /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                            TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN31);
            }
            else
            {
                /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TAMAGAWA_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                            TAMAGAWA_TIMESYNC_EVENT_ROUTER_IN27);
            }
        }
    }
}
#endif

/**
 * \brief Enable IEP counter
 *
 * \param handle Tamagawa driver handle
 */
static void tamagawa_enable_iep_counter(tamagawa_handle handle)
{
    const tamagawa_attrs *attrs;
    tamagawa_priv *priv;
    int32_t status;

    attrs = tamagawa_get_attrs(handle);
    priv = tamagawa_get_priv(handle);

    /* Configure and enable IEP counter */
    status = PRUICSS_setIepCounterIncrementValue(priv->pruicss_handle, attrs->iep_instance, TAMAGAWA_IEP_COUNTER_INCREMENT);
    DebugP_assert(status == SystemP_SUCCESS);

    status = PRUICSS_controlIepCounter(priv->pruicss_handle, attrs->iep_instance, TAMAGAWA_IEP_COUNTER_ENABLE);
    DebugP_assert(status == SystemP_SUCCESS);
}

/**
 * \brief Disable IEP counter
 *
 * \param handle Tamagawa driver handle
 */
static void tamagawa_disable_iep_counter(tamagawa_handle handle)
{
    const tamagawa_attrs *attrs;
    tamagawa_priv *priv;
    int32_t status;

    attrs = tamagawa_get_attrs(handle);
    priv = tamagawa_get_priv(handle);

    status = PRUICSS_controlIepCounter(priv->pruicss_handle, attrs->iep_instance, TAMAGAWA_IEP_COUNTER_DISABLE);
    DebugP_assert(status == SystemP_SUCCESS);
}

/**
 * \brief Enable IEP counter reset on CMP0 event
 *
 * \param handle Tamagawa driver handle
 * \param iep_reset_count IEP counter reset value (period)
 */
static void tamagawa_enable_iep_reset_on_cmp0(tamagawa_handle handle, uint64_t iep_reset_count)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint16_t event;
    uint16_t event_clear;
    uint32_t reg0;
    uint32_t reg1;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    reg0 = TAMAGAWA_GET_LOWER_32BITS(iep_reset_count);
    reg1 = TAMAGAWA_GET_UPPER_32BITS(iep_reset_count);

    HW_WR_REG32((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0,  reg0);
    HW_WR_REG32((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1,  reg1);

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /* Enable IEP reset by CMP0 event */
    event |= (1 << TAMAGAWA_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* CMP0 enable bit */
    event |= (1 << TAMAGAWA_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Reset counter enable bit */
    event_clear |= 1;

    /* Clear event */
    HW_WR_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /* Enable event */
    HW_WR_REG32((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}

/**
 * \brief Disable IEP counter reset on CMP0 event
 *
 * \param handle Tamagawa driver handle
 */
static void tamagawa_disable_iep_reset_on_cmp0(tamagawa_handle handle)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint16_t event;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Disable IEP reset on CMP0 */
    event = HW_RD_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    /* Disable IEP reset by CMP0 event */
    event &= ~(1 << TAMAGAWA_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* Clear CMP0 enable bit */
    event &= ~(1 << TAMAGAWA_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Clear Reset counter enable bit */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}

/**
 * \brief Enable IEP CMP event for periodic trigger
 *
 * \param handle Tamagawa driver handle
 * \param trigger_point IEP counter value when CMP event is generated
 * \param event_num CMP event number (0-15)
 */
static void tamagawa_enable_iep_cmp_event(tamagawa_handle handle, uint64_t trigger_point, uint8_t event_num)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;
    uint32_t reg1;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Configure the CMP event */
    reg0 = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    /* Set the CMP_EN bit (bit position = event_num + 1) */
    reg0 |= ((uint32_t)1U << (event_num + 1));
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, reg0);

    reg0 = TAMAGAWA_GET_LOWER_32BITS(trigger_point);
    reg1 = TAMAGAWA_GET_UPPER_32BITS(trigger_point);

    /* CMP8 to CMP15 registers have an 8-byte gap after CMP7 in the IEP memory map */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET + TAMAGAWA_8_BYTE_REG_OFFSET),  reg0);
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET + TAMAGAWA_8_BYTE_REG_OFFSET),  reg1);
    }
    else
    {
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET),  reg0);
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET),  reg1);
    }
}

/**
 * \brief Disable IEP CMP event
 *
 * \param handle Tamagawa driver handle
 * \param event_num CMP event number (0-15)
 */
static void tamagawa_disable_iep_cmp_event(tamagawa_handle handle, uint8_t event_num)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Disable the CMP event */
    reg0 = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    /* Clear the CMP_EN bit */
    reg0 &= ~(((uint32_t)1U << event_num) << TAMAGAWA_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, reg0);

    /* Clear CMP register values */
    /* CMP8 to CMP15 registers have an 8-byte gap after CMP7 in the IEP memory map */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET + TAMAGAWA_8_BYTE_REG_OFFSET),  0);
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET + TAMAGAWA_8_BYTE_REG_OFFSET),  0);
    }
    else
    {
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET),  0);
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*TAMAGAWA_8_BYTE_REG_OFFSET),  0);
    }
}

/**
 * \brief Enable IEP CAP event for periodic trigger
 *
 * \param handle Tamagawa driver handle
 * \param event_num CAP event number (0-7)
 */
static void tamagawa_enable_iep_cap_event(tamagawa_handle handle, uint8_t event_num)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Configure the CAP event */
    reg0 = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG);
    /* Set the CAP_EN bit */
    reg0 |= ((uint32_t)1U << event_num);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG, reg0);
}

/**
 * \brief Disable IEP CAP event
 *
 * \param handle Tamagawa driver handle
 * \param event_num CAP event number (0-7)
 */
static void tamagawa_disable_iep_cap_event(tamagawa_handle handle, uint8_t event_num)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Disable the CAP event */
    reg0 = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG);
    /* Clear the CAP_EN bit */
    reg0 &= ~((uint32_t)1U << event_num);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG, reg0);
}

#if defined(SOC_AM243X)
/**
 * \brief Disable IEP SYNC OUT0 cyclic generation
 *
 * \details Disables SYNC OUT0 cyclic generation and the associated CMP1 event
 *          that is used to generate the SYNC OUT0 signal
 *
 * \param handle Tamagawa handle
 */
static void tamagawa_disable_iep_sync(tamagawa_handle handle)
{
    const tamagawa_attrs *attrs;
    void *pru_iep;
    uint32_t reg_value;

    attrs = tamagawa_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Disable SYNC OUT0 */
    reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value &= ~(TAMAGAWA_IEP_SYNC_CTRL_SYNC01_EN_MASK | TAMAGAWA_IEP_SYNC_CTRL_SYNC0_EN_MASK);
    reg_value &= ~TAMAGAWA_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK;
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Disable CMP1 event (used for SYNC OUT0) */
    tamagawa_disable_iep_cmp_event(handle, TAMAGAWA_IEP_CMP_EVENT_FOR_SYNC0);
}
#endif /* SOC_AM243X */

static void tamagawa_config_iep(tamagawa_periodic_interface *tamagawa_periodic_interface)
{
    /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
    const tamagawa_attrs *attrs = tamagawa_get_attrs(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
    uint8_t ch_idx;

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* Get attrs for second slice if enabled */
    const tamagawa_attrs *attrs1 = tamagawa_get_attrs(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA1]);
#endif

    /* Configure IEP based on mode */
    if(tamagawa_periodic_interface->is_cap_mode)
    {
#if defined(SOC_AM243X)
        /*
         * Configure IEP for sync and route it to IEP latch (CAP mode)
         *
         * For AM243x device, PRU-ICSS Level Global Configuration and Time sync router configuration
         * is done for CONFIG_TAMAGAWA0 instance.
         *
         * For other Instance channels, it need to be added based on availability of Latch events
         * if CONFIG_TAMAGAWA_NUM_INSTANCES > 1 and using CAP for device am243x.
         *
         * First CONFIG_BISSC0 instance code can be used as reference.
         */
        tamagawa_config_iep_cap_for_sync(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], tamagawa_periodic_interface->iep_reset_count);
#endif
        /* Configure CAP events for channels */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    tamagawa_enable_iep_cap_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], attrs->iep_cap_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode */
            tamagawa_enable_iep_cap_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], attrs->iep_cap_event[0]);
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        /* Configure CAP event for second slice if enabled */
        /* Configuring CAP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share need to add similar logic as above */
        tamagawa_enable_iep_cap_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA1],
                                      attrs1->iep_cap_event[0]);
#endif
    }
    else
    {
        /* CMP mode: Enable IEP reset on CMP0 */
        tamagawa_enable_iep_reset_on_cmp0(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], tamagawa_periodic_interface->iep_reset_count);

        /* Configure CMP events for channels */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    tamagawa_enable_iep_cmp_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0],
                                                  tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA0][ch_idx],
                                                  attrs->iep_cmp_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode */
            tamagawa_enable_iep_cmp_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0],
                                          tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA0][0],
                                          attrs->iep_cmp_event[0]);
        }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        /* Configure CMP event for second slice if enabled */
        /* Configuring CMP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share need to add similar logic as above */
        tamagawa_enable_iep_cmp_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA1],
                                      tamagawa_periodic_interface->periodic_trigger_count[CONFIG_TAMAGAWA1][0],
                                      attrs1->iep_cmp_event[0]);
#endif
    }

    /* Enable IEP counter */
    tamagawa_enable_iep_counter(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
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
    const tamagawa_attrs *attrs;
    uint8_t ch_idx;
    uint32_t i;
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    const tamagawa_attrs *attrs1;
#endif

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

    attrs = tamagawa_get_attrs(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    attrs1 = tamagawa_get_attrs(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA1]);
#endif

    /* Disable IEP counter first */
    /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
    /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple Tamagawa handles in this example */
    tamagawa_disable_iep_counter(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);

    /* Disable events based on mode */
    if(tamagawa_periodic_interface->is_cap_mode)
    {
        /* CAP mode: Disable capture events */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    tamagawa_disable_iep_cap_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], attrs->iep_cap_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            tamagawa_disable_iep_cap_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], attrs->iep_cap_event[0]);
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        /* Disable CAP event for second slice if enabled */
        /* Configuring CAP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            tamagawa_disable_iep_cap_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA1], attrs1->iep_cap_event[0]);
        }
#endif

        /* Disable IEP SYNC generation for CAP mode */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple Tamagawa handles in this example */
#if defined(SOC_AM243X)
        tamagawa_disable_iep_sync(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
#endif /* SOC_AM243X */
    }
    else
    {
        /* CMP mode: Disable compare events */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    tamagawa_disable_iep_cmp_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], attrs->iep_cmp_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            tamagawa_disable_iep_cmp_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0], attrs->iep_cmp_event[0]);
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        /* Disable CMP event for second slice if enabled */
        /* Configuring CMP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            tamagawa_disable_iep_cmp_event(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA1], attrs1->iep_cmp_event[0]);
        }
#endif

        /* Disable IEP reset on CMP0 event */
        /* PRU-ICSS Level Global Configuration uses first Tamagawa handle */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple Tamagawa handles in this example */
        tamagawa_disable_iep_reset_on_cmp0(tamagawa_periodic_interface->handle[CONFIG_TAMAGAWA0]);
    }

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][0]);
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][1]);
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][2]);
#endif
#else
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA0][0]);
#endif

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    HwiP_destruct(&gTamagawaHwiObject[CONFIG_TAMAGAWA1][0]);
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