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
 * \file  endat_periodic_trigger.c
 *
 * \brief EnDAT periodic trigger mode implementation using IEP timer
 *
 * This file implements periodic trigger mode for EnDAT encoder interface.
 * In periodic mode, encoder position command is automatically triggered at regular
 * intervals by PRU using the PRU-ICSS Industrial Ethernet Peripheral (IEP) timer,
 * eliminating the need for host (R5F) intervention to trigger a command. After the
 * response is received, PRU triggers host (R5F) interrupt.
 *
 * \par IEP Timer Configuration:
 * The IEP timer is a PRU-ICSS instance-level resource shared between slices.
 * Therefore, IEP configuration uses the first handle (CONFIG_ENDAT0) to access
 * the PRU-ICSS hardware attributes, regardless of how many slices are active.
 * Each slice/instance can have different trigger counts per channel, but they
 * share the same IEP reset count (period).
 * - Trigger Count: IEP counter value when EnDAT transaction is initiated
 * - Reset Count: IEP counter value when counter resets to 0 (defines period)
 *
 * \par First instance (CONFIG_ENDAT0) is used for shared resources:
 * Several operations use gAppEndatHandle[CONFIG_ENDAT0] to access shared PRU-ICSS
 * resources:
 * 1. IEP timer configuration (endat_config_iep()): IEP is PRU-ICSS instance-level,
 *    not slice-specific. Using first handle ensures consistent access.
 * 2. INTC initialization (endat_config_periodic_mode()): INTC is initialized once
 *    per PRU-ICSS instance, not per slice.
 * 3. This approach works correctly because validation in endat_pruicss_init()
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
#include "endat_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/endat/include/endat_drv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_ENDAT_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_ENDAT_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#else
/* ICSSG Interrupt Numbers */
#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_ENDAT_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_ENDAT_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_ENDAT_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
/** \brief RTU-PRU EnDAT interrupt event number (18 = 2 + 16) */
#define RTU_TRIGGER_HOST_ENDAT_EVT      (2+16)
/** \brief PRU EnDAT interrupt event number (19 = 3 + 16) */
#define PRU_TRIGGER_HOST_ENDAT_EVT      (3+16)
/** \brief TX-PRU EnDAT interrupt event number (20 = 4 + 16) */
#define TXPRU_TRIGGER_HOST_ENDAT_EVT    (4+16)
#else
/** \brief PRU EnDAT interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_ENDAT_EVT      (2+16)
#endif

/** \brief IEP Compare event number for Channel 0 trigger */
#define IEP_CH0_CMP_EVENT               (3)

/** \brief IEP Compare event number for Channel 1 trigger */
#define IEP_CH1_CMP_EVENT               (5)

/** \brief IEP Compare event number for Channel 2 trigger */
#define IEP_CH2_CMP_EVENT               (6)

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other
 * combinations, update code and remove this line.
 */

#if (CONFIG_ENDAT1_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT_INT_NUM_SECOND_SLICE         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#else
#define ICSS_PRU_ENDAT_INT_NUM_SECOND_SLICE         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#endif

#if (CONFIG_ENDAT1_PRUICSS_SLICE == 1)
#define IEP_CH0_CMP_EVENT_SECOND_SLICE              (3)
#define PRU_TRIGGER_HOST_ENDAT_EVT_SECOND_SLICE     (2+16)
#else
#define IEP_CH0_CMP_EVENT_SECOND_SLICE              (4)
#define PRU_TRIGGER_HOST_ENDAT_EVT_SECOND_SLICE     (3+16)
#endif /* CONFIG_ENDAT1_PRUICSS_SLICE */

#endif /* ENDAT_DUAL_PRU_SLICE_ENABLE */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gEndatHwiObject[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
uint32_t gPruEndatIrqCnt[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX] = {0};

/* PRU-ICSS INTC Configuration uses first EnDAT instance */
/* ASSUMPTION: Same PRU-ICSS instance is used for multiple EnDAT handles in this example */
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* IEP Configuration Functions */
#if defined(SOC_AM243X)
static void endat_config_iep_cap_for_sync(endat_handle handle, uint64_t iep_sync0_period);
static void endat_disable_iep_cap_sync(endat_handle handle);
#endif

static void endat_config_iep(endat_periodic_interface *endat_periodic_interface);

/* IEP Counter Control */
static void endat_enable_iep_counter(endat_handle handle);
static void endat_disable_iep_counter(endat_handle handle);

/* IEP Reset Control */
static void endat_enable_iep_reset_on_cmp0(endat_handle handle, uint64_t iep_reset_count);
static void endat_disable_iep_reset_on_cmp0(endat_handle handle);

/* IEP CAP Event Functions */
static void endat_enable_iep_cap_event(endat_handle handle, uint8_t event_num);
static void endat_disable_iep_cap_event(endat_handle handle, uint8_t event_num);

/* IEP CMP Event Functions */
static void endat_enable_iep_cmp_event(endat_handle handle, uint64_t trigger_point, uint8_t event_num);
static void endat_disable_iep_cmp_event(endat_handle handle, uint8_t event_num);

/* Interrupt Configuration */
static void endat_interrupt_config(endat_periodic_interface *endat_periodic_interface);

/* IRQ Handlers */
void endat_pru_irq_handler(void *pruicss_handle);

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
void endat_rtupru_irq_handler(void *pruicss_handle);
void endat_txpru_irq_handler(void *pruicss_handle);
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Only PRU IRQ
 * handler is defined.
 */
void endat_pru_irq_handler_second_slice(void *pruicss_handle);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if defined(SOC_AM243X)
/**
 * \brief Configure IEP CAP mode for periodic trigger using SYNC signal
 *
 * \details This function configures IEP SYNC OUT0 generation and routes it to CAP inputs:
 * - AM243x/AM64x: IEP SYNC OUT0 routed to LATCH inputs via Time Sync Router
 * - AM26x: Uses EPWM crossbar configuration (done in SysConfig)
 *
 * Channel Configuration:
 * - Non-Load Share: CAP6 (LATCH0) for all channels
 * - Load Share Ch0: CAP6 via LATCH0_IN0
 * - Load Share Ch1: GPIO Mux to CAP0 (requires external GPIO connection)
 * - Load Share Ch2: CAP7 via LATCH1_IN0
 *
 * \param handle EnDAT driver handle
 * \param iep_sync0_period IEP SYNC OUT0 period in IEP clock cycles
 */
static void endat_config_iep_cap_for_sync(endat_handle handle, uint64_t iep_sync0_period)
{
    const endat_attrs *attrs = endat_get_attrs(handle);
    void *pru_iep = attrs->iep_base_addr;
    uint32_t reg_value;

    /* Configure IEP CMP1 to start SYNC OUT0 after 100 cycles */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG0, ENDAT_IEP_CMP1_START_DELAY);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG1, 0);

    /* Enable CMP1 event */
    reg_value = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    reg_value |= ((uint32_t)1U <<ENDAT_IEP_CMP_EVENT_FOR_SYNC0) << ENDAT_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT;  /* CMP1 enable bit */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, reg_value);

    /* Enable SYNC OUT0 cyclic generation */
    reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value |= (ENDAT_IEP_SYNC_CTRL_SYNC01_EN_MASK | ENDAT_IEP_SYNC_CTRL_SYNC0_EN_MASK);
    reg_value |= ENDAT_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK;
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Configure SYNC OUT0 pulse width and period */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_PWIDTH_REG, ENDAT_IEP_SYNC0_PULSE_WIDTH);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC0_PERIOD_REG, iep_sync0_period);

#if defined(SOC_AM243X)
    /* Route SYNC OUT0 to LATCH inputs via Time Sync Event Router */

    if(attrs->load_share_enabled == 1)
    {
        if(attrs->iep_instance == 0)
        {
            if(attrs->channel0_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN29);
                }
                else
                {
                    /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN25);
                }
            }
            if(attrs->channel2_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT13_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN29);
                }
                else
                {
                    /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT9_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN25);
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
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN31);
                }
                else
                {
                    /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN27);
                }
            }
            if(attrs->channel2_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT15_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN31);
                }
                else
                {
                    /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT11_OFFSET),
                                ENDAT_TIMESYNC_EVENT_ROUTER_IN27);
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
                HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + ENDAT_GPIOMUX_INTROUTER0_IEP0_CAP_OFFSET),
                            ENDAT_GPIOMUX_INTROUTER0_CAP_GPIO_IN);
            }
            else
            {
                /* Connect GPIO to IEP1 CAP_IN */
                HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + ENDAT_GPIOMUX_INTROUTER0_IEP1_CAP_OFFSET),
                            ENDAT_GPIOMUX_INTROUTER0_CAP_GPIO_IN);
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
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                            ENDAT_TIMESYNC_EVENT_ROUTER_IN29);
            }
            else
            {
                /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                            ENDAT_TIMESYNC_EVENT_ROUTER_IN25);
            }
        }
        else
        {
            if(attrs->pruicss_instance == 1)
            {
                /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                            ENDAT_TIMESYNC_EVENT_ROUTER_IN31);
            }
            else
            {
                /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                            ENDAT_TIMESYNC_EVENT_ROUTER_IN27);
            }
        }
    }
#endif /* SOC_AM243X */
}

/**
 * \brief Disable IEP CAP mode SYNC generation
 *
 * \details This function disables IEP SYNC OUT0 generation and CMP1 event used for sync.
 * This is typically called during periodic mode shutdown.
 *
 * \param handle EnDAT driver handle
 */
static void endat_disable_iep_cap_sync(endat_handle handle)
{
#if defined(SOC_AM243X)
    const endat_attrs *attrs = endat_get_attrs(handle);
    void *pru_iep = attrs->iep_base_addr;
    uint32_t reg_value;

    /* Disable SYNC OUT0 generation */
    reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value &= ~(ENDAT_IEP_SYNC_CTRL_SYNC01_EN_MASK | ENDAT_IEP_SYNC_CTRL_SYNC0_EN_MASK); /* SYNC OUT0 disable */
    reg_value &= ~ENDAT_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK; /* SYNC OUT0 cyclic disable */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Disable CMP1 event (configured for SYNC OUT0) */
    endat_disable_iep_cmp_event(handle, ENDAT_IEP_CMP_EVENT_FOR_SYNC0);
#endif /* SOC_AM243X */
}
#endif /* SOC_AM243X */

static void endat_enable_iep_reset_on_cmp0(endat_handle handle, uint64_t iep_reset_count)
{
    const endat_attrs *attrs;
    void *pru_iep;
    uint16_t event;
    uint32_t event_clear;
    uint32_t reg0;
    uint32_t reg1;

    attrs = endat_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    reg0 = (iep_reset_count & 0XFFFFFFFF);
    reg1 = (iep_reset_count >> 32 & 0XFFFFFFFF);

    HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0),  reg0);
    HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1),  reg1);

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /* Enable IEP reset by CMP0 event */
    event |= (1 << ENDAT_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* CMP0 enable bit */
    event |= (1 << ENDAT_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Reset counter enable bit */
    event_clear |= 1;

    /* Clear event */
    HW_WR_REG32((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /* Enable event */
    HW_WR_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}

static void endat_disable_iep_reset_on_cmp0(endat_handle handle)
{
    const endat_attrs *attrs;
    void *pru_iep;
    uint16_t event;

    attrs = endat_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);

    /* Disable IEP reset by CMP0 event */
    event &= ~(1 << ENDAT_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* Clear CMP0 enable bit */
    event &= ~(1 << ENDAT_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Clear Reset counter enable bit */

    /* Write back the modified value */
    HW_WR_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}

static void endat_enable_iep_counter(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;

    priv = endat_get_priv(handle);
    attrs = endat_get_attrs(handle);

    /* Configure and enable IEP counter */
    PRUICSS_setIepCounterIncrementValue(priv->pruicss_handle, attrs->iep_instance, ENDAT_IEP_COUNTER_INCREMENT);
    PRUICSS_controlIepCounter(priv->pruicss_handle, attrs->iep_instance, ENDAT_IEP_COUNTER_ENABLE);
}

static void endat_disable_iep_counter(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;

    priv = endat_get_priv(handle);
    attrs = endat_get_attrs(handle);

    /* Disable IEP counter */
    PRUICSS_controlIepCounter(priv->pruicss_handle, attrs->iep_instance, ENDAT_IEP_COUNTER_DISABLE);
}

static void endat_disable_iep_cmp_event(endat_handle handle, uint8_t event_num)
{
    const endat_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;

    attrs = endat_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Disable the cmp event */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Clear the CMP_EN bit (AND with the negated new value) */
    reg0 &= ~(((uint32_t)1U << event_num) << ENDAT_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);
}

static void endat_disable_iep_cap_event(endat_handle handle, uint8_t event_num)
{
    const endat_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;

    attrs = endat_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Disable the cap event */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));
    /* Clear the CAP_EN bit (AND with the negated new value) */
    reg0 &= ~((uint32_t)1U << event_num);
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);
}

static void endat_enable_iep_cap_event(endat_handle handle, uint8_t event_num)
{
    const endat_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;

    attrs = endat_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Configure the cap event in IEP hardware register */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));
    /* Set the CAP_EN bit (OR with the new value) */
    reg0 |= ((uint32_t)1U << event_num);
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);
}

static void endat_enable_iep_cmp_event(endat_handle handle, uint64_t trigger_point, uint8_t event_num)
{
    const endat_attrs *attrs;
    void *pru_iep;
    uint32_t reg0;
    uint32_t reg1;

    attrs = endat_get_attrs(handle);
    pru_iep = attrs->iep_base_addr;

    /* Configure the IEP CMP event in hardware registers */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Set the CMP_EN bit (OR with the new value) */
    reg0 |= ((uint32_t)1U << event_num) << ENDAT_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT;
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);

    /* Write trigger point to CMP registers */
    reg0 = (trigger_point & 0XFFFFFFFF);
    reg1 = (trigger_point >> 32 & 0XFFFFFFFF);
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*ENDAT_8_BYTE_REG_OFFSET + ENDAT_8_BYTE_REG_OFFSET),  reg0);
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*ENDAT_8_BYTE_REG_OFFSET + ENDAT_8_BYTE_REG_OFFSET),  reg1);
    }
    else
    {
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*ENDAT_8_BYTE_REG_OFFSET),  reg0);
        HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*ENDAT_8_BYTE_REG_OFFSET),  reg1);
    }
}

static void endat_config_iep(endat_periodic_interface *endat_periodic_interface)
{
    const endat_attrs *attrs = endat_get_attrs(endat_periodic_interface->handle[CONFIG_ENDAT0]);
    endat_handle *handle = endat_periodic_interface->handle;
    uint8_t ch_idx;
    uint64_t iep_count = endat_periodic_interface->iep_reset_count;
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    const endat_attrs *attrs1 = endat_get_attrs(endat_periodic_interface->handle[CONFIG_ENDAT1]);
#endif

    /* Configure IEP reset/sync based on mode */
    if(endat_periodic_interface->is_cap_mode)
    {
#if defined(SOC_AM243X)
        /*
         * Configure IEP for sync and route it to IEP latch (CAP mode)
         *
         * For AM243x device, PRU-ICSS Level Global Configuration and Time sync router configuration
         * is done for CONFIG_ENDAT0 instance.
         *
         * For other Instance channels, it need to be added based on availability of Latch events
         * if CONFIG_ENDAT_NUM_INSTANCES > 1 and using CAP for device am243x.
         *
         * First CONFIG_ENDAT0 instance code can be used as reference.
         */
        endat_config_iep_cap_for_sync(handle[CONFIG_ENDAT0], iep_count);
#endif
        /* Configure CAP events for channels */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < ENDAT_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    endat_enable_iep_cap_event(handle[CONFIG_ENDAT0], attrs->iep_cap_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode*/
            endat_enable_iep_cap_event(handle[CONFIG_ENDAT0], attrs->iep_cap_event[0]);
        }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        /* Configure CAP event for second slice if enabled */
        /* Configuring CAP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            endat_enable_iep_cap_event(handle[CONFIG_ENDAT1], attrs1->iep_cap_event[0]);
        }
#endif
    }
    else
    {
        /* CMP mode: Enable IEP reset on CMP0 */
        /* PRU-ICSS Level Global Configuration uses first EnDAT handle */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance are used for multiple EnDAT handles in this example */
        endat_enable_iep_reset_on_cmp0(handle[CONFIG_ENDAT0], iep_count);

        /* Configure CMP events for channels */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < ENDAT_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    endat_enable_iep_cmp_event(handle[CONFIG_ENDAT0], endat_periodic_interface->periodic_trigger_count[CONFIG_ENDAT0][ch_idx], attrs->iep_cmp_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            endat_enable_iep_cmp_event(handle[CONFIG_ENDAT0], endat_periodic_interface->periodic_trigger_count[CONFIG_ENDAT0][0], attrs->iep_cmp_event[0]);
        }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        /* Configure CMP event for second slice if enabled */
        /* Configuring CMP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            endat_enable_iep_cmp_event(handle[CONFIG_ENDAT1], endat_periodic_interface->periodic_trigger_count[CONFIG_ENDAT1][0], attrs1->iep_cmp_event[0]);
        }
#endif
    }

    /* PRU-ICSS Level Global Configuration uses first EnDAT handle */
    /* ASSUMPTION: Same PRU-ICSS instance and IEP Insatnce are used for multiple EnDAT handles in this example */
    endat_enable_iep_counter(handle[CONFIG_ENDAT0]);
}

static void endat_interrupt_config(endat_periodic_interface *endat_periodic_interface)
{
    /* PRU-ICSS Level Global Configuration uses first EnDAT handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple EnDAT handles in this example */
    endat_priv *priv = endat_get_priv(endat_periodic_interface->handle[CONFIG_ENDAT0]);
    void *pruicss_handle = (void *)(priv->pruicss_handle);
    int32_t status;
    HwiP_Params hwi_params;

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    /* Register and enable RTU-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_RTU_ENDAT_INT_NUM;
    hwi_params.callback = &endat_rtupru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndatHwiObject[CONFIG_ENDAT0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_ENDAT_INT_NUM;
    hwi_params.callback = &endat_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndatHwiObject[CONFIG_ENDAT0][1], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)

    /* Register and enable TX-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_TXPRU_ENDAT_INT_NUM;
    hwi_params.callback = &endat_txpru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndatHwiObject[CONFIG_ENDAT0][2], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#else
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_ENDAT_INT_NUM;
    hwi_params.callback = &endat_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndatHwiObject[CONFIG_ENDAT0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_ENDAT_INT_NUM_SECOND_SLICE;
    hwi_params.callback = &endat_pru_irq_handler_second_slice;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndatHwiObject[CONFIG_ENDAT1][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
}

int32_t endat_config_periodic_mode(endat_periodic_interface *endat_periodic_interface)
{
    int32_t status;
    uint32_t    i;
    endat_priv  *priv;
    void        *pruicss_handle;

    /* NULL check on interface pointer and handle(s) */
    if(endat_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        if(endat_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }
    /* PRU-ICSS Level Global Configuration uses first EnDAT handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple EnDAT handles in this example */
    priv = endat_get_priv(endat_periodic_interface->handle[CONFIG_ENDAT0]);
    pruicss_handle = (void *)(priv->pruicss_handle);
    
    /* Configure IEP*/
    endat_config_iep(endat_periodic_interface);

    /* Initialize PRU-ICSS Interrupt Controller */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple EnDAT handles in this example */
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE == 1)
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
    endat_interrupt_config(endat_periodic_interface);
    return SystemP_SUCCESS;
}

int32_t endat_stop_periodic_mode(endat_periodic_interface *endat_periodic_interface)
{
    const endat_attrs *attrs;
    endat_handle *handle;
    uint8_t ch_idx;
    uint32_t i;
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    const endat_attrs *attrs1;
#endif

    /* NULL check on interface pointer and handle(s) */
    if(endat_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        if(endat_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    attrs = endat_get_attrs(endat_periodic_interface->handle[CONFIG_ENDAT0]);
    handle = endat_periodic_interface->handle;
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    attrs1 = endat_get_attrs(endat_periodic_interface->handle[CONFIG_ENDAT1]);
#endif

    /* Disable IEP counter first */
    /* PRU-ICSS Level Global Configuration uses first EnDAT handle */
    /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple EnDAT handles in this example */
    endat_disable_iep_counter(handle[CONFIG_ENDAT0]);

    /* Disable events based on mode */
    if(endat_periodic_interface->is_cap_mode)
    {
        /* CAP mode: Disable capture events */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < ENDAT_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    endat_disable_iep_cap_event(handle[CONFIG_ENDAT0], attrs->iep_cap_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            endat_disable_iep_cap_event(handle[CONFIG_ENDAT0], attrs->iep_cap_event[0]);
        }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        /* Disable CAP event for second slice if enabled */
        /* Configuring CAP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            endat_disable_iep_cap_event(handle[CONFIG_ENDAT1], attrs1->iep_cap_event[0]);
        }
#endif

        /* Disable IEP SYNC generation for CAP mode */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple EnDAT handles in this example */
        endat_disable_iep_cap_sync(handle[CONFIG_ENDAT0]);

    }
    else
    {
        /* CMP mode: Disable compare events */
        if(attrs->load_share_enabled == 1)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < ENDAT_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    endat_disable_iep_cmp_event(handle[CONFIG_ENDAT0], attrs->iep_cmp_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            endat_disable_iep_cmp_event(handle[CONFIG_ENDAT0], attrs->iep_cmp_event[0]);
        }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        /* Disable CMP event for second slice if enabled */ 
        /* Configuring CMP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            endat_disable_iep_cmp_event(handle[CONFIG_ENDAT1], attrs1->iep_cmp_event[0]);
        }
#endif

        /* Disable IEP reset on CMP0 event */
        /* PRU-ICSS Level Global Configuration uses first EnDAT handle */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple EnDAT handles in this example */
        endat_disable_iep_reset_on_cmp0(handle[CONFIG_ENDAT0]);
    }

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    HwiP_destruct(&gEndatHwiObject[CONFIG_ENDAT0][0]);
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    HwiP_destruct(&gEndatHwiObject[CONFIG_ENDAT0][1]);
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)
    HwiP_destruct(&gEndatHwiObject[CONFIG_ENDAT0][2]);
#endif
#else
    HwiP_destruct(&gEndatHwiObject[CONFIG_ENDAT0][0]);
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    HwiP_destruct(&gEndatHwiObject[CONFIG_ENDAT1][0]);
#endif
    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void endat_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* In load share mode, index 1 is used for channel 1 connected to PRU */
    gPruEndatIrqCnt[CONFIG_ENDAT0][1]++;
#else
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruEndatIrqCnt[CONFIG_ENDAT0][0]++;
#endif
    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_ENDAT_EVT);
}

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
/* RTU-PRU FW IRQ handler */
void endat_rtupru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruEndatIrqCnt[CONFIG_ENDAT0][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, RTU_TRIGGER_HOST_ENDAT_EVT);

}

/* TX-PRU FW IRQ handler */
void endat_txpru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruEndatIrqCnt[CONFIG_ENDAT0][2]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, TXPRU_TRIGGER_HOST_ENDAT_EVT);

}
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Only PRU IRQ
 * handler is defined.
 */

/* PRU FW IRQ handler */
void endat_pru_irq_handler_second_slice(void *pruicss_handle)
{
    /* Increment IRQ count */
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruEndatIrqCnt[CONFIG_ENDAT1][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_ENDAT_EVT);
}
#endif