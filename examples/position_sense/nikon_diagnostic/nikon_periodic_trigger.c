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
 * In periodic mode, encoder position command is automatically triggered at regular
 * intervals by PRU using the PRU-ICSS Industrial Ethernet Peripheral (IEP) timer,
 * eliminating the need for host (R5F) intervention to trigger a command. After the
 * response is received, PRU triggers host (R5F) interrupt.
 *
 * \par Periodic Trigger Modes:
 * The Nikon driver supports two IEP-based periodic trigger modes:
 * - **CMP Mode (Compare)**: Time-based periodic sampling using IEP compare events (CMP0-CMP15)
 *   - Firmware monitors IEP counter and triggers when counter matches compare value
 * - **CAP Mode (Capture)**: Event-driven sampling using IEP capture events (CAP0-CAP7)
 *   - Firmware waits for external hardware signal routed through TIMESYNC/GPIOMUX router (AM243x)
 *     or XBAR (AM26x)
 *
 * \par IEP Timer Configuration:
 * The IEP timer is a PRU-ICSS instance-level resource shared between slices.
 * Therefore, IEP configuration uses the first handle (CONFIG_NIKON0) to access
 * the PRU-ICSS hardware attributes, regardless of how many slices are active.
 *
 * \par First instance (CONFIG_NIKON0) is used for shared resources:
 * Several operations use gAppNikonHandle[CONFIG_NIKON0] to access shared PRU-ICSS
 * resources:
 * 1. IEP timer configuration (nikon_config_iep()): IEP is PRU-ICSS instance-level,
 *    not slice-specific. Using first handle ensures consistent access.
 * 2. INTC initialization (nikon_config_periodic_mode()): INTC is initialized once
 *    per PRU-ICSS instance, not per slice.
 * 3. This approach works correctly because validation in nikon_pruicss_init()
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
#include "nikon_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/nikon/include/nikon_drv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if(CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if(CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#else
/* ICSSG Interrupt Numbers */
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if(CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2)
#else
#define ICSS_RTU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSS_TXPRU_NIKON_INT_NUM       (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#endif
#else
#if(CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_NIKON_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#endif
#endif
#endif

#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
/** \brief RTU-PRU Nikon interrupt event number (18 = 2 + 16) */
#define RTU_TRIGGER_HOST_NIKON_EVT      (2+16)
/** \brief PRU Nikon interrupt event number (19 = 3 + 16) */
#define PRU_TRIGGER_HOST_NIKON_EVT      (3+16)
/** \brief TX-PRU Nikon interrupt event number (20 = 4 + 16) */
#define TXPRU_TRIGGER_HOST_NIKON_EVT    (4+16)
#else
/** \brief PRU Nikon interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_NIKON_EVT      (2+16)
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other
 * combinations, update code and remove this line.
 */

#if(CONFIG_NIKON1_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_NIKON_INT_NUM_SECOND_SLICE         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1)
#else
#define ICSS_PRU_NIKON_INT_NUM_SECOND_SLICE         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1)
#endif

#if(CONFIG_NIKON1_PRUICSS_SLICE == 1)
#define PRU_TRIGGER_HOST_NIKON_EVT_SECOND_SLICE     (2+16)
#else
#define PRU_TRIGGER_HOST_NIKON_EVT_SECOND_SLICE     (3+16)
#endif /* CONFIG_NIKON1_PRUICSS_SLICE */

#endif /* NIKON_DUAL_PRU_SLICE_ENABLE */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gNikonHwiObject[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX];
uint32_t gPruNikonIrqCnt[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX] = {0};

/* PRU-ICSS INTC Configuration uses first Nikon instance */
/* ASSUMPTION: Same PRU-ICSS instance is used for multiple Nikon handles in this example */
#if(CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* IEP Configuration Functions */
#if defined(SOC_AM243X)
static void nikon_config_iep_cap_for_sync(nikon_handle handle, uint32_t iep_sync0_period);
static void nikon_disable_iep_cap_sync(void *pru_iep);
#endif /* SOC_AM243X */

static void nikon_config_iep(nikon_periodic_interface *nikon_periodic_interface);

/* IEP Counter Control */
static void nikon_enable_iep_counter(nikon_handle handle);
static void nikon_disable_iep_counter(nikon_handle handle);

/* IEP Reset Control */
static void nikon_enable_iep_reset_on_cmp0(void *pru_iep, uint64_t iep_reset_count);
static void nikon_disable_iep_reset_on_cmp0(void *pru_iep);

/* IEP CAP Event Functions */
static void nikon_enable_iep_cap_event(void *pru_iep, uint8_t event_num);
static void nikon_disable_iep_cap_event(void *pru_iep, uint8_t event_num);

/* IEP CMP Event Functions */
static void nikon_enable_iep_cmp_event(void *pru_iep, uint64_t trigger_point, uint8_t event_num);
static void nikon_disable_iep_cmp_event(void *pru_iep, uint8_t event_num);

/* Interrupt Configuration */
static void nikon_interrupt_config(void *pruicss_handle);

/* IRQ Handlers */
void nikon_pru_irq_handler(void *pruicss_handle);

#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
void nikon_rtupru_irq_handler(void *pruicss_handle);
void nikon_txpru_irq_handler(void *pruicss_handle);
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Only PRU IRQ
 * handler is defined.
 */
void nikon_pru_irq_handler_second_slice(void *pruicss_handle);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Configure IEP CAP mode for periodic trigger using SYNC signal
 *
 * \details This function configures IEP SYNC OUT0 generation and routes it to CAP inputs via TIMESYNC/GPIOMUX router
 *
 * Channel Configuration:
 * - Non-Load Share: CAP6 (LATCH0_IN0) via TIMESYNC router
 * - Load Share Ch0: CAP6 (LATCH0_IN0) via TIMESYNC router
 * - Load Share Ch1: CAP0 from GPIO0_GPIO_4 via GPIOMUX router (requires external GPIO connection)
 * - Load Share Ch2: CAP7 (LATCH1_IN0) via TIMESYNC router
 *
 * \param handle Nikon driver handle
 * \param iep_sync0_period IEP SYNC OUT0 period in IEP clock cycles
 */
#if defined(SOC_AM243X)
static void nikon_config_iep_cap_for_sync(nikon_handle handle, uint32_t iep_sync0_period)
{
    const nikon_attrs *attrs = nikon_get_attrs(handle);
    void *pru_iep = attrs->iep_base_addr;
    uint32_t reg_value;

    /* Configure IEP CMP1 to start SYNC OUT0 after 100 cycles */
    nikon_enable_iep_cmp_event(pru_iep, NIKON_IEP_CMP1_START_DELAY, NIKON_IEP_CMP_EVENT_FOR_SYNC0);

    /* Enable SYNC OUT0 cyclic generation */
    reg_value = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value |= (NIKON_IEP_SYNC_CTRL_SYNC01_EN_MASK | NIKON_IEP_SYNC_CTRL_SYNC0_EN_MASK);
    reg_value |= NIKON_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK;
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Configure SYNC OUT0 pulse width and period */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_PWIDTH_REG, NIKON_IEP_SYNC0_PULSE_WIDTH);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC0_PERIOD_REG, (iep_sync0_period-1));

    /* Route SYNC OUT0 to LATCH inputs via TIMESYNC/GPIOMUX router */
    if(attrs->load_share_enabled)
    {
        if(attrs->iep_instance == 0)
        {
            if(attrs->channel0_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN29);
                }
                else
                {
                    /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN25);
                }
            }
            if(attrs->channel2_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT13_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN29);
                }
                else
                {
                    /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT9_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN25);
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
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN31);
                }
                else
                {
                    /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN27);
                }
            }
            if(attrs->channel2_enabled)
            {
                if(attrs->pruicss_instance == 1)
                {
                    /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT15_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN31);
                }
                else
                {
                    /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
                    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT11_OFFSET),
                                NIKON_TIMESYNC_EVENT_ROUTER_IN27);
                }
            }
        }

        /* GPIOMUX router configuration for channel 1 */
        if(attrs->channel1_enabled)
        {
            if(attrs->pruicss_instance == 1)
            {
                DebugP_log("\r\n ERROR: GPIOMUX router can not be configured for routing event to PRU-ICSSG1\r\n");
                DebugP_assert(0);
            }

            /* Route GPIO to IEP CAP0 input*/
            if(attrs->iep_instance == 0)
            {
                /* Connect GPIO to IEP0 CAP_IN */
                HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + NIKON_GPIOMUX_INTROUTER0_IEP0_CAP_OFFSET),
                            NIKON_GPIOMUX_INTROUTER0_CAP_GPIO_IN);
            }
            else
            {
                /* Connect GPIO to IEP1 CAP_IN */
                HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + NIKON_GPIOMUX_INTROUTER0_IEP1_CAP_OFFSET),
                            NIKON_GPIOMUX_INTROUTER0_CAP_GPIO_IN);
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
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                            NIKON_TIMESYNC_EVENT_ROUTER_IN29);
            }
            else
            {
                /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                            NIKON_TIMESYNC_EVENT_ROUTER_IN25);
            }
        }
        else
        {
            if(attrs->pruicss_instance == 1)
            {
                /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                            NIKON_TIMESYNC_EVENT_ROUTER_IN31);
            }
            else
            {
                /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
                HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + NIKON_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                            NIKON_TIMESYNC_EVENT_ROUTER_IN27);
            }
        }
    }
}
#endif /* SOC_AM243X */

/**
 * \brief Disable IEP CAP mode SYNC generation
 *
 * \details This function disables IEP SYNC OUT0 generation and CMP1 event used for sync.
 * This is typically called during periodic mode shutdown.
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 */
#if defined(SOC_AM243X)
/**
 * \brief Disable IEP SYNC OUT0 generation for CAP mode
 *
 * \details This function disables IEP SYNC OUT0 signal generation that was
 *          configured for CAP mode periodic triggering. Called during cleanup
 *          when stopping periodic mode.
 *
 *          **Operations performed:**
 *          - Disables SYNC OUT0 enable bit
 *          - Disables SYNC OUT0 cyclic generation
 *          - Disables CMP1 event
 *
 * \param[in] pru_iep IEP register base address
 *
 * \note Only used on AM243x for CAP mode cleanup
 * \note Companion function to nikon_config_iep_cap_for_sync()
 */
static void nikon_disable_iep_cap_sync(void *pru_iep)
{
    uint32_t reg_value;

    /* Disable SYNC OUT0 generation */
    reg_value = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value &= ~(NIKON_IEP_SYNC_CTRL_SYNC01_EN_MASK | NIKON_IEP_SYNC_CTRL_SYNC0_EN_MASK); /* SYNC OUT0 disable */
    reg_value &= ~NIKON_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK; /* SYNC OUT0 cyclic disable */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Disable CMP1 event (configured for SYNC OUT0) */
    nikon_disable_iep_cmp_event(pru_iep, NIKON_IEP_CMP_EVENT_FOR_SYNC0);
}
#endif /* SOC_AM243X */

/**
 * \brief Enable IEP reset on CMP0 event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param iep_reset_count IEP counter value for reset (period)
 */
static void nikon_enable_iep_reset_on_cmp0(void *pru_iep, uint64_t iep_reset_count)
{
    uint16_t event;
    uint32_t reg0;
    uint32_t reg1;

    /* Clear event */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, (1 << NIKON_IEP_CMP_EVENT_FOR_RESET));

    /* Set IEP_CMP0_REG0 and IEP_CMP0_REG1 registers */
    reg0 = NIKON_GET_LOWER_32BITS(iep_reset_count);
    reg1 = NIKON_GET_UPPER_32BITS(iep_reset_count);

    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0), reg0);
    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1), reg1);

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);

    /* Enable IEP reset by CMP0 event */
    event |= (1 << NIKON_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* CMP0 enable bit */
    event |= (1 << NIKON_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Reset counter enable bit */

    /* Enable event */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}

/**
 * \brief Disable IEP reset on CMP0 event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 */
static void nikon_disable_iep_reset_on_cmp0(void *pru_iep)
{
    uint16_t event;

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);

    /* Disable IEP reset by CMP0 event */
    event &= ~(1 << NIKON_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* Clear CMP0 enable bit */
    event &= ~(1 << NIKON_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Clear Reset counter enable bit */

    /* Write back the modified value */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /* Clear IEP_CMP0_REG0 and IEP_CMP0_REG1 registers */
    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0), 0);
    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1), 0);
}

/**
 * \brief Enable IEP counter
 *
 * \param handle Nikon driver handle
 */
static void nikon_enable_iep_counter(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    int32_t status;

    priv = nikon_get_priv(handle);
    attrs = nikon_get_attrs(handle);

    /* Configure and enable IEP counter */
    status = PRUICSS_setIepCounterIncrementValue(priv->pruicss_handle, attrs->iep_instance, NIKON_IEP_COUNTER_INCREMENT);
    DebugP_assert(status == SystemP_SUCCESS);

    status = PRUICSS_controlIepCounter(priv->pruicss_handle, attrs->iep_instance, NIKON_IEP_COUNTER_ENABLE);
    DebugP_assert(status == SystemP_SUCCESS);
}

/**
 * \brief Disable IEP counter
 *
 * \param handle Nikon driver handle
 */
static void nikon_disable_iep_counter(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    int32_t status;

    priv = nikon_get_priv(handle);
    attrs = nikon_get_attrs(handle);

    /* Disable IEP counter */
    status = PRUICSS_controlIepCounter(priv->pruicss_handle, attrs->iep_instance, NIKON_IEP_COUNTER_DISABLE);
    DebugP_assert(status == SystemP_SUCCESS);
}

/**
 * \brief Disable IEP CMP event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param event_num CMP event number (0-15)
 */
static void nikon_disable_iep_cmp_event(void *pru_iep, uint8_t event_num)
{
    uint32_t reg0;

    /* Disable the CMP event */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Clear the CMP_EN bit (AND with the negated new value) */
    reg0 &= ~(((uint32_t)1U << event_num) << NIKON_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);

    /* Clear CMP register values */
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*NIKON_8_BYTE_REG_OFFSET + NIKON_8_BYTE_REG_OFFSET), 0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*NIKON_8_BYTE_REG_OFFSET + NIKON_8_BYTE_REG_OFFSET), 0);
    }
    else
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*NIKON_8_BYTE_REG_OFFSET), 0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*NIKON_8_BYTE_REG_OFFSET), 0);
    }
}

/**
 * \brief Disable IEP CAP event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param event_num CAP event number (0-7)
 */
static void nikon_disable_iep_cap_event(void *pru_iep, uint8_t event_num)
{
    uint32_t reg0;

    /* Disable the CAP event */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));

    /*
     * Clear the CAP_EN bit (AND with the negated new value)
     * NOTE: IEP CAP6 and CAP7 has 2 register bits each. So bit 8 needs
     * to be cleared for CAP7. Only clearing capture rise bit for CAP6 and CAP7.
     */
    if(event_num == 7)
    {
        reg0 &= ~((uint32_t)1U << (event_num + 1));
    }
    else
    {
        reg0 &= ~((uint32_t)1U << event_num);
    }
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);
}

/**
 * \brief Enable IEP CAP event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param event_num CAP event number (0-7)
 */
static void nikon_enable_iep_cap_event(void *pru_iep, uint8_t event_num)
{
    uint32_t reg0;

    /* Configure the CAP event in IEP hardware register */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));

    /*
     * Set the CAP_EN bit (OR with the new value)
     * NOTE: IEP CAP6 and CAP7 has 2 register bits each. So bit 8 needs
     * to be set for CAP7. Only setting capture rise bit for CAP6 and CAP7.
     */
    if(event_num == 7)
    {
        reg0 |= ((uint32_t)1U << (event_num + 1));
    }
    else
    {
        reg0 |= ((uint32_t)1U << event_num);
    }
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);
}

/**
 * \brief Enable IEP CMP event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param trigger_point IEP counter value for trigger
 * \param event_num CMP event number (0-15)
 */
static void nikon_enable_iep_cmp_event(void *pru_iep, uint64_t trigger_point, uint8_t event_num)
{
    uint32_t reg0;
    uint32_t reg1;

    /* Clear event */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, (uint16_t)(1 << event_num));

    /* Write trigger point to CMP registers */
    reg0 = NIKON_GET_LOWER_32BITS(trigger_point);
    reg1 = NIKON_GET_UPPER_32BITS(trigger_point);
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*NIKON_8_BYTE_REG_OFFSET + NIKON_8_BYTE_REG_OFFSET), reg0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*NIKON_8_BYTE_REG_OFFSET + NIKON_8_BYTE_REG_OFFSET), reg1);
    }
    else
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*NIKON_8_BYTE_REG_OFFSET), reg0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*NIKON_8_BYTE_REG_OFFSET), reg1);
    }

    /* Configure the IEP CMP event in hardware registers */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));

    /* Set the CMP_EN bit (OR with the new value) */
    reg0 |= ((uint32_t)1U << event_num) << NIKON_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT;

    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);
}

/**
 * \brief Configure IEP timer for Nikon periodic trigger mode
 *
 * \details This function configures the PRU-ICSS IEP (Industrial Ethernet Peripheral) timer
 *          to support periodic trigger mode for Nikon encoder transactions. It handles
 *          both CMP (compare) and CAP (capture) modes based on configuration.
 *
 *          **Configuration performed:**
 *          1. Disables IEP counter
 *          2. Resets IEP counter to zero
 *          3. **CMP Mode (is_cap_mode = 0):**
 *             - Enables IEP counter reset on CMP0 event (defines period)
 *             - Configures CMP events for each enabled channel with trigger counts
 *             - IEP counter automatically resets when reaching iep_reset_count
 *          4. **CAP Mode (is_cap_mode = 1):**
 *             - On AM243x: Configures IEP SYNC output and routes to capture pins
 *             - Enables CAP events for each enabled channel
 *             - CAP events triggered by external signals
 *          5. Re-enables IEP counter
 *
 *          **Multi-instance handling:**
 *          - Uses CONFIG_NIKON0 handle for PRU-ICSS instance-level resources (IEP is shared)
 *          - Assumes all Nikon instances use the same PRU-ICSS and IEP instance
 *          - Supports dual PRU slice configuration when NIKON_DUAL_PRU_SLICE_ENABLE defined
 *
 *          **Load share mode:**
 *          - Iterates through enabled channels based on channel_mask
 *          - Configures CMP/CAP events for each active channel independently
 *
 * \param[in] nikon_periodic_interface Pointer to periodic interface structure
 * \note This function assumes the handle and IEP base address are valid (set by nikon_init())
 */
static void nikon_config_iep(nikon_periodic_interface *nikon_periodic_interface)
{
    const nikon_attrs *attrs = nikon_get_attrs(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    uint8_t ch_idx;
    uint64_t iep_count = nikon_periodic_interface->iep_reset_count;
    void *pru_iep = attrs->iep_base_addr;
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    const nikon_attrs *attrs1 = nikon_get_attrs(nikon_periodic_interface->handle[CONFIG_NIKON1]);
#endif

    /* PRU-ICSS Level Global Configuration uses first Nikon handle */
    /* ASSUMPTION: Same PRU-ICSS instance and IEP instance are used for multiple Nikon handles in this example */

    /* Disable IEP counter */
    nikon_disable_iep_counter(nikon_periodic_interface->handle[CONFIG_NIKON0]);

    /* Set IEP counter to ZERO */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /* Configure IEP reset/sync based on mode */
    if(nikon_periodic_interface->is_cap_mode)
    {
#if defined(SOC_AM243X)
        /*
         * Configure IEP for generating SYNC and route it to IEP capture pins using TIMESYNC/GPIOMUX router on AM243x.
         *
         * If CONFIG_NIKON_NUM_INSTANCES > 1 and CAP mode is used, configuration for signal routing to
         * capture pins needs to be added based on availability.
         */
        nikon_config_iep_cap_for_sync(nikon_periodic_interface->handle[CONFIG_NIKON0], NIKON_GET_LOWER_32BITS(iep_count));
#endif
        /* Configure CAP events for channels */
        if(attrs->load_share_enabled)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < NIKON_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    nikon_enable_iep_cap_event(pru_iep, attrs->iep_cap_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode*/
            nikon_enable_iep_cap_event(pru_iep, attrs->iep_cap_event[0]);
        }
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
        /* Configure CAP event for second slice if enabled */
        /* Configuring CAP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            nikon_enable_iep_cap_event(pru_iep, attrs1->iep_cap_event[0]);
        }
#endif
    }
    else
    {
        /* CMP mode: Enable IEP reset on CMP0 */
        /* PRU-ICSS Level Global Configuration uses first Nikon handle */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance are used for multiple Nikon handles in this example */
        nikon_enable_iep_reset_on_cmp0(pru_iep, iep_count);

        /* Configure CMP events for channels */
        if(attrs->load_share_enabled)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < NIKON_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    nikon_enable_iep_cmp_event(pru_iep, nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][ch_idx], attrs->iep_cmp_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            nikon_enable_iep_cmp_event(pru_iep, nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON0][0], attrs->iep_cmp_event[0]);
        }
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
        /* Configure CMP event for second slice if enabled */
        /* Configuring CMP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            nikon_enable_iep_cmp_event(pru_iep, nikon_periodic_interface->periodic_trigger_count[CONFIG_NIKON1][0], attrs1->iep_cmp_event[0]);
        }
#endif
    }

    /* PRU-ICSS Level Global Configuration uses first Nikon handle */
    /* ASSUMPTION: Same PRU-ICSS instance and IEP instance are used for multiple Nikon handles in this example */
    /* Enable IEP counter */
    nikon_enable_iep_counter(nikon_periodic_interface->handle[CONFIG_NIKON0]);
}

/**
 * \brief Configure and register PRU interrupt handlers for Nikon periodic mode
 *
 * \details This function registers interrupt service routines (ISRs) for PRU firmware
 *          interrupts in periodic trigger mode. When PRU firmware completes a Nikon
 *          encoder transaction, it triggers an interrupt to notify the R5F host.
 *
 *          **Load share mode (NIKON_MODE_MULTI_CHANNEL_MULTI_PRU):**
 *          - Channel 0: RTU-PRU interrupt (nikon_rtupru_irq_handler)
 *          - Channel 1: PRU interrupt (nikon_pru_irq_handler)
 *          - Channel 2: TX-PRU interrupt (nikon_txpru_irq_handler)
 *          - Each channel has independent ISR registration based on enabled channels
 *
 *          **Single/Multi-channel single PRU mode:**
 *          - Single PRU interrupt (nikon_pru_irq_handler)
 *          - One ISR handles all enabled channels on the PRU
 *
 *          **Dual PRU slice mode (NIKON_DUAL_PRU_SLICE_ENABLE):**
 *          - Registers second slice interrupt if CONFIG_NIKON1 is enabled
 *          - Uses separate interrupt number and handler for second slice
 *
 * \param[in] pruicss_handle PRU-ICSS handle obtained from nikon_priv structure.
 *                           Passed to ISR callbacks for PRU-ICSS register access.
 *
 * \note This function uses HwiP_construct() which asserts on failure
 * \note Interrupt numbers are device and configuration specific (defined by macros)
 */
static void nikon_interrupt_config(void *pruicss_handle)
{
    int32_t status;
    HwiP_Params hwi_params;

#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if(CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    /* Register and enable RTU-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_RTU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_rtupru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[CONFIG_NIKON0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if(CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[CONFIG_NIKON0][1], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if(CONFIG_NIKON0_CHANNEL2_ENABLED == 1)

    /* Register and enable TX-PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_TXPRU_NIKON_INT_NUM;
    hwi_params.callback = &nikon_txpru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[CONFIG_NIKON0][2], &hwi_params);
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
    status              = HwiP_construct(&gNikonHwiObject[CONFIG_NIKON0][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_NIKON_INT_NUM_SECOND_SLICE;
    hwi_params.callback = &nikon_pru_irq_handler_second_slice;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gNikonHwiObject[CONFIG_NIKON1][0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
}

int32_t nikon_config_periodic_mode(nikon_periodic_interface *nikon_periodic_interface)
{
    int32_t status;
    uint32_t    i;
    nikon_priv  *priv;
    void        *pruicss_handle;

    /* NULL check on interface pointer and handle(s) */
    if(nikon_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        if(nikon_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }
    /* PRU-ICSS Level Global Configuration uses first Nikon handle */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Nikon handles in this example */
    priv = nikon_get_priv(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    pruicss_handle = (void *)(priv->pruicss_handle);

    /* Configure IEP */
    nikon_config_iep(nikon_periodic_interface);

    /* Initialize PRU-ICSS Interrupt Controller */
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Nikon handles in this example */
#if(CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
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
    nikon_interrupt_config(pruicss_handle);
    return SystemP_SUCCESS;
}

int32_t nikon_stop_periodic_mode(nikon_periodic_interface *nikon_periodic_interface)
{
    const nikon_attrs *attrs;
    uint8_t ch_idx;
    uint32_t i;
    void *pru_iep;
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    const nikon_attrs *attrs1;
#endif

    /* NULL check on interface pointer and handle(s) */
    if(nikon_periodic_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        if(nikon_periodic_interface->handle[i] == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    attrs = nikon_get_attrs(nikon_periodic_interface->handle[CONFIG_NIKON0]);
    pru_iep = attrs->iep_base_addr;
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    attrs1 = nikon_get_attrs(nikon_periodic_interface->handle[CONFIG_NIKON1]);
#endif

    /* Disable IEP counter first */
    /* PRU-ICSS Level Global Configuration uses first Nikon handle */
    /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple Nikon handles in this example */
    nikon_disable_iep_counter(nikon_periodic_interface->handle[CONFIG_NIKON0]);

    /* Disable events based on mode */
    if(nikon_periodic_interface->is_cap_mode)
    {
        /* CAP mode: Disable capture events */
        if(attrs->load_share_enabled)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < NIKON_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    nikon_disable_iep_cap_event(pru_iep, attrs->iep_cap_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            nikon_disable_iep_cap_event(pru_iep, attrs->iep_cap_event[0]);
        }
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
        /* Disable CAP event for second slice if enabled */
        /* Configuring CAP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            nikon_disable_iep_cap_event(pru_iep, attrs1->iep_cap_event[0]);
        }
#endif

        /* Disable IEP SYNC generation for CAP mode */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple Nikon handles in this example */
#if defined(SOC_AM243X)
        nikon_disable_iep_cap_sync(pru_iep);
#endif /* SOC_AM243X */
    }
    else
    {
        /* CMP mode: Disable compare events */
        if(attrs->load_share_enabled)
        {
            /* Load share mode: Iterate through enabled channels using channel_mask */
            for(ch_idx = 0; ch_idx < NIKON_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    nikon_disable_iep_cmp_event(pru_iep, attrs->iep_cmp_event[ch_idx]);
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            nikon_disable_iep_cmp_event(pru_iep, attrs->iep_cmp_event[0]);
        }
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
        /* Disable CMP event for second slice if enabled */
        /* Configuring CMP events for SINGLE_CHANNEL_SINGLE_PRU mode, For load share mode it is not added*/
        /* This code is only tested on AM261x with DUAL PRU SLICE MODE*/
        if(attrs1->load_share_enabled == 0)
        {
            nikon_disable_iep_cmp_event(pru_iep, attrs1->iep_cmp_event[0]);
        }
#endif

        /* Disable IEP reset on CMP0 event */
        /* PRU-ICSS Level Global Configuration uses first Nikon handle */
        /* ASSUMPTION: Same PRU-ICSS instance and IEP instance is used for multiple Nikon handles in this example */
        nikon_disable_iep_reset_on_cmp0(pru_iep);
    }

#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if(CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    HwiP_destruct(&gNikonHwiObject[CONFIG_NIKON0][0]);
#endif
#if(CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    HwiP_destruct(&gNikonHwiObject[CONFIG_NIKON0][1]);
#endif
#if(CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    HwiP_destruct(&gNikonHwiObject[CONFIG_NIKON0][2]);
#endif
#else
    HwiP_destruct(&gNikonHwiObject[CONFIG_NIKON0][0]);
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    /* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
    * NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Configuration is
    * done for one channel only, assuming single PRU mode.
    */
    HwiP_destruct(&gNikonHwiObject[CONFIG_NIKON1][0]);
#endif
    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void nikon_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* In load share mode, index 1 is used for channel 1 connected to PRU */
    gPruNikonIrqCnt[CONFIG_NIKON0][1]++;
#else
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruNikonIrqCnt[CONFIG_NIKON0][0]++;
#endif
    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_NIKON_EVT);
}

#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
/* RTU-PRU FW IRQ handler */
void nikon_rtupru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruNikonIrqCnt[CONFIG_NIKON0][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, RTU_TRIGGER_HOST_NIKON_EVT);
}

/* TX-PRU FW IRQ handler */
void nikon_txpru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruNikonIrqCnt[CONFIG_NIKON0][2]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, TXPRU_TRIGGER_HOST_NIKON_EVT);
}
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
/* NOTE: Dual handle example using PRU0 and PRU1 is tested only with
 * NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. Only PRU IRQ
 * handler is defined.
 */

/* PRU FW IRQ handler */
void nikon_pru_irq_handler_second_slice(void *pruicss_handle)
{
    /* Increment IRQ count */
    /* In single PRU mode, index 0 is used for any channel connected to PRU */
    gPruNikonIrqCnt[CONFIG_NIKON1][0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_NIKON_EVT);
}
#endif