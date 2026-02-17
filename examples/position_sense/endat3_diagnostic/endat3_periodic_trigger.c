/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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
 * \file  endat3_periodic_trigger.c
 *
 * \brief EnDAT3 periodic trigger mode implementation using IEP timer
 *
 * This file implements periodic trigger mode for EnDAT3 encoder interface.
 * In periodic mode, encoder position command is automatically triggered at regular
 * intervals by PRU using the PRU-ICSS Industrial Ethernet Peripheral (IEP) timer,
 * eliminating the need for host (R5F) intervention to trigger a command. After the
 * response is received, PRU triggers host (R5F) interrupt.
 *
 * \par Periodic Trigger Modes:
 * The EnDAT3 driver supports two IEP-based periodic trigger modes:
 * - **CMP Mode (Compare)**: Time-based periodic sampling using IEP compare events (CMP0-CMP15)
 *   - Firmware monitors IEP counter and triggers when counter matches compare value
 * - **CAP Mode (Capture)**: Event-driven sampling using IEP capture events (CAP0-CAP7)
 *   - Firmware waits for external hardware signal routed through TIMESYNC router (AM243x)
 *     or XBAR (AM26x)
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
#include "endat3_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/endat3/include/endat3_drv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * NOTE: Host Interrupt Number Synchronization
 *
 * The host interrupt numbers defined below (e.g., ICSS_PRU_ENDAT3_INT_NUM)
 * must match the interrupt channel assignments in SysConfig:
 *     PRU(ICSS) module -> INTC section -> INTC Host Interrupt
 *
 * Host interrupt channels route PRU events to the R5F core. The mapping is:
 *     Host Interrupt 2-9 in SysConfig = HOST_INTR_PEND_0-7 registers
 *
 * If the host interrupt assignments are changed in SysConfig, following macros need
 * to be updated accordingly to maintain proper interrupt delivery from PRU to R5F.
 */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif
#else
/* ICSSG Interrupt Numbers */
#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#endif
#endif

/*
 * NOTE: The INTC event numbers defined below must match the corresponding
 * definitions in the PRU firmware header file:
 *     "source/position_sense/endat3/firmware/endat3_params.h"
 *
 * These event numbers are used for communication between the R5F and
 * PRU firmware. Any changes to these values must be synchronized between
 * both files to ensure proper interrupt handling.
 */
#if(CONFIG_ENDAT3_0_PRUICSS_SLICE == 1)
/** \brief PRU EnDAT3 interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_ENDAT3_EVT      (2+16)
#else
/** \brief PRU EnDAT3 interrupt event number (21 = 5 + 16) */
#define PRU_TRIGGER_HOST_ENDAT3_EVT      (5+16)
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gEndat3HwiObject[CONFIG_ENDAT3_NUM_INSTANCES];
uint32_t gPruEndat3IrqCnt[CONFIG_ENDAT3_NUM_INSTANCES] = {0};

/* PRU-ICSS INTC Initialization Data Structure */
#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* IEP Configuration Functions */
#if defined(SOC_AM243X)
static int32_t endat3_config_iep_cap_for_sync(endat3_handle handle, uint32_t iep_sync0_period);
static void endat3_disable_iep_cap_sync(void *pru_iep);
#endif /* SOC_AM243X */

static int32_t endat3_config_iep(endat3_periodic_interface *endat3_periodic_interface);

/* IEP Counter Control */
static int32_t endat3_enable_iep_counter(PRUICSS_Handle pruicss_handle, uint8_t iep_instance);
static int32_t endat3_disable_iep_counter(PRUICSS_Handle pruicss_handle, uint8_t iep_instance);

/* IEP Reset Control */
static void endat3_enable_iep_reset_on_cmp0(void *pru_iep, uint64_t iep_reset_count);
static void endat3_disable_iep_reset_on_cmp0(void *pru_iep);

/* IEP CAP Event Functions */
static void endat3_enable_iep_cap_event(void *pru_iep, uint8_t event_num);
static void endat3_disable_iep_cap_event(void *pru_iep, uint8_t event_num);

/* IEP CMP Event Functions */
static void endat3_enable_iep_cmp_event(void *pru_iep, uint64_t trigger_point, uint8_t event_num);
static void endat3_disable_iep_cmp_event(void *pru_iep, uint8_t event_num);

/* Interrupt Configuration */
static void endat3_interrupt_config(void *pruicss_handle);

/* IRQ Handler */
void endat3_pru_irq_handler(void *pruicss_handle);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Configure IEP CAP mode for periodic trigger using SYNC signal
 *
 * \details This function configures IEP SYNC OUT0 generation and routes it to CAP6 (LATCH_IN0) via TIMESYNC router
 *
 * \param handle EnDAT3 driver handle
 * \param iep_sync0_period IEP SYNC OUT0 period in IEP clock cycles
 */
#if defined(SOC_AM243X)
static int32_t endat3_config_iep_cap_for_sync(endat3_handle handle, uint32_t iep_sync0_period)
{
    const endat3_attrs *attrs = NULL;
    void *pru_iep;
    uint32_t reg_value;

    attrs = endat3_get_attrs(handle);
    if((handle == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_iep_cap_for_sync() failed due to NULL handle/attrs");
        return SystemP_FAILURE;
    }

    pru_iep = attrs->iep_base_addr;
    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_iep_cap_for_sync() failed due to NULL iep_base_addr");
        return SystemP_FAILURE;
    }

    /* Configure IEP CMP1 to start SYNC OUT0 after 100 cycles */
    endat3_enable_iep_cmp_event(pru_iep, ENDAT3_IEP_CMP1_START_DELAY, ENDAT3_IEP_CMP_EVENT_FOR_SYNC0);

    /* Enable SYNC OUT0 cyclic generation */
    reg_value = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value |= (ENDAT3_IEP_SYNC_CTRL_SYNC01_EN_MASK | ENDAT3_IEP_SYNC_CTRL_SYNC0_EN_MASK);
    reg_value |= ENDAT3_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK;
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Configure SYNC OUT0 pulse width and period */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_PWIDTH_REG, ENDAT3_IEP_SYNC0_PULSE_WIDTH);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC0_PERIOD_REG, (iep_sync0_period-1));

    /* Route SYNC OUT0 to LATCH inputs via TIMESYNC router */
    if(attrs->iep_instance == 0)
    {
        if(attrs->pruicss_instance == 1)
        {
            /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT3_TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                        ENDAT3_TIMESYNC_EVENT_ROUTER_IN29);
        }
        else
        {
            /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT3_TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                        ENDAT3_TIMESYNC_EVENT_ROUTER_IN25);
        }
    }
    else
    {
        if(attrs->pruicss_instance == 1)
        {
            /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT3_TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                        ENDAT3_TIMESYNC_EVENT_ROUTER_IN31);
        }
        else
        {
            /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ENDAT3_TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                        ENDAT3_TIMESYNC_EVENT_ROUTER_IN27);
        }
    }

    return SystemP_SUCCESS;
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
 * \note Companion function to endat3_config_iep_cap_for_sync()
 */
static void endat3_disable_iep_cap_sync(void *pru_iep)
{
    uint32_t reg_value;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_cap_sync() failed due to NULL pru_iep");
        return;
    }

    /* Disable SYNC OUT0 generation */
    reg_value = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value &= ~(ENDAT3_IEP_SYNC_CTRL_SYNC01_EN_MASK | ENDAT3_IEP_SYNC_CTRL_SYNC0_EN_MASK); /* SYNC OUT0 disable */
    reg_value &= ~ENDAT3_IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK; /* SYNC OUT0 cyclic disable */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /* Disable CMP1 event (configured for SYNC OUT0) */
    endat3_disable_iep_cmp_event(pru_iep, ENDAT3_IEP_CMP_EVENT_FOR_SYNC0);
}
#endif /* SOC_AM243X */

/**
 * \brief Enable IEP reset on CMP0 event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param iep_reset_count IEP counter value for reset (period)
 */
static void endat3_enable_iep_reset_on_cmp0(void *pru_iep, uint64_t iep_reset_count)
{
    uint16_t event;
    uint32_t reg0;
    uint32_t reg1;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_enable_iep_reset_on_cmp0() failed due to NULL pru_iep");
        return;
    }

    /* Clear event */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, (1 << ENDAT3_IEP_CMP_EVENT_FOR_RESET));

    /* Set IEP_CMP0_REG0 and IEP_CMP0_REG1 registers */
    reg0 = ENDAT3_GET_LOWER_32BITS(iep_reset_count);
    reg1 = ENDAT3_GET_UPPER_32BITS(iep_reset_count);

    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0), reg0);
    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1), reg1);

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);

    /* Enable IEP reset by CMP0 event */
    event |= (1 << ENDAT3_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* CMP0 enable bit */
    event |= (1 << ENDAT3_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Reset counter enable bit */

    /* Enable event */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);
}

/**
 * \brief Disable IEP reset on CMP0 event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 */
static void endat3_disable_iep_reset_on_cmp0(void *pru_iep)
{
    uint16_t event;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_reset_on_cmp0() failed due to NULL pru_iep");
        return;
    }

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);

    /* Disable IEP reset by CMP0 event */
    event &= ~(1 << ENDAT3_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* Clear CMP0 enable bit */
    event &= ~(1 << ENDAT3_IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Clear Reset counter enable bit */

    /* Write back the modified value */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /* Clear IEP_CMP0_REG0 and IEP_CMP0_REG1 registers */
    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0), 0);
    HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1), 0);
}

/**
 * \brief Enable IEP counter
 *
 * \param handle EnDAT3 driver handle
 */
static int32_t endat3_enable_iep_counter(PRUICSS_Handle pruicss_handle, uint8_t iep_instance)
{
    int32_t status;

    if(pruicss_handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_enable_iep_counter() failed due to NULL pruicss_handle");
        return SystemP_FAILURE;
    }

    /* Configure and enable IEP counter */
    status = PRUICSS_setIepCounterIncrementValue(pruicss_handle, iep_instance, ENDAT3_IEP_COUNTER_INCREMENT);

    if(status == SystemP_SUCCESS)
    {
        status = PRUICSS_controlIepCounter(pruicss_handle, iep_instance, ENDAT3_IEP_COUNTER_ENABLE);
    }

    return status;
}

/**
 * \brief Disable IEP counter
 *
 * \param handle EnDAT3 driver handle
 */
static int32_t endat3_disable_iep_counter(PRUICSS_Handle pruicss_handle, uint8_t iep_instance)
{
    int32_t status;

    if(pruicss_handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_counter() failed due to NULL pruicss_handle");
        return SystemP_FAILURE;
    }

    /* Disable IEP counter */
    status = PRUICSS_controlIepCounter(pruicss_handle, iep_instance, ENDAT3_IEP_COUNTER_DISABLE);
    return status;
}

/**
 * \brief Disable IEP CMP event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param event_num CMP event number (0-15)
 */
static void endat3_disable_iep_cmp_event(void *pru_iep, uint8_t event_num)
{
    uint32_t reg0;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_cmp_event() failed due to NULL pru_iep");
        return;
    }

    /* Disable the CMP event */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Clear the CMP_EN bit (AND with the negated new value) */
    reg0 &= ~(((uint32_t)1U << event_num) << ENDAT3_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);

    /* Clear CMP register values */
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*ENDAT3_8_BYTE_REG_OFFSET + ENDAT3_8_BYTE_REG_OFFSET), 0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*ENDAT3_8_BYTE_REG_OFFSET + ENDAT3_8_BYTE_REG_OFFSET), 0);
    }
    else
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*ENDAT3_8_BYTE_REG_OFFSET), 0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*ENDAT3_8_BYTE_REG_OFFSET), 0);
    }
}

/**
 * \brief Disable IEP CAP event
 *
 * \param pru_iep Pointer to PRU-ICSS IEP Base Address
 * \param event_num CAP event number (0-7)
 */
static void endat3_disable_iep_cap_event(void *pru_iep, uint8_t event_num)
{
    uint32_t reg0;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_cap_event() failed due to NULL pru_iep");
        return;
    }

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
static void endat3_enable_iep_cap_event(void *pru_iep, uint8_t event_num)
{
    uint32_t reg0;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_enable_iep_cap_event() failed due to NULL pru_iep");
        return;
    }

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
static void endat3_enable_iep_cmp_event(void *pru_iep, uint64_t trigger_point, uint8_t event_num)
{
    uint32_t reg0;
    uint32_t reg1;

    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_enable_iep_cmp_event() failed due to NULL pru_iep");
        return;
    }

    /* Clear event */
    HW_WR_REG16((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, (uint16_t)(1 << event_num));

    /* Write trigger point to CMP registers */
    reg0 = ENDAT3_GET_LOWER_32BITS(trigger_point);
    reg1 = ENDAT3_GET_UPPER_32BITS(trigger_point);
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(event_num > 7)
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*ENDAT3_8_BYTE_REG_OFFSET + ENDAT3_8_BYTE_REG_OFFSET), reg0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*ENDAT3_8_BYTE_REG_OFFSET + ENDAT3_8_BYTE_REG_OFFSET), reg1);
    }
    else
    {
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*ENDAT3_8_BYTE_REG_OFFSET), reg0);
        HW_WR_REG32((uint8_t *)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*ENDAT3_8_BYTE_REG_OFFSET), reg1);
    }

    /* Configure the IEP CMP event in hardware registers */
    /* Read the current register value */
    reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));

    /* Set the CMP_EN bit (OR with the new value) */
    reg0 |= ((uint32_t)1U << event_num) << ENDAT3_IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT;

    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);
}

/**
 * \brief Configure IEP timer for EnDAT3 periodic trigger mode
 *
 * \details This function configures the PRU-ICSS IEP (Industrial Ethernet Peripheral) timer
 *          to support periodic trigger mode for encoder transactions. It handles
 *          both CMP (compare) and CAP (capture) modes based on configuration.
 *
 *          **Configuration performed:**
 *          1. Disables IEP counter
 *          2. Resets IEP counter to zero
 *          3. **CMP Mode (is_cap_mode = 0):**
 *             - Enables IEP counter reset on CMP0 event (defines period)
 *             - Configures CMP event
 *             - IEP counter automatically resets when reaching iep_reset_count
 *          4. **CAP Mode (is_cap_mode = 1):**
 *             - On AM243x: Configures IEP SYNC output and routes to capture pins
 *             - Enables CAP event
 *             - CAP events triggered by external signals
 *          5. Re-enables IEP counter
 *
 * \param[in] endat3_periodic_interface Pointer to periodic interface structure
 * \note This function assumes the handle and IEP base address are valid (set by endat3_init())
 */
static int32_t endat3_config_iep(endat3_periodic_interface *endat3_periodic_interface)
{
    const endat3_attrs *attrs[CONFIG_ENDAT3_NUM_INSTANCES] = {NULL};
    endat3_priv *priv[CONFIG_ENDAT3_NUM_INSTANCES] = {NULL};
    uint32_t i;
    void *pru_iep;
    int32_t status;

    /* NULL check on interface pointer and handle(s) */
    if(endat3_periodic_interface == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_iep() failed due to NULL endat3_periodic_interface pointer");
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
    {
        attrs[i] = endat3_get_attrs(endat3_periodic_interface->handle[i]);
        priv[i] = endat3_get_priv(endat3_periodic_interface->handle[i]);
        if((endat3_periodic_interface->handle[i] == NULL) || (attrs[i] == NULL) || (priv[i] == NULL))
        {
            DebugP_log("\r\n\n|ERROR: endat3_config_iep() failed due to NULL handle/attrs/priv for EnDAT3 instance %u", i);
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first EnDAT3 handle */
    pru_iep = attrs[CONFIG_ENDAT3_0]->iep_base_addr;
    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_iep() failed due to NULL iep_base_addr");
        return SystemP_FAILURE;
    }

    if(priv[CONFIG_ENDAT3_0]->pruicss_handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_iep() failed due to NULL pruicss_handle");
        return SystemP_FAILURE;
    }

    /* Disable IEP counter */
    status = endat3_disable_iep_counter(priv[CONFIG_ENDAT3_0]->pruicss_handle, attrs[CONFIG_ENDAT3_0]->iep_instance);
    if(status == SystemP_FAILURE)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_counter() failed");
        return SystemP_FAILURE;
    }

    /* Set IEP counter to ZERO */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /* Configure IEP reset/sync based on mode */
    if(endat3_periodic_interface->is_cap_mode)
    {
#if defined(SOC_AM243X)
        /*
         * Configure IEP for generating SYNC and route it to IEP capture pins using TIMESYNC router on AM243x.
         *
         * If CONFIG_ENDAT3_NUM_INSTANCES > 1 and CAP mode is used, configuration for signal routing to
         * capture pins needs to be added based on availability.
         */
        status = endat3_config_iep_cap_for_sync(endat3_periodic_interface->handle[CONFIG_ENDAT3_0], ENDAT3_GET_LOWER_32BITS(endat3_periodic_interface->iep_reset_count));
        if(status == SystemP_FAILURE)
        {
            DebugP_log("\r\n\n|ERROR: endat3_config_iep_cap_for_sync() failed");
            return SystemP_FAILURE;
        }
#endif
        for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
        {
            /* Configure CAP events for channels */
            endat3_enable_iep_cap_event(pru_iep, attrs[i]->iep_cap_event);
        }
    }
    else
    {
        /* CMP mode: Enable IEP reset on CMP0 */
        endat3_enable_iep_reset_on_cmp0(pru_iep, endat3_periodic_interface->iep_reset_count);

        for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
        {
            /* Configure CMP events for channels */
            endat3_enable_iep_cmp_event(pru_iep, endat3_periodic_interface->periodic_trigger_count[i], attrs[i]->iep_cmp_event);
        }
    }

    /* PRU-ICSS Level Global Configuration uses first EnDAT3 handle */

    /* Enable IEP counter */
    status = endat3_enable_iep_counter(priv[CONFIG_ENDAT3_0]->pruicss_handle, attrs[CONFIG_ENDAT3_0]->iep_instance);
    if(status == SystemP_FAILURE)
    {
        DebugP_log("\r\n\n|ERROR: endat3_enable_iep_counter() failed");
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

/**
 * \brief Configure and register PRU interrupt handlers for EnDAT3 periodic mode
 *
 * \details This function registers interrupt service routines (ISRs) for PRU firmware
 *          interrupts in periodic trigger mode. When PRU firmware completes a EnDAT3
 *          encoder transaction, it triggers an interrupt to notify the R5F host.
 *
 * \param[in] pruicss_handle PRU-ICSS handle obtained from endat3_priv structure.
 *                           Passed to ISR callbacks for PRU-ICSS register access.
 *
 * \note This function uses HwiP_construct() which asserts on failure
 * \note Interrupt numbers are device and configuration specific (defined by macros)
 */
static void endat3_interrupt_config(void *pruicss_handle)
{
    int32_t status;
    HwiP_Params hwi_params;

    if(pruicss_handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_interrupt_config() failed due to NULL pruicss_handle");
        return;
    }

    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_ENDAT3_INT_NUM;
    hwi_params.callback = &endat3_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndat3HwiObject[CONFIG_ENDAT3_0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
}

int32_t endat3_config_periodic_mode(endat3_periodic_interface *endat3_periodic_interface)
{
    int32_t status;
    uint32_t i;
    endat3_priv *priv = NULL;
    void *pruicss_handle = NULL;

    /* NULL check on interface pointer and handle(s) */
    if(endat3_periodic_interface == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_periodic_mode() failed due to NULL endat3_periodic_interface pointer");
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
    {
        if(endat3_periodic_interface->handle[i] == NULL)
        {
            DebugP_log("\r\n\n|ERROR: endat3_config_periodic_mode() failed due to NULL handle for EnDAT3 instance %u", i);
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first EnDAT3 handle */
    priv = endat3_get_priv(endat3_periodic_interface->handle[CONFIG_ENDAT3_0]);
    if(priv == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_periodic_mode() failed due to NULL priv pointer");
        return SystemP_FAILURE;
    }

    pruicss_handle = (void *)(priv->pruicss_handle);
    if(pruicss_handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_periodic_mode() failed due to NULL pruicss_handle");
        return SystemP_FAILURE;
    }

    /* Configure IEP */
    status = endat3_config_iep(endat3_periodic_interface);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n\n|ERROR: endat3_config_iep() failed inside endat3_config_periodic_mode()");
        return status;
    }

    /* Initialize PRU-ICSS Interrupt Controller */
#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
    status = PRUICSS_intcInit(pruicss_handle, &icss1_intc_initdata);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n\n|ERROR: PRUICSS_intcInit() failed inside endat3_config_periodic_mode()");
        return status;
    }
#else
    status = PRUICSS_intcInit(pruicss_handle, &icss0_intc_initdata);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n\n|ERROR: PRUICSS_intcInit() failed inside endat3_config_periodic_mode()");
        return status;
    }
#endif
    /* Configure Interrupts */
    endat3_interrupt_config(pruicss_handle);
    return SystemP_SUCCESS;
}

int32_t endat3_stop_periodic_mode(endat3_periodic_interface *endat3_periodic_interface)
{
    const endat3_attrs *attrs[CONFIG_ENDAT3_NUM_INSTANCES] = {NULL};
    endat3_priv *priv[CONFIG_ENDAT3_NUM_INSTANCES] = {NULL};
    uint32_t i;
    void *pru_iep;
    int32_t status;

    /* NULL check on interface pointer and handle(s) */
    if(endat3_periodic_interface == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_stop_periodic_mode() failed due to NULL endat3_periodic_interface pointer");
        return SystemP_FAILURE;
    }

    for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
    {
        attrs[i] = endat3_get_attrs(endat3_periodic_interface->handle[i]);
        priv[i] = endat3_get_priv(endat3_periodic_interface->handle[i]);
        if((endat3_periodic_interface->handle[i] == NULL) || (attrs[i] == NULL) || (priv[i] == NULL))
        {
            DebugP_log("\r\n\n|ERROR: endat3_stop_periodic_mode() failed due to NULL handle/attrs/priv for EnDAT3 instance %u", i);
            return SystemP_FAILURE;
        }
    }

    /* PRU-ICSS Level Global Configuration uses first EnDAT3 handle */
    pru_iep = attrs[CONFIG_ENDAT3_0]->iep_base_addr;
    if(pru_iep == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_stop_periodic_mode() failed due to NULL iep_base_addr");
        return SystemP_FAILURE;
    }

    if(priv[CONFIG_ENDAT3_0]->pruicss_handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: endat3_stop_periodic_mode() failed due to NULL pruicss_handle");
        return SystemP_FAILURE;
    }

    /* Disable IEP counter first */
    status = endat3_disable_iep_counter(priv[CONFIG_ENDAT3_0]->pruicss_handle, attrs[CONFIG_ENDAT3_0]->iep_instance);
    if(status == SystemP_FAILURE)
    {
        DebugP_log("\r\n\n|ERROR: endat3_disable_iep_counter() failed");
        return SystemP_FAILURE;
    }

    /* Disable events based on mode */
    if(endat3_periodic_interface->is_cap_mode)
    {
        for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
        {
            /* CAP mode: Disable capture events */
            endat3_disable_iep_cap_event(pru_iep, attrs[i]->iep_cap_event);
        }
        /* Disable IEP SYNC generation for CAP mode */
#if defined(SOC_AM243X)
        endat3_disable_iep_cap_sync(pru_iep);
#endif /* SOC_AM243X */
    }
    else
    {
        for(i = 0; i < CONFIG_ENDAT3_NUM_INSTANCES; i++)
        {
            /* CMP mode: Disable compare events */
            endat3_disable_iep_cmp_event(pru_iep, attrs[i]->iep_cmp_event);
        }
        /* Disable IEP reset on CMP0 event */
        endat3_disable_iep_reset_on_cmp0(pru_iep);
    }

    HwiP_destruct(&gEndat3HwiObject[CONFIG_ENDAT3_0]);

    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void endat3_pru_irq_handler(void *pruicss_handle)
{
    if(pruicss_handle == NULL)
    {
        return;
    }

    /* Increment IRQ count */
    gPruEndat3IrqCnt[CONFIG_ENDAT3_0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_ENDAT3_EVT);
}