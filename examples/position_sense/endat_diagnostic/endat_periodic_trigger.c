/*
 *  Copyright (C) 2023-25 Texas Instruments Incorporated
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
#include "endat_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/endat/include/endat_interface.h>

HwiP_Params hwiPrms;
static HwiP_Object gIcssgEncoder0HwiObject[3];

/* ICSS Interrupt settings */
#ifdef PRUICSSM
#if (CONFIG_ENDAT0_PRUICSSx == 1)
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_2 )
#else
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2 )
#endif
#else
#if (CONFIG_ENDAT0_PRUICSSx == 1)
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_2 )
#else
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
#define ICSS_RTU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1 )
#define ICSS_TXPRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2 )
#endif
#endif
uint32_t gPruEnDatIrqCnt0;
uint32_t gPruEnDatIrqCnt1;
uint32_t gPruEnDatIrqCnt2;

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
#ifdef PRUICSSM
#if (CONFIG_ENDAT1_PRUICSSx == 1)
#define ICSS_PRU_ENDAT1_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_3 )
#else
#define ICSS_PRU_ENDAT1_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_3 )
#endif
#endif
uint32_t gPruEnDat1IrqCnt0;
void pruEnDat1IrqHandler(void *args);
static HwiP_Object gIcssgEncoder1HwiObject;
#endif

/* global variable */
void *gpruicss_iep;

PRUICSS_Handle gPruIcssXHandle;

/* ICSS INTC configuration */
#if (CONFIG_ENDAT0_PRUICSSx == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
/* ICSS INTC configuration */
#if (CONFIG_ENDAT1_PRUICSSx == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
 #endif
#endif

/**
 * \brief Configure IEP CAP mode for periodic trigger using SYNC signal
 *
 * This function configures the IEP (Industrial Ethernet Peripheral) to generate
 * a periodic SYNC signal and routes it to CAP (capture) inputs for triggering
 * position commands in periodic CAP trigger mode (CMD 201).
 *
 * \par Device-Specific Configuration:
 *
 * \b AM26x Devices:
 * - Uses crossbar  to configure CAP mode
 * - IEP CAP6 and CAP7 events can be used for periodic triggering
 * - EPWM SYNC output is routed to LATCH inputs using cross bar
 *
 * \b AM243x Devices:
 * - IEP SYNC OUT0 is used as the source event for Time Sync Router
 * - SYNC OUT0 output is routed to IEP LATCHs
 *
 * \par Channel Configuration:
 *
 * \b Non-Load Share Mode (Single channel or multiple channels with same encoder):
 * - Uses CAP6 (LATCH0) to trigger position commands
 * - SYNC routed to LATCH0_IN0 via Time Sync Event Router
 * - All enabled channels share the same CAP event
 *
 * \b Load Share Mode (Multi-channel with different encoders):
 * - \b Channel 0: Uses CAP6 to trigger position commands
 *   - SYNC routed to LATCH0_IN0 via Time Sync Event Router
 * - \b Channel 2: Uses CAP7 to trigger position commands
 *   - SYNC routed to LATCH1_IN0 via Time Sync Event Router
 * - \b Channel 1: Uses GPIO Mux for  trigger
 *   - Requires external GPIO connection to IEP CAP input pin
 *   - GPIO is routed to IEP CAP_IN via GPIOMUX_INTROUTER0
 *   - Default example uses CAP0 connected to GPIOx
 *   - External signal on GPIO pin triggers position command
 *
 * \par SYNC OUT0 Configuration:
 * - CMP1 event triggers SYNC OUT0 generation after 100 IEP cycles
 * - SYNC OUT0 operates in cyclic generation mode
 * - SYNC OUT0 pulse width is 10 IEP clock cycles
 * - SYNC OUT0 period is configurable via iep_reset_count parameter
 *
 * \param[in]  handle          EnDAT driver handle
 * \param[in]  iep_reset_count SYNC OUT0 period in IEP clock cycles (determines trigger frequency)
 *
 * \return None
 *
 * \note For Channel 1 in load share mode, user must physically connect an external
 *       GPIO pin to the configured IEP CAP input pin on the hardware.
 */
void endat_config_iep_cap_for_sync(Endat_Handle handle, uint64_t iep_reset_count)
{
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint32_t reg_value;
    uint8_t event_num;

    /* Configure IEP sync out0*/

    /*Configure IEP CMP1 to start sync, after 100 cycles*/
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG0, IEP_CMP1_START_DELAY);
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG1, 0);

    /*Enable cmp1 event*/
    /* Read the current register value */
    event_num = IEP_CMP_EVENT_FOR_SYNC0; /* CMP1 event */
    reg_value = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Set the CMP_EN bit (OR with the new value) */
    reg_value |= ((uint32_t)1U << event_num) << 1;
    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg_value);

    /*Set sync ctrl register: SYNC OUT0 cyclic generation , SYNC OUT0 enable*/
    reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    reg_value |= (IEP_SYNC_CTRL_SYNC01_EN_MASK | IEP_SYNC_CTRL_SYNC0_EN_MASK); /*SYNC OUT0 enable*/
    reg_value |= IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK; /*SYNC OUT0 cyclic generation */

    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

    /*Set SYNC OUT0 high pulse time to 10 iep clock cycles  */
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_PWIDTH_REG, IEP_SYNC0_PULSE_WIDTH);
    /*Set SYNC OUT0 period*/
    HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC0_PERIOD_REG, iep_reset_count);

#if defined(SOC_AM243X)
    /* Time sync router for IEP sync to latch */
#if CONFIG_ENDAT0_LOAD_SHARE_MODE == 1

    if(handle->pru_cfg.iep_instance == 0)
    {
        if(handle->pruicss_xchg->config[0].channel)
        {
#if CONFIG_ENDAT0_PRUICSSx == 1
            /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN29);
#else
            /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN25);
#endif
        }
        if(handle->pruicss_xchg->config[2].channel)
        {
#if CONFIG_ENDAT0_PRUICSSx == 1
            /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT13_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN29);
#else
            /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH1_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT9_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN25);
#endif
        }
    }
    else
    {
        if(handle->pruicss_xchg->config[0].channel)
        {
#if CONFIG_ENDAT0_PRUICSSx == 1
            /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN31);
#else
            /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN27);
#endif
        }
        if(handle->pruicss_xchg->config[2].channel)
        {
#if CONFIG_ENDAT0_PRUICSSx == 1
            /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT15_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN31);
#else
            /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH1_IN0 */
            HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT11_OFFSET),
                        TIMESYNC_EVENT_ROUTER_IN27);
#endif
        }
    }

   if(handle->pruicss_xchg->config[1].channel)
    {
        /* Route GPIO to IEP CAP0 input*/
        if(handle->pru_cfg.iep_instance == 0)
        {
            /* Connect GPIO to IEP0 CAP_IN */
            HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + GPIOMUX_INTROUTER0_IEP0_CAP_OFFSET),
                        GPIOMUX_INTROUTER0_CAP_GPIO_IN);
        }
        else
        {
            /* Connect GPIO to IEP1 CAP_IN */
            HW_WR_REG32((CSL_MAIN_GPIOMUX_INTROUTER0_CFG_BASE + GPIOMUX_INTROUTER0_IEP1_CAP_OFFSET),
                        GPIOMUX_INTROUTER0_CAP_GPIO_IN);
        }
    }
#else
#if CONFIG_ENDAT0_PRUICSSx == 1
    if(handle->pru_cfg.iep_instance == 0)
    {
        /* ICSSG1: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
        HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT12_OFFSET),
                    TIMESYNC_EVENT_ROUTER_IN29);
    }
    else
    {
        /* ICSSG1: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
        HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT14_OFFSET),
                    TIMESYNC_EVENT_ROUTER_IN31);
    }
#else
    if(handle->pru_cfg.iep_instance == 0)
    {
        /* ICSSG0: Connect IEP0 SYNC OUT0 output to LATCH0_IN0 */
        HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT8_OFFSET),
                    TIMESYNC_EVENT_ROUTER_IN25);
    }
    else
    {
        /* ICSSG0: Connect IEP1 SYNC OUT0 output to LATCH0_IN0 */
        HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT10_OFFSET),
                    TIMESYNC_EVENT_ROUTER_IN27);
    }
#endif /* CONFIG_ENDAT0_PRUICSSx */
#endif /* CONFIG_ENDAT0_LOAD_SHARE_MODE */
#endif
}
void endat_config_iep_cap_events(struct endat_periodic_interface *endat_periodic_interface)
{
    Endat_Handle handle = endat_periodic_interface->endat_handle;
    uint8_t event_num;
    /* Configure the capture event */
#if (CONFIG_ENDAT0_LOAD_SHARE_MODE == 1)
    {
        if(handle->pruicss_xchg->config[0].channel)
        {
            event_num = CONFIG_ENDAT0_CH0_IEP_CAP_EVENT_NUM;
            endat_config_iep_cap_event(handle, 0, event_num);
        }
        if(handle->pruicss_xchg->config[1].channel)
        {
            event_num = CONFIG_ENDAT0_CH1_IEP_CAP_EVENT_NUM;
            handle->current_channel = 1;
            endat_config_iep_cap_event(handle, 1, event_num);
        }

        if(handle->pruicss_xchg->config[2].channel)
        {
            event_num = CONFIG_ENDAT0_CH2_IEP_CAP_EVENT_NUM;
            endat_config_iep_cap_event(handle, 2, event_num);
        }
    }
#else
    {
        event_num = CONFIG_ENDAT0_IEP_CAP_EVENT_NUM;
        endat_config_iep_cap_event(handle, handle->current_channel, event_num);
    }
#endif

}
void endat_config_iep_cmp_events(struct endat_periodic_interface *endat_periodic_interface)
{
    Endat_Handle handle = endat_periodic_interface->endat_handle;
    uint8_t event_num;
    uint64_t trigger_count;
    /* Clear all event & configure */
#if (CONFIG_ENDAT0_LOAD_SHARE_MODE == 1)
    {

        if(handle->pruicss_xchg->config[0].channel)
        {
            event_num = CONFIG_ENDAT0_CH0_IEP_CMP_EVENT_NUM;
            trigger_count = endat_periodic_interface->ch_trigger_count[0];

            endat_config_iep_cmp_event(handle, 0, trigger_count, event_num);
        }
        if(handle->pruicss_xchg->config[1].channel)
        {
            event_num = CONFIG_ENDAT0_CH1_IEP_CMP_EVENT_NUM;
            trigger_count = endat_periodic_interface->ch_trigger_count[1];

            endat_config_iep_cmp_event(handle, 1, trigger_count, event_num);
        }

        if(handle->pruicss_xchg->config[2].channel)
        {
            event_num = CONFIG_ENDAT0_CH2_IEP_CMP_EVENT_NUM;
            trigger_count = endat_periodic_interface->ch_trigger_count[2];

            endat_config_iep_cmp_event(handle, 2, trigger_count, event_num);
        }

    }
#else
    {
        event_num = CONFIG_ENDAT0_IEP_CMP_EVENT_NUM;
        trigger_count = endat_periodic_interface->ch_trigger_count[0];
        endat_config_iep_cmp_event(handle, handle->current_channel, trigger_count, event_num);
    }
#endif
}

void endat_interrupt_config(struct endat_periodic_interface *endat_periodic_interface)
{
    Endat_Handle handle = endat_periodic_interface->endat_handle;
    int32_t status;
    if(handle->pru_cfg.load_share_enable)
    {
        if(handle->pruicss_xchg->config[1].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_PRU_ENDAT_INT_NUM;
            hwiPrms.callback    = &pruEnDatIrqHandler;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoder0HwiObject[1], &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
        if(handle->pruicss_xchg->config[0].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_RTU_ENDAT_INT_NUM;
            hwiPrms.callback    = &rtuEnDatIrqHandler;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoder0HwiObject[0], &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
        if(handle->pruicss_xchg->config[2].channel)
        {
            /* Register & enable ICSSG EnDat PRU FW interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = ICSS_TXPRU_ENDAT_INT_NUM;
            hwiPrms.callback    = &txpruEnDatIrqHandler;
            hwiPrms.args        = 0;
            hwiPrms.isPulse     = FALSE;
            hwiPrms.isFIQ       = FALSE;
            status              = HwiP_construct(&gIcssgEncoder0HwiObject[2], &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

        }
    }
    else
    {
        /* Register & enable ICSSG EnDat PRU FW interrupt */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_PRU_ENDAT_INT_NUM;
        hwiPrms.callback    = &pruEnDatIrqHandler;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEncoder0HwiObject[0], &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);
    }

}

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
void endat1_interrupt_config(struct endat_periodic_interface *endat_periodic_interface)
{
    int32_t status;
    /* Register & enable ICSSG EnDat PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSS_PRU_ENDAT1_INT_NUM;
    hwiPrms.callback    = &pruEnDat1IrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoder1HwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
}
#endif

uint32_t  endat_config_periodic_mode(struct endat_periodic_interface *endat_periodic_interface)
{
    int32_t  status;
    gPruIcssXHandle = endat_periodic_interface->endat_handle->pru_cfg.pruicss_handle;
    gpruicss_iep = endat_periodic_interface->endat_handle->pru_cfg.iep_base_addr;
    Endat_Handle handle = endat_periodic_interface->endat_handle;
    uint64_t iep_count;
    /* configure IEP */
    /* configure IEP events */
    if (endat_periodic_interface->is_cap_mode)
    {
        endat_config_iep_cap_events(endat_periodic_interface);
        iep_count = endat_periodic_interface->iep_sync0_period;
    }
    else
    {
        endat_config_iep_cmp_events(endat_periodic_interface);
        iep_count = endat_periodic_interface->iep_reset_count;
    }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(handle->instance_index == 1)
    {
        if (endat_periodic_interface->is_cap_mode)
        {
#if defined(SOC_AM243X)
           /* Configure IEP for sync and route it to IEP latch*/
            endat_config_iep_cap_for_sync(handle, iep_count);
#endif
        }
        else
        {
            endat_enable_iep_reset_on_cmp0(handle, iep_count);
        }
        endat_enable_iep_counter(handle);
    }
#else
    if (endat_periodic_interface->is_cap_mode)
    {
        endat_config_iep_cap_for_sync(handle, iep_count);
    }
    else
    {
        endat_enable_iep_reset_on_cmp0(handle, iep_count);
    }
    endat_enable_iep_counter(handle);
#endif
    /* Initialize ICSS INTC */
#if (CONFIG_ENDAT0_PRUICSSx == 1)
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss1_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
#else
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
    if (status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
#endif
    /* config Interrupt */
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(handle->instance_index == 1)
    {
        endat1_interrupt_config(endat_periodic_interface);
    }
    else
    {
        endat_interrupt_config(endat_periodic_interface);
    }
#else
    endat_interrupt_config(endat_periodic_interface);
#endif
    return SystemP_SUCCESS;

}

void endat_stop_periodic_continuous_mode(struct endat_periodic_interface *endat_periodic_interface)
{
    Endat_Handle handle = endat_periodic_interface->endat_handle;
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint8_t event_num;
    uint32_t reg_value;
    endat_disable_iep_counter(handle);
    /*disable interrupt */
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    HwiP_destruct(&gIcssgEncoder1HwiObject);
#endif
    /*disable cmp event */
#if CONFIG_ENDAT0_LOAD_SHARE_MODE == 1
    {
        if(handle->pruicss_xchg->config[0].channel)
        {
            HwiP_destruct(&gIcssgEncoder0HwiObject[0]);
            if(endat_periodic_interface->is_cap_mode)
            {
                event_num = CONFIG_ENDAT0_CH0_IEP_CAP_EVENT_NUM;
                endat_disable_iep_cap_event(handle, event_num);
            }
            else
            {
                event_num = CONFIG_ENDAT0_CH0_IEP_CMP_EVENT_NUM;
                endat_disable_iep_cmp_event(handle, event_num);
            }
        }
        if(handle->pruicss_xchg->config[1].channel)
        {
            HwiP_destruct(&gIcssgEncoder0HwiObject[1]);
            if(endat_periodic_interface->is_cap_mode)
            {
                event_num = CONFIG_ENDAT0_CH1_IEP_CAP_EVENT_NUM;
                endat_disable_iep_cap_event(handle, event_num);
            }
            else
            {
                event_num = CONFIG_ENDAT0_CH1_IEP_CMP_EVENT_NUM;
                endat_disable_iep_cmp_event(handle, event_num);
            }
        }
        if(handle->pruicss_xchg->config[2].channel)
        {
            HwiP_destruct(&gIcssgEncoder0HwiObject[2]);
            if(endat_periodic_interface->is_cap_mode)
            {
                event_num = CONFIG_ENDAT0_CH2_IEP_CAP_EVENT_NUM;
                endat_disable_iep_cap_event(handle, event_num);
            }
            else
            {
                event_num = CONFIG_ENDAT0_CH2_IEP_CMP_EVENT_NUM;
                endat_disable_iep_cmp_event(handle, event_num);
            }
        }

    }
#else
    {
        if(endat_periodic_interface->is_cap_mode)
        {
            event_num = CONFIG_ENDAT0_IEP_CAP_EVENT_NUM;
            endat_disable_iep_cap_event(handle, event_num);
        }
        else
        {
            event_num = CONFIG_ENDAT0_IEP_CMP_EVENT_NUM;
            endat_disable_iep_cmp_event(handle, event_num);
        }

        HwiP_destruct(&gIcssgEncoder0HwiObject[0]);
    }
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(endat_periodic_interface->is_cap_mode && handle->instance_index == 1)
#else
    if(endat_periodic_interface->is_cap_mode)
#endif
    {
#if defined(SOC_AM243X)
        /* disable sync event */
        reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
        reg_value &= ~(IEP_SYNC_CTRL_SYNC01_EN_MASK | IEP_SYNC_CTRL_SYNC0_EN_MASK); /*SYNC OUT0 disable*/
        reg_value &= ~IEP_SYNC_CTRL_SYNC0_CYCLIC_EN_MASK; /*SYNC OUT0 cyclic disable */

        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_value);

        /*disbale cmp1 event*/
        event_num = 1; /* CMP1 event, configured for sync out0  */
        endat_disable_iep_cmp_event(handle, event_num);
#endif

    }
    else
    {
        /*disable cmp0 event*/
        event_num = IEP_CMP_EVENT_FOR_IEP_RESET; /* CMP0 event, configured for iep reset */
        endat_disable_iep_cmp_event(handle, event_num);
        reg_value = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
        reg_value &= ~(1 << IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* CMP0 reset counter enable bit at position 0 */
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, reg_value);
    }

}

/* PRU EnDat FW IRQ handler */
void pruEnDatIrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt0++;
#if CONFIG_ENDAT0_LOAD_SHARE_MODE == 1
    /* Clear interrupt at source for channel 1*/
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT1);
#else
    /* Clear interrupt at source for channel 0*/
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT0);
#endif
}
/* PRU EnDat FW IRQ handler */
void rtuEnDatIrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt1++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT0);
}
/* PRU EnDat FW IRQ handler */
void txpruEnDatIrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt2++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT2);
}

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
/* PRU EnDat FW IRQ handler */
void pruEnDat1IrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDat1IrqCnt0++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT1_EVT);
}
#endif
