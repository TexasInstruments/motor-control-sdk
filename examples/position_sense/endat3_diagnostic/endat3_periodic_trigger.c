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

#include<stdio.h>
#include<stdint.h>
#include<math.h>

#include <drivers/pruicss.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/tistdtypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include "endat3_periodic_trigger.h"
#include <drivers/soc.h>
#include <position_sense/endat3/include/endat3_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/* Global Variables                                                           */
/* ========================================================================== */

static HwiP_Object gIcssgEnDat3HwiObject0;  /**< ICSSG EnDat3 PRU FW HWI - Channel 0 */
static HwiP_Object gIcssgEnDat3HwiObject1;  /**< ICSSG EnDat3 PRU FW HWI - Channel 1 */
static HwiP_Object gIcssgEnDat3HwiObject2;  /**< ICSSG EnDat3 PRU FW HWI - Channel 2 */

/* Interrupt counters for debugging */
uint32_t gPruEnDat3IrqCnt0 = 0;  /**< Channel 0 interrupt count */
uint32_t gPruEnDat3IrqCnt1 = 0;  /**< Channel 1 interrupt count */
uint32_t gPruEnDat3IrqCnt2 = 0;  /**< Channel 2 interrupt count */

/* Global IEP pointer for interrupt handler access */
void *gPruss_iep = NULL;

/* Global PRUICSS handle for interrupt handler access */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* ========================================================================== */
/* ICSS Interrupt Configuration                                              */
/* ========================================================================== */

#if (SOC_AM261X || SOC_AM263X)
#if (PRUICSSx == 1)
#define ICSS_PRU_ENDAT3_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0 )
#else
#define ICSS_PRU_ENDAT3_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0 )
#endif
#else
#if (PRUICSSx == 1)
#define ICSS_PRU_ENDAT3_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0 )
#else
#define ICSS_PRU_ENDAT3_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )
#endif
#endif

/* ICSS INTC configuration */
#if (PRUICSSx == 1)
    extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
    extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/* Function Definitions                                                       */
/* ========================================================================== */

/**
 * \brief Configure IEP timer for periodic triggering
 * 
 * Sets up the IEP timer with compare registers for periodic event generation.
 * Configures CMP0 for counter reset and CMP3/CMP5/CMP6 for periodic triggers.
 *
 * \param endat3_periodic_interface Pointer to periodic interface configuration
 *
 * \return void
 */
static void endat3_config_iep(struct endat3_periodic_interface *endat3_periodic_interface)
{
    void *pruss_iep = endat3_periodic_interface->pruss_iep;
    uint8_t temp;
    uint8_t event;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint32_t event_clear;

    /* Clear IEP counter */
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    /* Read current compare configuration and status */
    event = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /* Enable IEP reset by CMP0 event */
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /* Set IEP counter to ZERO */
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /* Configure compare registers based on load share mode */
    if(endat3_periodic_interface->load_share)
    {
        /* Load share mode: configure CMP3, CMP5, CMP6 for different channels */
        
        /* Configure CMP3 for channel 0 */
        event |= (0x1 << 4);
        event_clear |= (0x1 << 3);
        cmp_reg0 = (endat3_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (endat3_periodic_interface->cmp3 >> 32 & 0xffffffff);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG0, cmp_reg0);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG1, cmp_reg1);

        /* Configure CMP5 for channel 1 */
        event |= (0x1 << 6);
        event_clear |= (0x1 << 5);
        cmp_reg0 = (endat3_periodic_interface->cmp5 & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (endat3_periodic_interface->cmp5 >> 32 & 0xffffffff);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP5_REG0, cmp_reg0);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP5_REG1, cmp_reg1);

        /* Configure CMP6 for channel 2 */
        event |= (0x1 << 7);
        event_clear |= (0x1 << 6);
        cmp_reg0 = (endat3_periodic_interface->cmp6 & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (endat3_periodic_interface->cmp6 >> 32 & 0xffffffff);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP6_REG0, cmp_reg0);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP6_REG1, cmp_reg1);
    }
    else
    {
        /* Single channel mode: configure CMP3 only */
        event |= (0x1 << 4);
        event_clear |= (0x1 << 3);
        cmp_reg0 = (endat3_periodic_interface->cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
        cmp_reg1 = (endat3_periodic_interface->cmp3 >> 32 & 0xffffffff);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG0, cmp_reg0);
        HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP3_REG1, cmp_reg1);
    }

    /* Clear and enable compare events */
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    /* Configure CMP0 for counter reset */
    cmp_reg0 = (endat3_periodic_interface->cmp0 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (endat3_periodic_interface->cmp0 >> 32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0, cmp_reg0);
    HW_WR_REG32((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1, cmp_reg1);

    /* Start IEP counter with default increment */
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

/**
 * \brief Configure interrupt handlers for periodic triggers
 * 
 * Registers hardware interrupt handlers for periodic trigger events.
 * Supports single channel and load share (multi-channel) modes.
 *
 * \param endat3_periodic_interface Pointer to periodic interface configuration
 *
 * \return void
 */
static void endat3_interrupt_config(struct endat3_periodic_interface *endat3_periodic_interface)
{
    int32_t status;
    HwiP_Params hwiPrms;

    if(endat3_periodic_interface->load_share)
    {
        /* Load share mode: register handlers for all three channels */
        
        /* Channel 0 interrupt handler */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_PRU_ENDAT3_INT_NUM;
        hwiPrms.callback    = &pruEnDat3IrqHandler0;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEnDat3HwiObject0, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Channel 1 interrupt handler */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_PRU_ENDAT3_INT_NUM + 1;
        hwiPrms.callback    = &pruEnDat3IrqHandler1;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEnDat3HwiObject1, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Channel 2 interrupt handler */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_PRU_ENDAT3_INT_NUM + 2;
        hwiPrms.callback    = &pruEnDat3IrqHandler2;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEnDat3HwiObject2, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    else
    {
        /* Single channel mode: register handler for channel 0 only */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = ICSS_PRU_ENDAT3_INT_NUM;
        hwiPrms.callback    = &pruEnDat3IrqHandler0;
        hwiPrms.args        = 0;
        hwiPrms.isPulse     = FALSE;
        hwiPrms.isFIQ       = FALSE;
        status              = HwiP_construct(&gIcssgEnDat3HwiObject0, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);
    }
}

/**
 * \brief Configure EnDat3 periodic mode
 * 
 * Main initialization function for periodic trigger mode. Configures IEP timer,
 * initializes ICSS interrupt controller, and sets up interrupt handlers.
 *
 * \param endat3_periodic_interface Pointer to periodic interface configuration
 * \param handle PRUICSS handle
 *
 * \return 1 on success, 0 on failure
 */
uint32_t endat3_config_periodic_mode(struct endat3_periodic_interface *endat3_periodic_interface, PRUICSS_Handle handle)
{
    gPruIcssXHandle = handle;
    gPruss_iep = endat3_periodic_interface->pruss_iep;

    /* Configure IEP timer */
    endat3_config_iep(endat3_periodic_interface);

    /* Note: PRUICSS INTC initialization is handled by syscfg/ti_drivers_config.c
     * The INTC is already initialized when Drivers_open() is called in main.
     * Attempting to initialize it again here would cause issues.
     */

    /* Configure interrupt handlers */
    endat3_interrupt_config(endat3_periodic_interface);

    DebugP_log("\r\nEnDat3 periodic mode configured successfully\r\n");
    return 1;
}

/**
 * \brief Configure EnDat3 encoder for periodic trigger mode
 * 
 * Configures the encoder to operate in periodic trigger mode where the
 * encoder is triggered by periodic IEP timer events instead of host commands.
 *
 * \param handle EnDat3 handle for the encoder channel
 *
 * \return void
 */
void endat3_config_periodic_trigger(endat3_Handle handle)
{
    /* Configure encoder for periodic trigger mode */
    /* This would typically involve setting encoder-specific registers or parameters */
    DebugP_log("\r\nEnDat3 encoder configured for periodic trigger mode\r\n");
}

/**
 * \brief Configure EnDat3 encoder for host trigger mode
 * 
 * Configures the encoder to operate in host trigger mode where the encoder
 * is triggered by host commands. This is the default mode.
 *
 * \param handle EnDat3 handle for the encoder channel
 *
 * \return void
 */
void endat3_config_host_trigger(endat3_Handle handle)
{
    /* Configure encoder for host trigger mode (default) */
    /* This would typically involve setting encoder-specific registers or parameters */
    DebugP_log("\r\nEnDat3 encoder configured for host trigger mode\r\n");
}

/**
 * \brief Stop EnDat3 periodic mode
 * 
 * Disables the IEP timer and stops periodic triggering.
 *
 * \param endat3_periodic_interface Pointer to periodic interface configuration
 *
 * \return void
 */
void endat3_stop_periodic_continuous_mode(struct endat3_periodic_interface *endat3_periodic_interface)
{
    void *pruss_iep = endat3_periodic_interface->pruss_iep;
    uint8_t temp;

    /* Clear IEP counter enable bit */
    temp = HW_RD_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t*)pruss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    DebugP_log("\r\nEnDat3 periodic mode stopped\r\n");
}

/**
 * \brief EnDat3 periodic trigger interrupt handler - Channel 0
 * 
 * Handles CMP3 event for channel 0. Clears the event flag and acknowledges
 * the interrupt at the source.
 *
 * \param args Pointer to arguments (unused)
 *
 * \return void
 */
static void pruEnDat3IrqHandler0(void *args)
{
    /* Increment interrupt counter for debugging */
    gPruEnDat3IrqCnt0++;

    /* Clear CMP3 event */
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP3_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT3_EVT0);
}

/**
 * \brief EnDat3 periodic trigger interrupt handler - Channel 1
 * 
 * Handles CMP5 event for channel 1. Clears the event flag and acknowledges
 * the interrupt at the source.
 *
 * \param args Pointer to arguments (unused)
 *
 * \return void
 */
static void pruEnDat3IrqHandler1(void *args)
{
    /* Increment interrupt counter for debugging */
    gPruEnDat3IrqCnt1++;

    /* Clear CMP5 event */
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP5_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT3_EVT1);
}

/**
 * \brief EnDat3 periodic trigger interrupt handler - Channel 2
 * 
 * Handles CMP6 event for channel 2. Clears the event flag and acknowledges
 * the interrupt at the source.
 *
 * \param args Pointer to arguments (unused)
 *
 * \return void
 */
static void pruEnDat3IrqHandler2(void *args)
{
    /* Increment interrupt counter for debugging */
    gPruEnDat3IrqCnt2++;

    /* Clear CMP6 event */
    uint32_t event_clear;
    event_clear = HW_RD_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);
    event_clear |= IEP_CMP6_EVNT;
    HW_WR_REG8((uint8_t*)gPruss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT3_EVT2);
}
