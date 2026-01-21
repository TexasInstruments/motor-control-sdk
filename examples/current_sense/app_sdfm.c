/*
 *  Copyright (C) 2023-2026 Texas Instruments Incorporated
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
 *  \file   app_sdfm.c
 *
 *  \brief  SDFM application file for all current sense examples.
 *
 *  \details
 *  Architecture:
 *  - 9 independent interrupt handlers (SDFM_irqHandlerCh0-8), one per channel
 *  - 9 separate sample buffers and index counters for independent timing
 *  - 9 HWI objects for flexible interrupt registration
 *  - Conditional compilation eliminates unused code at build time
 *
 *  Supported Configurations:
 *  - 3-channel or 9-channel operation
 *  - Single PRU mode (channels 0-8 on PRU core)
 *  - Load-share multi-PRU mode (RTU PRU: CH0-2, PRU: CH3-5, TX PRU: CH6-8)
 *  - Snoop/trigger mode (common interrupt per PRU core)
 *  - Continuous mode (individual interrupt per channel)
 *  - Phase compensation via IEP SYNC1 delay
 *  - EPWM synchronization for synchronized sampling
 *
 *  Key Functions:
 *  - sdfmConfigIrq()         : Register interrupt handlers based on operation mode
 *  - sdfmMain()              : Main entry point with EPWM init, SDFM init, and main loop
 *  - sdfmIrqHandlerChX()     : Nine independent handlers for data collection
 *
 *  Channel-to-PRU Core Mapping (Load-Share Mode):
 *  - Channels 0-2 : RTU PRU (HOST_INTR_PEND_0, 1, 2)
 *  - Channels 3-5 : PRU     (HOST_INTR_PEND_3, 4, 5)
 *  - Channels 6-8 : TX PRU  (HOST_INTR_PEND_6, 7 shared for 2 channels)
 *
 *  Host Interrupt Limitation:
 *  - Only 8 host interrupts are available for PRU-ICSS (HOST_INTR_PEND_0-7)
 *  - Channels 0-6 have individual R5F interrupts (one per channel)
 *  - Channels 7-8 share one common R5F host interrupt (HOST_INTR_PEND_7)
 *  - PRU firmware triggers individual events for CH7 and CH8
 *  - Current implementation uses one common handler (SDFM_irqHandlerCh7) for both CH7-8
 *
 *  Clock Source Configuration:
 *  - Three clock source options: PRU IEP, PRU ECAP, or PRU GPIO1
 *  - Default configuration: 20 MHz SDFM clock from 300 MHz PRU clock
 *  - IEP: Divider = 15 (Period: 15 cycles, High pulse: 7 cycles)
 *  - ECAP: Divider = 0x0F (300 MHz / 15 = 20 MHz)
 *  - GPIO1: DIV0 = 0x1C, DIV1 = 0x00 (cascaded divisors: 15 × 1 = 15)
 *  - Clock divider values must be updated when changing SDFM clock frequency
 *
 *  Phase Compensation:
 *  - Supported only for channel 0 in current firmware
 *  - Uses IEP SYNC0 and SYNC1 for phase delay compensation
 *  - Phase compensation should be configured after enabling SDFM firmware
 *
 *  Firmware Limitations:
 *  - Single PRU mode with snoop: Only channels 0-2 are supported
 *  - Phase delay measurement: Only supported for channel 0
 *
 *  \note Individual interrupts for CH7/CH8 can be implemented by remapping unused host interrupts
 *        via SysConfig. For example, if channels 0-2 are unused, their host interrupts can be
 *        reassigned to CH7 and CH8. This requires SysConfig changes to remap PRU event outputs
 *        to different host interrupt channels.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "sdfm_epwm.h"
#include "sdfm_example.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* R5F interrupt numbers for ICSSG SDFM - Continuous CH0-CH8 */
#define ICSSG_SDFM_HOST_INTR_NUM_CH0              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#define ICSSG_SDFM_HOST_INTR_NUM_CH1              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1)
#define ICSSG_SDFM_HOST_INTR_NUM_CH2              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2)
#define ICSSG_SDFM_HOST_INTR_NUM_CH3              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3)
#define ICSSG_SDFM_HOST_INTR_NUM_CH4              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_4)
#define ICSSG_SDFM_HOST_INTR_NUM_CH5              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_5)
#define ICSSG_SDFM_HOST_INTR_NUM_CH6              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_6)
#define ICSSG_SDFM_HOST_INTR_NUM_CH7              (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_7)

/* Sample buffer size */
#define MAX_SAMPLES                         (128)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* IRQ handlers for all 9 channels */
void sdfmIrqHandlerCh0(void *args);
void sdfmIrqHandlerCh1(void *args);
void sdfmIrqHandlerCh2(void *args);
void sdfmIrqHandlerCh3(void *args);
void sdfmIrqHandlerCh4(void *args);
void sdfmIrqHandlerCh5(void *args);
void sdfmIrqHandlerCh6(void *args);
void sdfmIrqHandlerCh7(void *args);

/*
 * Note: This function can be used for CH8 as an individual IRQ handler by remapping
 * unused host interrupts via SysConfig. For example, if channels 0-2 are unused,
 * their host interrupts can be reassigned to CH7 and CH8.
 * - Requires SysConfig changes to remap PRU event outputs to different host interrupts
 * - When using this as the CH8 IRQ handler, remove CH8-specific code from SDFM_irqHandlerCh7
 */
#ifdef SDFM_CHANNEL8_IRQ_HANDLER_USED
void sdfmIrqHandlerCh8(void *args);
#endif

/* Local function declarations */
static void sdfmDisplayModeInfo(SDFM_Handle handle);
static void sdfmConfigIrq(SDFM_Handle handle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* HWI objects  */
static HwiP_Object gSdfmHwiObject[SDFM_NUM_OF_CH_PER_PRU_SLICE];

/* Handles */
PRUICSS_Handle gPruIcssHandle = NULL;
PRUICSS_PWM_Handle gPruIcssPwmHandle = NULL;
SDFM_Handle gPruIcssSdfmHandle = NULL;

/* SDFM output samples, written by PRU cores */
__attribute__((section(".gSdfmSampleOutput"))) uint32_t gSdfmSampleOutput[SDFM_NUM_OF_CH_PER_PRU_SLICE];

/* Flag for continuing to execute test */
volatile uint8_t gRunFlag = 1;

/* ICSS SDFM Output samples - 9 channels */
uint32_t gSdfmChSamples[SDFM_NUM_OF_CH_PER_PRU_SLICE][MAX_SAMPLES] = {0};

/* Sample indices for each channel */
uint32_t gSdfmIdxCnt[SDFM_NUM_OF_CH_PER_PRU_SLICE] = {0};

/* IRQ counters for debugging - one per channel */
volatile uint32_t gSdfmIrqCnt[SDFM_NUM_OF_CH_PER_PRU_SLICE] = {0};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Display SDFM configuration and operation mode information
 *
 *  \details Displays comprehensive information about SDFM configuration including:
 *           - PRU-ICSS instance and slice
 *           - Load share mode vs single PRU mode
 *           - Operation mode per PRU core (Trigger/Snoop/Continuous)
 *           - Enabled channels per core
 *           - Channel limitations and notes
 *
 *  \param[in] handle  SDFM handle
 */
static void sdfmDisplayModeInfo(SDFM_Handle handle)
{
    uint8_t ch;
    const SDFM_Attrs *attrs;
    /* Get attrs using accessor function */
    attrs = SDFM_getAttrs(handle);

    if((handle == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n ERROR: NULL handle/attrs");
        return;
    }

    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n PRU-ICSS instance: %u, PRU-ICSS slice: %u", attrs->pruicss_instance, attrs->pruicss_slice);

    /* Display load share mode information */
    if(attrs->load_share_enabled == 1U)
    {
        DebugP_log("\r\n Operation Mode: Load Share Mode (Multi-PRU)");
        DebugP_log("\r\n|------------------------------------------------------------------------------|");

        /* RTU PRU core (Channels 0-2) */
        if(attrs->pru_core_mask & (1U << SDFM_RTUPRU_CORE_INDEX))
        {
            DebugP_log("\r\n RTU PRU core enabled:");
            if(attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_snoop_mode == 1U)
            {
                DebugP_log("\r\n Mode: Snoop mode");
            }
            else if(attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_trigger_mode == 1U)
            {
                DebugP_log("\r\n Mode: Trigger mode");
            }
            else
            {
                DebugP_log("\r\n Mode: Continuous mode");
            }
            DebugP_log("\r\n Channels: ");
            for(ch = SDFM_CHANNEL0; ch <= SDFM_CHANNEL2; ch++)
            {
                if(attrs->channel_mask & (1U << ch))
                {
                    DebugP_log("%d ", ch);
                }
            }
        }

        /* PRU core (Channels 3-5) */
        if(attrs->pru_core_mask & (1U << SDFM_PRU_CORE_INDEX))
        {
            DebugP_log("\r\n PRU core enabled:");
            if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1U)
            {
                DebugP_log("\r\n Mode: Snoop mode");
            }
            else if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1U)
            {
                DebugP_log("\r\n Mode: Trigger mode");
            }
            else
            {
                DebugP_log("\r\n Mode: Continuous mode");
            }
            DebugP_log("\r\n Channels: ");
            for(ch = SDFM_CHANNEL3; ch <= SDFM_CHANNEL5; ch++)
            {
                if(attrs->channel_mask & (1U << ch))
                {
                    DebugP_log("%d ", ch);
                }
            }
        }

        /* TX PRU core (Channels 6-8) */
        if(attrs->pru_core_mask & (1U << SDFM_TXPRU_CORE_INDEX))
        {
            DebugP_log("\r\n TX PRU core enabled:");
            if(attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_snoop_mode == 1U)
            {
                DebugP_log("\r\n Mode: Snoop mode");
            }
            else if(attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_trigger_mode == 1U)
            {
                DebugP_log("\r\n Mode: Trigger mode");
            }
            else
            {
                DebugP_log("\r\n Mode: Continuous mode");
            }
            DebugP_log("\r\n Channels: ");
            for(ch = SDFM_CHANNEL6; ch <= SDFM_CHANNEL8; ch++)
            {
                if(attrs->channel_mask & (1U << ch))
                {
                    DebugP_log("%d ", ch);
                }
            }

        }
    }
    else
    {
        /* Single PRU mode */
        DebugP_log("\r\n Operation Mode: Single PRU Mode");
        DebugP_log("\r\n|------------------------------------------------------------------------------|");

        if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1U)
        {
            DebugP_log("\r\n PRU core mode: Snoop mode");
            if(attrs->channel_mask > 7U)
            {
                /* Current firmware only supports channels 0-2 */
                DebugP_log("\r\n WARNING: Only Channels 0-2 are supported in snoop mode");
            }
        }
        else if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1U)
        {
            DebugP_log("\r\n PRU core mode: Trigger mode");
        }
        else
        {
            DebugP_log("\r\n PRU core mode: Continuous mode");
        }

        DebugP_log("\r\n Enabled channels: ");
        for(ch = SDFM_CHANNEL0; ch <= SDFM_CHANNEL8; ch++)
        {
            if(attrs->channel_mask & (1U << ch))
            {
                DebugP_log("%d ", ch);
            }
        }

    }

    DebugP_log("\r\n|------------------------------------------------------------------------------|");
}

/**
 *  \brief Configure SDFM interrupts based on operation mode
 *
 *  \details Registers interrupt handlers for enabled SDFM channels based on:
 *           - Load share mode vs single PRU mode
 *           - Trigger/snoop mode vs continuous mode
 *           - Channel enable mask from configuration
 *
 *  \param[in] handle  SDFM handle
 */
static void sdfmConfigIrq(SDFM_Handle handle)
{
    int32_t status;
    HwiP_Params sdfm_hwi_prms;
    const SDFM_Attrs *attrs;

    /* Get attrs using accessor function */
    attrs = SDFM_getAttrs(handle);
    if((handle == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n ERROR: NULL handle/attrs");
        return;
    }

    /* Register & enable interrupt handlers based on enabled channels and load sharing */
    if(attrs->load_share_enabled == 1U)
    {
        /* Load share mode - register handlers for all enabled channels */
        if(attrs->pru_core_mask & (1U << SDFM_RTUPRU_CORE_INDEX))
        {
            /* RTU PRU core enabled - register handlers for channels 0-2 */
            if(attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_trigger_mode == 1U)
            {
                /* Common interrupt handler for all three RTU PRU channels (trigger/snoop mode) */
                DebugP_log("\r\nRTU PRU core: Common interrupt used for trigger/snoop mode.");
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh0;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                /* Individual interrupt handler for each channel (continuous mode) */
                DebugP_log("\r\nRTU PRU core: Individual interrupts used for continuous mode.");
                if(attrs->channel_mask & (1<<SDFM_CHANNEL0))
                {
                    /* Channel 0 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh0;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(attrs->channel_mask & (1<<SDFM_CHANNEL1))
                {
                    /* Channel 1 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH1;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh1;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL1], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(attrs->channel_mask & (1<<SDFM_CHANNEL2))
                {
                    /* Channel 2 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH2;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh2;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL2], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }

        }
        if(attrs->pru_core_mask & (1U << SDFM_PRU_CORE_INDEX))
        {
            /* PRU core enabled - register handlers for channels 3-5 */
            if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1U)
            {
                /* Common interrupt handler for all three PRU channels (trigger/snoop mode) */
                DebugP_log("\r\nPRU core: Common interrupt used for trigger/snoop mode.");
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh3;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL3], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                /* Individual interrupt handler for each channel (continuous mode) */
                DebugP_log("\r\nPRU core: Individual interrupts used for continuous mode.");
                if(attrs->channel_mask & (1<<SDFM_CHANNEL3))
                {
                    /* Channel 3 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh3;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL3], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(attrs->channel_mask & (1<<SDFM_CHANNEL4))
                {
                    /* Channel 4 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH4;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh4;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL4], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(attrs->channel_mask & (1<<SDFM_CHANNEL5))
                {
                    /* Channel 5 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH5;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh5;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL5], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
        }
        if(attrs->pru_core_mask & (1U << SDFM_TXPRU_CORE_INDEX))
        {
            /* TX PRU core enabled - register handlers for channels 6-8 */
            if(attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_trigger_mode == 1U)
            {
                /* Common interrupt handler for all three TX PRU channels (trigger/snoop mode) */
                DebugP_log("\r\nTX PRU core: Common interrupt used for trigger/snoop mode.");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh6;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL6], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                /* Individual interrupt handler for each channel (continuous mode) */
                DebugP_log("\r\nTX PRU core: Individual interrupts used for continuous mode.");
                if(attrs->channel_mask & (1<<SDFM_CHANNEL6))
                {
                    /* Channel 6 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh6;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL6], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                /*
                 * Note: Due to host interrupt limitation, channels 7-8 share HOST_INTR_PEND_7.
                 * A common IRQ handler is used for both channels. The PRU firmware generates
                 * individual events for CH7 and CH8.
                 * For individual interrupts, remap unused host interrupts via SysConfig.
                 */
                if(attrs->channel_mask & (1<<SDFM_CHANNEL7) && attrs->channel_mask & (1<<SDFM_CHANNEL8))
                {
                    DebugP_log("\r\nNote: Due to host interrupt number limitation, common IRQ is used to read samples for channels 7 and 8.");
                }
                if(attrs->channel_mask & (1<<SDFM_CHANNEL7) || attrs->channel_mask & (1<<SDFM_CHANNEL8))
                {
                    /* Channel 7/8 enabled */
                    HwiP_Params_init(&sdfm_hwi_prms);
                    sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH7;
                    sdfm_hwi_prms.callback = &sdfmIrqHandlerCh7;
                    sdfm_hwi_prms.args = 0;
                    sdfm_hwi_prms.isPulse = FALSE;
                    sdfm_hwi_prms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL7], &sdfm_hwi_prms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
        }

    }
    else
    {
        /* Single PRU mode - configure interrupts based on operation mode */
        if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1 || attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1)
        {
            /* Common interrupt handler for trigger/snoop mode */
            DebugP_log("\r\nPRU core: Common interrupt used for trigger/snoop mode.");
            HwiP_Params_init(&sdfm_hwi_prms);
            sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
            sdfm_hwi_prms.callback = &sdfmIrqHandlerCh0;
            sdfm_hwi_prms.args = 0;
            sdfm_hwi_prms.isPulse = FALSE;
            sdfm_hwi_prms.isFIQ = FALSE;
            status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfm_hwi_prms);
            DebugP_assert(status == SystemP_SUCCESS);
        }
        else
        {
            /* Individual interrupt handler for each channel (continuous mode) */
            DebugP_log("\r\nPRU core: Individual interrupts used for continuous mode.");
            if(attrs->channel_mask & (1<<SDFM_CHANNEL0))
            {
                /* Channel 0 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh0;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(attrs->channel_mask & (1<<SDFM_CHANNEL1))
            {
                /* Channel 1 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH1;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh1;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL1], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(attrs->channel_mask & (1<<SDFM_CHANNEL2))
            {
                /* Channel 2 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH2;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh2;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL2], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(attrs->channel_mask & (1<<SDFM_CHANNEL3))
            {
                /* Channel 3 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh3;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL3], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(attrs->channel_mask & (1<<SDFM_CHANNEL4))
            {
                /* Channel 4 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH4;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh4;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL4], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(attrs->channel_mask & (1<<SDFM_CHANNEL5))
            {
                /* Channel 5 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH5;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh5;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL5], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(attrs->channel_mask & (1<<SDFM_CHANNEL6))
            {
                /* Channel 6 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh6;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL6], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            /*
             * Note: Due to host interrupt limitation, channels 7-8 share HOST_INTR_PEND_7.
             * A common IRQ handler is used for both channels. The PRU firmware generates
             * individual events for CH7 and CH8, but both map to the same R5F host interrupt.
             * For individual interrupts, remap unused host interrupts via SysConfig.
             */
            if(attrs->channel_mask & (1<<SDFM_CHANNEL7) || attrs->channel_mask & (1<<SDFM_CHANNEL8))
            {
                /* Channel 7/8 enabled */
                HwiP_Params_init(&sdfm_hwi_prms);
                sdfm_hwi_prms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH7;
                sdfm_hwi_prms.callback = &sdfmIrqHandlerCh7;
                sdfm_hwi_prms.args = 0;
                sdfm_hwi_prms.isPulse = FALSE;
                sdfm_hwi_prms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL7], &sdfm_hwi_prms);
                DebugP_assert(status == SystemP_SUCCESS);
                if(attrs->channel_mask & (1<<SDFM_CHANNEL7) && attrs->channel_mask & (1<<SDFM_CHANNEL8))
                {
                    DebugP_log("\r\nNote: Due to host interrupt number limitation, common IRQ is used to read samples for channels 7 and 8.");
                }
            }
        }
    }

}

/**
 *  \brief Main SDFM application entry point
 *
 *  \param args [IN] User arguments (unused)
 */
void sdfmMain(void *args)
{
    int32_t status;
    uint32_t i;
    const SDFM_Attrs *attrs = NULL;
    SDFM_Params sdfm_params;
    SDFM_Priv *priv = NULL;

    /* Clock configuration variables */
    uint32_t high_pulse_width, period_time, sync_start_time;
    uint8_t ecap_divider, div0, div1;

    /* Phase delay compensation variables */
#if (CONFIG_SDFM0_PHASE_DELAY != 0)
    float delay;
    uint32_t iep_count;
#endif

    /* ========================================================================== */
    /* STEP 1: Initialize SoC drivers and board drivers                           */
    /* ========================================================================== */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Sample SDFM example running!...\r\n");

    /* Output build time */
    DebugP_log("Build timestamp      : %s %s\r\n", __DATE__, __TIME__);

    /* ========================================================================== */
    /* STEP 2: Initialize EPWM if SDFM EPWM sync is enabled                       */
    /* ========================================================================== */
#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Configure EPWM */
    status = sdfmEpwmInit();
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: sdfmEpwmInit() failed.\r\n");
        return;
    }
    DebugP_log("EPWM Configured!\r\n");
#endif

    /* ========================================================================== */
    /* STEP 3: Initialize PRU-ICSS                                                */
    /* ========================================================================== */
    status = sdfmPruicssInit(&gPruIcssHandle, CONFIG_PRU_ICSS0, CONFIG_SDFM0_SLICE, CONFIG_SDFM0_LOAD_SHARE);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: sdfmPruicssInit() failed.\r\n");
        return;
    }

    /* ========================================================================== */
    /* STEP 4: Initialize SDFM driver                                            */
    /* ========================================================================== */
    /* Initialize SDFM parameters with sample buffer and PRU handle */
    SDFM_paramsInit(&sdfm_params);
    sdfm_params.sample_base_addr = (uint32_t)&gSdfmSampleOutput;
    sdfm_params.pruicss_handle = gPruIcssHandle;

    /* Open PRU-ICSS PWM handle if overcurrent detection or fast detect is enabled */
#if (CONFIG_SDFM0_ENABLE_ICSS_PWM)
    gPruIcssPwmHandle = PRUICSS_PWM_open(CONFIG_PRUICSS_PWM0, gPruIcssHandle);
    if (gPruIcssPwmHandle == NULL)
    {
        DebugP_log("Error: PRUICSS_PWM_open() failed.\r\n");
        return;
    }
#endif
    sdfm_params.pwm_handle = gPruIcssPwmHandle;

    /* Initialize SDFM driver with configured parameters */
    gPruIcssSdfmHandle = SDFM_init(CONFIG_SDFM0, &sdfm_params);
    /* Get SDFM attrs and priv data for configuration */
    attrs = SDFM_getAttrs(gPruIcssSdfmHandle);
    priv = SDFM_getPriv(gPruIcssSdfmHandle);

    if((gPruIcssSdfmHandle == NULL) || (attrs == NULL) || (priv == NULL))
    {
        DebugP_log("\r\nERROR: SDFM initialization failed\n");
        goto deinit;
    }

    /* Display SDFM configuration and operation mode information */
    sdfmDisplayModeInfo(gPruIcssSdfmHandle);
    DebugP_log("\n\n\n");
    /* ========================================================================== */
    /* STEP 5: Configure SDFM IRQs                                                */
    /* ========================================================================== */
    sdfmConfigIrq(gPruIcssSdfmHandle);
    DebugP_log("\r\nSDFM IRQ configured! \r\n");

    /* ========================================================================== */
    /* STEP 6: Load SDFM firmware into PRU cores                                 */
    /* ========================================================================== */
    status = sdfmLoadFirmware(gPruIcssHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: sdfmLoadFirmware() failed.\r\n");
        goto deinit;
    }

    /* Display SDFM firmware version information */
    i = SDFM_getFirmwareVersion(gPruIcssSdfmHandle);
    DebugP_log("\n\n\n");
    DebugP_log("SDFM firmware version \t: %x.%x.%x (%s)\r\n", (i >> 24) & 0x7FU,
                (i >> 16) & 0xFFU, i & 0xFFFFU, i & (1U << 31) ? "internal" : "release");
    DebugP_log("\n\n\n");
    /* ========================================================================== */
    /* STEP 7: Configure SDFM clock source to generate SDFM clock                 */
    /* ========================================================================== */
    if(attrs->sdfm_clock_source == SDFM_CLOCK_SOURCE_IEP)
    {
        /* Configure IEP to generate SDFM clock (default: 20 MHz from 300 MHz IEP clock)
         * Divider = 300/20 = 15, Period = 15 IEP cycles, High pulse = 7 IEP cycles */
        high_pulse_width = 6; /* 7 - 1 */
        period_time = 14;  /* 15 - 1 */
        sync_start_time = 0; /* Clock generation start time */
        status = SDFM_configIepSyncMode(gPruIcssSdfmHandle, high_pulse_width, period_time, sync_start_time);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log("Error: SDFM_configIepSyncMode() failed.\r\n");
            goto deinit;
        }
        DebugP_log("SDFM clock source: IEP (Internal)\r\n");
        DebugP_log("Divider: %u (Period: %u, High pulse: %u)\r\n", (period_time + 1), (period_time + 1), (high_pulse_width + 1));
        DebugP_log("Note: Default divider configured for 20 MHz. For other frequencies, update divider values accordingly.\r\n");
    }
    else if(attrs->sdfm_clock_source == SDFM_CLOCK_SOURCE_ECAP)
    {
        ecap_divider = 0x0F; /* PRU clock at 300 MHz: SD clock = 300/15 = 20 MHz */
        status = SDFM_configEcap(gPruIcssSdfmHandle, ecap_divider);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log("Error: SDFM_configEcap() failed.\r\n");
            goto deinit;
        }
        DebugP_log("SDFM clock source: PRU ECAP\r\n");
        DebugP_log("Divider: %u (PRU clock 300 MHz / %u = %u MHz)\r\n", ecap_divider, ecap_divider, 300 / ecap_divider);
        DebugP_log("Note: Default divider configured for 20 MHz. For other frequencies, update ecap_divider value accordingly.\r\n");
    }
    else if(attrs->sdfm_clock_source == SDFM_CLOCK_SOURCE_PRU_GPIO1)
    {
        /*
         * Configure GPO1 divisors for 20 MHz SDFM clock at 300 MHz PRU core clock
         * Two cascaded divisors: DIV0 and DIV1
         * Total division: 15 × 1 = 15, resulting in 300/15 = 20 MHz
         * PRU0_GPO_DIV0 = 0x1C (divisor value 15)
         * PRU0_GPO_DIV1 = 0x00 (divisor value 1)
         */
        div0 = 0x1C;
        div1 = 0x0;
        status = SDFM_configClockFromGPO1(gPruIcssSdfmHandle, div0, div1);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log("Error: SDFM_configClockFromGPO1() failed.\r\n");
            goto deinit;
        }
        DebugP_log("SDFM clock source: PRU GPIO1 (GPO1)\r\n");
        DebugP_log("Dividers: DIV0=0x%02X, DIV1=0x%02X (PRU clock 300 MHz / %u = %u MHz)\r\n", div0, div1, ((div0 >> 2) + 1) * ((div1 >> 2) + 1), 300 / (((div0 >> 2) + 1) * ((div1 >> 2) + 1)));
        DebugP_log("Note: Default dividers configured for 20 MHz. For other frequencies, update div0 and div1 values accordingly.\r\n");
    }
    else
    {
        DebugP_log("External clock source is used for SDFM clock.\r\n");
    }
    DebugP_log("Clock frequency: %u MHz\r\n", attrs->sdfm_sampling_freq / 1000000);
    DebugP_log("\n\n\n");
    /* ========================================================================== */
    /* STEP 8: Configure SDFM settings and enable firmware to start sampling     */
    /* ========================================================================== */
    status = sdfmConfigureAndEnable(gPruIcssSdfmHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: sdfmConfigureAndEnable() failed.\r\n");
        goto deinit;
    }

    /*Enable IEP counter if trigger mode is enabled or IEP is used to generate clock*/
    if(attrs->sdfm_clock_source == SDFM_CLOCK_SOURCE_IEP ||
        attrs->pru_core_config[0].enable_trigger_mode == 1||
        attrs->pru_core_config[1].enable_trigger_mode == 1||
        attrs->pru_core_config[2].enable_trigger_mode == 1)
    {
        status = SDFM_enableIep(gPruIcssSdfmHandle);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log("Error: SDFM_enableIep() failed.\r\n");
            goto deinit;
        }
    }


#if (CONFIG_SDFM0_PHASE_DELAY != 0)
    /* Configure IEP SYNC1 delay for phase compensation based on measured clock delay */
    delay = SDFM_getClockPhaseDelay(gPruIcssSdfmHandle, 0);
    /* Convert delay from nanoseconds to IEP cycle count */
    iep_count = (uint32_t)(((uint64_t)delay * (uint64_t)attrs->iep_clk_freq) / 1000000000ULL);
    /* Configure IEP SYNC1 delay for phase compensation */
    status = SDFM_configSync1Delay(gPruIcssSdfmHandle, iep_count);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: SDFM_configSync1Delay() failed.\r\n");
        goto deinit;
    }
#endif

    DebugP_log("\r\n");
    DebugP_log("=======================================================================\r\n");
    DebugP_log("Configuration complete! SDFM firmware enabled and sampling started.\r\n");
    DebugP_log("=======================================================================\r\n");
    DebugP_log("\n\n\n");

    /* Main loop */
    while (gRunFlag == 1)
    {
        ;
    }

deinit:
    /* Deinitialize SDFM driver */
    if (gPruIcssSdfmHandle != NULL)
    {
        SDFM_deinit(gPruIcssSdfmHandle);
    }

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Disable and clear interrupts for EPWM */
    sdfmEpwmDeinit();
#endif

    /* Destroy interrupt handlers for all enabled channels */
    if (attrs != NULL)
    {
        for (int8_t i = 0; i < SDFM_NUM_OF_CH_PER_PRU_SLICE; i++)
        {
            if (attrs->channel_mask & (1U << i))
            {
                /* Skip channel 8 if channel 7 is also enabled (they share the same HWI object) */
                if (i == SDFM_CHANNEL8 && (attrs->channel_mask & (1U << SDFM_CHANNEL7)))
                {
                    continue;
                }
                HwiP_destruct(&gSdfmHwiObject[i]);
            }
        }
    }

    DebugP_log("SDFM application cleanup complete\r\n");

    Board_driversClose();
    Drivers_close();
}

/* ========================================================================== */
/*                       Interrupt Handler Definitions                        */
/* ========================================================================== */

/**
 *  \brief SDFM IRQ handler for Channel 0
 *
 *  In continuous mode: Reads only Channel 0 sample
 *  In trigger/snoop mode: Reads all enabled channels
 */
void sdfmIrqHandlerCh0(void *args)
{
    const SDFM_Attrs *attrs;

    gSdfmIrqCnt[SDFM_CHANNEL0]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH0);

    if (gSdfmIdxCnt[SDFM_CHANNEL0] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL0] = 0U;
    }

    /* Get attrs from handle */
    attrs = SDFM_getAttrs(gPruIcssSdfmHandle);
    if(attrs == NULL)
    {
        return;
    }

    if(attrs->load_share_enabled == 1U)
    {
       if(attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_trigger_mode == 1U)
       {
           /* Load share mode - Trigger/snoop mode - read all enabled RTU PRU channels (CH0-CH2) */
           for (int8_t i = SDFM_CHANNEL0; i <= SDFM_CHANNEL2; i++)
           {
               if (attrs->channel_mask & (1 << i))
               {
                   gSdfmChSamples[i][gSdfmIdxCnt[SDFM_CHANNEL0]] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
               }
           }
       }
       else
       {
           gSdfmChSamples[SDFM_CHANNEL0][gSdfmIdxCnt[SDFM_CHANNEL0]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL0);
       }
    }
    else
    {
        if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1U)
        {
            /* Single PRU mode - Trigger/snoop mode - read all enabled channels (CH0-CH8) */
            for (int8_t i = SDFM_CHANNEL0; i <= SDFM_CHANNEL8; i++)
            {
                if (attrs->channel_mask & (1U << i))
                {
                    gSdfmChSamples[i][gSdfmIdxCnt[SDFM_CHANNEL0]] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
                }
            }
        }
        else
        {
            /* Single PRU mode - Continuous mode - Channel 0 only */
            gSdfmChSamples[SDFM_CHANNEL0][gSdfmIdxCnt[SDFM_CHANNEL0]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL0);
        }
    }

    gSdfmIdxCnt[SDFM_CHANNEL0]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 1
 *
 *  Only used in continuous mode - reads Channel 1 sample
 */
void sdfmIrqHandlerCh1(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL1]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH1);

    if (gSdfmIdxCnt[SDFM_CHANNEL1] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL1] = 0;
    }

    gSdfmChSamples[SDFM_CHANNEL1][gSdfmIdxCnt[SDFM_CHANNEL1]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL1);
    gSdfmIdxCnt[SDFM_CHANNEL1]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 2
 *
 *  Only used in continuous mode - reads Channel 2 sample
 */
void sdfmIrqHandlerCh2(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL2]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH2);

    if (gSdfmIdxCnt[SDFM_CHANNEL2] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL2] = 0;
    }

    gSdfmChSamples[SDFM_CHANNEL2][gSdfmIdxCnt[SDFM_CHANNEL2]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL2);
    gSdfmIdxCnt[SDFM_CHANNEL2]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 3
 *
 *  In continuous mode: Reads only Channel 3 sample
 *  In load-share trigger/snoop mode: Reads all enabled PRU channels (CH3, CH4, CH5)
 */
void sdfmIrqHandlerCh3(void *args)
{
    const SDFM_Attrs *attrs;

    gSdfmIrqCnt[SDFM_CHANNEL3]++;

    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH3);

    if (gSdfmIdxCnt[SDFM_CHANNEL3] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL3] = 0U;
    }

    /* Get attrs from handle */
    attrs = SDFM_getAttrs(gPruIcssSdfmHandle);
    if(attrs == NULL)
    {
        return;
    }

    if(attrs->load_share_enabled == 1U)
    {
        if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1U)
        {
            /* Load share mode - Trigger/snoop mode - read all enabled PRU channels (CH3-CH5) */
            for (int8_t i = SDFM_CHANNEL3; i <= SDFM_CHANNEL5; i++)
            {
                if (attrs->channel_mask & (1U << i))
                {
                    gSdfmChSamples[i][gSdfmIdxCnt[SDFM_CHANNEL3]] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
                }
            }
        }
        else
        {
            /* Load share mode - Continuous mode - Channel 3 only */
            gSdfmChSamples[SDFM_CHANNEL3][gSdfmIdxCnt[SDFM_CHANNEL3]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL3);
        }
    }
    else
    {
        /* Single PRU mode - Channel 3 only */
        gSdfmChSamples[SDFM_CHANNEL3][gSdfmIdxCnt[SDFM_CHANNEL3]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL3);
    }
    gSdfmIdxCnt[SDFM_CHANNEL3]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 4
 *
 *  Only used in continuous mode - reads Channel 4 sample
 */
void sdfmIrqHandlerCh4(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL4]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH4);

    if (gSdfmIdxCnt[SDFM_CHANNEL4] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL4] = 0;
    }

    gSdfmChSamples[SDFM_CHANNEL4][gSdfmIdxCnt[SDFM_CHANNEL4]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL4);
    gSdfmIdxCnt[SDFM_CHANNEL4]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 5
 *
 *  Only used in continuous mode - reads Channel 5 sample
 */
void sdfmIrqHandlerCh5(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL5]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH5);

    if (gSdfmIdxCnt[SDFM_CHANNEL5] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL5] = 0;
    }

    gSdfmChSamples[SDFM_CHANNEL5][gSdfmIdxCnt[SDFM_CHANNEL5]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL5);
    gSdfmIdxCnt[SDFM_CHANNEL5]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 6
 *
 *  In continuous mode: Reads only Channel 6 sample
 *  In load share trigger/snoop mode: Reads all enabled TX PRU channels (CH6, CH7, CH8)
 */
void sdfmIrqHandlerCh6(void *args)
{
    const SDFM_Attrs *attrs;

    gSdfmIrqCnt[SDFM_CHANNEL6]++;

    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH6);

    if (gSdfmIdxCnt[SDFM_CHANNEL6] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL6] = 0U;
    }

    /* Get attrs from handle */
    attrs = SDFM_getAttrs(gPruIcssSdfmHandle);
    if(attrs == NULL)
    {
        return;
    }

    if(attrs->load_share_enabled == 1U)
    {
       if(attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_snoop_mode == 1U || attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_trigger_mode == 1U)
       {
           /* Load share mode - Trigger/snoop mode - read all enabled TX PRU channels (CH6-CH8) */
           for (int8_t i = SDFM_CHANNEL6; i <= SDFM_CHANNEL8; i++)
           {
               if (attrs->channel_mask & (1U << i))
               {
                   gSdfmChSamples[i][gSdfmIdxCnt[SDFM_CHANNEL6]] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
               }
           }
       }
       else
       {
           /* Load share mode - Continuous mode - Channel 6 only */
           gSdfmChSamples[SDFM_CHANNEL6][gSdfmIdxCnt[SDFM_CHANNEL6]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL6);
       }
    }
    else
    {
        /* Single PRU mode - Channel 6 only */
        gSdfmChSamples[SDFM_CHANNEL6][gSdfmIdxCnt[SDFM_CHANNEL6]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL6);
    }
    gSdfmIdxCnt[SDFM_CHANNEL6]++;
}

/**
 *  \brief SDFM IRQ handler for Channel 7
 *
 *  In continuous mode: Reads both Channel 7 and Channel 8 samples
 *  (Due to host interrupt limitation, CH7 and CH8 share HOST_INTR_PEND_7)
 *
 *  Only used in continuous mode
 */
void sdfmIrqHandlerCh7(void *args)
{
    const SDFM_Attrs *attrs;

    gSdfmIrqCnt[SDFM_CHANNEL7]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH7);

    /* Get attrs from handle */
    attrs = SDFM_getAttrs(gPruIcssSdfmHandle);
    if(attrs == NULL)
    {
        return;
    }

    if(attrs->channel_mask & (1U << SDFM_CHANNEL8))
    {
        PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH8);
    }

    if (gSdfmIdxCnt[SDFM_CHANNEL7] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL7] = 0U;
    }
    gSdfmChSamples[SDFM_CHANNEL7][gSdfmIdxCnt[SDFM_CHANNEL7]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL7);
    gSdfmIdxCnt[SDFM_CHANNEL7]++;

    /* Channel 8 - only process if enabled */
    if(attrs->channel_mask & (1U << SDFM_CHANNEL8))
    {
        if (gSdfmIdxCnt[SDFM_CHANNEL8] >= MAX_SAMPLES)
        {
            gSdfmIdxCnt[SDFM_CHANNEL8] = 0U;
        }
        gSdfmChSamples[SDFM_CHANNEL8][gSdfmIdxCnt[SDFM_CHANNEL8]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL8);
        gSdfmIdxCnt[SDFM_CHANNEL8]++;
    }
}
/**
 *  \brief SDFM IRQ handler for Channel 8
 *
 *  Note: This handler exists for completeness but is not currently used
 *  because channels 7 and 8 share the same host interrupt (HOST_INTR_PEND_7)
 *  and are handled together by SDFM_irqHandlerCh7.
 */
#ifdef SDFM_CHANNEL8_IRQ_HANDLER_USED
void sdfmIrqHandlerCh8(void *args)
{
    gSdfmIrqCntChannel8++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH8);

    if (sdfmIdxCntChannel8 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel8 = 0;
    }

    sdfm_ch_samples[SDFM_CHANNEL8][sdfmIdxCntChannel8] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL8);
    sdfmIdxCntChannel8++;
}
#endif
