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

/**
 *  \file   app_sdfm.c
 *
 *  \brief  SDFM application file for all current sense examples.
 *
 *  \details
 *  Architecture:
 *  - 9 independent interrupt handlers (sdfmIrqHandlerChannel0-8), one per channel
 *  - 9 separate sample buffers and index counters for independent timing
 *  - 9 HWI objects for flexible interrupt registration
 *  - Conditional compilation eliminates unused code at build time
 *
 *  Supported configurations:
 *  - 3-channel or 9-channel operation
 *  - Single PRU mode (channels 0-8 on PRU core)
 *  - Load-share multi-PRU mode (RTU PRU: CH0-2, PRU: CH3-5, TX PRU: CH6-8)
 *  - Snoop/trigger mode (common interrupt per PRU core)
 *  - Continuous mode (individual interrupt per channel)
 *  - Phase compensation via IEP SYNC1 delay
 *  - EPWM synchronization 
 *
 *  Key Functions:
 *  - appSdfmInit()      : Initialize ICSSG, configure parameters, register interrupts
 *  - sdfmMain()     : Main entry point with EPWM init, SDFM init, and main loop
 *  - sdfmIrqHandlerChannelX() : Nine independent handlers for data collection
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
 *  - PRU firmware triggers individual events for CH7 and CH8, but both map to same host interrupt
 *  - Current implementation uses one common handler (sdfmIrqHandlerChannel7) for both CH7-8
 *
 *  Note: Individual interrupts for CH7/CH8 can be implemented by remapping unused host interrupts via SysConfig.
 *  For example, if channels 0-2 are unused, their host interrupts can be reassigned to CH7 and CH8.
 *  - This requires SysConfig changes to remap PRU event outputs to different
 *    host interrupt channels
 *
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

#define PRUICSS_G_MUX_EN    ( 0x1 ) /* ICSSG_SA_MX_REG:G_MUX_EN */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* IRQ handlers for all 9 channels */
void sdfmIrqHandlerChannel0(void *handle);
void sdfmIrqHandlerChannel1(void *handle);
void sdfmIrqHandlerChannel2(void *handle);
void sdfmIrqHandlerChannel3(void *handle);
void sdfmIrqHandlerChannel4(void *handle);
void sdfmIrqHandlerChannel5(void *handle);
void sdfmIrqHandlerChannel6(void *handle);
void sdfmIrqHandlerChannel7(void *handle);

/*Note: This function can be used for CH8 as an individual IRQ function by remapping unused host interrupts via SysConfig.
 *  For example, if channels 0-2 are unused, their host interrupts can be reassigned to CH7 and CH8.
 *  - This requires SysConfig changes to remap PRU event outputs to different host interrupts.
 *  - The code available in the sdfmIrqHandlerChannel7 function is not applicable for channel8, so it should be commented when using the channel8 individual IRQ function.
 * 
/*void sdfmIrqHandlerChannel8(void *handle);*/

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
__attribute__((section(".gSdfmSampleOutput"))) uint32_t gSdfm_sampleOutput[SDFM_NUM_OF_CH_PER_PRU_SLICE];

extern SDFM_Params gSdfmParams;

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
 *  \brief Initialize SDFM
 *
 *  This function initializes the ICSSG, configures SDFM parameters,
 *  registers interrupt handlers, and initializes the SDFM firmware.
 */
void appSdfmInit(void)
{
    int32_t status;
    HwiP_Params sdfmHwiPrms;

    /* Initialize ICSSG */
    status = SDFM_pruIcssInit(CONFIG_PRU_ICSS0, CONFIG_SDFM0_SLICE, PRUICSS_G_MUX_EN,
                      CONFIG_SDFM0_LOAD_SHARE, &gPruIcssHandle);
    if (status != SDFM_ERR_NERR)
    {
        DebugP_log("Error: SDFM_pruIcssInit() fail.\r\n");
        return;
    }

    /* Register & enable interrupt handlers based on enabled channels and load sharing */
    if(gSdfmParams.load_share_enable == 1)
    {
        /* Load share mode - register handlers for all enabled channels */
        if(gSdfmParams.enable_pru_core_mask & (1<< SDFM_RTUPRU_CORE_INDEX))
        {
            /* RTU PRU core enabled - register handlers for channels 0-2 */
            if(gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_trigger_mode == 1)
            {
                DebugP_log("RTU PRU core: Trigger mode enabled.\r\n");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel0;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                DebugP_log("RTU PRU core: Continuous mode enabled.\r\n");
                /* Individual interrupt handler for each channel */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL0))
                {
                    /* Channel 0 enabled */
                    HwiP_Params_init(&sdfmHwiPrms);
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel0;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL1))
                {
                    /* Channel 1 enabled */
                    HwiP_Params_init(&sdfmHwiPrms); 
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH1;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel1;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL1], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL2))
                {
                    /* Channel 2 enabled */
                    HwiP_Params_init(&sdfmHwiPrms); 
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH2;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel2;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL2], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
         
        }
        if(gSdfmParams.enable_pru_core_mask & (1<< SDFM_PRU_CORE_INDEX))
        {
            /* PRU core enabled - register handlers for channels 3-5 */
            if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1)
            {
                DebugP_log("PRU core: Trigger mode enabled.\r\n");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel3;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL3], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                DebugP_log("PRU core: Continuous mode enabled.\r\n");
                /* individual interrupt handler for all three channels */
                /* Channels 3-5 (PRU) */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL3))
                {
                    /* Channel 3 enabled */
                    HwiP_Params_init(&sdfmHwiPrms);
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel3;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL3], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL4))
                {
                    /* Channel 4 enabled */
                    HwiP_Params_init(&sdfmHwiPrms);
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH4;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel4;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL4], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);       
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL5))
                {
                    /* Channel 5 enabled */
                    HwiP_Params_init(&sdfmHwiPrms);
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH5;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel5;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL5], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
        }
        if(gSdfmParams.enable_pru_core_mask & (1<< SDFM_TXPRU_CORE_INDEX))
        {
            /* TX PRU core enabled - register handlers for channels 6-8 */
            if(gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_trigger_mode == 1)
            {
                DebugP_log("TX PRU core: Trigger mode enabled.\r\n");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel6;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL6], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                DebugP_log("TX PRU core: Continuous mode enabled.\r\n");
                /* individual interrupt handler for all three channels */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL6))
                {
                    /* Channel 6 enabled */
                    HwiP_Params_init(&sdfmHwiPrms);
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel6;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL6], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                /*Note: due to host interrupt number limitation for 2 channels7 - 8, common IRQ is used to read samples for channels 7-8*/
                /* Firmware writes trigger the R5 interrupt for individual channel 6-8, to used individual IRQ same as other channels IRQ can be defined and used as needed */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL7) || gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL8))
                {
                    /* Channel 7/8 enabled */
                    HwiP_Params_init(&sdfmHwiPrms);
                    sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH7;
                    sdfmHwiPrms.callback = &sdfmIrqHandlerChannel7;
                    sdfmHwiPrms.args = 0;
                    sdfmHwiPrms.isPulse = FALSE;
                    sdfmHwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL7], &sdfmHwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
        }

    }
    else
    {
        /* Snoop mode enabled */
        if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1 )
        {
            if(gSdfmParams.enable_channel_mask > 7)
            {
                DebugP_log("Error: Only Channel0-2 are supported in snoop mode.\r\n");
                return;
            }
            DebugP_log("PRU core: Snoop mode enabled.\r\n");
        }
        else if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1)
        {
            DebugP_log("PRU core: Trigger mode enabled.\r\n");
            /* common interrupt handler for all nine channels */
            HwiP_Params_init(&sdfmHwiPrms);
            sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
            sdfmHwiPrms.callback = &sdfmIrqHandlerChannel0;
            sdfmHwiPrms.args = 0;
            sdfmHwiPrms.isPulse = FALSE;
            sdfmHwiPrms.isFIQ = FALSE;
            status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfmHwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
        }
        else 
        {
            DebugP_log("PRU core: Continuous mode enabled.\r\n");
            /* individual interrupt handler for all nine channels */
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL0))
            {
                /* Channel 0 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel0;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL0], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL1))
            {
                /* Channel 1 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH1;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel1;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL1], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL2))
            {
                /* Channel 2 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH2;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel2;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL2], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL3))
            {
                /* Channel 3 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel3;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;          
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL3], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL4))
            {
                /* Channel 4 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH4;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel4;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;          
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL4], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);       
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL5))
            {
                /* Channel 5 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH5;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel5;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;          
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL5], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL6))
            {
                /* Channel 6 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel6;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL6], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            /*Note: due to host interrupt number limitation for 2 channels7 - 8, common IRQ is used to read samples for channels 7-8*/
            /* Firmware writes trigger the R5 interrupt for individual channel 6-8, to used individual IRQ same as other channels IRQ can be defined and used as needed */
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL7) || gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL8))
            {
                /* Channel 7/8 enabled */
                HwiP_Params_init(&sdfmHwiPrms);
                sdfmHwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH7;
                sdfmHwiPrms.callback = &sdfmIrqHandlerChannel7;
                sdfmHwiPrms.args = 0;
                sdfmHwiPrms.isPulse = FALSE;
                sdfmHwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObject[SDFM_CHANNEL7], &sdfmHwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL7) && gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL8))
                {
                    DebugP_log("Note: Due to host interrupt number limitation, common IRQ is used to read samples for channels 7 and 8\r\n");                
                }
            }
        }
    }

    /* Open PRUICSS PWM handle if overcurrent (comparator) or fast detect is enabled */
#if (CONFIG_SDFM0_ENABLE_ICSS_PWM)
    gPruIcssPwmHandle = PRUICSS_PWM_open(CONFIG_PRUICSS_PWM0, gPruIcssHandle);
    if (gPruIcssPwmHandle == NULL)
    {
        DebugP_log("Error: PRUICSS_PWM_open() failed.\r\n");
        return;
    }
    gSdfmParams.pwm_handle = gPruIcssPwmHandle;
#else
    gSdfmParams.pwm_handle = NULL;
#endif
    gSdfmParams.pruicss_handle = gPruIcssHandle;

    /* Sample output base address for all channels */
    gSdfmParams.sample_base_addr = (uint32_t)&gSdfm_sampleOutput;

    /* Log connected channels */
    DebugP_log("Connected SDFM channels: ");
    for (int8_t i = 0; i < 9; i++)
    {
        if (gSdfmParams.enable_channel_mask & (1 << i))
        {
            DebugP_log("%d ", i);
        }
    }
    DebugP_log("\r\n");

    /* Initialize SDFM driver and load the SDFM firmware*/
    status = appSdfmPruInit(gPruIcssHandle, gSdfmParams, &gPruIcssSdfmHandle);
    if (status != SDFM_ERR_NERR)
    {
        DebugP_log("Error: appSdfmPruInit() fail.\r\n");
        return;
    }
}

/**
 *  \brief Main SDFM application entry point
 *
 *  \param args [IN] User arguments (unused)
 */
void sdfmMain(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Sample SDFM example running!...\r\n");

    /* Output build time */
    DebugP_log("Build timestamp      : %s %s\r\n", __DATE__, __TIME__);

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Configure EPWM */
    int32_t status = sdfmEpwmInit();
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: sdfmEpwmInit() failed.\r\n");
        return;
    }
    DebugP_log("EPWM Configured!\r\n");
#endif

    /* Configure SDFM */
    appSdfmInit();
    DebugP_log("SDFM Configured!\r\n");

#if (CONFIG_SDFM0_PHASE_DELAY != 0)
    /* Read measured delay and configure IEP SYNC1 for phase compensation */
    float delay = SDFM_getClockPhaseDelay(gPruIcssSdfmHandle, 0);
    /* Convert nanoseconds into IEP cycle count */
    uint32_t iepCount = (delay * (gPruIcssSdfmHandle->pru_config.iep_clock)) / 1000000000;
    /* Config IEP SYNC1 delay based on phase compensation */
    SDFM_configSync1Delay(gPruIcssSdfmHandle, iepCount);
#endif

    /* Main loop */
    while (gRunFlag == 1)
    {
        ;
    }

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Disable and clear interrupts for EPWM */
    sdfmEpwmDeinit();
#endif

    /* Destroy interrupt handlers for all enabled channels */
    for (int8_t i = 0; i < 9; i++)
    {
        if (gSdfmParams.enable_channel_mask & (1 << i))
        {
            /* Skip channel 8 if channel 7 is also enabled (they share the same HWI object) */
            if (i == SDFM_CHANNEL8 && (gSdfmParams.enable_channel_mask & (1 << SDFM_CHANNEL7)))
            {
                continue;
            }
            HwiP_destruct(&gSdfmHwiObject[i]);
        }
    }

    DebugP_log("All tests have passed!!\r\n");

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
void sdfmIrqHandlerChannel0(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL0]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH0);

    if (gSdfmIdxCnt[SDFM_CHANNEL0] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL0] = 0;
    }

    if(gSdfmParams.load_share_enable == 1)
    {
       if(gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_trigger_mode == 1)
       {
           /* Load share mode - Trigger/snoop mode - read all enabled RTU PRU channels (CH0-CH2) */
           for (int8_t i = SDFM_CHANNEL0; i <= SDFM_CHANNEL2; i++)
           {
               if (gSdfmParams.enable_channel_mask & (1 << i))
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
        if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1)
        {
            /* Single PRU mode - Trigger/snoop mode - read all enabled channels (CH0-CH8) */
            for (int8_t i = SDFM_CHANNEL0; i <= SDFM_CHANNEL8; i++)
            {
                if (gSdfmParams.enable_channel_mask & (1 << i))
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
void sdfmIrqHandlerChannel1(void *args)
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
void sdfmIrqHandlerChannel2(void *args)
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
void sdfmIrqHandlerChannel3(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL3]++;

    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH3);

    if (gSdfmIdxCnt[SDFM_CHANNEL3] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL3] = 0;
    }

    if(gSdfmParams.load_share_enable == 1)
    {
        if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDEX].enable_trigger_mode == 1)
        {
            /* Load share mode - Trigger/snoop mode - read all enabled PRU channels (CH3-CH5) */
            for (int8_t i = SDFM_CHANNEL3; i <= SDFM_CHANNEL5; i++)
            {
                if (gSdfmParams.enable_channel_mask & (1 << i))
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
 *  \brief PRU SDFM IRQ handler for Channel 4
 *
 *  Only used continuous mode - reads Channel 4 sample
 */
void sdfmIrqHandlerChannel4(void *args)
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
 *  \brief PRU SDFM IRQ handler for Channel 5
 *
 *  Only used in continuous mode - reads Channel 5 sample
 */
void sdfmIrqHandlerChannel5(void *args)
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
void sdfmIrqHandlerChannel6(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL6]++;

    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH6);

    if (gSdfmIdxCnt[SDFM_CHANNEL6] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL6] = 0;
    }
    if(gSdfmParams.load_share_enable == 1)
    {
       if(gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_trigger_mode == 1)
       {
           /* Load share mode - Trigger/snoop mode - read all enabled TX PRU channels (CH6-CH8) */
           for (int8_t i = SDFM_CHANNEL6; i <= SDFM_CHANNEL8; i++)
           {
               if (gSdfmParams.enable_channel_mask & (1 << i))
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
void sdfmIrqHandlerChannel7(void *args)
{
    gSdfmIrqCnt[SDFM_CHANNEL7]++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH7);

    if(gSdfmParams.enable_channel_mask & (1 << SDFM_CHANNEL8))
    {
        PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH8);
    }

    if (gSdfmIdxCnt[SDFM_CHANNEL7] >= MAX_SAMPLES)
    {
        gSdfmIdxCnt[SDFM_CHANNEL7] = 0;
    }
    gSdfmChSamples[SDFM_CHANNEL7][gSdfmIdxCnt[SDFM_CHANNEL7]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL7);
    gSdfmIdxCnt[SDFM_CHANNEL7]++;

    /* Channel 8 - only process if enabled */
    if(gSdfmParams.enable_channel_mask & (1 << SDFM_CHANNEL8))
    {
        if (gSdfmIdxCnt[SDFM_CHANNEL8] >= MAX_SAMPLES)
        {
            gSdfmIdxCnt[SDFM_CHANNEL8] = 0;
        }
        gSdfmChSamples[SDFM_CHANNEL8][gSdfmIdxCnt[SDFM_CHANNEL8]] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL8);
        gSdfmIdxCnt[SDFM_CHANNEL8]++;
    }
}
/**
 *  \brief PRU SDFM IRQ handler for Channel 8
 *
 *  Note: This handler exists for completeness but is not currently used
 *  because Channel 7 and Channel 8 share the same host interrupt (HOST_INTR_PEND_7)
 *  and are handled together by sdfmIrqHandlerChannel7.
 */
/*void sdfmIrqHandlerChannel8(void *args)
{
    gSdfmIrqCntChannel8++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH8);

    if (sdfmIdxCntChannel8 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel8 = 0;
    }

    sdfm_ch_samples[SDFM_CHANNEL8][sdfmIdxCntChannel8] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL8);
    sdfmIdxCntChannel8++;
}*/
