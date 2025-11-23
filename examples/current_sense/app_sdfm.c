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
 *  - initSdfm()      : Initialize ICSSG, configure parameters, register interrupts
 *  - sdfm_main()     : Main entry point with EPWM init, SDFM init, and main loop
 *  - sdfmIrqHandlerChannelX() : Nine independent handlers for data collection
 *
 *  Channel-to-PRU Core Mapping (Load-Share Mode):
 *  - Channels 0-2 : RTU PRU (HOST_INTR_PEND_0, 1, 2)
 *  - Channels 3-5 : PRU     (HOST_INTR_PEND_3, 4, 5)
 *  - Channels 6-8 : TX PRU  (HOST_INTR_PEND_6, 7 shared for 2 channels)
 *
 *  Host Interrupt Limitation:
 *  - Only 8 host interrupts are mapped with PRU interrupts (HOST_INTR_PEND_0-7) 
 *  - Channels 0-7 have individual R5F interrupts (one per channel)
 *  - Channels 7-8 share one common R5F host interrupt (HOST_INTR_PEND_7)
 *  - PRU firmware triggers individual interrupts for CH7, CH8
 *  - Current implementation uses one common handler for CH7-8 due to R5F host
 *
 *  Note: Individual interrupts for CH7 and CH8 can be implemented:
 *  - If other channels (0-6) are not connected, their host interrupts can be
 *    reassigned to CH7 and CH8
 *  - Any available HOST_INTR_PEND (0-7) can be mapped to CH7/CH8
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
void sdfmIrqHandlerChannel8(void *handle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* HWI objects for all 9 channels */
static HwiP_Object gSdfmHwiObjectChannel0;
static HwiP_Object gSdfmHwiObjectChannel1;
static HwiP_Object gSdfmHwiObjectChannel2;
static HwiP_Object gSdfmHwiObjectChannel3;
static HwiP_Object gSdfmHwiObjectChannel4;
static HwiP_Object gSdfmHwiObjectChannel5;
static HwiP_Object gSdfmHwiObjectChannel6;
static HwiP_Object gSdfmHwiObjectChannel7;
static HwiP_Object gSdfmHwiObjectChannel8;

/* Handles */
PRUICSS_Handle gPruIcssHandle;
PRUICSS_PWM_Handle gPruIcssPwmHandle;
SDFM_Handle gPruIcssSdfmHandle;

/* SDFM output samples, written by PRU cores */
__attribute__((section(".gSdfmSampleOutput"))) uint32_t gSdfm_sampleOutput[SDFM_NUM_OF_CH_PER_PRU_SLICE];

extern SDFM_Params gSdfmParams;

/* Flag for continuing to execute test */
volatile Bool gRunFlag = TRUE;

/* ICSS SDFM Output samples - 9 channels */
uint32_t sdfm_ch_samples[9][MAX_SAMPLES] = {0};

/* Sample indices for each channel */
uint32_t sdfmIdxCntChannel0 = 0;
uint32_t sdfmIdxCntChannel1 = 0;
uint32_t sdfmIdxCntChannel2 = 0;
uint32_t sdfmIdxCntChannel3 = 0;
uint32_t sdfmIdxCntChannel4 = 0;
uint32_t sdfmIdxCntChannel5 = 0;
uint32_t sdfmIdxCntChannel6 = 0;
uint32_t sdfmIdxCntChannel7 = 0;
uint32_t sdfmIdxCntChannel8 = 0;

/* IRQ counters for debugging - one per channel */
volatile uint32_t gSdfmIrqCntChannel0 = 0;
volatile uint32_t gSdfmIrqCntChannel1 = 0;
volatile uint32_t gSdfmIrqCntChannel2 = 0;
volatile uint32_t gSdfmIrqCntChannel3 = 0;
volatile uint32_t gSdfmIrqCntChannel4 = 0;
volatile uint32_t gSdfmIrqCntChannel5 = 0;
volatile uint32_t gSdfmIrqCntChannel6 = 0;
volatile uint32_t gSdfmIrqCntChannel7 = 0;
volatile uint32_t gSdfmIrqCntChannel8 = 0;

/* HWI parameters */
HwiP_Params hwiPrms;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Initialize SDFM
 *
 *  This function initializes the ICSSG, configures SDFM parameters,
 *  registers interrupt handlers, and initializes the SDFM firmware.
 */
void initSdfm(void)
{
    int32_t status;

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
        if(gSdfmParams.enable_pru_core_mask & 1<< SDFM_RTUPRU_CORE_INDX)
        {
            /* RTU PRU core enabled - register handlers for channels 0-2 */
            if(gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDX].enable_trigger_mode == 1)
            {
                DebugP_log("RTU PRU core: Trigger mode enabled.\r\n");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                hwiPrms.callback = &sdfmIrqHandlerChannel0;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel0, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                DebugP_log("RTU PRU core: Continuous mode enabled.\r\n");
                /* Individual interrupt handler for each channel */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL0))
                {
                    /* Channel 0 enabled */
                    HwiP_Params_init(&hwiPrms);
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                    hwiPrms.callback = &sdfmIrqHandlerChannel0;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel0, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL1))
                {
                    /* Channel 1 enabled */
                    HwiP_Params_init(&hwiPrms); 
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH1;
                    hwiPrms.callback = &sdfmIrqHandlerChannel1;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel1, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL2))
                {
                    /* Channel 2 enabled */
                    HwiP_Params_init(&hwiPrms); 
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH2;
                    hwiPrms.callback = &sdfmIrqHandlerChannel2;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel2, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
         
        }
        if(gSdfmParams.enable_pru_core_mask & 1<< SDFM_PRU_CORE_INDX)
        {
            /* PRU core enabled - register handlers for channels 3-5 */
            if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_trigger_mode == 1)
            {
                DebugP_log("PRU core: Trigger mode enabled.\r\n");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                hwiPrms.callback = &sdfmIrqHandlerChannel3;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel3, &hwiPrms);
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
                    HwiP_Params_init(&hwiPrms);
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                    hwiPrms.callback = &sdfmIrqHandlerChannel3;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel3, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL4))
                {
                    /* Channel 4 enabled */
                    HwiP_Params_init(&hwiPrms);
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH4;
                    hwiPrms.callback = &sdfmIrqHandlerChannel4;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel4, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);       
                }
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL5))
                {
                    /* Channel 5 enabled */
                    HwiP_Params_init(&hwiPrms);
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH5;
                    hwiPrms.callback = &sdfmIrqHandlerChannel5;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel5, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
        }
        if(gSdfmParams.enable_pru_core_mask & 1<< SDFM_TXPRU_CORE_INDX)
        {
            /* TX PRU core enabled - register handlers for channels 6-8 */
            if(gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDX].enable_trigger_mode == 1)
            {
                DebugP_log("TX PRU core: Trigger mode enabled.\r\n");
                /* common interrupt handler for all three channels */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                hwiPrms.callback = &sdfmIrqHandlerChannel6;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel6, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            else
            {
                DebugP_log("TX PRU core: Continuous mode enabled.\r\n");
                /* individual interrupt handler for all three channels */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL6))
                {
                    /* Channel 6 enabled */
                    HwiP_Params_init(&hwiPrms);
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                    hwiPrms.callback = &sdfmIrqHandlerChannel6;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel6, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                /*Note: due to host interrupt number limitation for 2 channels7 - 8, common IRQ is used to read samples for channels 7-8*/
                /* Firmware writes trigger the R5 interrupt for individual channel 6-8, to used individual IRQ same as other channels IRQ can be defined and used as needed */
                if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL7) || gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL8))
                {
                    /* Channel 7/8 enabled */
                    HwiP_Params_init(&hwiPrms);
                    hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH7;
                    hwiPrms.callback = &sdfmIrqHandlerChannel7;
                    hwiPrms.args = 0;
                    hwiPrms.isPulse = FALSE;
                    hwiPrms.isFIQ = FALSE;
                    status = HwiP_construct(&gSdfmHwiObjectChannel7, &hwiPrms);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }
        }

    }
    else
    {
        /* Snoop mode enabled */
        if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_snoop_mode == 1 )
        {
            if(gSdfmParams.enable_channel_mask > 7)
            {
                DebugP_log("Error: Only Channel0-2 are supported in snoop mode.\r\n");
                return;
            }
            DebugP_log("PRU core: Snoop mode enabled.\r\n");
        }
        else if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_trigger_mode == 1)
        {
            DebugP_log("PRU core: Trigger mode enabled.\r\n");
            /* common interrupt handler for all nine channels */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
            hwiPrms.callback = &sdfmIrqHandlerChannel0;
            hwiPrms.args = 0;
            hwiPrms.isPulse = FALSE;
            hwiPrms.isFIQ = FALSE;
            status = HwiP_construct(&gSdfmHwiObjectChannel0, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
        }
        else 
        {
            DebugP_log("PRU core: Continuous mode enabled.\r\n");
            /* individual interrupt handler for all nine channels */
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL0))
            {
                /* Channel 0 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH0;
                hwiPrms.callback = &sdfmIrqHandlerChannel0;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel0, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL1))
            {
                /* Channel 1 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH1;
                hwiPrms.callback = &sdfmIrqHandlerChannel1;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel1, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL2))
            {
                /* Channel 2 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH2;
                hwiPrms.callback = &sdfmIrqHandlerChannel2;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel2, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL3))
            {
                /* Channel 3 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH3;
                hwiPrms.callback = &sdfmIrqHandlerChannel3;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;          
                status = HwiP_construct(&gSdfmHwiObjectChannel3, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL4))
            {
                /* Channel 4 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH4;
                hwiPrms.callback = &sdfmIrqHandlerChannel4;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;          
                status = HwiP_construct(&gSdfmHwiObjectChannel4, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);       
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL5))
            {
                /* Channel 5 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH5;
                hwiPrms.callback = &sdfmIrqHandlerChannel5;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;          
                status = HwiP_construct(&gSdfmHwiObjectChannel5, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL6))
            {
                /* Channel 6 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH6;
                hwiPrms.callback = &sdfmIrqHandlerChannel6;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel6, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
            /*Note: due to host interrupt number limitation for 2 channels7 - 8, common IRQ is used to read samples for channels 7-8*/
            /* Firmware writes trigger the R5 interrupt for individual channel 6-8, to used individual IRQ same as other channels IRQ can be defined and used as needed */
            if(gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL7) || gSdfmParams.enable_channel_mask & (1<<SDFM_CHANNEL8))
            {
                /* Channel 7/8 enabled */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum = ICSSG_SDFM_HOST_INTR_NUM_CH7;
                hwiPrms.callback = &sdfmIrqHandlerChannel7;
                hwiPrms.args = 0;
                hwiPrms.isPulse = FALSE;
                hwiPrms.isFIQ = FALSE;
                status = HwiP_construct(&gSdfmHwiObjectChannel7, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
            }
        }
    }

    /* Open PRUICSS PWM handle */
    /*FIXME call if only Overcurrent or fast detect is enabeled */
    gPruIcssPwmHandle = PRUICSS_PWM_open(CONFIG_PRUICSS_PWM0, gPruIcssHandle);
    DebugP_assert(gPruIcssPwmHandle != NULL);
    gSdfmParams.pwm_handle = gPruIcssPwmHandle;

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
    status = initPruSdfm(gPruIcssHandle, gSdfmParams, &gPruIcssSdfmHandle);
    if (status != SDFM_ERR_NERR)
    {
        DebugP_log("Error: initPruSdfm() fail.\r\n");
        return;
    }
}

/**
 *  \brief Main SDFM application entry point
 *
 *  \param args [IN] User arguments (unused)
 */
void sdfm_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Sample SDFM example running!...\r\n");

    /* Output build time */
    DebugP_log("Build timestamp      : %s %s\r\n", __DATE__, __TIME__);

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Configure EPWM */
    int32_t status = SDFM_initEpwm();
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Error: SDFM_initEpwm() failed.\r\n");
        return;
    }
    DebugP_log("EPWM Configured!\r\n");
#endif

    /* Configure SDFM */
    initSdfm();
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
    while (gRunFlag == TRUE)
    {
        ;
    }

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Disable and clear interrupts for EPWM */
    SDFM_deinitEpwm();
#endif

    /* Array of pointers to HWI objects for loop-based cleanup */
    HwiP_Object* hwiObjects[9] = {
        &gSdfmHwiObjectChannel0,
        &gSdfmHwiObjectChannel1,
        &gSdfmHwiObjectChannel2,
        &gSdfmHwiObjectChannel3,
        &gSdfmHwiObjectChannel4,
        &gSdfmHwiObjectChannel5,
        &gSdfmHwiObjectChannel6,
        &gSdfmHwiObjectChannel7,
        &gSdfmHwiObjectChannel8  /* CH7 and CH8 share the same HWI object */
    };

    /* Destroy interrupt handlers for all enabled channels */
    for (int8_t i = 0; i < 9; i++)
    {
        if (gSdfmParams.enable_channel_mask & (1 << i))
        {
            /* Skip channel 8 to avoid double destruction (CH7 and CH8 share the same HWI) */
            if (i == SDFM_CHANNEL8)
            {
                continue;
            }
            HwiP_destruct(hwiObjects[i]);
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
    gSdfmIrqCntChannel0++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH0);

    if (sdfmIdxCntChannel0 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel0 = 0;
    }

    if(gSdfmParams.load_share_enable == 1)
    {
       if(gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_RTUPRU_CORE_INDX].enable_trigger_mode == 1)
       {
           /* Load share mode - Trigger/snoop mode - read all enabled RTU PRU channels (CH0-CH2) */
           for (int8_t i = SDFM_CHANNEL0; i <= SDFM_CHANNEL2; i++)
           {
               if (gSdfmParams.enable_channel_mask & (1 << i))
               {
                   sdfm_ch_samples[i][sdfmIdxCntChannel0] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
               }
           }
       }
       else
       {
           sdfm_ch_samples[SDFM_CHANNEL0][sdfmIdxCntChannel0] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL0);
       }
    }
    else
    {
        if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_trigger_mode == 1)
        {
            /* Single PRU mode - Trigger/snoop mode - read all enabled channels (CH0-CH8) */
            for (int8_t i = SDFM_CHANNEL0; i <= SDFM_CHANNEL8; i++)
            {
                if (gSdfmParams.enable_channel_mask & (1 << i))
                {
                    sdfm_ch_samples[i][sdfmIdxCntChannel0] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
                }
            }
        }
        else
        {
            /* Single PRU mode - Continuous mode - Channel 0 only */
            sdfm_ch_samples[SDFM_CHANNEL0][sdfmIdxCntChannel0] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL0);
        }
    }

    sdfmIdxCntChannel0++;
}

/**
 *  \brief SDFM IRQ handler for Channel 1
 *
 *  Only used in continuous mode - reads Channel 1 sample
 */
void sdfmIrqHandlerChannel1(void *args)
{
    gSdfmIrqCntChannel1++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH1);

    if (sdfmIdxCntChannel1 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel1 = 0;
    }

    sdfm_ch_samples[SDFM_CHANNEL1][sdfmIdxCntChannel1] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL1);
    sdfmIdxCntChannel1++;
}

/**
 *  \brief SDFM IRQ handler for Channel 2
 *
 *  Only used in continuous mode - reads Channel 2 sample
 */
void sdfmIrqHandlerChannel2(void *args)
{
    gSdfmIrqCntChannel2++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH2);

    if (sdfmIdxCntChannel2 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel2 = 0;
    }

    sdfm_ch_samples[SDFM_CHANNEL2][sdfmIdxCntChannel2] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL2);
    sdfmIdxCntChannel2++;
}

/**
 *  \brief SDFM IRQ handler for Channel 3
 *
 *  In continuous mode: Reads only Channel 3 sample
 *  In load-share trigger/snoop mode: Reads all enabled PRU channels (CH3, CH4, CH5)
 */
void sdfmIrqHandlerChannel3(void *args)
{
    gSdfmIrqCntChannel3++;

    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH3);

    if (sdfmIdxCntChannel3 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel3 = 0;
    }

    if(gSdfmParams.load_share_enable == 1)
    {
        if(gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_PRU_CORE_INDX].enable_trigger_mode == 1)
        {
            /* Load share mode - Trigger/snoop mode - read all enabled PRU channels (CH3-CH5) */
            for (int8_t i = SDFM_CHANNEL3; i <= SDFM_CHANNEL5; i++)
            {
                if (gSdfmParams.enable_channel_mask & (1 << i))
                {
                    sdfm_ch_samples[i][sdfmIdxCntChannel3] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
                }
            }
        }
        else
        {
            /* Load share mode - Continuous mode - Channel 3 only */
            sdfm_ch_samples[SDFM_CHANNEL3][sdfmIdxCntChannel3] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL3);
        }
    }
    else
    {
        /* Single PRU mode - Channel 3 only */
        sdfm_ch_samples[SDFM_CHANNEL3][sdfmIdxCntChannel3] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL3);
    }
    sdfmIdxCntChannel3++;
}

/**
 *  \brief PRU SDFM IRQ handler for Channel 4
 *
 *  Only used continuous mode - reads Channel 4 sample
 */
void sdfmIrqHandlerChannel4(void *args)
{
    gSdfmIrqCntChannel4++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH4);

    if (sdfmIdxCntChannel4 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel4 = 0;
    }

    sdfm_ch_samples[SDFM_CHANNEL4][sdfmIdxCntChannel4] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL4);
    sdfmIdxCntChannel4++;
}

/**
 *  \brief PRU SDFM IRQ handler for Channel 5
 *
 *  Only used in continuous mode - reads Channel 5 sample
 */
void sdfmIrqHandlerChannel5(void *args)
{
    gSdfmIrqCntChannel5++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH5);

    if (sdfmIdxCntChannel5 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel5 = 0;
    }

    sdfm_ch_samples[SDFM_CHANNEL5][sdfmIdxCntChannel5] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL5);
    sdfmIdxCntChannel5++;
}

/**
 *  \brief SDFM IRQ handler for Channel 6
 *
 *  In continuous mode: Reads only Channel 6 sample
 *  In load share trigger/snoop mode: Reads all enabled TX PRU channels (CH6, CH7, CH8)
 */
void sdfmIrqHandlerChannel6(void *args)
{
    gSdfmIrqCntChannel6++;
   
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH6);

    if (sdfmIdxCntChannel6 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel6 = 0;
    }
    if(gSdfmParams.load_share_enable == 1)
    {
       if(gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDX].enable_snoop_mode == 1 || gSdfmParams.pru_core_config[SDFM_TXPRU_CORE_INDX].enable_trigger_mode == 1)
       {
           /* Load share mode - Trigger/snoop mode - read all enabled TX PRU channels (CH6-CH8) */
           for (int8_t i = SDFM_CHANNEL6; i <= SDFM_CHANNEL8; i++)
           {
               if (gSdfmParams.enable_channel_mask & (1 << i))
               {
                   sdfm_ch_samples[i][sdfmIdxCntChannel6] = SDFM_getFilterData(gPruIcssSdfmHandle, i);
               }
           }
       }
       else
       {
           /* Load share mode - Continuous mode - Channel 6 only */
           sdfm_ch_samples[SDFM_CHANNEL6][sdfmIdxCntChannel6] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL6);
       }
    }
    else
    {
        /* Single PRU mode - Channel 6 only */
        sdfm_ch_samples[SDFM_CHANNEL6][sdfmIdxCntChannel6] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL6);
    }
    sdfmIdxCntChannel6++;
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
    gSdfmIrqCntChannel7++;
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH7);
    
    if (sdfmIdxCntChannel7 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel7 = 0;
    }
    sdfm_ch_samples[SDFM_CHANNEL7][sdfmIdxCntChannel7] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL7);
    sdfmIdxCntChannel7++;

    /*Channel 8*/
    PRUICSS_clearEvent(gPruIcssHandle, ICSS_SDFM_TRIGGER_EVNT_CH8);
    if (sdfmIdxCntChannel8 >= MAX_SAMPLES)
    {
        sdfmIdxCntChannel8 = 0;
    }
    sdfm_ch_samples[SDFM_CHANNEL8][sdfmIdxCntChannel8] = SDFM_getFilterData(gPruIcssSdfmHandle, SDFM_CHANNEL8);

    sdfmIdxCntChannel8++;
}

/**
 *  \brief PRU SDFM IRQ handler for Channel 8
 *
 *  Note: This handler exists for completeness but is not currently used
 *  because Channel 7 and Channel 8 share the same host interrupt (HOST_INTR_PEND_7)
 *  and are handled together by sdfmIrqHandlerChannel7.
 */
void sdfmIrqHandlerChannel8(void *args)
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
