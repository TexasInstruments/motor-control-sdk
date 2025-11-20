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

/* R5F interrupt settings for ICSSG */
#define ICSSG_PRU_SDFM_INT_NUM_CH0          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1 )
#define ICSSG_PRU_SDFM_INT_NUM_CH1          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_2 )
#define ICSSG_PRU_SDFM_INT_NUM_CH2          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3 )

/* HWI global variables */
static HwiP_Object gIcssgPruSdfmHwiObject;  /* ICSSG PRU SDFM FW HWI */

/* ICSSG PRU SDFM FW IRQ handler */
void pruSdfmIrqHandlerCh0(void *handle);
void pruSdfmIrqHandlerCh1(void *handle);
void pruSdfmIrqHandlerCh2(void *handle);

/* Test ICSSG handle */
PRUICSS_Handle gPruIcssHandle;

/* ICSSG PWM handle */
PRUICSS_PWM_Handle gPruIcssPwmHandle;

/* Test Sdfm handles */
SDFM_Handle gPruIcssSdfmHandle;

/* Sdfm output samples, written by PRU cores */
__attribute__((section(".gSdfmSampleOutput"))) uint32_t gSdfm_sampleOutput[SDFM_NUM_OF_CH_PER_PRU_SLICE];

/* Sdfm parameters */
SDFM_Params gSdfmParams;

#define PRUICSS_G_MUX_EN    ( 0x1 ) /* ICSSG_SA_MX_REG:G_MUX_EN */

/* Flag for continuing to execute test */
volatile Bool gRunFlag = TRUE;

/* ICSS SDFM Output sample for Channel 0 */
/*Sample size*/
#define MAX_SAMPLES (128)

/* ICSS SDFM Output samples */
uint32_t sdfm_ch_samples[SDFM_NUM_OF_CH_PER_PRU_SLICE][MAX_SAMPLES] = {0};
uint32_t sdfmPruIdxCntCh0 = 0;
uint32_t sdfmPruIdxCntCh1 = 0;
uint32_t sdfmPruIdxCntCh2 = 0;

/* IRQ counters */
volatile uint32_t gPruSdfmIrqCntCh0=0; /* PRU ICSS SDFM FW IRQ count */
volatile uint32_t gPruSdfmIrqCntCh1=0; /* PRU ICSS SDFM FW IRQ count */
volatile uint32_t gPruSdfmIrqCntCh2=0; /* PRU ICSS SDFM FW IRQ count */

HwiP_Params hwiPrms;

void init_sdfm()
{
    int32_t status;
    /* Initialize ICSSG */
    status = initIcss(CONFIG_PRU_ICSS0, CONFIG_SDFM0_SLICE, PRUICSS_G_MUX_EN, CONFIG_SDFM0_LOAD_SHARE, &gPruIcssHandle);
    if (status != SDFM_ERR_NERR) {
        DebugP_log("Error: initIcss() fail.\r\n");
        return;
    }
     
#if (CONFIG_SDFM0_CHANNEL0 == 1)
    /* Register & enable ICSSG PRU SDFM FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_SDFM_INT_NUM_CH0;
    hwiPrms.callback    = &pruSdfmIrqHandlerCh0;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgPruSdfmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_SDFM0_CHANNEL1 == 1) 
    /* Register & enable ICSSG PRU SDFM FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_SDFM_INT_NUM_CH1;
    hwiPrms.callback    = &pruSdfmIrqHandlerCh1;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgPruSdfmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#if (CONFIG_SDFM0_CHANNEL2 == 1)
    /* Register & enable ICSSG PRU SDFM FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_SDFM_INT_NUM_CH2;
    hwiPrms.callback    = &pruSdfmIrqHandlerCh2;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgPruSdfmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /* Configure Sdfm  parameters */
    sdfmGlobalParamsConfig(&gSdfmParams);

    for(int8_t i=0; i<9; i++)
    {
        if(gSdfmParams.sdfm_channel_mask & (1 << i))
        {
            /* enable channel number*/
            DebugP_log("Enable channel %d.\r\n", i);
            sdfmParamsConfig(i, &gSdfmParams);
        }
    }
#if(CONFIG_SDFM0_LOAD_SHARE == 1 )
#if(CONFIG_SDFM0_CHANNEL0 == 1 || CONFIG_SDFM0_CHANNEL1 == 1 || CONFIG_SDFM0_CHANNEL2 == 1)
    sdfmAxisParamsConfig(&gSdfmParams, SDFM_RTUPRU_CORE_INDX);
#endif
#if(CONFIG_SDFM0_CHANNEL3 == 1 || CONFIG_SDFM0_CHANNEL4 == 1 || CONFIG_SDFM0_CHANNEL5 == 1)
    sdfmAxisParamsConfig(&gSdfmParams, SDFM_PRU_CORE_INDX);
#endif
#if(CONFIG_SDFM0_CHANNEL6 == 1 || CONFIG_SDFM0_CHANNEL7 == 1 || CONFIG_SDFM0_CHANNEL8 == 1)
    sdfmAxisParamsConfig(&gSdfmParams, SDFM_TXPRU_CORE_INDX);
#endif
#else
    sdfmAxisParamsConfig(&gSdfmParams, SDFM_PRU_CORE_INDX);
#endif
    
    gPruIcssPwmHandle = PRUICSS_PWM_open(CONFIG_PRUICSS_PWM0, gPruIcssHandle);
    DebugP_assert(gPruIcssPwmHandle != NULL);

    gSdfmParams.pwm_handle = gPruIcssPwmHandle;
    gSdfmParams.pruicss_handle = gPruIcssHandle;

    /*sample output base address for all channel*/
    gSdfmParams.samplesBaseAddress = (uint32_t)&gSdfm_sampleOutput;

    status = initPruSdfm(gPruIcssHandle, gSdfmParams, &gPruIcssSdfmHandle);

    if (status != SDFM_ERR_NERR) 
    {
        DebugP_log("Error: initPruSdfm() fail.\r\n");
        return;
    }
}
void sdfm_main(void *args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    int32_t status;

    DebugP_log("Sample SDFM example running!...\r\n");

    /* Output build time */
    DebugP_log("Build timestamp      : %s %s\r\n", __DATE__, __TIME__);
#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /*
     *  Configure EPWM0
     */
    status = SDFM_initEpwm();
    if (status != SystemP_SUCCESS) {
        DebugP_log("Error: SDFM_initEpwm() failed.\r\n");
        return;
    }
    DebugP_log("EPWM Configured!\r\n");
#endif 
    /* Configure SDFM */
    init_sdfm();
    DebugP_log("SDFM Configured!\r\n");
    
    while(gRunFlag == TRUE)
    {
        ;
    }

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1 || APP_EPWM1_ENABLE == 1)
    /* Disable and clear interrupts for EPWM */
    SDFM_deinitEpwm();
#endif
    /* Destroy PRU SDFM HWI */
    HwiP_destruct(&gIcssgPruSdfmHwiObject);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

/* PRU SDFM FW IRQ handler */
void pruSdfmIrqHandlerCh0(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPruSdfmIrqCntCh0++;
    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssHandle, PRU_TRIGGER_HOST_SDFM_EVT_CH0);

    if(sdfmPruIdxCntCh0 >= MAX_SAMPLES)
    {
        sdfmPruIdxCntCh0 = 0;
    }

    sdfm_ch_samples[SDFM_CH0][sdfmPruIdxCntCh0] = SDFM_getFilterData(gPruIcssSdfmHandle, 0);

    sdfmPruIdxCntCh0++;
}

void pruSdfmIrqHandlerCh1(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPruSdfmIrqCntCh1++;
    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssHandle, PRU_TRIGGER_HOST_SDFM_EVT_CH1);

    if(sdfmPruIdxCntCh1 >= MAX_SAMPLES)
    {
        sdfmPruIdxCntCh1 = 0;
    }
    
    /* SDFM Output sample for Channel 1 */
    sdfm_ch_samples[SDFM_CH1][sdfmPruIdxCntCh1] = SDFM_getFilterData(gPruIcssSdfmHandle, 1);
    sdfmPruIdxCntCh1++;
}

/* PRU SDFM FW IRQ handler */
void pruSdfmIrqHandlerCh2(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPruSdfmIrqCntCh2++;
    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssHandle, PRU_TRIGGER_HOST_SDFM_EVT_CH2);

    if(sdfmPruIdxCntCh2 >= MAX_SAMPLES)
    {
        sdfmPruIdxCntCh2 = 0;
    }
    /* SDFM Output sample for Channel 2 */
    sdfm_ch_samples[SDFM_CH2][sdfmPruIdxCntCh2] = SDFM_getFilterData(gPruIcssSdfmHandle, 2);
    sdfmPruIdxCntCh2++;
}
