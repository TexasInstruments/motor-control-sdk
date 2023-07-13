/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include "epwm_dc.h"
#include "cfg_pad.h"
#include "sddf.h"

/* Output channel - A or B */
#define APP_EPWM_OUT_CH_EN              ( 0x1 ) /* ChA enabled */

/* EPWM functional clock */
/* Functional clock is the same for all EPWMs */
#define APP_EPWM_FCLK                   ( CONFIG_EPWM0_FCLK )

/* EPWM functional clock dividers */
#define APP_EPWM_FCLK_HSPCLKDIV         ( 0x0 ) /* EPWM_TBCTL:HSPCLKDIV, High-Speed Time-base Clock Prescale Bits */
#define APP_EPWM_FCLK_CLKDIV            ( 0x0 ) /* EPWM_TBCTL:CLKDIV, Time-base Clock Prescale Bits */

#if (APP_EPWM_FCLK_HSPCLKDIV != 0x0)
/* EPWM Time Base clock -- all EPWM TB clocks set the same */
#define APP_EPWM_TB_FREQ               ( APP_EPWM_FCLK / ( 2*APP_EPWM_FCLK_HSPCLKDIV * (1 << APP_EPWM_FCLK_CLKDIV)))
#else
/* EPWM Time Base clock -- all EPWM TB clocks set the same */
#define APP_EPWM_TB_FREQ               ( APP_EPWM_FCLK / ( 1 * (1 << APP_EPWM_FCLK_CLKDIV)))
#endif

/* Initial Duty Cycle of PWM output signal in %, 0 to 100 */
#define APP_EPWM0_DUTY_CYCLE            ( 50U )

/* Frequency of PWM output signal in Hz */
#define APP_EPWM_OUTPUT_FREQ_4K         ( 1U * 4000U )
#define APP_EPWM_OUTPUT_FREQ_8K         ( 1U * 8000U )
#define APP_EPWM_OUTPUT_FREQ_16K        ( 1U * 16000U )
#define APP_EPWM_OUTPUT_FREQ_20K        ( 1U * 20000U )
#define APP_EPWM_OUTPUT_FREQ            ( APP_EPWM_OUTPUT_FREQ_8K ) /* init freq */

/*sample Read time */
#define SAMPLE_READ_TIME                ((float)(1000000/(2*APP_EPWM_OUTPUT_FREQ))) //Middle of PWM loop

/* PWM count direction (Up, Down, Up/Down) */
#define APP_EPWM_TB_COUNTER_DIR         ( EPWM_TB_COUNTER_DIR_UP_DOWN )

/* Test ICSSG instance ID */
#define TEST_ICSSG_INST_ID              ( CONFIG_PRU_ICSS0 )
/* Test ICSSG slice ID */
#define TEST_ICSSG_SLICE_ID             ( ICSSG_SLICE_ID_0 )
/* Test PRU core instance IDs */
#define TEST_PRU_INST_ID                ( PRUICSS_PRU0 )
#define TEST_RTU_INST_ID                ( PRUICSS_RTU_PRU0 )

/* R5F interrupt settings for ICSSG */
#define ICSSG_PRU_SDDF_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )  /* VIM interrupt number */
#define ICSSG_RTU_SDDF_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1 )  /* VIM interrupt number */

/* EPWM0 IRQ handler */
static void epwmIrqHandler(void *handle);

/* HWI global variables */
static HwiP_Object gIcssgPruSddfHwiObject;  /* ICSSG PRU SDDF FW HWI */
//static HwiP_Object gIcssgRtuSddfHwiObject;  /* ICSSG RTU SDDF FW HWI */
static HwiP_Object gEpwm0HwiObject;         /* EPWM0 HWI */

/* EPWM global variables */
uint32_t gEpwm0BaseAddr;    /* EPWM0 base address */
EPwmObj_t gEpwm0Obj;        /* EPWM0 object */
Epwm_Handle hEpwm0;         /* EPWM0 handle */
volatile uint32_t gEpwmOutFreq = APP_EPWM_OUTPUT_FREQ; /* EPWM output frequency */

/* ICSSG PRU SDDF FW IRQ handler */
static void pruSddfIrqHandler(void *handle);


/* Test ICSSG handle */
PRUICSS_Handle gPruIcssHandle;

/* Test SDDF handles */
sdfm_handle gHPruSddf;


/* Test SDDF parameters */
SddfPrms gTestSddfPrms = {
    SAMPLE_READ_TIME,       // trigSampTime in us; always middel of PWM cycle
    APP_EPWM_OUTPUT_FREQ,        // PWM output frequency
    {{3500,2500,0},    //threshold parameters(High, low & reserevd)
    {3500,2500,0},
    {3500,2500,0}},
    {{0,0},                //clock sourse & clock inversion for all channels
    {0,0},
    {0,0}},
     32,   //OC osr
     64,   //NC osr
     1   //comparator enable
};

#define PRUICSS_G_MUX_EN    ( 0x1 ) /* ICSSG_SA_MX_REG:G_MUX_EN */

/* GPIO enable signal for EPWM0-2 on 3-axis breakout board */
uint32_t gMtr1PwnEnGpioBaseAddr = GPIO_MTR_1_PWM_EN_BASE_ADDR;
uint32_t gMtr1PwnEnGpioPin      = GPIO_MTR_1_PWM_EN_PIN;
uint32_t gMtr1PwnEnGpioPinDir   = GPIO_MTR_1_PWM_EN_DIR;

/* Flag for continuing to execute test */
volatile Bool gRunFlag = TRUE;


/* SDFM Output sample for Channel 0 */
/*Sample size*/
#define MAX_SAMPLES (128)
uint32_t sdfm_ch0_samples[MAX_SAMPLES] = {0};
uint32_t sdfm_ch0_idx = 0;
/* SDFM Output sample for Channel 1 */
uint32_t sdfm_ch1_samples[MAX_SAMPLES] = {0};
uint32_t sdfm_ch1_idx = 0;
/* SDFM Output sample for Channel 2 */
uint32_t sdfm_ch2_samples[MAX_SAMPLES] = {0};
uint32_t sdfm_ch2_idx = 0;


/* IRQ counters */
volatile uint32_t gPruSddfIrqCnt=0; /* PRU SDDF FW IRQ count */
volatile uint32_t gEpwmIsrCnt=0;    /* EPWM0 IRQ count */

/*PWM Parameters*/
HwiP_Params hwiPrms;
EPwmCfgPrms_t epwmCfgPrms;

void init_pwm()
{
    int32_t status;
    /* Initialize EPWM0 base address, perform address translation */
    gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);

    /* Register & enable EPWM0 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &epwmIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gEpwm0HwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure EPWM0 */
    epwmCfgPrms.epwmId = EPWM_ID_0;
    epwmCfgPrms.epwmBaseAddr = gEpwm0BaseAddr;
    epwmCfgPrms.epwmOutChEn = APP_EPWM_OUT_CH_EN;
    epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
    epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
    epwmCfgPrms.epwmTbFreq = APP_EPWM_TB_FREQ;
    epwmCfgPrms.epwmOutFreq = gEpwmOutFreq;
    epwmCfgPrms.epwmDutyCycle[EPWM_OUTPUT_CH_A] = APP_EPWM0_DUTY_CYCLE;
    epwmCfgPrms.epwmTbCounterDir = APP_EPWM_TB_COUNTER_DIR;
    epwmCfgPrms.cfgTbSyncIn = TRUE;
    epwmCfgPrms.tbPhsValue = 0;
    epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    epwmCfgPrms.cfgTbSyncOut = TRUE;
    epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
    epwmCfgPrms.aqCfg[EPWM_OUTPUT_CH_A].zeroAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg[EPWM_OUTPUT_CH_A].prdAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    epwmCfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpADownAction = EPWM_AQ_ACTION_LOW;
    epwmCfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.cfgDb = FALSE;
    epwmCfgPrms.cfgEt = TRUE;
    epwmCfgPrms.intSel = EPWM_ET_INTR_EVT_CNT_EQ_ZRO;
    epwmCfgPrms.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    hEpwm0 = epwmInit(&epwmCfgPrms, &gEpwm0Obj);
    DebugP_assert(hEpwm0 != NULL);

}

void init_sdfm()
{
    int32_t status;
    /* Initialize ICSSG */
    status = initIcss(TEST_ICSSG_INST_ID, TEST_ICSSG_SLICE_ID, PRUICSS_G_MUX_EN, &gPruIcssHandle);
    if (status != SDDF_ERR_NERR) {
        DebugP_log("Error: initIcss() fail.\r\n");
        return;
    }

    /* Register & enable ICSSG PRU SDDF FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_SDDF_INT_NUM;
    hwiPrms.callback    = &pruSddfIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgPruSddfHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Initialize PRU core for SDDF */
    status = initPruSddf(gPruIcssHandle, TEST_PRU_INST_ID, &gTestSddfPrms, &gHPruSddf);
    if (status != SDDF_ERR_NERR) {
        DebugP_log("Error: initPruSddf() fail.\r\n");
        return;
    }

}
void sddf_main(void *args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Sample SDFM example running!...\r\n");

    /* Output build time */
    DebugP_log("Build timestamp      : %s %s\r\n", __DATE__, __TIME__);


    /* Enable EPWM0-2 on 3-axis Breakout Board */
    GPIO_setDirMode(gMtr1PwnEnGpioBaseAddr, gMtr1PwnEnGpioPin, gMtr1PwnEnGpioPinDir);
    GPIO_pinWriteHigh(gMtr1PwnEnGpioBaseAddr, gMtr1PwnEnGpioPin);
    GPIO_pinWriteLow(gMtr1PwnEnGpioBaseAddr, gMtr1PwnEnGpioPin);

    /*
     *  Configure EPWM0
     */
    init_pwm();
    DebugP_log("EPWM Configured!\r\n");
    /*
     *  Configure SDDF
     */

    /* Configure SOC pads for SDDF.
       Normally handled via Pinmux_init(),
       but currently no way to pads for ICSSG from Sysconfig. */
    cfgPad();

     /* Configure SDFM */
    init_sdfm();
    DebugP_log("SDFM Configured!\r\n");

    /* Start EPWM0 clock */
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN, 1);

    /* Force SW sync for EPWM0 */
    EPWM_tbTriggerSwSync(gEpwm0BaseAddr);

    while(gRunFlag == TRUE)
    {
        ;
    }

    /* Disable and clear interrupts for EPWM0 */
    EPWM_etIntrDisable(gEpwm0BaseAddr); /* Disable interrupts */
    EPWM_etIntrClear(gEpwm0BaseAddr);   /* Clear pending interrupts */

    /* Destroy EPWM0 HWI */
    HwiP_destruct(&gEpwm0HwiObject);

    /* Destroy PRU SDDF HWI */
    HwiP_destruct(&gIcssgPruSddfHwiObject);

    /* Destroy RTU SDDF HWI */
    //HwiP_destruct(&gIcssgRtuSddfHwiObject);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

/* PRU SDDF FW IRQ handler */
void pruSddfIrqHandler(void *args)
{
    /* debug, inncrement PRU SDDF IRQ count */
    gPruSddfIrqCnt++;
    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDDF_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */
    PRUICSS_clearEvent(gPruIcssHandle, PRU_TRIGGER_HOST_SDDF_EVT);

   /* SDFM Output sample for Channel 0 */
    sdfm_ch0_samples[sdfm_ch0_idx++] = SDFM_getFilterData(0);
    /* SDFM Output sample for Channel 1 */
    sdfm_ch1_samples[sdfm_ch1_idx++] = SDFM_getFilterData(1);
    /* SDFM Output sample for Channel 2 */
    sdfm_ch2_samples[sdfm_ch2_idx++] = SDFM_getFilterData(2);


     if(sdfm_ch0_idx >= MAX_SAMPLES)
    {
        sdfm_ch0_idx = 0;
        sdfm_ch1_idx = 0;
        sdfm_ch2_idx = 0;
    }


}


/* EPWM0 IRQ handler */
static void epwmIrqHandler(void *args)
{
    volatile uint16_t status;

    /* debug, inncrement EPWM0 IRQ count */
    gEpwmIsrCnt++;

    status = EPWM_etIntrStatus(gEpwm0BaseAddr);
    if (status & EPWM_ETFLG_INT_MASK) {
        EPWM_etIntrClear(gEpwm0BaseAddr);
    }
    return;
}
