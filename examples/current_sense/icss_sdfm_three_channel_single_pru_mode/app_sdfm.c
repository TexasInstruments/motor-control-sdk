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
#include "sdfm_example.h"

/*EPWM1 configuration for sigma delta clock generation: */
#define APP_EPWM1_ENABLE  0 /*make sure EPWM1 is added in sysconfig before making true this macro */
/* Output channel - A or B */
#define APP_EPWM_OUT_CH_EN              ( 0x1 ) /* ChA enabled */

#define  NUM_CH_SUPPORTED          ( 3 )
#define ICSSG_PRU_LOAD_SHARE_MODE  ( 0 )
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
#define FIRST_SAMPLE_TRIGGER_TIME                ((float)((float)1000000/(4*APP_EPWM_OUTPUT_FREQ))) /*sample trigger time for Single Update */

#define SECOND_SAMPLE_TRIGGER_TIME               ((float)((float)3000000/(4*APP_EPWM_OUTPUT_FREQ))) /*Sample trigger time for double update*/

/* PWM count direction (Up, Down, Up/Down) */
#define APP_EPWM_TB_COUNTER_DIR         ( EPWM_TB_COUNTER_DIR_UP_DOWN )

/* Test ICSSG instance ID */
#define TEST_ICSSG_INST_ID              ( CONFIG_PRU_ICSS0 )
/* Test ICSSG slice ID */
#define TEST_ICSSG_SLICE_ID             ( ICSSG_SLICE_ID_0 )

/* R5F interrupt settings for ICSSG */
#define ICSSG_PRU_SDFM_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3 )  /* VIM interrupt number */

/* EPWM0 IRQ handler */
static void epwmIrqHandler(void *handle);

/* EPWM1 IRQ handler */
static void epwmIrqHandler1(void *handle);

/* HWI global variables */
static HwiP_Object gIcssgPruSdfmHwiObject;  /* ICSSG PRU SDFM FW HWI */

#if APP_EPWM1_ENABLE
#define APP_EPWM_OUTPUT_FREQ1            (1U*20000000 )
static HwiP_Object gIcssgRtuSDFMHwiObject;  /* ICSSG RTU SDFM FW HWI */

static HwiP_Object gEpwm1HwiObject;         /* EPWM1 HWI */

uint32_t gEpwm1BaseAddr;    /* EPWM1 base address */
EPwmObj_t gEpwm1Obj;        /* EPWM1 object */
Epwm_Handle hEpwm1;         /* EPWM1 handle */

volatile uint32_t gEpwmOutFreq1 = APP_EPWM_OUTPUT_FREQ1; /*EPWM1 output freq. */

#endif

static HwiP_Object gEpwm0HwiObject;         /* EPWM0 HWI */

/* EPWM global variables */
uint32_t gEpwm0BaseAddr;    /* EPWM0 base address */
EPwmObj_t gEpwm0Obj;        /* EPWM0 object */
Epwm_Handle hEpwm0;         /* EPWM0 handle */



volatile uint32_t gEpwmOutFreq = APP_EPWM_OUTPUT_FREQ; /* EPWM output frequency */


/* ICSSG PRU SDFM FW IRQ handler */
static void pruSdfmIrqHandler(void *handle);


/* Test ICSSG handle */
PRUICSS_Handle gPruIcssHandle;

/* Test Sdfm handles */
sdfm_handle gHPruSdfm;

/* Sdfm output samples, written by PRU cores */
__attribute__((section(".gSdfmSampleOutput"))) uint32_t gSdfm_sampleOutput[NUM_CH_SUPPORTED];

/* Test Sdfm parameters */
SdfmPrms gTestSdfmPrms = {
    0, /*Load share enable*/
    PRUICSS_PRU0,
    TEST_ICSSG_SLICE_ID,
    300000000,   /*PRU core clock*/
    {300000000, 0},   /*Value of G0IEP0,  second index reserved for G1IEP0 */
    20000000,    /*Value of SD clock (It should be exact equal to sd clock value)*/
    0,                        /*enable double update*/
    FIRST_SAMPLE_TRIGGER_TIME,       /*first sample  trigger time*/
    SECOND_SAMPLE_TRIGGER_TIME,       /*second sample trigger time*/
    APP_EPWM_OUTPUT_FREQ,     /*PWM output frequency*/
    {{3500, 1000,0},    /*threshold parameters(High, low & reserevd)*/
    {3500, 1000,0},
    {3500, 1000,0}},
    {{0,0},                /*clock sourse & clock inversion for all channels*/
    {0,0},
    {0,0}},
    15,   /*Over current osr: The effect count is OSR + 1*/
    128,   /*Normal current osr */
    1,   /*comparator enable*/
    (uint32_t)&gSdfm_sampleOutput, /*Output samples base address*/
    0,    /*Fast Detect enable */
    {{4, 18, 2},
    {4, 18, 2},
    {4, 18, 2}
    },   /*Fast detect fields {Window size, zero count max, zero count min}*/
    0,   /*reserved for phase delay*/
    0,   /*Enable zero cross*/
    {1700, 1700, 1700}, /*Zero cross threshold*/
};

#define PRUICSS_G_MUX_EN    ( 0x1 ) /* ICSSG_SA_MX_REG:G_MUX_EN */

/* GPIO enable signal for EPWM0-2 on 3-axis breakout board */
uint32_t gMtr1PwnEnGpioBaseAddr = GPIO_MTR_1_PWM_EN_BASE_ADDR;
uint32_t gMtr1PwnEnGpioPin      = GPIO_MTR_1_PWM_EN_PIN;
uint32_t gMtr1PwnEnGpioPinDir   = GPIO_MTR_1_PWM_EN_DIR;

/* Flag for continuing to execute test */
volatile Bool gRunFlag = TRUE;


/* ICSS SDFM Output sample for Channel 0 */
/*Sample size*/
#define MAX_SAMPLES (128)

/* ICSS SDFM Output samples */
uint32_t sdfm_ch_samples[NUM_CH_SUPPORTED][MAX_SAMPLES] = {0};
uint32_t sdfmPruIdxCnt = 0;

/* IRQ counters */
volatile uint32_t gPruSdfmIrqCnt=0; /* PRU ICSS SDFM FW IRQ count */

volatile uint32_t gEpwmIsrCnt=0;    /* EPWM0 IRQ count */
volatile uint32_t gEpwmIsrCnt1=0;
/*PWM Parameters*/
HwiP_Params hwiPrms;
HwiP_Params hwiPrms1;
EPwmCfgPrms_t epwmCfgPrms;
EPwmCfgPrms_t epwm1CfgPrms;

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

#if APP_EPWM1_ENABLE  // DEBUG code for SDFM clock generation from EPWM1
    /* EPWM1 for SD clock generation */
    /* Initialize EPWM1 base address, perform address translation */
    gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM1_BASE_ADDR);
    /* Register & enable EPWM0 interrupt */
    HwiP_Params_init(&hwiPrms1);
    hwiPrms1.intNum      = CONFIG_EPWM1_INTR;
    hwiPrms1.callback    = &epwmIrqHandler1;
    hwiPrms1.args        = 0;
    hwiPrms1.isPulse     = CONFIG_EPWM1_INTR_IS_PULSE;
    hwiPrms1.isFIQ       = FALSE;
    status              = HwiP_construct(&gEpwm1HwiObject, &hwiPrms1);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Configure EPWM0 */
    epwm1CfgPrms.epwmId = EPWM_ID_1;
    epwm1CfgPrms.epwmBaseAddr = gEpwm1BaseAddr;
    epwm1CfgPrms.epwmOutChEn = APP_EPWM_OUT_CH_EN;
    epwm1CfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
    epwm1CfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
    epwm1CfgPrms.epwmTbFreq = APP_EPWM_TB_FREQ;
    epwm1CfgPrms.epwmOutFreq = gEpwmOutFreq1;
    epwm1CfgPrms.epwmDutyCycle[EPWM_OUTPUT_CH_A] = APP_EPWM0_DUTY_CYCLE;
    epwm1CfgPrms.epwmTbCounterDir = APP_EPWM_TB_COUNTER_DIR;
    epwm1CfgPrms.cfgTbSyncIn = FALSE;
    epwm1CfgPrms.tbPhsValue = 0;
    epwm1CfgPrms.cfgTbSyncOut = FALSE;
    epwm1CfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
    epwm1CfgPrms.aqCfg[EPWM_OUTPUT_CH_A].zeroAction = EPWM_AQ_ACTION_DONOTHING;
    epwm1CfgPrms.aqCfg[EPWM_OUTPUT_CH_A].prdAction = EPWM_AQ_ACTION_DONOTHING;
    epwm1CfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    epwm1CfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpADownAction = EPWM_AQ_ACTION_LOW;
    epwm1CfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    epwm1CfgPrms.aqCfg[EPWM_OUTPUT_CH_A].cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    epwm1CfgPrms.cfgDb = FALSE;
    epwm1CfgPrms.cfgEt = FALSE;
    epwm1CfgPrms.intSel = EPWM_ET_INTR_EVT_CNT_EQ_ZRO;
    epwm1CfgPrms.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    hEpwm1 = epwmInit(&epwm1CfgPrms, &gEpwm1Obj);
    DebugP_assert(hEpwm1 != NULL);
#endif 
}

void init_sdfm()
{
    int32_t status;
    /* Initialize ICSSG */
    status = initIcss(TEST_ICSSG_INST_ID, TEST_ICSSG_SLICE_ID, PRUICSS_G_MUX_EN, ICSSG_PRU_LOAD_SHARE_MODE, &gPruIcssHandle);
    if (status != SDFM_ERR_NERR) {
        DebugP_log("Error: initIcss() fail.\r\n");
        return;
    }
     
    /* Register & enable ICSSG PRU SDFM FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_SDFM_INT_NUM;
    hwiPrms.callback    = &pruSdfmIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgPruSdfmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Initialize PRU cores for SDFM */
    status = initPruSdfm(gPruIcssHandle, PRUICSS_PRU0, &gTestSdfmPrms, &gHPruSdfm);
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

    /* Destroy PRU SDFM HWI */
    HwiP_destruct(&gIcssgPruSdfmHwiObject);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

/* PRU SDFM FW IRQ handler */
void pruSdfmIrqHandler(void *args)
{
    /* debug, inncrement PRU SDFM IRQ count */
    gPruSdfmIrqCnt++;
    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDFM_IRQ defined as 18
        18 = 16+2, 2 is Host Interrupt Number. See AM64x TRM.
    */
    PRUICSS_clearEvent(gPruIcssHandle, PRU_TRIGGER_HOST_SDFM_EVT);

    if(sdfmPruIdxCnt >= MAX_SAMPLES)
    {
        sdfmPruIdxCnt = 0;      
    }

    /* SDFM Output sample for Channel 0 */
    sdfm_ch_samples[SDFM_CH0][sdfmPruIdxCnt] = SDFM_getFilterData(gHPruSdfm, 0);
    /* SDFM Output sample for Channel 1 */
    sdfm_ch_samples[SDFM_CH1][sdfmPruIdxCnt] = SDFM_getFilterData(gHPruSdfm, 1);
    /* SDFM Output sample for Channel 2 */
    sdfm_ch_samples[SDFM_CH2][sdfmPruIdxCnt] = SDFM_getFilterData(gHPruSdfm, 2);

    sdfmPruIdxCnt++;
}

/* EPWM0 IRQ handler */
static void epwmIrqHandler(void *args)
{
    volatile uint16_t status;

    /* debug, inncrement EPWM0 IRQ count */
    gEpwmIsrCnt++;

    status = EPWM_etIntrStatus(gEpwm0BaseAddr);
    if(status & EPWM_ETFLG_INT_MASK) 
    {
        EPWM_etIntrClear(gEpwm0BaseAddr);
    }
    return;
}

#if APP_EPWM1_ENABLE //DEBUG code for EPWM1
/* EPWM0 IRQ handler */
static void epwmIrqHandler1(void *args)
{
    volatile uint16_t status;

    /* debug, inncrement EPWM0 IRQ count */
    gEpwmIsrCnt1++;

    status = EPWM_etIntrStatus(gEpwm1BaseAddr);
    if (status & EPWM_ETFLG_INT_MASK) 
    {
        EPWM_etIntrClear(gEpwm0BaseAddr);
    }
    return;
}
#endif
