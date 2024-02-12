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
#include <math.h>
#include <inttypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ipc_notify.h>
#include <drivers/pinmux.h>
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <position_sense/endat/include/endat_drv.h>
#include <position_sense/endat/firmware/endat_master_bin.h>
#include <position_sense/endat/firmware/endat_master_multi_bin.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "DRV8350_defs.h"
#include "pwm.h"
#include "settings.h"
#include "ti_r5fmath_trig.h"
#include <cmsis/DSP/Include/arm_math.h>

#include "tisddf_pruss_intc_mapping.h"
#include "mclk_iep_sync.h"
#include "sddf.h"
#include <sdfm_api.h>
#include "endat_periodic_trigger.h"

#include <board/ioexp/ioexp_tca6424.h>

#if defined(USE_RTLIB_FOC)
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"
#include "dcl.h"
#endif

#if PRU_ICSSGx_PRU_SLICE
#define PRUICSS_PRUx PRUICSS_PRU1
#define PRUICSS_TXPRUx PRUICSS_TX_PRU1
#define PRUICSS_RTUPRUx PRUICSS_RTU_PRU1
#else
#define PRUICSS_PRUx PRUICSS_PRU0
#define PRUICSS_TXPRUx PRUICSS_TX_PRU0
#define PRUICSS_RTUPRUx PRUICSS_RTU_PRU0
#endif
#define PRUICSS_SLICEx PRU_ICSSGx_PRU_SLICE

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
#include  <position_sense/endat/firmware/endat_master_multi_bin.h>
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#include <position_sense/endat/firmware/endat_master_multi_RTU_bin.h>
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#include <position_sense/endat/firmware/endat_master_multi_PRU_bin.h>
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#include <position_sense/endat/firmware/endat_master_multi_TXPRU_bin.h>
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
#include <position_sense/endat/firmware/endat_master_bin.h>
#endif

#define WAIT_5_SECOND  (5000)
#define TASK_STACK_SIZE (4096)
#define TASK_PRIORITY   (6)

#define ENDAT_RX_SAMPLE_SIZE    7
#define ENDAT_RX_SESQUI_DIV (1 << 15)

#define MRS_POS_VAL2_WORD1  0x42
#define MRS_POS_VAL2_WORD2  0x43
#define MRS_POS_VAL2_WORD3  0x44

/* Translate the TCM local view addr to SoC view addr */
#define CPU0_ATCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_ATCM_BASE+(x))
#define CPU1_ATCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_ATCM_BASE+(x))
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))
#define CPU1_BTCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_BTCM_BASE+(x - CSL_R5FSS1_BTCM_BASE))

#define  NUM_CH_SUPPORTED      ( 3 )

/* EPWM ISR information */
typedef struct _AppEPwmIsrInfo_t
{
    uint32_t            epwmBaseAddr;
    SemaphoreP_Object   *pEpwmSyncSemObject;
} AppEPwmIsrInfo_t;
/* Interrupt globals */
static HwiP_Object       gEpwm0HwiObject;
static SemaphoreP_Object gEpwm0SyncSemObject;
static SemaphoreP_Object gEpwm0SyncSemObject2;
/* variable to hold EPWM base address and semaphore address for ISR */
AppEPwmIsrInfo_t gAppEPwm0Info;
AppEPwmIsrInfo_t gAppEPwm0Info2;

/* variables to hold EPWM base addresses */
uint32_t gEpwm0BaseAddr;
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;

uint32_t gEpwm0BaseAddr2A;
uint32_t gEpwm0BaseAddr2B;
uint32_t gEpwm1BaseAddr2;
uint32_t gEpwm2BaseAddr2;

/* EPWM output frequency */
volatile uint32_t gEpwmOutFreq = APP_EPWM_OUTPUT_FREQ;
/* EPWM output frequency set value */
volatile uint32_t gEpwmOutFreqSet = APP_EPWM_OUTPUT_FREQ;
/* EPWM Rising Edge Delay */
uint32_t gDbRisingEdgeDelay = APP_EPWM_DB_RED_COUNT;
/* EPWM Falling Edge Delay */
uint32_t gDbFallingEdgeDelay = APP_EPWM_DB_FED_COUNT;

volatile Bool gSetEPwmOutFreq8K = TRUE;
volatile Bool gSetEPwmOutFreq16K = TRUE;
volatile Bool gSetEPwmOutFreq4K = TRUE;

// debug, mainloop count
uint32_t gLoopCnt = 0;

// debug, ISR counts
volatile uint32_t gEpwmIsrCnt = 0;
volatile uint32_t gEpwmIsrCnt2 = 0;
/* EPWM period */
volatile uint32_t gEpwmPrdVal;

/* For handling up-down count alternating period
   when period isn't divisible by 2 */
Bool gToggleEpwmPrd=FALSE;          /* Flag for EPWMs in alternating period mode */
uint8_t gToggleEpwmPrdState=0;      /* Alternating period state: 'Lower' or 'Upper' period written on alternating ISRs */
uint32_t gEpwmPrdValL;              /* 'Lower' EPWM period value written in 'Lower' state */
uint32_t gEpwmPrdValU;              /* 'Upper' EPWM period value written in 'Upper' state */

volatile Bool gRunFlag = TRUE;      /* Flag for continuing to execute test */

volatile uint32_t gPruSddfIrqCnt=0;         /* SDDF PRU FW IRQ count */
volatile uint32_t gRtuSddfIrqCnt=0;         /* SDDF RTU FW IRQ count */

/* Test ICSSG instance ID */
#define TEST_ICSSG_INST_ID              ( CONFIG_PRU_ICSS0 )
/* Test ICSSG slice ID */
#define TEST_ICSSG_SLICE_ID             ( ICSSG_SLICE_ID_0 )
/* Test PRU core instance IDs */
#define TEST_PRU_INST_ID                ( PRUICSS_PRU0 )
#define TEST_RTU_INST_ID                ( PRUICSS_RTU_PRU0 )

/* R5F interrupt settings for ICSSG */
#define ICSSG_PRU_SDDF_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3 )  /* VIM interrupt number */
#define ICSSG_RTU_SDDF_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_4 )  /* VIM interrupt number */
#define ICSSG_TX_PRU_SDDF_INT_NUM       ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_5 )  /* VIM interrupt number */

/* HWI global variables */
static HwiP_Object gIcssgPruSddfHwiObject;  /* ICSSG PRU SDDF FW HWI */
static HwiP_Object gIcssgRtuSddfHwiObject;  /* ICSSG RTU SDDF FW HWI */
static HwiP_Object gEpwm0HwiObject;         /* EPWM0 HWI */

/* Test ICSSG handle */
PRUICSS_Handle gPruIcssHandle;

/* Test SDDF handles */
//Sddf_handle gHPruSddf;
//Sddf_handle gHRtuSddf;
/* Test Sdfm handles */
sdfm_handle gHPruSddf;
sdfm_handle gHRtuSddf;

/* ICSSG_SA_MX_REG:G_MUX_EN */
#if defined(SDDF_HW_EVM_ADAPTER_BP)
#if defined(AM243X_ALV)
#define PRUICSS_G_MUX_EN    ( 0x00 )
#endif
#if defined(AM243X_ALX)
#define PRUICSS_G_MUX_EN    ( 0x80 )
#endif
#endif

#if (DEBUG_LEVEL == DEBUG_BUFFERS_ON)
/* debug, SDDF sample capture buffers */
#define DEBUG_BUFF_SZ     (32768)

__attribute__((section(".gDebugBuff1"))) volatile float gDebugBuff0[DEBUG_BUFF_SZ] = { 0 };
__attribute__((section(".gDebugBuff1"))) volatile float gDebugBuff1[DEBUG_BUFF_SZ] = { 0 };
__attribute__((section(".gDebugBuff2"))) volatile float gDebugBuff2[DEBUG_BUFF_SZ] = { 0 };
__attribute__((section(".gDebugBuff2"))) volatile float gDebugBuff3[DEBUG_BUFF_SZ] = { 0 };
uint32_t gDebugBuffIdx = 0;
#endif

uint32_t gCapSddfChSampsChNum = 0;
#ifdef SDDF_HW_EVM_ADAPTER_BP
#if defined(USE_PURE_OPEN_LOOP)
__attribute__((section(".gCtrlVars"))) float gSddfChOffsets[3] = { 335.125366, -647.005249, -536.584351 };
__attribute__((section(".gCtrlVars"))) float gSddfChOffsets2[3] = { 335.125366, -647.005249, -536.584351 };
#else
__attribute__((section(".gCtrlVars"))) float gSddfChOffsets[3] = { 0.0, 0.0, 0.0 };
__attribute__((section(".gCtrlVars"))) float gSddfChOffsets2[3] = { 0.0, 0.0, 0.0 };
#endif
#endif

/* SDDF output samples, written by PRU */
__attribute__((section(".gSddfChSampsRaw"))) uint32_t gSddfChSamps[ICSSG_NUM_SD_CH] = { 0 };

struct endat_priv *priv;

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
__attribute__((section(".gSharedPrivStep"))) int32_t priv_step;
__attribute__((section(".gSharedPruHandle"))) PRUICSS_Handle gPruIcss0Handle;
PRUICSS_Handle gRtuIcss0Handle;
#define gPruIcssXHandle gPruIcss0Handle

/* ADC offset calculation complete */
uint8_t gSDDFOffsetComplete = FALSE;
uint8_t gSDDFOffsetComplete2 = FALSE;

/* EnDat channel Info, written by PRU cores */
__attribute__((section(".gEnDatChInfo"))) struct endatChRxInfo gEndatChInfo;

#define ENDAT_MULTI_CH0 (1 << 0)
#define ENDAT_MULTI_CH1 (1 << 1)
#define ENDAT_MULTI_CH2 (1 << 2)

#define ENDAT_INPUT_CLOCK_UART_FREQUENCY 192000000
/* use uart clock only to start with */
#define ENDAT_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY

#define ENDAT_RX_SAMPLE_SIZE    7
#define ENDAT_RX_SESQUI_DIV (1 << 15)

static int gEndat_is_multi_ch;
static unsigned char gEndat_multi_ch_mask;
static uint8_t  gEndat_is_load_share_mode;

static unsigned int gEndat_prop_delay[3];
static unsigned int gEndat_prop_delay_max;

__attribute__((section(".gEncChData"))) struct endat_pruss_xchg local_pruss_xchg;

volatile Bool gUpdOutIsr = FALSE;   /* Flag for updating PWM output in ISR */

#if defined(USE_RTLIB_FOC)
DCL_PI gPiSpd;
DCL_PI gPiId;
DCL_PI gPiIq;
#else
arm_pid_instance_f32 gPiId;
arm_pid_instance_f32 gPiIq;
arm_pid_instance_f32 gPiSpd;
arm_pid_instance_f32 gPiPos;
#endif

volatile Bool gConstantsChanged = 0;

/* ICSSG PRU SDDF FW IRQ handler */
static void pruSddfIrqHandler(void *handle);
/* ICSSG RTU SDDF FW IRQ handler */
static void rtuSddfIrqHandler(void *handle);

#if defined(SINGLE_AXLE_USE_M1)
void enable_pwm_buffers(Bool enable)
{
    if(enable) {
        GPIO_pinWriteLow(MTR_1_PWM_EN_BASE_ADDR, MTR_1_PWM_EN_PIN);
        GPIO_pinWriteLow(MTR_2_PWM_EN_BASE_ADDR, MTR_2_PWM_EN_PIN);
        GPIO_pinWriteHigh(BP_MUX_SEL_BASE_ADDR, BP_MUX_SEL_PIN);
    }
    else {
        GPIO_pinWriteHigh(MTR_1_PWM_EN_BASE_ADDR, MTR_1_PWM_EN_PIN);
        GPIO_pinWriteHigh(MTR_2_PWM_EN_BASE_ADDR, MTR_2_PWM_EN_PIN);
        GPIO_pinWriteHigh(BP_MUX_SEL_BASE_ADDR, BP_MUX_SEL_PIN);
    }
}
#endif

uint64_t endat_get_fw_version(void)
{
#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
    return *((unsigned long *)EnDatFirmwareMulti_0 + 2);
#endif

#if (CONFIG_ENDAT0_CHANNEL0) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    return *((unsigned long *)EnDatFirmwareMultiMakeRTU_0 + 2);
#endif

#if (CONFIG_ENDAT0_CHANNEL1) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    return *((unsigned long *)EnDatFirmwareMultiMakePRU_0 + 2);
#endif

#if (CONFIG_ENDAT0_CHANNEL2) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    return *((unsigned long *)EnDatFirmwareMultiMakeTXPRU_0 + 2);
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
    return *((unsigned long *)EnDatFirmware_0 + 2);
#endif

}

static void endat_pruss_init(void)
{
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
     /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);

    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
    if(gEndat_is_load_share_mode)
    {
        PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
        PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_RTUPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
        /*Set in constant table C29 for  tx pru*/
        PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);

    }
     /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_SLICEx));
    if(gEndat_is_load_share_mode)
    {
        PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
        PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    }
    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);


}

void endat_pre_init(void)
{
    endat_pruss_init();
}

uint32_t endat_pruss_load_run_fw(struct endat_priv *priv)
{

    uint32_t status = SystemP_FAILURE;

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU /*enable loadshare mode*/



            status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status=PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) EnDatFirmwareMultiMakeRTU_0,
                                                        sizeof(EnDatFirmwareMultiMakeRTU_0));
            DebugP_assert(0 != status);
            status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
            DebugP_assert(SystemP_SUCCESS == status);


            status=PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx );
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                                      0, (uint32_t *) EnDatFirmwareMultiMakePRU_0,
                                                      sizeof(EnDatFirmwareMultiMakePRU_0));
            DebugP_assert(0 != status);
            status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
            DebugP_assert(SystemP_SUCCESS == status);


           status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
             DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) EnDatFirmwareMultiMakeTXPRU_0,
                                                        sizeof(EnDatFirmwareMultiMakeTXPRU_0));
            DebugP_assert(0 != status);
            status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_TXPRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
            DebugP_assert(SystemP_SUCCESS == status);


        status=endat_wait_initialization(priv, WAIT_5_SECOND, gEndat_multi_ch_mask);


#else

        status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
        DebugP_assert(SystemP_SUCCESS == status);

#if(CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)

            status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                0, (uint32_t *) EnDatFirmwareMulti_0,
                                sizeof(EnDatFirmwareMulti_0));

#else

            status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                0, (uint32_t *) EnDatFirmware_0,
                                sizeof(EnDatFirmware_0));
#endif
        DebugP_assert(0 != status);

        status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
        DebugP_assert(SystemP_SUCCESS == status);

         /*Run firmware */
        status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
        DebugP_assert(SystemP_SUCCESS == status);

        /* check initialization ack from firmware, with a timeout of 5 second */
        status = endat_wait_initialization(priv, WAIT_5_SECOND, gEndat_multi_ch_mask);
#endif

    return status;
}



static int endat_calc_clock(unsigned freq, struct endat_clk_cfg *clk_cfg)
{
    unsigned ns;

    if(freq > 16000000 || (freq > 12000000 && freq < 16000000))
    {
        DebugP_log("\r| ERROR: frequency above 16MHz, between 12 & 16MHz not allowed\n|\n|\n");
        return -1;
    }

    if((freq != 16000000) && (ENDAT_INPUT_CLOCK_FREQUENCY % (freq * 8)))
        DebugP_log("\r| WARNING: exact clock divider is not possible, frequencies set would be tx: %u\trx: %u\n",
                    ENDAT_INPUT_CLOCK_FREQUENCY / (ENDAT_INPUT_CLOCK_FREQUENCY / freq),
                    ENDAT_INPUT_CLOCK_FREQUENCY / (ENDAT_INPUT_CLOCK_FREQUENCY / (freq * 8)));

    ns = 2 * 1000000000 / freq; /* rx arm >= 2 clock */

    /* should be divisible by 5 */
    if(ns % 5)
    {
        ns /= 5, ns += 1,  ns *= 5;
    }

    clk_cfg->tx_div = ENDAT_INPUT_CLOCK_FREQUENCY / freq - 1;
    clk_cfg->rx_div = ENDAT_INPUT_CLOCK_FREQUENCY / (freq * 8) - 1;
    clk_cfg->rx_en_cnt = ns;
    clk_cfg->rx_div_attr = ENDAT_RX_SAMPLE_SIZE;

    if(freq == 16000000)
    {
        clk_cfg->rx_div_attr |= ENDAT_RX_SESQUI_DIV;
    }

    DebugP_logInfo("\r| clock config values - tx_div: %u\trx_div: %u\trx_en_cnt: %u\trx_div_attr: %x\n",
                    clk_cfg->tx_div, clk_cfg->rx_div, clk_cfg->rx_en_cnt, clk_cfg->rx_div_attr);

    return 0;
}

static void endat_handle_prop_delay(struct endat_priv *priv,
                                    unsigned short prop_delay)
{
    if(prop_delay > priv->rx_en_cnt)
    {
        unsigned short dis = (prop_delay - priv->rx_en_cnt) * 2 / priv->rx_en_cnt;

        endat_config_rx_arm_cnt(priv, prop_delay);
        /* propagation delay - 2T */
        endat_config_rx_clock_disable(priv, dis);
    }
    else
    {
        endat_config_rx_arm_cnt(priv, priv->rx_en_cnt);
        endat_config_rx_clock_disable(priv, 0);
    }
}

static void endat_print_encoder_info(struct endat_priv *priv)
{
    DebugP_log("EnDat 2.%d %s encoder\tID: %u %s\tSN: %c %u %c\n\n",
                priv->cmd_set_2_2 ? 2 : 1,
                (priv->type == rotary) ? "rotary" : "linear",
                priv->id.binary, (char *)&priv->id.ascii,
                (char)priv->sn.ascii_msb, priv->sn.binary, (char)priv->sn.ascii_lsb);
    DebugP_log("\rPosition: %d bits ", priv->pos_res);

    if(priv->type == rotary)
    {
        DebugP_log("(singleturn: %d, multiturn: %d) ", priv->single_turn_res,
                    priv->multi_turn_res);
    }

    DebugP_log("[resolution: %d %s]", priv->step,
                priv->type == rotary ? "M/rev" : "nm");
    DebugP_log("\r\n\nPropagation delay: %dns",
                gEndat_prop_delay[priv->channel]);
    DebugP_log("\n\n\n");
}

static unsigned endat_do_sanity_tst_delay(unsigned delay)
{
    /* (unsigned short)~0 is also a multiple of 5 */
    if(delay > (unsigned short)~0)
    {
        DebugP_log("\r| ERROR: delay greater than %uns, enter lesser value\n|\n|\n",
                    (unsigned short)~0);
        return delay;
    }

    if(delay % 5)
    {
        delay += 5, delay /= 5, delay *= 5;
        DebugP_log("\r| WARNING: delay not multiple of 5ns, rounding to %uns\n|\n|\n",
                    delay);
    }

    return delay;
}

static void endat_init_clock(uint32_t frequency, struct endat_priv *priv) {
    struct endat_clk_cfg clk_cfg;
    int32_t j;
    uint32_t delay;
    uint16_t d;

    if(endat_calc_clock(frequency, &clk_cfg) < 0)
    {
        return;
    }

    endat_config_clock(priv, &clk_cfg);

    priv->rx_en_cnt = clk_cfg.rx_en_cnt;

    for(j = 0; j < 3; j++)
    {
        if(gEndat_multi_ch_mask & 1 << j)
        {
            endat_multi_channel_set_cur(priv, j);
            endat_handle_prop_delay(priv, gEndat_prop_delay[priv->channel]);
            d = gEndat_prop_delay_max - gEndat_prop_delay[j];
            endat_config_wire_delay(priv, d);
        }
    }

    /* set tST to 2us if frequency > 1MHz, else turn it off */
    if(frequency >= 1000000)
    {
        frequency = 2000;
    }
    else
    {
        frequency = 0;
    }

    delay = endat_do_sanity_tst_delay(frequency);

    if(delay <= (unsigned short)~0)
    {
        endat_config_tst_delay(priv, (unsigned short) delay);
    }
}
volatile float gAngle = 0;
volatile uint32_t gRevolution = 0;

static void endat_handle_rx(struct endat_priv *priv, int cmd)
{
    union endat_format_data endat_format_data;

    endat_recvd_process(priv, cmd,  &endat_format_data);
    endat_recvd_validate(priv, cmd, &endat_format_data);
    gAngle = (float) endat_format_data.position_addinfo.position.position / priv->step * 360.0;
    gRevolution = endat_format_data.position_addinfo.position.revolution;

    return;
}


volatile uint32_t gPruEncoderIrqCnt=0;      /* EnDat PRU FW IRQ count */
volatile uint32_t gPruEncoderIrqCnt2=0;      /* EnDat PRU FW IRQ count */
volatile uint32_t gPruEncoderIrqCnt2_after=0;      /* EnDat PRU FW IRQ count */
static HwiP_Object gIcssgEncoderHwiObject;  /* ICSSG EnDat PRU FW HWI */

/* ICSSG Interrupt settings */
#define ICSSG_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )  /* interrupt number */

__STATIC_FORCEINLINE void space_vector_f32(
float32_t Ialpha,
float32_t Ibeta,
float32_t * pIa,
float32_t * pIb,
float32_t * pIc)
{
    float32_t tmp1, tmp2, tmp3;
    uint8_t vector = 3;

    tmp1 = Ibeta;
    tmp2 = (Ibeta / 2) + (0.8660254039f * Ialpha);
    tmp3 = tmp2 - tmp1;

    vector = (tmp2 > 0) ? vector - 1: vector;
    vector = (tmp3 > 0) ? vector - 1: vector;
    vector = (tmp1 < 0) ? (7 - vector): vector;

    if(vector == 1 || vector == 4){
        *pIa = tmp2;
        *pIb = tmp1 - tmp3;
        *pIc = -tmp2;
    }
    else if (vector == 2 || vector == 5){
        *pIa = tmp3 + tmp2;
        *pIb = tmp1;
        *pIc = -tmp1;
    }
    else {
        *pIa = tmp3;
        *pIb = -tmp3;
        *pIc = -(tmp1 + tmp2);
    }
}

volatile float gIq = 0.0;
volatile float gIqTarget, gIqSetPoint, gIqRef = 0.0;
__attribute__((section(".gCtrlVars"))) volatile float gIqArray[8] = {0,0.5,1.0,1.5,0,-.5,-1,0};

volatile float gId = 0.0;
volatile float gIdRef = 0.0;

volatile float gSpdTarget, gSpdSetPoint = 0, gSpdRef = 0;
///__attribute__((section(".gCtrlVars"))) volatile float gSpdArray[8] = {0,500,750,-500,-750,0,0,0};
__attribute__((section(".gCtrlVars"))) volatile float gSpdArray[8] = {MAX_SPD_RPM,MAX_SPD_RPM,MAX_SPD_RPM,MAX_SPD_RPM,
                                                                      MAX_SPD_RPM,MAX_SPD_RPM,MAX_SPD_RPM,MAX_SPD_RPM};

volatile float gPosTarget, gPosSetPoint, gPosRef = 0;
__attribute__((section(".gCtrlVars"))) volatile float gPosArray[8] = {180,180,359.5,359.5,0.5,0.5,180,180};

volatile uint8_t count = 0;

volatile uint8_t gInferFromLargestPhases = TRUE;
volatile uint8_t gInferA, gInferB = FALSE;

__attribute__((section(".gCtrlVars"))) float gMechAngleOffset = 0;
float gLastMechTheta = 0;
uint16_t gLastMultiTurn = 0;
uint16_t gStartingMultiTurn = 0;

uint8_t localEnDatGetSingleMulti(float32_t * mechTheta, uint16_t * multiTurn, int chan){
    uint32_t pos, rev;

    if ((chan<0)||(chan>2))
            return -1;

    /* Check CRC from the EnDat PRU */
    if(!(gEndatChInfo.ch[chan].crcStatus & ENDAT_CRC_DATA))
        return -1;

    /* Set CRC to 0 to ensure that the PRU re-populates it every time and we aren't reading old CRC flags*/
    gEndatChInfo.ch[chan].crcStatus = 0;

    /* grab position word 0/1 from the TCM */
    pos = gEndatChInfo.ch[chan].posWord0;
    rev = gEndatChInfo.ch[chan].posWord1;

    /* Reverse the bits since they arrive at the PRU in reverse order */
    asm("rbit %0,%1" : "=r"(pos) : "r"(pos));
    asm("rbit %0,%1" : "=r"(rev) : "r"(rev));

    /* Cobble the multiturn data together from pos0 and pos1 and create singleturn by shifting out F1/F2 and masking the multiturn bits */
    rev = ((rev & 0x07F00000) >> 15) | ((pos & 0xF8000000) >> 27);
    pos = (pos >> 2) & 0x1FFFFFF;

    *mechTheta = (float32_t) pos / (float32_t) priv->step * 360.0;
    *multiTurn = (uint16_t) rev;

    return 0;
}

/* Arm CMSIS 'arm_pid_f32' function modified here to remove the derivative portion since it is unused */
__STATIC_FORCEINLINE float32_t arm_pi_f32(
arm_pid_instance_f32 * S,
float32_t in)
{
  float32_t out;

  /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] */
  out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->state[2]);

  /* Update state */
  S->state[1] = S->state[0];
  S->state[0] = in;
  S->state[2] = out;

  /* return to application */
  return (out);
}

uint32_t gLastEnDatCMP2 = 2897;

float32_t gClarkeAlphaMeasured, gClarkeBetaMeasured;

struct pdo_tx_data {
    uint16_t controlWord;
    int8_t modeOfOperation;
    int32_t targetPosition;
    int32_t targetVelocity;
    int16_t targetTorque;
};

struct pdo_rx_data {
    uint16_t statusWord;
    int8_t modeOfOperationDisplay;
    int32_t actualPosition;
    int32_t actualVelocity;
    int16_t actualTorque;
};

/* Target data from the PLC */
__attribute__((section(".gTxDataSection"))) struct pdo_tx_data gTxData;
/* Actual data to send to the PLC */
__attribute__((section(".gRxDataSection"))) struct pdo_rx_data gRxData;

#ifdef SDDF_HW_EVM_ADAPTER_BP
/* EnDat PRU FW IRQ handler */
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
float32_t myMechTheta = 0.0;
float32_t myMechDelta = 0.0166667; /// for 50Khz PWM
float32_t myMechTheta2 = 0.0;
float32_t myMechDelta2 = 0.0166667; /// for 50Khz PWM
#endif

#ifdef SINGLE_AXLE_USE_M1
uint32_t gEncErrCnt = 0;
uint32_t gPruEncoderIrqCnt_after = 0;
__attribute__((section(".gEtherCatCia402"))) int32_t gCurTargetVelocity[3];
__attribute__((section(".gEtherCatCia402"))) int32_t gCurActualVelocity[3];
__attribute__((section(".critical_code"))) void pruEncoderIrqHandler(void *args)
{
    float32_t fdbkCurPhA, fdbkCurPhB;
    float32_t mechTheta, elecTheta;
    uint16_t multiTurn = 0;
    float32_t elecThetaSinCos[2];
    float32_t parkIdOut, parkIqOut, parkIdMeasured, parkIqMeasured;
    float32_t iparkAlphaOut, iparkBetaOut;
    float32_t spcVectAOut, spcVectBOut, spcVectCOut;
    float dc0, dc1, dc2;        /* EPWM duty cycle values */
    uint16_t cmp0, cmp1, cmp2;  /* EPWM CMPA values */
    float speed, angleDelta;
    float halfPeriod;

    /* Hard-coded for now to trigger the EnDat reading twice, not benchmarked because this should be moved to the PRU in the future */
#if 0
    if(gLastEnDatCMP2 == 2897) {
        HW_WR_REG32(0x3002E098, 5897);
        gLastEnDatCMP2 = 5897;
    }
    else {
        HW_WR_REG32(0x3002E098, 2897);
        gLastEnDatCMP2 = 2897;
    }
#endif
    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcss0Handle, ENDAT_EVENT);

#if defined(USE_PURE_OPEN_LOOP)
    gSDDFOffsetComplete = TRUE;
    gPruEncoderIrqCnt = OFFSET_FINISHED+1;
#endif

    if (gSDDFOffsetComplete) {
        /* debug, increment PRU SDDF IRQ count */
        gPruEncoderIrqCnt++;

        /* Start FOC loop and unlock the rotor */
        if (gPruEncoderIrqCnt > OFFSET_FINISHED) {
#if !defined(USE_PURE_OPEN_LOOP)&&!defined(USE_OPEN_LOOP_WITH_SDDF)
            /* Get the latest mechanical theta and multi-turn position from the encoder */
            if (localEnDatGetSingleMulti(&mechTheta, &multiTurn, 0) != 0)
            {
                gEncErrCnt++;
                return;
            }
            gPruEncoderIrqCnt_after++;
#endif
            /* Use calculated offset from electrical 0, 4 pole pairs */
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            elecTheta = myMechTheta - gMechAngleOffset;
#else
            elecTheta = mechTheta - gMechAngleOffset;
#endif
            if(elecTheta < 0)
                elecTheta += 90;
            elecTheta *= 4.0;

            /* ti_r5fmath_sincos expects input in the range of 0-2pi radians so normalize here to 0-360 degrees */
            while (elecTheta > 360.0)
                elecTheta -= 360.0;

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            angleDelta = myMechTheta - gLastMechTheta;
#else
            angleDelta = mechTheta - gLastMechTheta;
#endif
            /* Max angle change per period @ 3200RPMs with 50KHz period should be 0.384 degrees, outside that is rollover
             * Use 5 as the bound to provide plenty of room for max acceleration cases */
            if (angleDelta < -50)
                angleDelta += 360;
            else if (angleDelta > 50)
                angleDelta -= 360;

            /* Compute instantaneous RPMs using the change in angle and the PWM frequency */
            speed = (angleDelta / 360.0) / (ISR_PRD_IN_MINUTES);

#if (PRECOMPUTE_LEVEL == NO_PRECOMPUTE)
            /* Try to avoid the INA240 switching noise by selecting the 2 phases with the largest duty cycles */
            if (gInferA) {
                fdbkCurPhB = (-((float)gSddfChSamps[1] - gSddfChOffsets[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
                fdbkCurPhA = -fdbkCurPhB - ((-((float)gSddfChSamps[2] - gSddfChOffsets[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);

            }
            else if (gInferB){
                fdbkCurPhA = (-((float)gSddfChSamps[0] - gSddfChOffsets[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
                fdbkCurPhB = -fdbkCurPhA - ((-((float)gSddfChSamps[2] - gSddfChOffsets[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);
            }
            else {
                fdbkCurPhA = (-((float)gSddfChSamps[0] - gSddfChOffsets[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
                fdbkCurPhB = (-((float)gSddfChSamps[1] - gSddfChOffsets[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
            }

#if defined(USE_RTLIB_FOC)
            CLARKE_run_twoInput(fdbkCurPhA, fdbkCurPhB, &gClarkeAlphaMeasured, &gClarkeBetaMeasured);
#else
            /* Clarke transform */
            arm_clarke_f32(fdbkCurPhA, fdbkCurPhB, &gClarkeAlphaMeasured, &gClarkeBetaMeasured);
#endif
#endif
            /* Calculate sine and cosine of the electrical angle (elecTheta converted to radians) */
            ti_r5fmath_sincos((elecTheta * TI_R5FMATH_PIOVER180), ti_r5fmath_sincosPIconst, ti_r5fmath_sincosCoef, elecThetaSinCos);
#if defined(USE_RTLIB_FOC)
            /* Park transform */
            PARK_run(elecThetaSinCos[0], elecThetaSinCos[1], gClarkeAlphaMeasured, gClarkeBetaMeasured, &parkIdMeasured, &parkIqMeasured);
#else
            /* Park transform */
            arm_park_f32(gClarkeAlphaMeasured, gClarkeBetaMeasured, &parkIdMeasured, &parkIqMeasured, elecThetaSinCos[0], elecThetaSinCos[1]);
#endif

// Open loop Iq
#if (BUILDLEVEL == OPEN_LOOP_IQ_ID)
            if(gIq < IQ_TESTING)
                gIq += 0.01;

            parkIdOut = 0.0;
            parkIqOut = gIq;

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            myMechTheta += myMechDelta;

            if (myMechTheta>359.9999)
                myMechTheta = 0.0;
#endif
#endif
//Closed loop Iq and Id tuning
#if (BUILDLEVEL == CLOSED_LOOP_IQ_ID)
            gIqTarget = 0.5; ///gIqArray[count];
            gIqSetPoint = gIqTarget;

            if((gPruEncoderIrqCnt - OFFSET_FINISHED) % CYCLES_PER_TARGET == 0){
                count++;
                if(count == 8)
                    count = 0;
            }

            parkIqOut = arm_pi_f32(&gPiIq, gIqSetPoint - parkIqMeasured);
            parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
            if (parkIqOut>IQ_TESTING) parkIqOut = IQ_TESTING;
            if (parkIqOut<-1.0*IQ_TESTING) parkIqOut = -1.0*IQ_TESTING;
            if (parkIdOut>ID_TESTING) parkIdOut = ID_TESTING;
            if (parkIdOut<-1.0*ID_TESTING) parkIdOut = -1.0*ID_TESTING;
#endif
//Closed loop speed
#if (BUILDLEVEL == CLOSED_LOOP_SPEED)
            gSpdTarget = gCurTargetVelocity[0];
            if(gSpdSetPoint < gSpdTarget)
                gSpdSetPoint += MAX_SPD_CHANGE;
            else if (gSpdSetPoint > gSpdTarget)
                gSpdSetPoint -= MAX_SPD_CHANGE;

            // update the gCurActualVelocity[0] based on the gSpdSetPoint
            gCurActualVelocity[0] = (uint32_t)gSpdSetPoint;
#if defined(USE_RTLIB_FOC)
            gIqRef = -DCL_runPIParallel(&gPiSpd, gSpdSetPoint, speed);
            parkIqOut = DCL_runPIParallel(&gPiIq, gIqRef, parkIqMeasured);
            parkIdOut = DCL_runPIParallel(&gPiId, 0.0f, parkIdMeasured);
#else
            gIqRef = arm_pi_f32(&gPiSpd, gSpdSetPoint - speed);
            parkIqOut = arm_pi_f32(&gPiIq, gIqRef - parkIqMeasured);
            parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
            if (parkIqOut>IQ_TESTING) parkIqOut = IQ_TESTING;
            if (parkIqOut<-1.0*IQ_TESTING) parkIqOut = -1.0*IQ_TESTING;
            if (parkIdOut>ID_TESTING) parkIdOut = ID_TESTING;
            if (parkIdOut<-1.0*ID_TESTING) parkIdOut = -1.0*ID_TESTING;
#endif
#endif

//Closed loop CiA402 Data
#if (BUILDLEVEL == CLOSED_LOOP_CIA402)
            switch(gTxData.modeOfOperation){
            case CYCLIC_SYNC_POSITION_MODE:
                gRxData.actualPosition = (int32_t) (mechTheta * 1000.0);
                /* Create a 0-1080 degree reference frame using the multiturn data
                 * 0-360 is overflow with a counter-clockwise turn
                 * 360-720 is no overflow condition (same revolution as start point)
                 * 720-1080 is overflow with a clockwise turn
                 */
                if (multiTurn == gStartingMultiTurn) /* no overflow, move to center of the reference frame */
                    relativeMechTheta = mechTheta + 360;
                else if ((multiTurn == (gStartingMultiTurn + 1)) || ((gStartingMultiTurn > 1) && (multiTurn == 0))) /* clockwise overflow */
                    relativeMechTheta = mechTheta + 720;
                else /* counter-clockwise overflow, leave in the lower end of the reference frame */
                    relativeMechTheta = mechTheta;

                gPosTarget = (float) gTxData.targetPosition / 1000.0;
                if(gPosSetPoint < gPosTarget) {
                    gPosSetPoint += MAX_POS_CHANGE;
                    if (gPosSetPoint > gPosTarget) /* Increment went over the request */
                        gPosSetPoint = gPosTarget;
                }
                else if (gPosSetPoint > gPosTarget) {
                    gPosSetPoint -= MAX_POS_CHANGE;
                    if (gPosSetPoint < gPosTarget) /* Decrement went below the request */
                        gPosSetPoint = gPosTarget;
                }

                /* Move the setpoint to the 360-720 reference frame and compare*/
                gSpdRef = arm_pi_f32(&gPiPos, (gPosSetPoint + 360) - relativeMechTheta);
                gIqRef = arm_pi_f32(&gPiSpd, gSpdRef - speed);
                parkIqOut = arm_pi_f32(&gPiIq, gIqRef - parkIqMeasured);
                parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
                break;
            case CYCLIC_SYNC_VELOCITY_MODE:
                gRxData.actualVelocity = (int32_t) (speed * 1000.0);
                gSpdTarget = (float) gTxData.targetVelocity / 1000.0;
                if(gSpdSetPoint < gSpdTarget)
                    gSpdSetPoint += MAX_SPD_CHANGE;
                else if (gSpdSetPoint > gSpdTarget)
                    gSpdSetPoint -= MAX_SPD_CHANGE;

                gIqRef = arm_pi_f32(&gPiSpd, gSpdSetPoint - speed);
                parkIqOut = arm_pi_f32(&gPiIq, gIqRef - parkIqMeasured);
                parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
                break;
            case CYCLIC_SYNC_TORQUE_MODE:
                ///gRxData.actualTorque = (int16_t) (parkIqOut * 1000.0);
                gIqTarget = (float) gTxData.targetTorque / 1000.0;
                gIqSetPoint = gIqTarget;

                parkIqOut = arm_pi_f32(&gPiIq, gIqSetPoint - parkIqMeasured);
                parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
                gRxData.actualTorque = (int16_t) (parkIqOut * 1000.0);
                break;
            default:
                break;
            }
#endif

#if defined(USE_RTLIB_FOC)
            /* Inverse Park transform */
            IPARK_run(elecThetaSinCos[0], elecThetaSinCos[1], parkIdOut, parkIqOut, &iparkAlphaOut, &iparkBetaOut);
            /* Space Vector Generation */
            SVGEN_runCom(1.0f, iparkAlphaOut, iparkBetaOut, &spcVectAOut, &spcVectBOut, &spcVectCOut);
#else
            /* Inverse Park transform */
            arm_inv_park_f32(parkIdOut, parkIqOut, &iparkAlphaOut, &iparkBetaOut, elecThetaSinCos[0], elecThetaSinCos[1]);
            /* Space Vector Generation */
            space_vector_f32(iparkAlphaOut, iparkBetaOut, &spcVectAOut, &spcVectBOut, &spcVectCOut);
#endif

            /* Write next CMPA values. Swap cmp0 and cmp2 because the HW connect PWM0 to Phase C and PWM2 to Phase A */
            halfPeriod = (float)gEpwmPrdVal / 2.0;
#if defined(SINGLE_AXLE_USE_M1)
#if defined(USE_RTLIB_FOC)
            writeCmpA(gEpwm0BaseAddr, (uint16_t) ((1 + spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm1BaseAddr, (uint16_t) ((1 + spcVectBOut) * halfPeriod));
            writeCmpA(gEpwm2BaseAddr, (uint16_t) ((1 + spcVectAOut) * halfPeriod));
#else
            writeCmpA(gEpwm0BaseAddr, (uint16_t) ((1 - spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm1BaseAddr, (uint16_t) ((1 - spcVectBOut) * halfPeriod));
            writeCmpA(gEpwm2BaseAddr, (uint16_t) ((1 - spcVectAOut) * halfPeriod));
#endif
#endif
#if defined(SINGLE_AXLE_USE_M2)
            writeCmpA(gEpwm0BaseAddr2A, (uint16_t) ((1 - spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm0BaseAddr2B, (uint16_t) ((1 - spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm1BaseAddr2, (uint16_t) ((1 - spcVectBOut) * halfPeriod));
            writeCmpA(gEpwm2BaseAddr2, (uint16_t) ((1 - spcVectAOut) * halfPeriod));
#endif
            asm("    dsb");

            /* Determine if Phase A or Phase B currents need to be inferred in the next cycle based on the two largest phases */
            gInferA = FALSE;
            gInferB = FALSE;

            if (spcVectCOut > spcVectAOut || spcVectCOut > spcVectBOut) {   /* If C is larger than either A or B */
                if (spcVectAOut > spcVectBOut) {                            /* A and C are the largest, infer B in the next cycle */
                    gInferB = TRUE;
                }
                else {                                                      /* B and C are the largest, infer A in the next cycle */
                    gInferA = TRUE;
                }
            }

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            gLastMechTheta = myMechTheta;
            gLastMultiTurn = 0;
#else
            gLastMechTheta = mechTheta;
            gLastMultiTurn = multiTurn;
#endif

#if (DEBUG_LEVEL == DEBUG_BUFFERS_ON)
#if (BUILDLEVEL == OPEN_LOOP_IQ_ID)
        gDebugBuff1[gDebugBuffIdx] = spcVectBOut;
        ///gDebugBuff2[gDebugBuffIdx] = (uint32_t)spcVectAOut;
        ///gDebugBuff3[gDebugBuffIdx] = (uint32_t)gSddfChSamps[0];
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
        gDebugBuff3[gDebugBuffIdx] = (uint32_t)gSddfChSamps[3]; ///myMechTheta;
#else
        ///gDebugBuff3[gDebugBuffIdx] = mechTheta;
#endif
#endif
#if (BUILDLEVEL == CLOSED_LOOP_IQ_ID)
            gDebugBuff1[gDebugBuffIdx] = spcVectBOut;
            ///gDebugBuff2[gDebugBuffIdx] = parkIqMeasured;
            ///gDebugBuff3[gDebugBuffIdx] = parkIdMeasured;
#endif
#if (BUILDLEVEL == CLOSED_LOOP_SPEED)
            ///gDebugBuff1[gDebugBuffIdx] = spcVectBOut; ///speed; ///gClarkeBetaMeasured;
            ///gDebugBuff2[gDebugBuffIdx] = spcVectAOut; ///parkIdOut; ///parkIqMeasured;
            gDebugBuff0[gDebugBuffIdx] = (uint32_t)gSddfChSamps[0];
            gDebugBuff1[gDebugBuffIdx] = (uint32_t)mechTheta;
#endif
#endif

#if (PID_TUNE_LEVEL == NO_TUNING) /* No tuning */
#if (DEBUG_LEVEL == DEBUG_BUFFERS_ON)
#if (DEBUG_WRAP_TYPE == DEBUG_BUFFER_WRAP) /* No tuning, wrap buffer */
            /* debug, update capture buffers index */
            gDebugBuffIdx++;
            if (gDebugBuffIdx >= DEBUG_BUFF_SZ) {
                gDebugBuffIdx = 0;
            }
#elif (DEBUG_WRAP_TYPE == DEBUG_BUFFER_SINGLE) /* No tuning, single buffer store */

            /* debug, update capture buffers index */
            gDebugBuffIdx++;
            if (gDebugBuffIdx >= DEBUG_BUFF_SZ) {
                gDebugBuffIdx = DEBUG_BUFF_SZ - 1;
            }
#endif
#endif
#endif
        }
        /* Calculate mechanical/electrical angle offset */
        else {
            /* Get the latest mechanical theta from the encoder */
#if defined(SINGLE_AXLE_USE_M1)
            if (localEnDatGetSingleMulti(&mechTheta, &multiTurn, 0) != 0)
            {
                gEncErrCnt++;
                return;
            }
            gPruEncoderIrqCnt_after++;
#endif
#if defined(SINGLE_AXLE_USE_M2)
            localEnDatGetSingleMulti(&mechTheta, &multiTurn, 2);
#endif
            if (gPruEncoderIrqCnt > SETTLING_COUNT)
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
                gMechAngleOffset += myMechTheta;
#else
                gMechAngleOffset += mechTheta;
#endif
            if (gPruEncoderIrqCnt == OFFSET_FINISHED) {
                gMechAngleOffset /= SETTLING_COUNT;
                while (gMechAngleOffset > 90)
                    gMechAngleOffset -= 90;

                /* Electrical Angle offset complete, turn off all phases */
                computeCmpx(0.001, gEpwmPrdVal, &dc0, &cmp0);
                computeCmpx(-0.001, gEpwmPrdVal, &dc1, &cmp1);
                computeCmpx(-0.001, gEpwmPrdVal, &dc2, &cmp2);

                /* Write next CMPA values. Swap cmp0 and cmp2 because the HW connect PWM0 to Phase C and PWM2 to Phase A */
#if defined(SINGLE_AXLE_USE_M1)
                writeCmpA(gEpwm0BaseAddr, cmp2);
                writeCmpA(gEpwm1BaseAddr, cmp1);
                writeCmpA(gEpwm2BaseAddr, cmp0);
#endif
#if defined(SINGLE_AXLE_USE_M2)
                writeCmpA(gEpwm0BaseAddr2A, cmp2);
                writeCmpA(gEpwm0BaseAddr2B, cmp2);
                writeCmpA(gEpwm1BaseAddr2, cmp1);
                writeCmpA(gEpwm2BaseAddr2, cmp0);
#endif

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
                gLastMechTheta = myMechTheta;
                gLastMultiTurn = 0;
                gStartingMultiTurn = 0;

                /* Set initial position information */
                gPosSetPoint = myMechTheta;
                gTxData.targetPosition = myMechTheta;
#else
                gLastMechTheta = mechTheta;
                gLastMultiTurn = multiTurn;
                gStartingMultiTurn = multiTurn;

                /* Set initial position information */
                gPosSetPoint = mechTheta;
                gTxData.targetPosition = mechTheta;
#endif
            }
        }
    }
}
#endif

#ifdef SINGLE_AXLE_USE_M2
uint32_t gEncErrCnt2 = 0;
__attribute__((section(".critical_code"))) void pruEncoderIrqHandler2(void *args)
{
    float32_t fdbkCurPhA, fdbkCurPhB;
    float32_t mechTheta, elecTheta, relativeMechTheta = 0;
    uint16_t multiTurn = 0;
    float32_t elecThetaSinCos[2];
    float32_t parkIdOut, parkIqOut, parkIdMeasured, parkIqMeasured;
    float32_t iparkAlphaOut, iparkBetaOut;
    float32_t spcVectAOut, spcVectBOut, spcVectCOut;
    float dc0, dc1, dc2;        /* EPWM duty cycle values */
    uint16_t cmp0, cmp1, cmp2;  /* EPWM CMPA values */
    float speed, angleDelta;
    float halfPeriod;

    /* Hard-coded for now to trigger the EnDat reading twice, not benchmarked because this should be moved to the PRU in the future */
#if 0
    if(gLastEnDatCMP2 == 2897) {
        HW_WR_REG32(0x3002E098, 5897);
        gLastEnDatCMP2 = 5897;
    }
    else {
        HW_WR_REG32(0x3002E098, 2897);
        gLastEnDatCMP2 = 2897;
    }
#endif
    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcss0Handle, ENDAT_EVENT+2);

#if defined(USE_PURE_OPEN_LOOP)
    gSDDFOffsetComplete2 = TRUE;
    gPruEncoderIrqCnt2 = OFFSET_FINISHED+1;
#endif

    if (gSDDFOffsetComplete2) {
        /* debug, increment PRU SDDF IRQ count */
        gPruEncoderIrqCnt2++;

        /* Start FOC loop and unlock the rotor */
        if (gPruEncoderIrqCnt2 > OFFSET_FINISHED) {
#if !defined(USE_PURE_OPEN_LOOP)&&!defined(USE_OPEN_LOOP_WITH_SDDF)
            /* Get the latest mechanical theta and multi-turn position from the encoder */
            if (localEnDatGetSingleMulti(&mechTheta, &multiTurn, 2) != 0)
            {
                gEncErrCnt2++;
                return;
            }
            gPruEncoderIrqCnt2_after++;
#endif
            /* Use calculated offset from electrical 0, 4 pole pairs */
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            elecTheta = myMechTheta2 - gMechAngleOffset;
#else
            elecTheta = mechTheta - gMechAngleOffset;
#endif
            if(elecTheta < 0)
                elecTheta += 90;
            elecTheta *= 4.0;

            /* ti_r5fmath_sincos expects input in the range of 0-2pi radians so normalize here to 0-360 degrees */
            while (elecTheta > 360.0)
                elecTheta -= 360.0;

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            angleDelta = myMechTheta2 - gLastMechTheta;
#else
            angleDelta = mechTheta - gLastMechTheta;
#endif
            /* Max angle change per period @ 3200RPMs with 50KHz period should be 0.384 degrees, outside that is rollover
             * Use 5 as the bound to provide plenty of room for max acceleration cases */
            if (angleDelta < -50)
                angleDelta += 360;
            else if (angleDelta > 50)
                angleDelta -= 360;

            /* Compute instantaneous RPMs using the change in angle and the PWM frequency */
            speed = (angleDelta / 360.0) / (ISR_PRD_IN_MINUTES);

#if (PRECOMPUTE_LEVEL == NO_PRECOMPUTE)
            /* Try to avoid the INA240 switching noise by selecting the 2 phases with the largest duty cycles */
            if (gInferA) {
                fdbkCurPhB = (-((float)gSddfChSamps[4] - gSddfChOffsets2[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
                fdbkCurPhA = -fdbkCurPhB - ((-((float)gSddfChSamps[5] - gSddfChOffsets2[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);

            }
            else if (gInferB){
                fdbkCurPhA = (-((float)gSddfChSamps[3] - gSddfChOffsets2[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
                fdbkCurPhB = -fdbkCurPhA - ((-((float)gSddfChSamps[5] - gSddfChOffsets2[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);
            }
            else {
                fdbkCurPhA = (-((float)gSddfChSamps[3] - gSddfChOffsets2[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
                fdbkCurPhB = (-((float)gSddfChSamps[4] - gSddfChOffsets2[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
            }

            /* Clarke transform */
            arm_clarke_f32(fdbkCurPhA, fdbkCurPhB, &gClarkeAlphaMeasured, &gClarkeBetaMeasured);
#endif
            /* Calculate sine and cosine of the electrical angle (elecTheta converted to radians) */
            ti_r5fmath_sincos((elecTheta * TI_R5FMATH_PIOVER180), ti_r5fmath_sincosPIconst, ti_r5fmath_sincosCoef, elecThetaSinCos);
            /* Park transform */
            arm_park_f32(gClarkeAlphaMeasured, gClarkeBetaMeasured, &parkIdMeasured, &parkIqMeasured, elecThetaSinCos[0], elecThetaSinCos[1]);

// Open loop Iq
#if (BUILDLEVEL == OPEN_LOOP_IQ_ID)
            if(gIq < IQ_TESTING)
                gIq += 0.01;

            parkIdOut = 0.0;
            parkIqOut = gIq;

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            myMechTheta2 += myMechDelta2;

            if (myMechTheta2>359.9999)
                myMechTheta2 = 0.0;

#endif
#endif
//Closed loop Iq and Id tuning
#if (BUILDLEVEL == CLOSED_LOOP_IQ_ID)
            gIqTarget = 0.5; ///gIqArray[count];
            gIqSetPoint = gIqTarget;

            ///if((gPruEncoderIrqCnt - OFFSET_FINISHED) % CYCLES_PER_TARGET == 0){
            ///    count++;
            ///    if(count == 8)
            ///        count = 0;
            ///}

            parkIqOut = arm_pi_f32(&gPiIq, gIqSetPoint - parkIqMeasured);
            parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
            if (parkIqOut>IQ_TESTING) parkIqOut = IQ_TESTING;
            if (parkIqOut<-1.0*IQ_TESTING) parkIqOut = -1.0*IQ_TESTING;
            if (parkIdOut>ID_TESTING) parkIdOut = ID_TESTING;
            if (parkIdOut<-1.0*ID_TESTING) parkIdOut = -1.0*ID_TESTING;
#endif
//Closed loop speed
#if (BUILDLEVEL == CLOSED_LOOP_SPEED)
            gSpdTarget = gSpdArray[0];
            if(gSpdSetPoint < gSpdTarget)
                gSpdSetPoint += MAX_SPD_CHANGE;
            else if (gSpdSetPoint > gSpdTarget)
                gSpdSetPoint -= MAX_SPD_CHANGE;

            ///if((gPruEncoderIrqCnt - OFFSET_FINISHED) % CYCLES_PER_TARGET == 0){
            ///    count++;
            ///    if(count == 8)
            ///        count = 0;
            ///}

            gIqRef = arm_pi_f32(&gPiSpd, gSpdSetPoint - speed);
            parkIqOut = arm_pi_f32(&gPiIq, gIqRef - parkIqMeasured);
            parkIdOut = arm_pi_f32(&gPiId, 0.0 - parkIdMeasured);
            if (parkIqOut>IQ_TESTING) parkIqOut = IQ_TESTING;
            if (parkIqOut<-1.0*IQ_TESTING) parkIqOut = -1.0*IQ_TESTING;
            if (parkIdOut>ID_TESTING) parkIdOut = ID_TESTING;
            if (parkIdOut<-1.0*ID_TESTING) parkIdOut = -1.0*ID_TESTING;
#endif
            /* Inverse Park transform */
            arm_inv_park_f32(parkIdOut, parkIqOut, &iparkAlphaOut, &iparkBetaOut, elecThetaSinCos[0], elecThetaSinCos[1]);
            /* Space Vector Generation */
            space_vector_f32(iparkAlphaOut, iparkBetaOut, &spcVectAOut, &spcVectBOut, &spcVectCOut);

            /* Write next CMPA values. Swap cmp0 and cmp2 because the HW connect PWM0 to Phase C and PWM2 to Phase A */
            halfPeriod = (float)gEpwmPrdVal / 2.0;
#if defined(SINGLE_AXLE_USE_M1)
            writeCmpA(gEpwm0BaseAddr, (uint16_t) ((1 - spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm1BaseAddr, (uint16_t) ((1 - spcVectBOut) * halfPeriod));
            writeCmpA(gEpwm2BaseAddr, (uint16_t) ((1 - spcVectAOut) * halfPeriod));
#endif
#if defined(SINGLE_AXLE_USE_M2)
            writeCmpA(gEpwm0BaseAddr2A, (uint16_t) ((1 - spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm0BaseAddr2B, (uint16_t) ((1 - spcVectCOut) * halfPeriod));
            writeCmpA(gEpwm1BaseAddr2, (uint16_t) ((1 - spcVectBOut) * halfPeriod));
            writeCmpA(gEpwm2BaseAddr2, (uint16_t) ((1 - spcVectAOut) * halfPeriod));
#endif
            asm("    dsb");

            /* Determine if Phase A or Phase B currents need to be inferred in the next cycle based on the two largest phases */
            gInferA = FALSE;
            gInferB = FALSE;

            if (spcVectCOut > spcVectAOut || spcVectCOut > spcVectBOut) {   /* If C is larger than either A or B */
                if (spcVectAOut > spcVectBOut) {                            /* A and C are the largest, infer B in the next cycle */
                    gInferB = TRUE;
                }
                else {                                                      /* B and C are the largest, infer A in the next cycle */
                    gInferA = TRUE;
                }
            }

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
            gLastMechTheta = myMechTheta2;
            gLastMultiTurn = 0;
#else
            gLastMechTheta = mechTheta;
            gLastMultiTurn = multiTurn;
#endif

#if (DEBUG_LEVEL == DEBUG_BUFFERS_ON)
#if (BUILDLEVEL == OPEN_LOOP_IQ_ID)
        ///gDebugBuff1[gDebugBuffIdx] = (uint32_t)gSddfChSamps[4];
        gDebugBuff2[gDebugBuffIdx] = (uint32_t)gSddfChSamps[3];
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
        gDebugBuff3[gDebugBuffIdx] = (uint32_t)gSddfChSamps[3]; ///myMechTheta;
#else
        gDebugBuff3[gDebugBuffIdx] = mechTheta;
#endif
#endif
#if (BUILDLEVEL == CLOSED_LOOP_IQ_ID)
            gDebugBuff1[gDebugBuffIdx] = gClarkeBetaMeasured;
            gDebugBuff2[gDebugBuffIdx] = parkIqMeasured;
            gDebugBuff3[gDebugBuffIdx] = parkIdMeasured;
#endif
#if (BUILDLEVEL == CLOSED_LOOP_SPEED)
            ///gDebugBuff1[gDebugBuffIdx] = parkIqOut; ///gClarkeBetaMeasured;
            ///gDebugBuff2[gDebugBuffIdx] = parkIdOut; ///parkIqMeasured;
            gDebugBuff1[gDebugBuffIdx] = (uint32_t)gSddfChSamps[4];
            gDebugBuff2[gDebugBuffIdx] = (uint32_t)gSddfChSamps[5];

            gDebugBuff3[gDebugBuffIdx] = parkIdMeasured;
#endif
#endif

#if (PID_TUNE_LEVEL == NO_TUNING) /* No tuning */
#if (DEBUG_LEVEL == DEBUG_BUFFERS_ON)
#if (DEBUG_WRAP_TYPE == DEBUG_BUFFER_WRAP) /* No tuning, wrap buffer */
            /* debug, update capture buffers index */
            gDebugBuffIdx++;
            if (gDebugBuffIdx >= DEBUG_BUFF_SZ) {
                gDebugBuffIdx = 0;
            }
#elif (DEBUG_WRAP_TYPE == DEBUG_BUFFER_SINGLE) /* No tuning, single buffer store */

            /* debug, update capture buffers index */
            gDebugBuffIdx++;
            if (gDebugBuffIdx >= DEBUG_BUFF_SZ) {
                gDebugBuffIdx = DEBUG_BUFF_SZ - 1;
            }
#endif
#endif
#endif
        }
        /* Calculate mechanical/electrical angle offset */
        else {
            /* Get the latest mechanical theta from the encoder */
#ifdef SINGLE_AXLE_USE_M1
            localEnDatGetSingleMulti(&mechTheta, &multiTurn, 0);
#endif
#ifdef SINGLE_AXLE_USE_M2
            if (localEnDatGetSingleMulti(&mechTheta, &multiTurn, 2) != 0)
            {
                gEncErrCnt2++;
                return;
            }
            gPruEncoderIrqCnt2_after++;
#endif
            if (gPruEncoderIrqCnt2 > SETTLING_COUNT)
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
                gMechAngleOffset += myMechTheta;
#else
                gMechAngleOffset += mechTheta;
#endif
            if (gPruEncoderIrqCnt2 == OFFSET_FINISHED) {
                gMechAngleOffset /= SETTLING_COUNT;
                while (gMechAngleOffset > 90)
                    gMechAngleOffset -= 90;

                /* Electrical Angle offset complete, turn off all phases */
                computeCmpx(0.0001, gEpwmPrdVal, &dc0, &cmp0);
                computeCmpx(-0.0001, gEpwmPrdVal, &dc1, &cmp1);
                computeCmpx(-0.0001, gEpwmPrdVal, &dc2, &cmp2);

                /* Write next CMPA values. Swap cmp0 and cmp2 because the HW connect PWM0 to Phase C and PWM2 to Phase A */
#if defined(SINGLE_AXLE_USE_M1)
                writeCmpA(gEpwm0BaseAddr, cmp2);
                writeCmpA(gEpwm1BaseAddr, cmp1);
                writeCmpA(gEpwm2BaseAddr, cmp0);
#endif
#if defined(SINGLE_AXLE_USE_M2)
                writeCmpA(gEpwm0BaseAddr2A, cmp2);
                writeCmpA(gEpwm0BaseAddr2B, cmp2);
                writeCmpA(gEpwm1BaseAddr2, cmp1);
                writeCmpA(gEpwm2BaseAddr2, cmp0);
#endif

#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
                gLastMechTheta = myMechTheta;
                gLastMultiTurn = 0;
                gStartingMultiTurn = 0;

                /* Set initial position information */
                gPosSetPoint = myMechTheta;
                gTxData.targetPosition = myMechTheta;
#else
                gLastMechTheta = mechTheta;
                gLastMultiTurn = multiTurn;
                gStartingMultiTurn = multiTurn;

                /* Set initial position information */
                gPosSetPoint = mechTheta;
                gTxData.targetPosition = mechTheta;
#endif
            }
        }
    }
}
#endif

#endif

#if defined(SINGLE_AXLE_USE_M1)
void init_encoder(){
    int i, j;
    HwiP_Params hwiPrms;
    int32_t status;

    uint64_t icssgclk;

    void *pruss_cfg;
    void *pruss_iep;

#ifdef SDDF_HW_EVM_ADAPTER_BP
    /* Configure g_mux_en to 0 in ICSSG_SA_MX_REG Register. */
    ///HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x00));
    HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x80));
#endif
    i = endat_get_fw_version();

    // set initial velocity for 1st motor
    gCurTargetVelocity[0] = MAX_SPD_RPM;
    gCurActualVelocity[0] = MAX_SPD_RPM;

    DebugP_log("\n\n\n");
    DebugP_log("EnDat firmware \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

#ifdef SINGLE_AXLE_USE_M1
    /* Register & enable ICSSG EnDat PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM;
    hwiPrms.callback    = &pruEncoderIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoderHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#ifdef SINGLE_AXLE_USE_M2
    /* Register & enable ICSSG EnDat PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM + 2;
    hwiPrms.callback    = &pruEncoderIrqHandler2;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoderHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));

    /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRUx));

    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);

    /* Select multi-channel */
    gEndat_is_multi_ch = 1;
    /* Select the EnDat channel */
#ifdef SINGLE_AXLE_USE_M1
#ifdef DUAL_AXIS_USE_M1_M2
    /* for dual motor drive mode */
    gEndat_multi_ch_mask = ENDAT_MULTI_CH0|ENDAT_MULTI_CH2;
#else
    gEndat_multi_ch_mask = ENDAT_MULTI_CH0;
#endif
#endif

#ifdef SINGLE_AXLE_USE_M2
    gEndat_multi_ch_mask = ENDAT_MULTI_CH2;
#endif




    gEndat_is_multi_ch = CONFIG_ENDAT0_MODE & 1;
    gEndat_is_load_share_mode = CONFIG_ENDAT0_MODE & 2;
    endat_pre_init();

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        gEndat_multi_ch_mask=(CONFIG_ENDAT0_CHANNEL0<<0|CONFIG_ENDAT0_CHANNEL1<<1|CONFIG_ENDAT0_CHANNEL2<<2);

        DebugP_log("\r\nchannels %s %s %s selected\n",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH0 ? "0" : "",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH1 ? "1" : "",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH2 ? "2" : "");

        if(!gEndat_multi_ch_mask)
        {
            DebugP_log("\r\nERROR: please select channels to be used in multi channel configuration -\n\n");
            DebugP_log("\rexit %s as no channel selected in multichannel configuration\n",
                          __func__);
            return;
        }
    }
    else
    {

        i = CONFIG_ENDAT0_CHANNEL0 & 0;

        i += CONFIG_ENDAT0_CHANNEL1;

        i += CONFIG_ENDAT0_CHANNEL2<<1;

        if(i < 0 || i > 2)
        {
           DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
           i = 0;
        }
    }

    pruss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    pruss_iep  = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);

    /*Translate the TCM local view addr to globel view addr */
    uint64_t gEndatChInfoGlobalAddr = CPU0_BTCM_SOCVIEW((uint64_t)&gEndatChInfo);

    priv = endat_init((struct endat_pruss_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcssXHandle->hwAttrs))->pru1DramBase, &gEndatChInfo, gEndatChInfoGlobalAddr, pruss_cfg, pruss_iep, PRUICSS_SLICEx);

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        endat_config_multi_channel_mask(priv, gEndat_multi_ch_mask, gEndat_is_load_share_mode);
    }
    else
    {
        endat_config_channel(priv, i);
    }

    endat_config_host_trigger(priv);

    /* Read the ICSSG configured clock frequency. */
    if(gPruIcssXHandle->hwAttrs->instance)
    {
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK, &icssgclk);
    }
    else
    {
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, &icssgclk);
    }

    /* Configure Delays based on the ICSSG frequency*/
    /* Count = ((required delay * icssgclk)/1000) */
    priv->pruss_xchg->endat_delay_125ns = ((icssgclk*125)/1000000000);
    priv->pruss_xchg->endat_delay_51us = ((icssgclk*51)/1000000 );
    priv->pruss_xchg->endat_delay_5us = ((icssgclk*5)/1000000);
    priv->pruss_xchg->endat_delay_1ms = ((icssgclk/1000) * 1);
    priv->pruss_xchg->endat_delay_2ms = ((icssgclk/1000) * 2);
    priv->pruss_xchg->endat_delay_12ms = ((icssgclk/1000) * 12);
    priv->pruss_xchg->endat_delay_50ms = ((icssgclk/1000) * 50);
    priv->pruss_xchg->endat_delay_380ms = ((icssgclk/1000) * 380);
    priv->pruss_xchg->endat_delay_900ms = ((icssgclk/1000) * 900);
    priv->pruss_xchg->icssg_clk = icssgclk;



    i = endat_pruss_load_run_fw(priv);

    if (i < 0)
    {
        DebugP_log("\rERROR: EnDat initialization failed -\n\n");

        if(gEndat_is_multi_ch)
        {
            unsigned char tmp;

            tmp = endat_multi_channel_detected(priv) & gEndat_multi_ch_mask;
            tmp ^= gEndat_multi_ch_mask;
            DebugP_log("\r\tunable to detect encoder in channel %s %s %s\n",
                        tmp & ENDAT_MULTI_CH0 ? "0" : "",
                        tmp & ENDAT_MULTI_CH1 ? "1" : "",
                        tmp & ENDAT_MULTI_CH2 ? "2" : "");
        }
        else
        {
            DebugP_log("\r\tcheck whether encoder is connected and ensure proper connections\n");
        }

        DebugP_log("\rexit %s due to failed firmware initialization\n", __func__);
        return;
    }

    /* read encoder info at low frequency so that cable length won't affect */
    endat_init_clock(200*1000, priv);

    for(j = 0; j < 3; j++)
    {
        if(gEndat_multi_ch_mask & 1 << j)
        {
            endat_multi_channel_set_cur(priv, j);

            if(endat_get_encoder_info(priv) < 0)
            {
                DebugP_log("\rEnDat initialization channel %d failed\n", j);
                DebugP_log("\rexit %s due to failed initialization\n", __func__);
                return;
            }

            gEndat_prop_delay[priv->channel] = endat_get_prop_delay(priv);
            DebugP_log("\n\t\t\t\tCHANNEL %d\n\n", j);
            endat_print_encoder_info(priv);
        }
    }

    /* pass the encoder step to the R5F_0_1 */
    priv_step = priv->step;

    gEndat_prop_delay_max = gEndat_prop_delay[0] > gEndat_prop_delay[1] ?
                           gEndat_prop_delay[0] : gEndat_prop_delay[1];
    gEndat_prop_delay_max = gEndat_prop_delay_max > gEndat_prop_delay[2] ?
                           gEndat_prop_delay_max : gEndat_prop_delay[2];

#if 1
    /* default frequency - 16MHz for 2.2 encoders, 1MHz for 2.1 encoders */
    endat_init_clock(16 * 1000 * 1000, priv);
    /* JR: Hard code propagation delay to make 16MHz work @300MHz PRU */
    endat_handle_prop_delay(priv, 265);
#else
    /* default frequency - 8MHz for 2.2 encoders, 1MHz for 2.1 encoders */
    endat_init_clock(8 * 1000 * 1000, priv);
    /* JR: Hard code propagation delay to make 16MHz work @300MHz PRU */
    endat_handle_prop_delay(priv, 530);
#endif

#if 0
    /* set to EnDat 2.2 recovery time */
    memset(&cmd_supplement, 0, sizeof(cmd_supplement));
    cmd_supplement.address = 3;
    cmd_supplement.data = 1;
    endat_command_process(priv, 10, &cmd_supplement);
    /* reset encoder */
    endat_command_process(priv, 5, NULL);
    endat_addinfo_track(priv, 5, NULL);
#endif

    /* JR: Configures EnDat to trigger based on IEP CMP4 event */
    endat_config_periodic_trigger(priv);

    DebugP_assert(endat_command_process(priv, 8, NULL) >= 0);

    struct endat_periodic_interface endat_periodic_interface;
    endat_periodic_interface.pruss_cfg = priv->pruss_cfg;
    endat_periodic_interface.pruss_iep = priv->pruss_iep;
    endat_periodic_interface.pruss_dmem = priv->pruss_xchg;
    endat_periodic_interface.load_share = priv->load_share;
    endat_periodic_interface.cmp3 = 3000;
    endat_periodic_interface.cmp5 = 0;
    endat_periodic_interface.cmp6 = 3000;

    status = endat_config_periodic_mode(&endat_periodic_interface, gPruIcssXHandle);
    DebugP_assert(0 != status);

#ifdef SINGLE_AXLE_USE_M1
    endat_multi_channel_set_cur(priv, 0);
    DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", 0);
#endif
#ifdef SINGLE_AXLE_USE_M2
    endat_multi_channel_set_cur(priv, 2);
    DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", 2);
#endif
    endat_handle_rx(priv, 8);

    DebugP_log("Encoder Channel Init Completed!!!\n");
}
#endif

void init_encoder2(){
#ifdef SINGLE_AXLE_USE_M2
    HwiP_Params hwiPrms;
    int32_t status;

    /* Register & enable ICSSG EnDat PRU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_ENDAT_INT_NUM + 2;
    hwiPrms.callback    = &pruEncoderIrqHandler2;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgEncoderHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    DebugP_log("Encoder Channel Init Completed!!!\n");
}
/* Test Sdfm handles */
sdfm_handle gHPruSdfm;

/* Sdfm output samples, written by PRU cores */
__attribute__((section(".gSdfmSampleOutput"))) uint32_t gSdfm_sampleOutput[NUM_CH_SUPPORTED];
/* Test SDDF handle */
sdfm_handle gHSddf;

/* Test Sdfm parameters */
SdfmPrms gTestSdfmPrms = {
    300000000,   /*Value of IEP clock*/
    20000000,    /*Value of SD clock (It should be exact equal to sd clock value)*/
    0,                        /*enable double update*/
     10,       /*first sample  trigger time*/
     0,       /*second sample trigger time*/
    APP_EPWM_OUTPUT_FREQ,     /*PWM output frequency*/
    {{3500, 1000,0},    /*threshold parameters(High, low & reserevd)*/
    {3500, 1000,0},
    {3500, 1000,0}},
    {{0,0},                /*clock sourse & clock inversion for all channels*/
    {0,0},
    {0,0}},
     15,   /*Over current osr: The effect count is OSR + 1*/
     64,   /*Normal current osr */
     0,   /*comparator enable*/
     (uint32_t)&gSddfChSamps /*Output samples base address*/
};

/* Test SDDF parameters */
/*//SddfPrms gTestSddfPrms = {
    SDDF_MODE_TRIG,     // sddfMode
    {0, 0, 0, 0, 0, 0}, // fdPrms
    0x1FF,              // chEn
    3,                  // trigSampCnt
    //31,                  // trigSampCnt
    //10,                  // trigSampCnt
    //45200,              // trigSampTime
    //37500,
    3000,
    //0,              // trigSampTime
    //4500,              // trigSampTime
    63,                 // osr
    {0, 0},             // sdClkPrms
    (uint32_t)gSddfChSamps,       // trigOutSampBuf
    SDDF_CFG_OSR | SDDF_CFG_TRIG_SAMP_TIME | SDDF_CFG_TRIG_SAMP_CNT | SDDF_CFG_CH_EN | SDDF_CFG_TRIG_OUT_SAMP_BUF // cfgMask
};*/

#ifdef SDDF_HW_EVM_ADAPTER_BP
/* SDDF PRU FW IRQ handler */
void pruSddfIrqHandler(void *args)
{
    /* debug, show ISR timing */
    //GPIO_pinWriteHigh(SDDF_DEBUG_BASE_ADDR, SDDF_DEBUG_PIN);

    /* debug, increment PRU SDDF IRQ count */
    gPruSddfIrqCnt++;
    if(gPruSddfIrqCnt > SETTLING_COUNT && gPruSddfIrqCnt <= OFFSET_FINISHED){
        gSddfChOffsets2[0] += (int32_t) (gSddfChSamps[3] - SDDF_HALF_SCALE);
        gSddfChOffsets2[1] += (int32_t) (gSddfChSamps[4] - SDDF_HALF_SCALE);
        gSddfChOffsets2[2] += (int32_t) (gSddfChSamps[5] - SDDF_HALF_SCALE);

        if(gPruSddfIrqCnt == OFFSET_FINISHED) {
            float dc0, dc1, dc2;
            uint16_t cmp0, cmp1, cmp2;

            gSddfChOffsets2[0] = (gSddfChOffsets2[0] / SETTLING_COUNT);
            gSddfChOffsets2[1] = (gSddfChOffsets2[1] / SETTLING_COUNT);
            gSddfChOffsets2[2] = (gSddfChOffsets2[2] / SETTLING_COUNT);

            /* ADC offset complete, lock the rotor to electrical 0 */
            computeCmpx(0.167, gEpwmPrdVal, &dc0, &cmp0);
            computeCmpx(-0.167, gEpwmPrdVal, &dc1, &cmp1);
            computeCmpx(-0.167, gEpwmPrdVal, &dc2, &cmp2);

            /* Write next CMPA values. Swap cmp0 and cmp2 because the HW connect PWM0 to Phase C and PWM2 to Phase A */
            writeCmpA(gEpwm0BaseAddr2A, cmp2);
            writeCmpA(gEpwm0BaseAddr2B, cmp2);
            writeCmpA(gEpwm1BaseAddr2, cmp1);
            writeCmpA(gEpwm2BaseAddr2, cmp0);
            gSDDFOffsetComplete2 = TRUE;
#if (PRECOMPUTE_LEVEL == NO_PRECOMPUTE)
            HwiP_disableInt(ICSSG_PRU_SDDF_INT_NUM);
#endif
        }

    }
#if (PRECOMPUTE_LEVEL == PRECOMPUTE_CLARKE)
    /* Offset has been calculated, pull the value and convert to float and scale it */
    else {
        float32_t fdbkCurPhA, fdbkCurPhB;

       /* Try to avoid the INA240 switching noise by selecting the 2 phases with the largest duty cycles */
       if (gInferA) {
           fdbkCurPhB = (-((float)gSddfChSamps[1] - gSddfChOffsets[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
           fdbkCurPhA = -fdbkCurPhB - ((-((float)gSddfChSamps[2] - gSddfChOffsets[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);

       }
       else if (gInferB){
           fdbkCurPhA = (-((float)gSddfChSamps[0] - gSddfChOffsets[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
           fdbkCurPhB = -fdbkCurPhA - ((-((float)gSddfChSamps[2] - gSddfChOffsets[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);
       }
       else {
           fdbkCurPhA = (-((float)gSddfChSamps[0] - gSddfChOffsets[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
           fdbkCurPhB = (-((float)gSddfChSamps[1] - gSddfChOffsets[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
       }

       /* Clarke transform */
       arm_clarke_f32(fdbkCurPhA, fdbkCurPhB, &gClarkeAlphaMeasured, &gClarkeBetaMeasured);
    }
#endif

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDDF_IRQ defined as 18
        Host:       SDDF_EVT defined as TRIGGER_HOST_SDDF_IRQ
        18 = 16+2, 2 is Host Interrupt Number. See AM654x TRM, Table 6-391.
    */
    PRUICSS_clearEvent(gPruIcss0Handle, PRU_TRIGGER_HOST_SDFM_EVT);

    /* debug, show ISR timing */
    //GPIO_pinWriteLow(SDDF_DEBUG_BASE_ADDR, SDDF_DEBUG_PIN);
}

/* SDDF PRU FW IRQ handler */
void rtuSddfIrqHandler(void *args)
{
    /* debug, increment PRU SDDF IRQ count */
    gRtuSddfIrqCnt++;

    if(gRtuSddfIrqCnt > SETTLING_COUNT && gRtuSddfIrqCnt <= OFFSET_FINISHED){
        gSddfChOffsets[0] += (int32_t) (gSddfChSamps[0] - SDDF_HALF_SCALE);
        gSddfChOffsets[1] += (int32_t) (gSddfChSamps[1] - SDDF_HALF_SCALE);
        gSddfChOffsets[2] += (int32_t) (gSddfChSamps[2] - SDDF_HALF_SCALE);

        if(gRtuSddfIrqCnt == OFFSET_FINISHED) {
            float dc0, dc1, dc2;
            uint16_t cmp0, cmp1, cmp2;

            gSddfChOffsets[0] = (gSddfChOffsets[0] / SETTLING_COUNT);
            gSddfChOffsets[1] = (gSddfChOffsets[1] / SETTLING_COUNT);
            gSddfChOffsets[2] = (gSddfChOffsets[2] / SETTLING_COUNT);

            /* ADC offset complete, lock the rotor to electrical 0 */
            computeCmpx(0.167, gEpwmPrdVal, &dc0, &cmp0);
            computeCmpx(-0.167, gEpwmPrdVal, &dc1, &cmp1);
            computeCmpx(-0.167, gEpwmPrdVal, &dc2, &cmp2);

            /* Write next CMPA values. Swap cmp0 and cmp2 because the HW connect PWM0 to Phase C and PWM2 to Phase A */
            writeCmpA(gEpwm0BaseAddr, cmp2);
            writeCmpA(gEpwm1BaseAddr, cmp1);
            writeCmpA(gEpwm2BaseAddr, cmp0);
            gSDDFOffsetComplete = TRUE;
#if (PRECOMPUTE_LEVEL == NO_PRECOMPUTE)
            HwiP_disableInt(ICSSG_RTU_SDDF_INT_NUM);
#endif
        }

    }
#if (PRECOMPUTE_LEVEL == PRECOMPUTE_CLARKE)
    /* Offset has been calculated, pull the value and convert to float and scale it */
    else {
        float32_t fdbkCurPhA, fdbkCurPhB;

       /* Try to avoid the INA240 switching noise by selecting the 2 phases with the largest duty cycles */
       if (gInferA) {
           fdbkCurPhB = (-((float)gSddfChSamps[1] - gSddfChOffsets[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
           fdbkCurPhA = -fdbkCurPhB - ((-((float)gSddfChSamps[2] - gSddfChOffsets[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);

       }
       else if (gInferB){
           fdbkCurPhA = (-((float)gSddfChSamps[0] - gSddfChOffsets[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
           fdbkCurPhB = -fdbkCurPhA - ((-((float)gSddfChSamps[2] - gSddfChOffsets[2] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0);
       }
       else {
           fdbkCurPhA = (-((float)gSddfChSamps[0] - gSddfChOffsets[0] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
           fdbkCurPhB = (-((float)gSddfChSamps[1] - gSddfChOffsets[1] - SDDF_HALF_SCALE_FLT) / SDDF_HALF_SCALE_FLT) * 30.0;
       }

       /* Clarke transform */
       arm_clarke_f32(fdbkCurPhA, fdbkCurPhB, &gClarkeAlphaMeasured, &gClarkeBetaMeasured);
    }
#endif

    /* Clear interrupt at source */
    /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
        Firmware:   TRIGGER_HOST_SDDF_IRQ defined as 18
        Host:       SDDF_EVT defined as TRIGGER_HOST_SDDF_IRQ
        18 = 16+2, 2 is Host Interrupt Number. See AM654x TRM, Table 6-391.
    */
    ///PRUICSS_clearEvent(gRtuIcss0Handle, RTU_TRIGGER_HOST_SDFM_EVT);
    PRUICSS_clearEvent(gPruIcss0Handle, RTU_TRIGGER_HOST_SDFM_EVT);
}
#endif

#ifdef SDDF_HW_EVM_ADAPTER_BP
#if defined(SINGLE_AXLE_USE_M1)
void init_sddf(){
    HwiP_Params hwiPrms;
    int32_t status;

    /* Configure g_mux_en to PRUICSS_G_MUX_EN in ICSSG_SA_MX_REG Register. */
    HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), PRUICSS_G_MUX_EN);

    /* Configure ICSSG clock selection */
    /*status = cfgIcssgClkCfg(TEST_ICSSG_INST_ID,
        CORE_CLK_SEL_ICSSGn_CORE_CLK,
        ICSSGn_CORE_CLK_SEL_MAIN_PLL2_HSDIV0_CLKOUT,
        ICSSGn_CORE_CLK_FREQ,
        IEP_CLK_SEL_CORE_CLK,
        0, //ICSSGn_IEP_CLK_SEL_MAIN_PLL0_HSDIV6_CLKOUT,
        0  //ICSSGn_IEP_CLK_FREQ
    );
    if (status != SDDF_ERR_NERR) {
        DebugP_log("Error: cfgIcssgClkCfg() fail.\r\n");
        return;
    }*/

    /* Initialize IEP0, configure SYNC0/1 SD clocks */
    init_IEP0_SYNC();

    /* Initialize ICSSG */
    status = initIcss(TEST_ICSSG_INST_ID, TEST_ICSSG_SLICE_ID, PRUICSS_G_MUX_EN, &gPruIcssHandle);
    if (status != SDDF_ERR_NERR) {
        DebugP_log("Error: initIcss() fail.\r\n");
        return;
    }

    /* Register & enable ICSSG SDDF RTU FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_RTU_SDDF_INT_NUM;
    hwiPrms.callback    = &rtuSddfIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgRtuSddfHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Initialize RTU core for SDDF */
    status = initPruSddf(gPruIcssHandle, TEST_RTU_INST_ID, &gTestSdfmPrms, &gHRtuSddf);
    if (status != SDDF_ERR_NERR) {
        DebugP_log("Error: initPruSddf() fail.\r\n");
        return;
    }

    /* Initialize PRU core for SDDF */
    gTestSdfmPrms.samplesBaseAddress += 0x80;
    status = initPruSddf(gPruIcssHandle, TEST_PRU_INST_ID, &gTestSdfmPrms, &gHPruSddf);
    if (status != SDDF_ERR_NERR) {
        DebugP_log("Error: initPruSddf() fail.\r\n");
        return;
    }

    /* Start IEP0 */
    start_IEP0();

    /* Start EPWM0 clock */
    ///CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN, 1);

    /* Force SW sync for EPWM0. Other PWMs will be sync'd through HW sync daisy-chain. */
   EPWM_tbTriggerSwSync(gEpwm0BaseAddr);

   /* Enable the PWM output buffer until gate driver configured and PWM signals at 50% duty cycle */
   enable_pwm_buffers(FALSE);
}
#endif

void init_sddf2(){
    HwiP_Params hwiPrms;
    int32_t status;

    /* Register & enable ICSSG PRU SDDF FW interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ICSSG_PRU_SDDF_INT_NUM;
    hwiPrms.callback    = &pruSddfIrqHandler;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = FALSE;
    hwiPrms.isFIQ       = FALSE;
    status              = HwiP_construct(&gIcssgPruSddfHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
}
#endif

/* This example shows message exchange between multiple cores.
 *
 * One of the core is designated as the 'main' core
 * and other cores are desginated as `remote` cores.
 *
 * The main core initiates IPC with remote core's by sending it a message.
 * The remote cores echo the same message to the main core.
 *
 * The main core repeats this for gMsgEchoCount iterations.
 *
 * In each iteration of message exchange, the message value is incremented.
 *
 * When iteration count reaches gMsgEchoCount, a semaphore is posted and
 * the pending thread/task on that core is unblocked.
 *
 * When a message or its echo is received, a user registered callback is invoked.
 * The message is echoed from within the user callback itself.
 *
 * This is a example message exchange, in final systems, user can do more
 * sophisticated message exchanges as needed for their applications.
 */

/* number of iterations of message exchange to do */
uint32_t gMsgEchoCount = 1000000u;
/* client ID that is used to send and receive messages */
uint32_t gClientId = 4u;
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS1_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};

/* semaphore's used to indicate a main core has finished all message exchanges */
SemaphoreP_Object gMainDoneSem[CSL_CORE_ID_MAX];

/* semaphore used to indicate a remote core has finished all message xchange */
SemaphoreP_Object gRemoteDoneSem;

void ipc_notify_msg_handler_remote_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* on remote core, we have registered handler on the same client ID and current core client ID */
    IpcNotify_sendMsg(remoteCoreId, localClientId, msgValue, 1);

    /* if all messages received then post semaphore to exit */
    if(msgValue == (gMsgEchoCount-1))
    {
        SemaphoreP_post(&gRemoteDoneSem);
    }
}

#if defined(SINGLE_AXLE_USE_M1)
int32_t rw_drv8350_register(uint16_t r_w, uint16_t reg_addr, uint16_t reg_val, uint16_t* read_val)
{
    MCSPI_Transaction   spiTransaction;
    uint16_t mcspiTxBuffer, mcspiRxBuffer;
    int32_t transferOK;

    /* Initiate SPI transfer parameters */
    spiTransaction.channel  = 0U;
    spiTransaction.count    = 1;
    spiTransaction.txBuf    = &mcspiTxBuffer;
    spiTransaction.rxBuf    = &mcspiRxBuffer;
    spiTransaction.args     = NULL;

    mcspiTxBuffer = r_w | (reg_addr << DRV8350_ADDR_SHIFT) | reg_val;
#ifdef SINGLE_AXLE_USE_M1
    ////GPIO_pinWriteLow(MTR_1_CS_BASE_ADDR, MTR_1_CS_PIN);
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    ////GPIO_pinWriteHigh(MTR_1_CS_BASE_ADDR, MTR_1_CS_PIN);
#endif
#ifdef SINGLE_AXLE_USE_M2
    ////GPIO_pinWriteLow(MTR_2_CS_BASE_ADDR, MTR_2_CS_PIN);
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    ////GPIO_pinWriteHigh(MTR_2_CS_BASE_ADDR, MTR_2_CS_PIN);
#endif

    if (read_val != NULL)
        *read_val = mcspiRxBuffer;

    return transferOK;
}

int32_t init_drv8350()
{
    int32_t status = SystemP_SUCCESS;
    DRV8350_Vars drv8350;
    uint16_t reg_vals[6];

    /* Build the desired register values for the four control registers */
    drv8350.driver_control.all = 0;
    drv8350.gate_drive_hs.all = 0;
    drv8350.gate_drive_ls.all = 0;
    drv8350.ocp_control.all = 0;

    drv8350.driver_control.bit.OCP_ACT = DRV8350_OCP_disable_three_phases;

    drv8350.gate_drive_hs.bit.IDRIVEN_HS = DRV8350_idriveN_600mA;
    drv8350.gate_drive_hs.bit.IDRIVEP_HS = DRV8350_idriveP_300mA;
    drv8350.gate_drive_hs.bit.LOCK = DRV8350_lock_disable;

    drv8350.gate_drive_ls.bit.CBC = DRV8350_OCP_ClrFaults_Cycle_by_Cycle_Yes;
    drv8350.gate_drive_ls.bit.IDRIVEN_LS = DRV8350_idriveN_600mA;
    drv8350.gate_drive_ls.bit.IDRIVEP_LS = DRV8350_idriveP_300mA;
    drv8350.gate_drive_ls.bit.TDRIVE = DRV8350_tdrive_4000nS;

    drv8350.ocp_control.bit.DEAD_TIME = DRV8350_deadTime_50nS;
    drv8350.ocp_control.bit.OCP_MODE = DRV8350_OCP_Latch_Fault;
    drv8350.ocp_control.bit.OCP_DEG = DRV8350_deglitch_1us;
    drv8350.ocp_control.bit.VDS_LVL = DRV8350_VDS_LVL_1000mV;

    /* Write the control registers */
    rw_drv8350_register(DRV8350_WRITE, DRV8350_DRIVER_CONTROL_ADDR, drv8350.driver_control.all, NULL);
    rw_drv8350_register(DRV8350_WRITE, DRV8350_GATE_DRIVE_HS_ADDR, drv8350.gate_drive_hs.all, NULL);
    rw_drv8350_register(DRV8350_WRITE, DRV8350_GATE_DRIVE_LS_ADDR, drv8350.gate_drive_ls.all, NULL);
    rw_drv8350_register(DRV8350_WRITE, DRV8350_OCP_CONTROL_ADDR, drv8350.ocp_control.all, NULL);

    /* Read back the control registers to check for correctness */
    rw_drv8350_register(DRV8350_READ, DRV8350_DRIVER_CONTROL_ADDR, 0, &reg_vals[DRV8350_DRIVER_CONTROL_ADDR]);
    rw_drv8350_register(DRV8350_READ, DRV8350_GATE_DRIVE_HS_ADDR, 0, &reg_vals[DRV8350_GATE_DRIVE_HS_ADDR]);
    rw_drv8350_register(DRV8350_READ, DRV8350_GATE_DRIVE_LS_ADDR, 0, &reg_vals[DRV8350_GATE_DRIVE_LS_ADDR]);
    rw_drv8350_register(DRV8350_READ, DRV8350_OCP_CONTROL_ADDR, 0, &reg_vals[DRV8350_OCP_CONTROL_ADDR]);

    /* Check against desired values */
    if ((drv8350.driver_control.all != reg_vals[DRV8350_DRIVER_CONTROL_ADDR]) ||
        (drv8350.gate_drive_hs.all != reg_vals[DRV8350_GATE_DRIVE_HS_ADDR]) ||
        (drv8350.gate_drive_ls.all != reg_vals[DRV8350_GATE_DRIVE_LS_ADDR]) ||
        (drv8350.ocp_control.all != reg_vals[DRV8350_OCP_CONTROL_ADDR])) {
        status = SystemP_FAILURE;
        DebugP_log("[Single Chip Servo] DRV8350 configuration failed!\r\n");
    }
    else {
        DebugP_log("[Single Chip Servo] DRV8350 configured!\r\n");
    }

    /* Lock the control registers of the DRV8350 to ensure there are no accidental writes */
    drv8350.gate_drive_hs.bit.LOCK = DRV8350_lock_enable;
    rw_drv8350_register(DRV8350_WRITE, DRV8350_GATE_DRIVE_HS_ADDR, drv8350.gate_drive_hs.all, NULL);
    rw_drv8350_register(DRV8350_READ, DRV8350_GATE_DRIVE_HS_ADDR, 0, &reg_vals[DRV8350_GATE_DRIVE_HS_ADDR]);

    if (drv8350.gate_drive_hs.all == reg_vals[DRV8350_GATE_DRIVE_HS_ADDR])
        DebugP_log("[Single Chip Servo] DRV8350 Control registers locked!\r\n");

    return status;
}

int32_t check_adapter_board_power()
{
    int32_t status = SystemP_SUCCESS;

    /* Check the nRESET_SVS pin to ensure power on the 3-axis board is good */
    ////if(!GPIO_pinRead(NRESET_SVS_BASE_ADDR, NRESET_SVS_PIN)) {
    ////    DebugP_log("[Single Chip Servo] Adapter board is not powered or 3.3V rail is out of specification.\r\n");
    ////    status = SystemP_FAILURE;
    ////}
    ////else {
    ////    DebugP_log("[Single Chip Servo] Adapter board power is good.\r\n");
    ////}

    return status;
}

void init_gpio_state()
{
    /* Set the initial GPIO output state before configuring direction. This avoids ambiguity on outputs. */
    GPIO_pinWriteHigh(MTR_1_PWM_EN_BASE_ADDR, MTR_1_PWM_EN_PIN);
    GPIO_pinWriteHigh(MTR_2_PWM_EN_BASE_ADDR, MTR_2_PWM_EN_PIN);
    GPIO_pinWriteHigh(BP_MUX_SEL_BASE_ADDR, BP_MUX_SEL_PIN);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);

    GPIO_setDirMode(MTR_1_PWM_EN_BASE_ADDR, MTR_1_PWM_EN_PIN, MTR_1_PWM_EN_DIR);
    GPIO_setDirMode(MTR_2_PWM_EN_BASE_ADDR, MTR_2_PWM_EN_PIN, MTR_2_PWM_EN_DIR);
    GPIO_setDirMode(BP_MUX_SEL_BASE_ADDR, BP_MUX_SEL_PIN, BP_MUX_SEL_DIR);
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);

#ifdef SDDF_HW_EVM_ADAPTER_BP
    ////GPIO_pinWriteHigh(HDSL_EN_BASE_ADDR, HDSL_EN_PIN);
    ////GPIO_setDirMode(HDSL_EN_BASE_ADDR, HDSL_EN_PIN, GPIO_DIRECTION_OUTPUT);
#endif
}
#endif

#if defined(SDDF_HW_EVM_ADAPTER_BP)
static TCA6424_Config  gTCA6424_Config;
void i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;
    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    uint32_t            ioIndex;

    if(status == SystemP_SUCCESS)
    {
        /* set IO expander P12 to high */
        ioIndex = 0x0a;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
    }
    TCA6424_close(&gTCA6424_Config);
}
#endif

/* GPIO PIN Macros */
#define GPIO0_BASE_ADDR (CSL_MCU_GPIO0_BASE)
#define GPIO_LED_PIN ()
#define GPIO_LED_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_LED_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)

#if defined(SINGLE_AXLE_USE_M1)
void misc_pinmux(){
#if 0
    /* Sigma Delta pin mux */
    Pinmux_PerCfg_t miscPinMuxCfg[] = {
#if 0
           /* EHRPWM3 pin config */
#ifdef SINGLE_AXLE_USE_M2
   /* EHRPWM5_A -> GPMC0_BE1n (P21) */
   {
       PIN_GPMC0_BE1N,
       ( PIN_MODE(3) | PIN_PULL_DISABLE )
   },
#endif
#if defined(SDDF_HW_EVM_ADAPTER_BP)
    /* PRG0_PRU0_GPO19, PRG0_IEP0_EDC_SYNC_OUT0, W1, BP J5.45 */
    {
        PIN_PRG0_PRU0_GPO19,
        ( PIN_MODE(2) | PIN_PULL_DISABLE )
    },
    /* PRG0_PRU0_GPO17, PRG0_IEP0_EDC_SYNC_OUT1, U1, BP J2.18 */
    {
        PIN_PRG0_PRU0_GPO17,
        ( PIN_MODE(2) | PIN_PULL_DISABLE )
    },
#endif
    /* SD8_CLK,
       PRG0_PRU0_GPI16, SD8_CLK,    U4, J2E:P9 */
    {
        PIN_PRG0_PRU0_GPO16,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* PRG0_PRU1_GPI9 -> PRG0_PRU1_GPO9 (Y5) */
    {
        PIN_PRG0_PRU1_GPO9,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* PRG0_PRU1_GPI10 -> PRG0_PRU1_GPO10 (V6) */
    {
        PIN_PRG0_PRU1_GPO10,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* GPIO0_27 -> GPMC0_AD12 (W21) */
    {
        PIN_GPMC0_AD12,
        ( PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
#ifdef SINGLE_AXLE_USE_M1
#if defined(AM243X_ALX)
       /* SD0_D,
         PRG0_PRU0_GPI1,   SD0_D,      J4, J4:32 */
       {
           PIN_PRG0_PRU0_GPO1,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
       /* SD1_D,
          PRG0_PRU0_GPI3, SD1_D,      H1, J2:19  */
       {
           PIN_PRG0_PRU0_GPO3,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
       /* SD2_D,
          PRG0_PRU0_GPI5,  SD2_D,      F2, J2:13  */
       {
           PIN_PRG0_PRU0_GPO5,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
#else
       /* SD0_D,
         PRG0_PRU0_GPI1,   SD0_D,      R4, J2E:P8 */
       {
           PIN_PRG0_PRU0_GPO1,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
       /* SD1_D,
          PRG0_PRU0_GPI3m, SD1_D,      V2, J2A:P9  */
       {
           PIN_PRG0_PRU0_GPO3,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
       /* SD2_D,
          PRG0_PRU0_GPI5,  SD2_D,      R3, J2C:P6  */
       {
           PIN_PRG0_PRU0_GPO5,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
#endif
#endif
#ifdef SINGLE_AXLE_USE_M1 ////// do pinmux for M2 here
       /* SD3_D,
         PRG0_PRU0_GPI7,   SD3_D,      T1, J2B:P7 */
       {
           PIN_PRG0_PRU0_GPO7,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
#ifdef SDDF_HW_EVM_ADAPTER_BP
       /* SD4_D,
          PRG0_PRU0_GPI9, SD4_D,      W6, J2D:P28  */
       {
           PIN_PRG0_PRU0_GPO9,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
#endif
       /* SD5_D,
          PRG0_PRU0_GPI11,  SD5_D,      Y3, J2B:P14  */
       {
           PIN_PRG0_PRU0_GPO11,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
       },
#endif
#endif

        /* PRU_ICSSG0_PRU0 pin config */
        /* PRG0_PRU0_GPI1 -> PRG0_PRU0_GPO1 (J4) */
        {
           PIN_PRG0_PRU0_GPO1,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI11 -> PRG0_PRU0_GPO11 (L1) */
        {
           PIN_PRG0_PRU0_GPO11,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI16 -> PRG0_PRU0_GPO16 (N3) */
        {
           PIN_PRG0_PRU0_GPO16,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI18 -> PRG0_PRU0_GPO18 (K4) */
        {
           PIN_PRG0_PRU0_GPO18,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI3 -> PRG0_PRU0_GPO3 (H1) */
        {
           PIN_PRG0_PRU0_GPO3,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI5 -> PRG0_PRU0_GPO5 (F2) */
        {
           PIN_PRG0_PRU0_GPO5,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI7 -> PRG0_PRU0_GPO7 (E2) */
        {
           PIN_PRG0_PRU0_GPO7,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },
        /* PRG0_PRU0_GPI8 -> PRG0_PRU0_GPO8 (H5) */
        {
           PIN_PRG0_PRU0_GPO8,
           ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },

               /* PRU_ICSSG0_IEP0 pin config */
        /* PRG0_IEP0_EDC_SYNC_OUT0 -> PRG0_PRU0_GPO19 (G2) */
        {
           PIN_PRG0_PRU0_GPO19,
           ( PIN_MODE(2) | PIN_PULL_DISABLE )
        },
        /* PRG0_IEP0_EDC_SYNC_OUT1 -> PRG0_PRU0_GPO17 (E1) */
        {
           PIN_PRG0_PRU0_GPO17,
           ( PIN_MODE(2) | PIN_PULL_DISABLE )
        },

       {PINMUX_END, PINMUX_END}
    };
#endif

    ///Pinmux_config(miscPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);

#if defined(SDDF_HW_EVM_ADAPTER_BP)
    /* set IO expander P12 to high for using PIN_PRG0_PRU0_GPO9 */
    ////i2c_io_expander(NULL);
#endif

#ifdef SINGLE_AXLE_USE_M2
    ///Pinmux_config(miscPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);
#endif

#if defined(AM243X_ALV)
    GPIO_setDirMode(CSL_GPIO0_BASE, 27, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(CSL_GPIO0_BASE, 27);

    GPIO_setDirMode(CSL_GPIO0_BASE, 42, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(CSL_GPIO0_BASE, 42);
#endif
#if defined(AM243X_ALX)
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif
}

/* EPWM ISR */
__attribute__((section(".critical_code"))) static void App_epwmIntrISR(void *args)
{
    AppEPwmIsrInfo_t  *pAppEpwmIsrInfo;
    uint32_t epwmBaseAddr;
    SemaphoreP_Object *pEpwmSyncSemObject;
    volatile uint16_t status;

    gEpwmIsrCnt++;

    /* Get EPWM info */
    pAppEpwmIsrInfo = (AppEPwmIsrInfo_t *)args;
    epwmBaseAddr = pAppEpwmIsrInfo->epwmBaseAddr;
    pEpwmSyncSemObject = pAppEpwmIsrInfo->pEpwmSyncSemObject;

    status = EPWM_etIntrStatus(epwmBaseAddr);
    if (status & EPWM_ETFLG_INT_MASK) {
        SemaphoreP_post(pEpwmSyncSemObject);
        EPWM_etIntrClear(epwmBaseAddr);
    }

#if defined(SDDF_HW_EVM_ADAPTER_BP)
    /* Call FOC loop here */
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
    pruEncoderIrqHandler(NULL);
#endif
#endif

    return;
}
#endif

__attribute__((section(".critical_code"))) static void App_epwmIntrISR2(void *args)
{
    AppEPwmIsrInfo_t  *pAppEpwmIsrInfo;
    uint32_t epwmBaseAddr;
    SemaphoreP_Object *pEpwmSyncSemObject;
    volatile uint16_t status;

    gEpwmIsrCnt2++;

    /* Get EPWM info */
    pAppEpwmIsrInfo = (AppEPwmIsrInfo_t *)args;
    epwmBaseAddr = pAppEpwmIsrInfo->epwmBaseAddr;
    pEpwmSyncSemObject = pAppEpwmIsrInfo->pEpwmSyncSemObject;

    status = EPWM_etIntrStatus(epwmBaseAddr);
    if (status & EPWM_ETFLG_INT_MASK) {
        SemaphoreP_post(pEpwmSyncSemObject);
        EPWM_etIntrClear(epwmBaseAddr);
    }

#if defined(SDDF_HW_EVM_ADAPTER_BP)
#if defined(USE_PURE_OPEN_LOOP)||defined(USE_OPEN_LOOP_WITH_SDDF)
    /* Call FOC loop here */
    pruEncoderIrqHandler2(NULL);
#endif
#endif

    return;
}

void init_pwms(){
    int32_t         status;
    AppEPwmCfg_t    appEpwmCfg;
    HwiP_Params     hwiPrms;
    uint32_t        epwm0PrdVal, epwm1PrdVal, epwm2PrdVal;
    uint32_t        epwm0CmpAVal, epwm1CmpAVal, epwm2CmpAVal;

    /* Address translate */
#ifdef SINGLE_AXLE_USE_M1
    gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM0_BASE_ADDR);
    gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM1_BASE_ADDR);
    gEpwm2BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM2_BASE_ADDR);
#endif
#ifdef  SINGLE_AXLE_USE_M2
    gEpwm0BaseAddr2A = (uint32_t)AddrTranslateP_getLocalAddr(EPWM3_OUTA_BASE_ADDR);
    gEpwm0BaseAddr2B = (uint32_t)AddrTranslateP_getLocalAddr(EPWM3_OUTB_BASE_ADDR);
    gEpwm1BaseAddr2 = (uint32_t)AddrTranslateP_getLocalAddr(EPWM4_BASE_ADDR);
    gEpwm2BaseAddr2 = (uint32_t)AddrTranslateP_getLocalAddr(EPWM5_BASE_ADDR);
#endif

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);
#ifdef SINGLE_AXLE_USE_M1
    /* Configure the SYNCI/SYNCO mapping to tie the three PWM groups together and have PWM0 SYNC from Time Sync Router 38 */
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_SYNCIN_SEL_SHIFT));
    /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 38 (0x26 + 4 = 0x2A + Time Sync Router Base */
    CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((38 * 4) + 4), (0x10000 | 29));
#endif
#ifdef SINGLE_AXLE_USE_M2
    /* Configure the SYNCI/SYNCO mapping to tie the three PWM groups together and have PWM3 SYNC from Time Sync Router 39 */
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_SHIFT));
    /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 39 (0x26 + 4 = 0x2A + Time Sync Router Base */
    CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((39 * 4) + 4), (0x10000 | 29));
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_SHIFT));
    /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 39 (0x26 + 4 = 0x2A + Time Sync Router Base */
    CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((40 * 4) + 4), (0x10000 | 29));
#endif

    /* Configure the ISR triggered by EPWM0 */
    /* Create semaphore */
    status = SemaphoreP_constructBinary(&gEpwm0SyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gEpwm0SyncSemObject2, 0);
    DebugP_assert(SystemP_SUCCESS == status);


#if defined(SINGLE_AXLE_USE_M1)
    /* Register & enable EPWM0 interrupt */
    gAppEPwm0Info.epwmBaseAddr = gEpwm0BaseAddr;
    gAppEPwm0Info.pEpwmSyncSemObject = &gEpwm0SyncSemObject;
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = EPWM0_INTR;
    hwiPrms.callback    = &App_epwmIntrISR;
    hwiPrms.args        = &gAppEPwm0Info;
    hwiPrms.isPulse     = EPWM0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwm0HwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWMs */
    /* Configure EPWM0 (Phase C) */
    appEpwmCfg.epwmBaseAddr = gEpwm0BaseAddr;
    appEpwmCfg.epwmCh = EPWM_OUTPUT_CH_A;
    appEpwmCfg.epwmFuncClk = EPWM0_FCLK;
    appEpwmCfg.epwmTbFreq = EPWM0_FCLK;
    appEpwmCfg.epwmOutFreq = gEpwmOutFreq;
    appEpwmCfg.epwmDutyCycle = 50;
    appEpwmCfg.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
    appEpwmCfg.cfgTbSyncIn = TRUE;
    appEpwmCfg.tbPhsValue = 7;
    appEpwmCfg.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_DOWN;
    appEpwmCfg.cfgTbSyncOut = TRUE;
    appEpwmCfg.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
#if defined(USE_RTLIB_FOC)
    appEpwmCfg.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_LOW;
    appEpwmCfg.aqCfg.cmpADownAction = EPWM_AQ_ACTION_HIGH;
#else
    appEpwmCfg.aqCfg.zeroAction = EPWM_AQ_ACTION_LOW;
    appEpwmCfg.aqCfg.prdAction = EPWM_AQ_ACTION_HIGH;
    appEpwmCfg.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    appEpwmCfg.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
#endif
    appEpwmCfg.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.cfgDb = TRUE;
    appEpwmCfg.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    appEpwmCfg.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
    appEpwmCfg.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
    appEpwmCfg.dbCfg.risingEdgeDelay = gDbRisingEdgeDelay;
    appEpwmCfg.dbCfg.fallingEdgeDelay = gDbFallingEdgeDelay;
    appEpwmCfg.cfgEt = TRUE;
    appEpwmCfg.intSel = EPWM_ET_INTR_EVT_CNT_EQ_PRD;
    appEpwmCfg.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    App_epwmConfig(&appEpwmCfg, &epwm0PrdVal, &epwm0CmpAVal);

    /* Configure EPWM1 (Phase B), EPWM2 (Phase A) */
    appEpwmCfg.tbPhsValue = 0;
    appEpwmCfg.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    appEpwmCfg.cfgEt = FALSE;
    appEpwmCfg.epwmBaseAddr = gEpwm1BaseAddr;
    App_epwmConfig(&appEpwmCfg, &epwm1PrdVal, &epwm1CmpAVal);
    appEpwmCfg.epwmBaseAddr = gEpwm2BaseAddr;
    App_epwmConfig(&appEpwmCfg, &epwm2PrdVal, &epwm2CmpAVal);

    DebugP_assert(epwm0PrdVal == epwm1PrdVal);
    DebugP_assert(epwm1PrdVal == epwm2PrdVal);

    /* Period is the same for all EPWMs */
    gEpwmPrdVal = epwm0PrdVal;

    /* Force SW sync for EPWM0. Other PWMs will be sync'd through HW sync daisy-chain. */
    EPWM_tbTriggerSwSync(gEpwm0BaseAddr);
#endif

#if defined(SINGLE_AXLE_USE_M2)
    /* Register & enable EPWM0 interrupt */
    gAppEPwm0Info2.epwmBaseAddr = gEpwm0BaseAddr2A;
    gAppEPwm0Info2.pEpwmSyncSemObject = &gEpwm0SyncSemObject2;
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = EPWM3_OUTA_INTR;
    hwiPrms.callback    = &App_epwmIntrISR2;
    hwiPrms.args        = &gAppEPwm0Info2;
    hwiPrms.isPulse     = EPWM3_OUTA_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwm0HwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWMs */
    /* Configure EPWM3 (Phase C) */
    appEpwmCfg.epwmBaseAddr = gEpwm0BaseAddr2A;
    appEpwmCfg.epwmCh = EPWM_OUTPUT_CH_A;
    appEpwmCfg.epwmFuncClk = EPWM3_OUTA_FCLK;
    appEpwmCfg.epwmTbFreq = EPWM3_OUTA_FCLK;
    appEpwmCfg.epwmOutFreq = gEpwmOutFreq;
    appEpwmCfg.epwmDutyCycle = 50;
    appEpwmCfg.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
    appEpwmCfg.cfgTbSyncIn = TRUE;
    appEpwmCfg.tbPhsValue = 7;
    appEpwmCfg.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_DOWN;
    appEpwmCfg.cfgTbSyncOut = TRUE;
    appEpwmCfg.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
    appEpwmCfg.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    appEpwmCfg.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
    appEpwmCfg.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.cfgDb = TRUE;
    appEpwmCfg.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    appEpwmCfg.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
    appEpwmCfg.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
    appEpwmCfg.dbCfg.risingEdgeDelay = gDbRisingEdgeDelay;
    appEpwmCfg.dbCfg.fallingEdgeDelay = gDbFallingEdgeDelay;
    appEpwmCfg.cfgEt = TRUE;
    appEpwmCfg.intSel = EPWM_ET_INTR_EVT_CNT_EQ_PRD;
    appEpwmCfg.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    App_epwmConfig(&appEpwmCfg, &epwm0PrdVal, &epwm0CmpAVal);
    appEpwmCfg.epwmBaseAddr = gEpwm0BaseAddr2B;
    App_epwmConfig(&appEpwmCfg, &epwm0PrdVal, &epwm0CmpAVal);

    /* Configure EPWM1 (Phase B), EPWM2 (Phase A) */
    appEpwmCfg.tbPhsValue = 0;
    appEpwmCfg.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    appEpwmCfg.cfgEt = FALSE;
    appEpwmCfg.epwmBaseAddr = gEpwm1BaseAddr2;
    App_epwmConfig(&appEpwmCfg, &epwm1PrdVal, &epwm1CmpAVal);
    appEpwmCfg.epwmBaseAddr = gEpwm2BaseAddr2;
    App_epwmConfig(&appEpwmCfg, &epwm2PrdVal, &epwm2CmpAVal);

    DebugP_assert(epwm0PrdVal == epwm1PrdVal);
    DebugP_assert(epwm1PrdVal == epwm2PrdVal);

    /* Period is the same for all EPWMs */
    gEpwmPrdVal = epwm0PrdVal;

    /* Force SW sync for EPWM0. Other PWMs will be sync'd through HW sync daisy-chain. */
    EPWM_tbTriggerSwSync(gEpwm0BaseAddr2A);
#endif

    /* Enable the PWM output buffer until gate driver configured and PWM signals at 50% duty cycle */
    ///enable_pwm_buffers(FALSE);
}

void init_pids(){
    /* 50KHz PWM frequency PID constants */
#if defined(USE_RTLIB_FOC)
    gPiSpd.Kp = MAX_SPD_CHANGE;
    gPiSpd.Ki = 0.0003;
    gPiSpd.Umax = 1000;
    gPiSpd.Umin = -1000;
    gPiSpd.Imax = 0.0;
    gPiSpd.Imin = 0.0;
    gPiSpd.i6 = 1.0;
    gPiSpd.i10 = 0.0;
    gPiSpd.i11 = 0.0;
    gPiSpd.sps = &(DCL_PI_SPS)PI_SPS_DEFAULTS;
    gPiSpd.css = &(DCL_CSS)DCL_CSS_DEFAULTS;

    gPiId.Kp = 0.2;
    gPiId.Ki = 0.0001;
    gPiId.Umax = ID_TESTING;
    gPiId.Umin = -1.0*ID_TESTING;
    gPiId.Imax = 0.0;
    gPiId.Imin = 0.0;
    gPiId.i6 = 1.0;
    gPiId.i10 = 0.0;
    gPiId.i11 = 0.0;
    gPiId.sps = &(DCL_PI_SPS)PI_SPS_DEFAULTS;
    gPiId.css = &(DCL_CSS)DCL_CSS_DEFAULTS;

    gPiIq.Kp = 0.2;
    gPiIq.Ki = 0.0001;
    gPiIq.Umax = IQ_TESTING;
    gPiIq.Umin = -1.0*IQ_TESTING;
    gPiIq.Imax = 0.0;
    gPiIq.Imin = 0.0;
    gPiIq.i6 = 1.0;
    gPiIq.i10 = 0.0;
    gPiIq.i11 = 0.0;
    gPiIq.sps = &(DCL_PI_SPS)PI_SPS_DEFAULTS;
    gPiIq.css = &(DCL_CSS)DCL_CSS_DEFAULTS;
#else
    gPiId.Kp = 0.2;
    gPiId.Ki = 0.0001;
    gPiId.Kd = 0;
    arm_pid_init_f32(&gPiId, 1);

    gPiIq.Kp = 0.2;
    gPiIq.Ki = 0.0001;
    gPiIq.Kd = 0;
    arm_pid_init_f32(&gPiIq, 1);

    /* Mmax 0.12 RPM step between periods */
    gPiSpd.Kp = MAX_SPD_CHANGE; ///0.06
    gPiSpd.Ki = 0.0003;
    gPiSpd.Kd = 0;
    arm_pid_init_f32(&gPiSpd, 1);

    /* Max 0.06 angle step between periods (500 RPM) */
    gPiPos.Kp = MAX_POS_CHANGE*MAX_SPD_RPM; ///30;
    gPiPos.Ki = 0;
    gPiPos.Kd = 0;
    arm_pid_init_f32(&gPiPos, 1);
#endif

    /* 20KHz PWM frequency PID constants */
#if 0
    gPiId.Kp = 0.225;
    gPiId.Ki = 0.005;
    gPiId.Kd = 0;
    arm_pid_init_f32(&gPiId, 1);

    gPiIq.Kp = 0.2;
    gPiIq.Ki = 0.005;
    gPiIq.Kd = 0;
    arm_pid_init_f32(&gPiIq, 1);

    /* Max 0.3 RPM step between periods */
    gPiSpd.Kp = 0.045;
    gPiSpd.Ki = 0.0001;
    gPiSpd.Kd = 0;
    arm_pid_init_f32(&gPiSpd, 1);

    /* Max 0.15 angle step between periods (500 RPM) */
    gPiPos.Kp = 13;
    gPiPos.Ki = 0.000025;
    gPiPos.Kd = 0;
    arm_pid_init_f32(&gPiPos, 1);
#endif
}

void single_chip_servo_remote_core_start()
{
#if (DEBUG_LEVEL == DEBUG_BUFFERS_ON)
    /* Make sure that the Debug buffer size is larger than the target buffer size so that there are no index out of bounds issues */
    DebugP_assert(DEBUG_BUFF_SZ > TARGET_BUFF_SIZE);
#endif

#if defined(SINGLE_AXLE_USE_M1)
    /* Initialize GPIO state */
    init_gpio_state();

#if !defined(SDDF_HW_EVM_ADAPTER_BP)
    /* Check the power on the adapter board */
    status = check_adapter_board_power();
    DebugP_assert(status==SystemP_SUCCESS);
#endif

    /* Disable the PWM output buffer until gate driver configured and PWM signals at 50% duty cycle */
    enable_pwm_buffers(FALSE);

#if !defined(SDDF_HW_EVM_ADAPTER_BP)
    /* Initialize the DRV8350 gate driver */
    status = init_drv8350();
    DebugP_assert(status==SystemP_SUCCESS);
#endif

    ///misc_pinmux();
#endif

    /* Configure PWMs for complementary 3-phase and set duty cycle to 50% */
    init_pwms();

#if defined(SINGLE_AXLE_USE_M1)
#if !defined(USE_PURE_OPEN_LOOP) && !defined(USE_OPEN_LOOP_WITH_SDDF)
    init_encoder();

    init_sddf();
#endif
#if defined(USE_OPEN_LOOP_WITH_SDDF)
    init_sddf();
#endif

    init_pids();

#if (BUILDLEVEL == CLOSED_LOOP_CIA402)
    /* Default the CiA402 commands to torque control with a target value of 0 */
    gTxData.modeOfOperation = CYCLIC_SYNC_VELOCITY_MODE; ///CYCLIC_SYNC_TORQUE_MODE;
    gTxData.targetPosition = 0;
    gTxData.targetVelocity = 0;
    gTxData.targetTorque = 0;
#endif

    /* Enable the PWM output buffers */
    enable_pwm_buffers(TRUE);
#endif

#if defined(SINGLE_AXLE_USE_M2)
#if !defined(USE_PURE_OPEN_LOOP) && !defined(USE_OPEN_LOOP_WITH_SDDF)
    init_pids();
    init_sddf2();

    init_encoder2();
#endif
#if defined(USE_OPEN_LOOP_WITH_SDDF)
    init_sddf2();
#endif

#endif

    SemaphoreP_constructBinary(&gRemoteDoneSem, 0);

    /* register a handler to receive messages */
    ///status = IpcNotify_registerClient(gClientId, ipc_notify_msg_handler_remote_core, NULL);
    ///DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    ///IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("\n\n[Single Chip Servo] All cores synchronized...\n\n");

    while(gRunFlag == TRUE)
        {
#if defined(SINGLE_AXLE_USE_M1)
            SemaphoreP_pend(&gEpwm0SyncSemObject, SystemP_WAIT_FOREVER);
#endif
#if defined(SINGLE_AXLE_USE_M2)
            SemaphoreP_pend(&gEpwm0SyncSemObject2, SystemP_WAIT_FOREVER);
#endif
            gLoopCnt++;
        }

    DebugP_log("Exiting!\r\n");
}

void single_chip_servo_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    single_chip_servo_remote_core_start();

    Board_driversClose();
    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    /* Drivers_close(); */
}
