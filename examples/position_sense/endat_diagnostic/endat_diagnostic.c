/*
 *  Copyright (C) 2021-2025 Texas Instruments Incorporated
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>

#if defined(SOC_AM243X) || defined(SOC_AM64X)
#include <drivers/sciclient.h>
#endif

#include <kernel/dpl/TaskP.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <position_sense/endat/include/endat_drv.h>

#include "endat_periodic_trigger.h"

/* Size of the PRU instruction memory in bytes.*/
#define PRU_IRAM_SIZE   ( 12 * 1024 )    /* 12KB */

/* Size of the RTU PRU instruction memory in bytes. */
#define RTUPRU_IRAM_SIZE   ( 8 * 1024 )    /* 8KB */

/* Size of the TX PRU instruction memory in bytes.*/
#define TXPRU_IRAM_SIZE   ( 6 * 1024 )    /* 6KB */

#define ENDAT0_PRUICSS_SLICEx CONFIG_ENDAT0_PRUICSS_PRUx

#if (ENDAT0_PRUICSS_SLICEx == 1)
#define ENDAT0_PRUICSS_PRUx PRUICSS_PRU1
#ifndef PRUICSSM
#define ENDAT0_PRUICSS_TXPRUx PRUICSS_TX_PRU1
#define ENDAT0_PRUICSS_RTUPRUx PRUICSS_RTU_PRU1
#endif
#else
#define ENDAT0_PRUICSS_PRUx PRUICSS_PRU0
#ifndef PRUICSSM
#define ENDAT0_PRUICSS_TXPRUx PRUICSS_TX_PRU0
#define ENDAT0_PRUICSS_RTUPRUx PRUICSS_RTU_PRU0
#endif
#endif

/*define macros for dual channels*/
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)

#define ENDAT1_PRUICSS_SLICEx CONFIG_ENDAT1_PRUICSS_PRUx

#if (ENDAT1_PRUICSS_SLICEx == 1)
#define ENDAT1_PRUICSS_PRUx PRUICSS_PRU1
#ifndef PRUICSSM
#define ENDAT1_PRUICSS_TXPRUx PRUICSS_TX_PRU1
#define ENDAT1_PRUICSS_RTUPRUx PRUICSS_RTU_PRU1
#endif
#else
#define ENDAT1_PRUICSS_PRUx PRUICSS_PRU0
#ifndef PRUICSSM
#define ENDAT1_PRUICSS_TXPRUx PRUICSS_TX_PRU0
#define ENDAT1_PRUICSS_RTUPRUx PRUICSS_RTU_PRU0
#endif
#endif 

__attribute__((section(".gEnDat1ChInfo")))Endat_ChRxInfoArray gEndat1ChInfo;

Endat_Handle gEndatHandle2;

static uint32_t gEndat1_prop_delay[3];
static uint32_t gEndat1_prop_delay_max;
static uint8_t gEndat1_is_multi_ch;
static uint8_t gEndat1_multi_ch_mask;
static uint8_t  gEndat1_is_load_share_mode;

/* Add separate global variables for second channel */
static Endat_FormatData gEndat1_format_data_mtrctrl[3];
static uint32_t gEndat1_mtrctrl_crc_err[3];
static uint32_t gEndat1_2_2_crc_position_err_cnt[3];
static uint32_t gEndat1_2_2_crc_addinfo1_err_cnt[3];
static uint64_t gEndat1_2_2_pos_val2[3];
static int32_t gEndat1_2_2_loop_mrs;
static void (*endat1_fn_position_loop)(unsigned int);
#endif 

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
#if ENDAT0_PRUICSS_SLICEx == 1
#include <endat_receiver_multi_pru1_bin.h>
#else
#include <endat_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if ENDAT0_PRUICSS_SLICEx == 1
#include <endat_receiver_multi_rtu_pru1_bin.h>
#include <endat_receiver_multi_pru1_bin.h>
#include <endat_receiver_multi_tx_pru1_bin.h>
#else
#include <endat_receiver_multi_rtu_pru0_bin.h>
#include <endat_receiver_multi_pru0_bin.h>
#include <endat_receiver_multi_tx_pru0_bin.h>
#endif
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
#if ENDAT0_PRUICSS_SLICEx == 1
#include <endat_receiver_pru1_bin.h>
#else
#include <endat_receiver_pru0_bin.h>
#endif
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
#if ENDAT1_PRUICSS_SLICEx == 1
#include <endat_receiver_pru1_bin.h>
#else
#include <endat_receiver_pru0_bin.h>
#endif
#endif


#define WAIT_5_SECOND  (5000)
#define TASK_STACK_SIZE (4096)
#define TASK_PRIORITY   (6)

#define ENDAT_RX_SAMPLE_SIZE    7
#define ENDAT_RX_SESQUI_DIV (1 << 15)

#define MRS_POS_VAL2_WORD1  0x42
#define MRS_POS_VAL2_WORD2  0x43
#define MRS_POS_VAL2_WORD3  0x44

#if defined(SOC_AM261X) || defined(SOC_AM263X) || defined(SOC_AM263PX)
/* Translate the TCM local view addr to SoC view addr */
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5SS0_CORE0_TCMB_U_BASE+(x - CSL_MSS_TCMB_RAM_BASE))
#else
/* Translate the TCM local view addr to SoC view addr */
#define CPU0_ATCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_ATCM_BASE+(x))
#define CPU1_ATCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_ATCM_BASE+(x))
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))
#define CPU1_BTCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_BTCM_BASE+(x - CSL_R5FSS1_BTCM_BASE))
#endif

/*Use soc driver instead it when available */
#if SOC_AM263PX
/**
 *  \anchor TCA6416_Mode
 *  \name IO pin mode - Input or Output
 *  @{
 */
/** \brief Configure IO pin as input */
#define TCA6416_MODE_INPUT              (0U)
/** \brief Configure IO pin as output */
#define TCA6416_MODE_OUTPUT             (1U)
/** @} */

/**
 *  \anchor TCA6416_OutState
 *  \name IO pin output state - HIGH or LOW
 *  @{
 */
/** \brief Configure IO pin output as LOW */
#define TCA6416_OUT_STATE_LOW           (0U)
/** \brief Configure IO pin output as HIGH */
#define TCA6416_OUT_STATE_HIGH          (1U)
/** @} */


#define TCA6416_REG_INPUT_PORT_0        (0x00U)
#define TCA6416_REG_INPUT_PORT_1        (0x01U)
#define TCA6416_REG_OUTPUT_PORT_0       (0x02U)
#define TCA6416_REG_OUTPUT_PORT_1       (0x03U)
#define TCA6416_REG_POL_INV_PORT_0      (0x04U)
#define TCA6416_REG_POL_INV_PORT_1      (0x05U)
#define TCA6416_REG_CONFIG_PORT_0       (0x06U)
#define TCA6416_REG_CONFIG_PORT_1       (0x07U)
#endif

static Endat_FormatData gEndat_format_data_mtrctrl[3];
static uint32_t gEndat_mtrctrl_crc_err[3];
static uint32_t gEndat_2_2_crc_position_err_cnt[3];
static uint32_t gEndat_2_2_crc_addinfo1_err_cnt[3];
static uint64_t gEndat_2_2_pos_val2[3];
static int32_t gEndat_2_2_loop_mrs;
static void (*endat_fn_position_loop)(unsigned int);

uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

#define VALID_CONT_MODE_CMD(x) ((x) == 101 || (x) == 104 || (x) == 107 || (x) == 111 || (x) == 200)

#define VALID_HOST_CMD(x) ((x == 100) || ((x) == 102) || ((x) == 103) || ((x) == 105) || \
                           ((x) == 106) || ((x) == 108) || ((x) == 109) || ((x) == 110) || ((x)== 112))

#define HAVE_COMMAND_SUPPLEMENT(x) (((x) == 2) || ((x) == 3) || ((x) == 4) || ((x) == 7) || \
                                    ((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 13) || ((x) == 14) || \
                                    ((x) == 100) || ((x) == 101) || ((x)== 103) || ((x) == 105) || ((x) == 106) || ((x) == 107) || ((x) == 108) || ((x) == 109)  || ((x) == 200) || ((x) == 112))


#define ICSS_PRU_CORE_CLOCK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#define ENDAT_INPUT_CLOCK_UART_FREQUENCY CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ

#if CONFIG_ENDAT0_TX_RX_FIFO_CLOCK_SOURCE == 1
#define ENDAT_RX_INPUT_CLOCK_FREQUENCY ICSS_PRU_CORE_CLOCK
#define ENDAT_TX_INPUT_CLOCK_FREQUENCY ICSS_PRU_CORE_CLOCK
#else
#define ENDAT_RX_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY
#define ENDAT_TX_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY
#endif

#define ENDAT_POSITION_LOOP_STOP    0
#define ENDAT_POSITION_LOOP_START   1


union position
{
    float angle;
    uint64_t length;
};

Endat_Handle gEndatHandle1;

#define PRUSS_PRU1_CTRL_CTPPR1      (CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE + 0x2C)

#define ENDAT_MULTI_CH0 (1 << 0)
#define ENDAT_MULTI_CH1 (1 << 1)
#define ENDAT_MULTI_CH2 (1 << 2)
static uint32_t gEndat_prop_delay[3];
static uint32_t gEndat_prop_delay_max;
static uint8_t gEndat_is_multi_ch;
static uint8_t gEndat_multi_ch_mask;
static uint8_t  gEndat_is_load_share_mode;
static char gUart_buffer[256];

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcssXHandle;

/* buffer to handle long long printf */
char gPrintf_dump_buffer[21];

/* This function is used to convert uint64_t
 * variable tp a string as printf doesn't support
 * printing 64-bit variables
 */

/* EnDat channel Info, written by PRU cores */
__attribute__((section(".gEnDatChInfo"))) Endat_ChRxInfoArray gEndatChInfo;
char * uint64_to_str (uint64_t x)
{
    char *b = gPrintf_dump_buffer + sizeof(gPrintf_dump_buffer);
    *(--b) = '\0';
    do
    {
        *(--b) = '0' + (x % 10);
        x /= 10;
    } while (x);

    return b;
}

static void endat_pruicss_init(void)
{

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
     /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
#ifdef CONFIG_ENDAT0_G_MUX_EN
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
#endif
    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));

#ifdef CONFIG_ENDAT1_LOAD_SHARE_MODE

    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_RTUPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
    /*
    * Set the constant table C28 for tx pru
    * configuring the constant table C28 to point to the TX counter
    * register (CNTR). The counter is needed in firmware for adding waits and time stemps.
    */
#if CONFIG_ENDAT1_PRUICSSx == 1
#if ENDAT1_PRUICSS_SLICEx == 1
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA58);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA50);
#endif /* ENDAT0_PRUICSS_SLICEx == 1 */
#else
#if ENDAT1_PRUICSS_SLICEx == 1
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
#endif /* ENDAT0_PRUICSS_SLICEx == 1 */
#endif /* CONFIG_ENDAT0_PRUICSSx == 1 */
#endif /* CONFIG_ENDAT0_LOAD_SHARE_MODE */

#endif /* ENDAT_DUAL_PRU_SLICE_ENABLE */

#ifdef CONFIG_ENDAT0_LOAD_SHARE_MODE

    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_RTUPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
    /*
    * Set the constant table C28 for tx pru
    * configuring the constant table C28 to point to the TX counter
    * register (CNTR). The counter is needed in firmware for adding waits and time stemps.
    */
#if CONFIG_ENDAT0_PRUICSSx == 1
#if ENDAT0_PRUICSS_SLICEx == 1
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA58);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA50);
#endif /* ENDAT0_PRUICSS_SLICEx == 1 */
#else
#if ENDAT0_PRUICSS_SLICEx == 1
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
#endif /* ENDAT0_PRUICSS_SLICEx == 1 */
#endif /* CONFIG_ENDAT0_PRUICSSx == 1 */
#endif

     /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(ENDAT0_PRUICSS_SLICEx));
#ifdef CONFIG_ENDAT0_LOAD_SHARE_MODE
        PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_RTUPRUx);
        PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx);
#endif
    PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(ENDAT1_PRUICSS_SLICEx));
#ifdef CONFIG_ENDAT1_LOAD_SHARE_MODE
    PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_RTUPRUx);
    PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx);
#endif
    PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_SLICEx);
#endif

}

void endat_pre_init(void)
{
    endat_pruicss_init();
}

uint32_t endat_pruicss_load_run_fw(Endat_Handle handle)
{
    uint32_t status = SystemP_FAILURE;

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU /*enable loadshare mode*/
    /* Set up variables based on PRU slice*/
    const uint32_t *rtuFirmware;
    const uint32_t *pruFirmware;
    const uint32_t *txFirmware;
    uint32_t rtuFirmwareSize;
    uint32_t pruFirmwareSize;
    uint32_t txFirmwareSize;

#if ENDAT0_PRUICSS_SLICEx == 1
    rtuFirmware = EnDatFirmwareMultiMakeRtuPru1_0;
    pruFirmware = EnDatFirmwareMultiMakePru1_0;
    txFirmware = EnDatFirmwareMultiMakeTxPru1_0;
    rtuFirmwareSize = sizeof(EnDatFirmwareMultiMakeRtuPru1_0);
    pruFirmwareSize = sizeof(EnDatFirmwareMultiMakePru1_0);
    txFirmwareSize = sizeof(EnDatFirmwareMultiMakeTxPru1_0);
#else
    rtuFirmware = EnDatFirmwareMultiMakeRtuPru0_0;
    pruFirmware = EnDatFirmwareMultiMakePru0_0;
    txFirmware = EnDatFirmwareMultiMakeTxPru0_0;
    rtuFirmwareSize = sizeof(EnDatFirmwareMultiMakeRtuPru0_0);
    pruFirmwareSize = sizeof(EnDatFirmwareMultiMakePru0_0);
    txFirmwareSize = sizeof(EnDatFirmwareMultiMakeTxPru0_0);
#endif

    /* Load and run RTU firmware when Channel 0 is enabled */
#if CONFIG_ENDAT0_CHANNEL0
    if(rtuFirmwareSize > RTUPRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: RTU Firmware binary size (%d) exceeds available IRAM size (%d)\n", rtuFirmwareSize, RTUPRU_IRAM_SIZE);
    }

    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(ENDAT0_PRUICSS_SLICEx),
                                0, (uint32_t *)rtuFirmware, rtuFirmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT0_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT0_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Load and run PRU firmware when Channel 1 is enabled */
#if CONFIG_ENDAT0_CHANNEL1
    if(pruFirmwareSize > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", pruFirmwareSize, PRU_IRAM_SIZE);
    }

    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(ENDAT0_PRUICSS_SLICEx),
                                0, (uint32_t *)pruFirmware, pruFirmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Load and run TX PRU firmware when Channel 2 is enabled*/
#if CONFIG_ENDAT0_CHANNEL2
    if(txFirmwareSize > TXPRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: TX PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", txFirmwareSize, TXPRU_IRAM_SIZE);
    }

    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(ENDAT0_PRUICSS_SLICEx),
                                0, (uint32_t *)txFirmware, txFirmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT0_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    status = endat_wait_initialization(handle, WAIT_5_SECOND, gEndat_multi_ch_mask);

#else
    const uint32_t *firmware;
    uint32_t firmwareSize;

#if(CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if ENDAT0_PRUICSS_SLICEx == 1
    firmware = EnDatFirmwareMultiPru1_0;
    firmwareSize = sizeof(EnDatFirmwareMultiPru1_0);
#else
    firmware = EnDatFirmwareMultiPru0_0;
    firmwareSize = sizeof(EnDatFirmwareMultiPru0_0);
#endif
#else
#if ENDAT0_PRUICSS_SLICEx == 1
    firmware = EnDatFirmwarePru1_0;
    firmwareSize = sizeof(EnDatFirmwarePru1_0);
#else
    firmware = EnDatFirmwarePru0_0;
    firmwareSize = sizeof(EnDatFirmwarePru0_0);
#endif
#endif

    /*Validate firmware size*/ 
    if(firmwareSize > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: Firmware binary size (%d) exceeds available IRAM size (%d)\n", firmwareSize, PRU_IRAM_SIZE);
    }

    /* Load and run firmware */
    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(ENDAT0_PRUICSS_SLICEx),
                               0, (uint32_t *)firmware, firmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);

    status = endat_wait_initialization(handle, WAIT_5_SECOND, gEndat_multi_ch_mask);
#endif

    return status;
}

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
uint32_t endat1_pruicss_load_run_fw(Endat_Handle handle)
{
    uint32_t status = SystemP_FAILURE;

#if CONFIG_ENDAT1_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU /*enable loadshare mode*/
    /* Set up variables based on PRU slice*/
    const uint32_t *rtuFirmware;
    const uint32_t *pruFirmware;
    const uint32_t *txFirmware;
    uint32_t rtuFirmwareSize;
    uint32_t pruFirmwareSize;
    uint32_t txFirmwareSize;

#if ENDAT1_PRUICSS_SLICEx == 1
    rtuFirmware = EnDatFirmwareMultiMakeRtuPru1_0;
    pruFirmware = EnDatFirmwareMultiMakePru1_0;
    txFirmware = EnDatFirmwareMultiMakeTxPru1_0;
    rtuFirmwareSize = sizeof(EnDatFirmwareMultiMakeRtuPru1_0);
    pruFirmwareSize = sizeof(EnDatFirmwareMultiMakePru1_0);
    txFirmwareSize = sizeof(EnDatFirmwareMultiMakeTxPru1_0);
#else
    rtuFirmware = EnDatFirmwareMultiMakeRtuPru0_0;
    pruFirmware = EnDatFirmwareMultiMakePru0_0;
    txFirmware = EnDatFirmwareMultiMakeTxPru0_0;
    rtuFirmwareSize = sizeof(EnDatFirmwareMultiMakeRtuPru0_0);
    pruFirmwareSize = sizeof(EnDatFirmwareMultiMakePru0_0);
    txFirmwareSize = sizeof(EnDatFirmwareMultiMakeTxPru0_0);
#endif

    /* Load and run RTU firmware when Channel 0 is enabled */
#if CONFIG_ENDAT1_CHANNEL0
    if(rtuFirmwareSize > RTUPRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: RTU Firmware binary size (%d) exceeds available IRAM size (%d)\n", rtuFirmwareSize, RTUPRU_IRAM_SIZE);
    }

    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(ENDAT1_PRUICSS_SLICEx),
                                0, (uint32_t *)rtuFirmware, rtuFirmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT1_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT1_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Load and run PRU firmware when Channel 1 is enabled */
#if CONFIG_ENDAT1_CHANNEL1
    if(pruFirmwareSize > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", pruFirmwareSize, PRU_IRAM_SIZE);
    }

    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(ENDAT1_PRUICSS_SLICEx),
                                0, (uint32_t *)pruFirmware, pruFirmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Load and run TX PRU firmware when Channel 2 is enabled*/
#if CONFIG_ENDAT1_CHANNEL2
    if(txFirmwareSize > TXPRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: TX PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", txFirmwareSize, TXPRU_IRAM_SIZE);
    }

    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(ENDAT1_PRUICSS_SLICEx),
                                0, (uint32_t *)txFirmware, txFirmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT1_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    status = endat_wait_initialization(handle, WAIT_5_SECOND, gEndat_multi_ch_mask);

#else
    const uint32_t *firmware;
    uint32_t firmwareSize;

#if(CONFIG_ENDAT1_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if ENDAT1_PRUICSS_SLICEx == 1
    firmware = EnDatFirmwareMultiPru1_0;
    firmwareSize = sizeof(EnDatFirmwareMultiPru1_0);
#else
    firmware = EnDatFirmwareMultiPru0_0;
    firmwareSize = sizeof(EnDatFirmwareMultiPru0_0);
#endif
#else
#if ENDAT1_PRUICSS_SLICEx == 1
    firmware = EnDatFirmwarePru1_0;
    firmwareSize = sizeof(EnDatFirmwarePru1_0);
#else
    firmware = EnDatFirmwarePru0_0;
    firmwareSize = sizeof(EnDatFirmwarePru0_0);
#endif
#endif

    /*Validate firmware size*/ 
    if(firmwareSize > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: Firmware binary size (%d) exceeds available IRAM size (%d)\n", firmwareSize, PRU_IRAM_SIZE);
    }

    /* Load and run firmware */
    status = PRUICSS_disableCore(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(ENDAT1_PRUICSS_SLICEx),
                               0, (uint32_t *)firmware, firmwareSize);
    DebugP_assert(0 != status);
    status = PRUICSS_resetCore(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, ENDAT1_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);

    status = endat_wait_initialization(handle, WAIT_5_SECOND, gEndat_multi_ch_mask);
#endif

    return status;
}

#endif

uint64_t endat_get_fw_version(void)
{
#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
    #if ENDAT0_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiPru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiPru0_0 + 2);
    #endif
#endif

#if (CONFIG_ENDAT0_CHANNEL0) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    #if ENDAT0_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiMakeRtuPru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiMakeRtuPru0_0 + 2);
    #endif
#endif

#if (CONFIG_ENDAT0_CHANNEL1) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    #if ENDAT0_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiMakePru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiMakePru0_0 + 2);
    #endif
#endif

#if (CONFIG_ENDAT0_CHANNEL2) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    #if ENDAT0_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiMakeTxPru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiMakeTxPru0_0 + 2);
    #endif
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
    #if ENDAT0_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwarePru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwarePru0_0 + 2);
    #endif
#endif

    /* Default return in case no conditions are met */
    return 0;
}

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
uint64_t endat1_get_fw_version(void)
{
#if CONFIG_ENDAT1_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
    #if ENDAT1_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiPru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiPru0_0 + 2);
    #endif
#endif

#if (CONFIG_ENDAT1_CHANNEL0) && (CONFIG_ENDAT1_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    #if ENDAT1_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiMakeRtuPru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiMakeRtuPru0_0 + 2);
    #endif
#endif

#if (CONFIG_ENDAT1_CHANNEL1) && (CONFIG_ENDAT1_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    #if ENDAT1_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiMakePru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiMakePru0_0 + 2);
    #endif
#endif

#if (CONFIG_ENDAT1_CHANNEL2) && (CONFIG_ENDAT1_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    #if ENDAT1_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwareMultiMakeTxPru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwareMultiMakeTxPru0_0 + 2);
    #endif
#endif

#if CONFIG_ENDAT1_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
    #if ENDAT1_PRUICSS_SLICEx == 1
        return *((unsigned long *)EnDatFirmwarePru1_0 + 2);
    #else
        return *((unsigned long *)EnDatFirmwarePru0_0 + 2);
    #endif
#endif

    /* Default return in case no conditions are met */
    return 0;
}
#endif

static void endat_print_menu(void)
{
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r|              Select value for the encoder command from following             |\n");
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| 1 : Encoder send position values                                             |\n");
    DebugP_log("\r| 2 : Selection of memory area                                                 |\n");
    DebugP_log("\r| 3 : Encoder receive parameter                                                |\n");
    DebugP_log("\r| 4 : Encoder send parameter                                                   |\n");
    DebugP_log("\r| 5 : Encoder receive reset                                                    |\n");
    DebugP_log("\r| 6 : Encoder send test values                                                 |\n");
    DebugP_log("\r| 7 : Encoder receive test command                                             |\n");
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(gEndatHandle1->cmd_set_2_2 && gEndatHandle2->cmd_set_2_2)
#else
    if(gEndatHandle1->cmd_set_2_2)
#endif
    {
        DebugP_log("\r| 8 : Encoder to send position + AI(s)                                         |\n");
        DebugP_log("\r| 9 : Encoder to send position + AI(s) and receive selection of memory area    |\n");
        DebugP_log("\r| 10: Encoder to send position + AI(s) and receive parameter                   |\n");
        DebugP_log("\r| 11: Encoder to send position + AI(s) and send parameter                      |\n");
        DebugP_log("\r| 12: Encoder to send position + AI(s) and receive error reset                 |\n");
        DebugP_log("\r| 13: Encoder to send position + AI(s) and receive test command                |\n");
        DebugP_log("\r| 14: Encoder receive communication command                                    |\n");
    }

    DebugP_log("\r|                                                                              |\n");
    DebugP_log("\r|100: Configure clock                                                          |\n");
    DebugP_log("\r|101: Simulate motor control 2.1 position loop                                 |\n");
    DebugP_log("\r|102: Toggle raw data display                                                  |\n");
    DebugP_log("\r|103: Configure tST delay                                                      |\n");
    DebugP_log("\r|104: Start continuous mode                                                    |\n");


    DebugP_log("\r|105: Configure rx arm counter (account tD)                                    |\n");
    DebugP_log("\r|106: Configure rx clock disable time (for tD)                                 |\n");

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(gEndatHandle1->cmd_set_2_2 && gEndatHandle2->cmd_set_2_2)
#else
    if(gEndatHandle1->cmd_set_2_2)
#endif
    {
        DebugP_log("\r|107: Simulate motor control 2.2 position loop %s                        |\n",
                    gEndat_is_multi_ch || gEndat_is_load_share_mode ? "        " : "(safety)");
    }

    DebugP_log("\r|108: Configure propagation delay (tD)                                         |\n");
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(((gEndat_is_multi_ch) || (gEndat_is_load_share_mode)) && (gEndat1_is_multi_ch || gEndat1_is_load_share_mode))
#else
    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
#endif
    {
        DebugP_log("\r|109: Configure wire delay                                                     |\n");
    }

    DebugP_log("\r|110: Recovery Time (RT)                                                       |\n");
    DebugP_log("\r|111: Simulate motor control 2.1 position loop for long time                   |\n");
    DebugP_log("\r|112: Start/Stop Recovery Time measurement                                     |\n");
    DebugP_log("\r|200: Start periodic continuous mode                                           |\n");

    DebugP_log("\r|------------------------------------------------------------------------------|\n\r|\n");
    DebugP_log("\r| enter value: ");
}

static inline int32_t endat_get_command(void)
{
    volatile int32_t cmd = -1;


    if(DebugP_scanf("%d", &cmd) < 0)
    {
        cmd = -1;
    }

    if(VALID_2_1_CMD(cmd) || (gEndatHandle1->cmd_set_2_2 && VALID_2_2_CMD(cmd))
            || VALID_HOST_CMD(cmd) || VALID_CONT_MODE_CMD(cmd))
    {
        return cmd;
    }

    DebugP_log("\r| WARNING: invalid command, EnDat 2.1 send position values command will be sent\n");
    return 1;
}

static void endat_recvd_print(int32_t cmd, Endat_Handle handle,
                              Endat_FormatData *u, int32_t crc)
{
    uint32_t addinfo, byte1;
    uint64_t max = pow(2, handle->single_turn_res[handle->current_channel]);
    union position position;

    /* this would give wrong values if cmd is not position related, but that is okay as then this value won't be used */
    if(handle->type[handle->current_channel] == rotary)
    {
        position.angle = ((float) u->position_addinfo.position.position) /
                         (float)max * (float)360;
    }

    else
    {
        position.length = u->position_addinfo.position.position * handle->step[handle->current_channel];
    }


    DebugP_log("\r|\n\r|\n");

    switch(cmd)
    {
        case 2:
        case 3:
        case 5:
        case 7:
        case 14:
            DebugP_log("\r| crc: %s\n", crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("| crc: %x\n", u->addr_params.crc);
            break;

        case 4:
            DebugP_log("\r| parameter: 0x%x, crc: %s\n", u->addr_params.params,
                        crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("\r| crc: %x\n", u->addr_params.crc);
            break;

        case 6:
            DebugP_log("\r| test value: 0x%02x%08x, crc: %s\n",
                        (uint32_t)((u->test.value & 0xFF00000000) >> 32),
                        (uint32_t)(u->test.value & 0xFFFFFFFF), crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("\r| crc: %x\n", u->test.crc);
            break;

        case 1:
            if(handle->multi_turn_res[handle->current_channel])
            {
                sprintf(gUart_buffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(handle->type[handle->current_channel] == rotary)
                {
                    sprintf(gUart_buffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUart_buffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUart_buffer);

            DebugP_log("f1: %x, crc: %s\n", u->position_addinfo.position.f1,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            if(handle->multi_turn_res[handle->current_channel])
            {
                sprintf(gUart_buffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(handle->type[handle->current_channel] == rotary)
                {
                    sprintf(gUart_buffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUart_buffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUart_buffer);

            DebugP_log("f1/f2: %x/%x, crc: %s\n", u->position_addinfo.position.f1,
                        u->position_addinfo.position.f2,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);

            if(handle->flags.info1)
            {
                addinfo = u->position_addinfo.addinfo1.addinfo;
                byte1 = (addinfo >> 16) & ((1 << 8) - 1);

                DebugP_log("\r|\n| WRN: %x  RM: %x  Busy: %x  I4-I0: %x\n",
                            (byte1 & ENDAT_STATUS_WARN_MASK) >> ENDAT_STATUS_WARN_SHIFT,
                            (byte1 & ENDAT_STATUS_RM_MASK) >> ENDAT_STATUS_RM_SHIFT,
                            (byte1 & ENDAT_STATUS_BUSY_MASK) >> ENDAT_STATUS_BUSY_SHIFT,
                            byte1 & ENDAT_INFORMATION_MASK);
                DebugP_log("\r|\n| Additional Information 1: 0x%x, crc: %s\n", addinfo,
                            crc & 0x2 ? "success" : "failure");
                DebugP_logInfo("| addinfo1 crc: %x\n",
                                u->position_addinfo.addinfo1.crc);
            }

            if(handle->flags.info2)
            {
                addinfo = u->position_addinfo.addinfo2.addinfo;
                byte1 = (addinfo >> 16) & ((1 << 8) - 1);

                DebugP_log("\r|\n| Additional Information 2: 0x%x, crc: %s\n", addinfo,
                            crc & 0x4 ? "success" : "failure");
                DebugP_logInfo("| addinfo2 crc: %x\n",
                                u->position_addinfo.addinfo2.crc);
            }

            break;

        default:
            DebugP_log("\r|\n| ERROR: print requested for invalid command\n");
            break;
    }

    DebugP_log("\r|\n\r|\n");
}

static void endat_display_raw_data(int32_t cmd, Endat_Handle handle)
{
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    switch(cmd)
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
            DebugP_log("\r|\n| raw data: %x %x %x %x\n|\n",
                        channel_rx_info->ch[ch].posWord0, channel_rx_info->ch[ch].posWord1,
                        channel_rx_info->ch[ch].posWord2, channel_rx_info->ch[ch].posWord3);
            break;

        default:
            DebugP_log("\r|\n| Nothing raw to display - this is not a valid command\n|\n");
            break;
    }
}

/*
 * check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that this function will be called after endat_get_command
 */
static int32_t endat_get_command_supplement(int32_t cmd,
                                        Endat_CmdSupplement *cmd_supplement)
{
    /* erase previous cmd supplements */
    memset(cmd_supplement, 0, sizeof(*cmd_supplement));

    switch(cmd)
    {
        case 2:
            DebugP_log("\r| enter MRS code (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 9:
            DebugP_log("\r| enter MRS code (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address == ENDAT_SECTION2_MEMORY)
            {
                DebugP_log("\r| enter block address (hex value): ");

                if(DebugP_scanf("%x\n", &cmd_supplement->block) < 0)
                {
                    DebugP_log("\r| ERROR: invalid block address\n|\n|\n|\n");
                    return -EINVAL;
                }

                /* better compare it with number of blocks information available in eeprom */
                if(cmd_supplement->block > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid block address\n|\n|\n|\n");
                    return -EINVAL;
                }

                cmd_supplement->has_block_address = TRUE;
            }

            break;

        case 3:
        case 10:
            DebugP_log("\r| enter parameter address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter parameter (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->data) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->data > 0xFFFF)
            {
                DebugP_log("\r| ERROR: invalid parameter\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 4:
        case 11:
            DebugP_log("\r| enter parameter address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 7:
        case 13:
            DebugP_log("\r| enter port address (hex value):");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid port address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("| ERROR: invalid port address\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 14:
            DebugP_log("\r| enter encoder address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid encoder address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid encoder address\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter instruction (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->data) < 0)
            {
                DebugP_log("\r| ERROR: invalid instruction\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->data > 0xFFFF)
            {
                DebugP_log("\r| ERROR: invalid instruction\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 100:
        case 101:
        case 107:
            DebugP_log("\r| enter frequency in Hz: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid frequency\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 103:
            DebugP_log("\r| enter tST delay counter value in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle1->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<gEndatHandle1->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                DebugP_log("\r| Select 2nd Slice Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle2->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            
                if(!((gEndat1_multi_ch_mask) & (1<<gEndatHandle2->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#endif
            break;

        case 105:
            DebugP_log("\r| enter rx arm counter in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle1->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<gEndatHandle1->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                DebugP_log("\r| Select 2nd Slice Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle2->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            
                if(!((gEndat1_multi_ch_mask) & (1<<gEndatHandle2->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#endif
            break;

        case 106:
            DebugP_log("\r| enter rx clock disable time (for tD) in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle1->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<gEndatHandle1->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                DebugP_log("\r| Select 2nd Slice Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle2->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            
                if(!((gEndat1_multi_ch_mask) & (1<<gEndatHandle2->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#endif
            break;

        case 108:
            DebugP_log("\r| enter propagation delay in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle1->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<gEndatHandle1->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                DebugP_log("\r| Select 2nd Slice Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle2->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            
                if(!((gEndat1_multi_ch_mask) & (1<<gEndatHandle2->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#endif

            break;

        case 109:
            DebugP_log("\r| enter channel number: ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                return -EINVAL;
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(!(gEndat_multi_ch_mask & 1 << cmd_supplement->address) || 
               !(gEndat1_multi_ch_mask & 1 << cmd_supplement->address))
#else
            if(!(gEndat_multi_ch_mask & 1 << cmd_supplement->address))
#endif
            {
                DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter wire delay in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;
        case 112:
            DebugP_log("\r| enter 1 to enable recovery time measurement and 0 to disable recovery time measurement: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(cmd_supplement->frequency > 1)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                               return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle1->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<gEndatHandle1->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                DebugP_log("\r| Select 2nd Slice Channel: ");
                if(DebugP_scanf("%u\n", &gEndatHandle2->current_channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            
                if(!((gEndat1_multi_ch_mask) & (1<<gEndatHandle2->current_channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
#endif
            break;
        case 200:


            DebugP_log("\r| Enter IEP reset cycle count (must be greater than EnDat cycle time including timeout period, in IEP cycles): ");
            if(DebugP_scanf("%u\n", &cmd_supplement->iep_reset_count))
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(gEndat_is_load_share_mode && gEndat1_is_load_share_mode)
#else
            if(gEndat_is_load_share_mode)
#endif
            {
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                if(gEndat_multi_ch_mask & (1<<0) || gEndat1_multi_ch_mask & (1<<0))
#else
                if(gEndat_multi_ch_mask & (1<<0))
#endif
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel0: \n");
                    if(DebugP_scanf("%u\n", &cmd_supplement->ch0_trigger_count) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        return -EINVAL;
                    }
                }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                if(gEndat_multi_ch_mask & (1<<1) || gEndat1_multi_ch_mask & (1<<1))
#else
                if(gEndat_multi_ch_mask & (1<<1))
#endif
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel1: \n");
                    if(DebugP_scanf("%u\n", &cmd_supplement->ch1_trigger_count) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        return -EINVAL;
                    }
                }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                if(gEndat_multi_ch_mask & (1<<2) || gEndat1_multi_ch_mask & (1<<2))
#else
                if(gEndat_multi_ch_mask & (1<<2))
#endif
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel2: \n");
                    if(DebugP_scanf("%u\n", &cmd_supplement->ch2_trigger_count) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        return -EINVAL;
                    }
                }

            }
            else
            {
                DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
                if(DebugP_scanf("%u\n", &cmd_supplement->ch0_trigger_count) < 0)
                {
                    DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
            break;

        default:
            cmd = -EINVAL;
            DebugP_log("\r| ERROR: no command data required for the command\n");
            break;
    }

    return cmd;
}

static int32_t endat_handle_user(Endat_CmdSupplement *cmd_supplement)
{
    int32_t cmd;

    endat_print_menu();
    cmd = endat_get_command();

    if(HAVE_COMMAND_SUPPLEMENT(cmd))
    {
        return endat_get_command_supplement(cmd, cmd_supplement);
    }

    return cmd;
}

static int32_t endat_calc_clock(Endat_Handle handle, uint32_t freq, Endat_ClkCfg_Internal *clk_cfg)
{
    uint32_t ns;
    uint64_t rx_source_freq;
    uint64_t tx_source_freq;

    if(handle->clk_cfg->rx_clock_source == 1)
    {
        rx_source_freq = handle->pru_cfg.pru_clock;
    }
    else {
        rx_source_freq = handle->pru_cfg.uart_clock;
    }
    if(handle->clk_cfg->tx_clock_source == 1)
    {
        tx_source_freq = handle->pru_cfg.pru_clock;
    }
    else {
        tx_source_freq = handle->pru_cfg.uart_clock;
    }

    if(freq > 16000000 || (freq > 12000000 && freq < 16000000))
    {
        DebugP_log("\r| ERROR: frequency above 16MHz, between 12 & 16MHz not allowed\n|\n|\n");
        return -1;
    }

    if((freq != 16000000) && (rx_source_freq % (freq * 8))&&(tx_source_freq % (freq)))
        DebugP_log("\r| WARNING: exact clock divider is not possible, frequencies set would be tx: %u\trx: %u\n",
                    tx_source_freq / (tx_source_freq / freq),
                    rx_source_freq / (rx_source_freq / (freq * 8)));

    ns = ENDAT_DELAY_COUNTER_INCREMENT*(2*ICSS_PRU_CORE_CLOCK/freq); /* rx arm >= 2 clock */

    /* should be divisible by 5 */
    if(ns % 5)
    {
        ns /= 5, ns += 1,  ns *= 5;
    }

    clk_cfg->tx_div = tx_source_freq / freq - 1;
    clk_cfg->rx_div = rx_source_freq / (freq * 8) - 1;
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

static uint32_t endat_do_sanity_tst_delay(uint32_t delay)
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

static int32_t endat_position_loop_status;

/* position period in microsecond */
static int32_t endat_calc_position_period(uint32_t freq)
{
    /* 16KHz limitation due to the timer */
    if(freq > 16000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 16KHz\n|\n|\n");
        return -1;
    }
    else if((gEndat_is_multi_ch || gEndat_is_load_share_mode)&& freq > 8000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 8KHz in multi channel configuration\n|\n|\n");
        return -1;
    }

    return 1000000 / freq;
}

static void endat_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        endat_position_loop_status = ENDAT_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

void endat_process_position_command(uint32_t a0)
{
    uint32_t crc;

    endat_command_process(gEndatHandle1, 1, NULL);
    endat_recvd_process(gEndatHandle1, 1, &gEndat_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(gEndatHandle1, 1, &gEndat_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat_mtrctrl_crc_err[a0]++;
    }
}

uint16_t _endat_process_2_2_position_command(int32_t cmd,
        Endat_CmdSupplement *cmd_supplement, uint32_t a0)
{
    uint32_t crc;

    endat_command_process(gEndatHandle1, cmd, cmd_supplement);
    endat_recvd_process(gEndatHandle1, cmd, &gEndat_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(gEndatHandle1, cmd, &gEndat_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat_2_2_crc_position_err_cnt[a0]++;
    }

    if(gEndatHandle1->flags.info1 && !(crc & 0x2))
    {
        gEndat_2_2_crc_addinfo1_err_cnt[a0]++;
    }

    endat_addinfo_track(gEndatHandle1, cmd, cmd_supplement);

    return gEndat_format_data_mtrctrl[a0].position_addinfo.addinfo1.addinfo & 0xFFFF;
}

void endat_process_2_2_position_command(uint32_t a0)
{
    uint32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint16_t pos_word;

    if(((!gEndat_is_multi_ch || !gEndat_is_load_share_mode) && gEndatHandle1->has_safety[gEndatHandle1->current_channel]) )
    {
        cmd = 9, cmd_supplement.address = gEndat_2_2_loop_mrs;
    }
    else
    {
        cmd = 8;
    }

    pos_word = _endat_process_2_2_position_command(cmd, &cmd_supplement, a0);

    if((gEndat_is_multi_ch) || (!gEndatHandle1->has_safety[gEndatHandle1->current_channel]) || (gEndat_is_load_share_mode))
    {
        return;
    }

    /* WORD3 in addinfo1 */
    if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD1)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFF0000FFFFFFFF;
        gEndat_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 32;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD2;
        /* WORD1 in addinfo1 */
    }
    else if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD2)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFFFFFFFFFF0000;
        gEndat_2_2_pos_val2[a0] |= (uint64_t)pos_word;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD3;
        /* WORD2 in addinfo1 */
    }
    else if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD3)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFFFFFF0000FFFF;
        gEndat_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 16;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD1;
    }
}

void endat_position_loop(uint32_t a0)
{
    if(endat_fn_position_loop != NULL)
    {
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(gEndatHandle1, j);
                    endat_fn_position_loop(j);
                }
            }
        }
        else
        {
            endat_fn_position_loop(0);
        }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        /* Process second channel */
        if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
        {
            int32_t j;

            for(j = 0; j < 3; j++)
            {
                if(gEndat1_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(gEndatHandle2, j);
                    endat1_fn_position_loop(j);
                }
            }
        }
        else
        {
            endat1_fn_position_loop(0);
        }
#endif
    }
}
/* New function to print the appropriate header based on channel configurations */
static void endat_print_position_header(Endat_Handle handle, int32_t continuous,
                                        int32_t is_2_2, int32_t channel_mask)
{
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(handle->instance_index == 0)
    {
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            /*For multi-channel, we print a general header that accommodates all channels*/ 
            if(continuous)
            {
                DebugP_log("         position,    [revolution],  f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
            else
            {
                DebugP_log("         position,    [revolution], crc errors, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
        }
        else
        {
            /* For single channel, we check the specific channel configuration */
            if(handle->multi_turn_res[handle->current_channel])
            {
                if(continuous)
                {
                    DebugP_log("         position,       revolution, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
                else
                {
                    DebugP_log("         position,       revolution, crc errors, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
            }
            else
            {
                if(continuous)
                {
                    DebugP_log("         position, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
                else
                {
                    DebugP_log("         position, crc errors, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
            }
        }
    }
    else
    {
        if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
        {
            /*For multi-channel, we print a general header that accommodates all channels*/ 
            if(continuous)
            {
                DebugP_log("         position,    [revolution],  f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
            else
            {
                DebugP_log("         position,    [revolution], crc errors, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
        }
        else
        {
            /* For single channel, we check the specific channel configuration */
            if(handle->multi_turn_res[handle->current_channel])
            {
                if(continuous)
                {
                    DebugP_log("         position,       revolution, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
                else
                {
                    DebugP_log("         position,       revolution, crc errors, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
            }
            else
            {
                if(continuous)
                {
                    DebugP_log("         position, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
                else
                {
                    DebugP_log("         position, crc errors, f1");
                    if(is_2_2)
                    {
                        DebugP_log(", f2");
                    }
                    DebugP_log("\r\n| ");
                }
            }
        }
    }
#else
    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        /*For multi-channel, we print a general header that accommodates all channels*/ 
        if(continuous)
        {
            DebugP_log("         position,    [revolution],  f1");
            if(is_2_2)
            {
                DebugP_log(", f2");
            }
            DebugP_log("\r\n| ");
        }
        else
        {
            DebugP_log("         position,    [revolution], crc errors, f1");
            if(is_2_2)
            {
                DebugP_log(", f2");
            }
            DebugP_log("\r\n| ");
        }
    }
    else
    {
        /* For single channel, we check the specific channel configuration */
        if(handle->multi_turn_res[handle->current_channel])
        {
            if(continuous)
            {
                DebugP_log("         position,       revolution, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
            else
            {
                DebugP_log("         position,       revolution, crc errors, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
        }
        else
        {
            if(continuous)
            {
                DebugP_log("         position, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
            else
            {
                DebugP_log("         position, crc errors, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
        }
    }
#endif
}

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)

/* Process second channel position command */
void endat1_process_position_command(uint32_t a0)
{
    uint32_t crc;

    endat_command_process(gEndatHandle2, 1, NULL);
    endat_recvd_process(gEndatHandle2, 1, &gEndat1_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(gEndatHandle2, 1, &gEndat1_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat1_mtrctrl_crc_err[a0]++;
    }
}

/* Process second channel EnDat 2.2 position command */
uint16_t _endat1_process_2_2_position_command(int32_t cmd,
        Endat_CmdSupplement *cmd_supplement, uint32_t a0)
{
    uint32_t crc;

    endat_command_process(gEndatHandle2, cmd, cmd_supplement);
    endat_recvd_process(gEndatHandle2, cmd, &gEndat1_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(gEndatHandle2, cmd, &gEndat1_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat1_2_2_crc_position_err_cnt[a0]++;
    }

    if(gEndatHandle2->flags.info1 && !(crc & 0x2))
    {
        gEndat1_2_2_crc_addinfo1_err_cnt[a0]++;
    }

    endat_addinfo_track(gEndatHandle2, cmd, cmd_supplement);

    return gEndat1_format_data_mtrctrl[a0].position_addinfo.addinfo1.addinfo & 0xFFFF;
}

void endat1_process_2_2_position_command(uint32_t a0)
{
    uint32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint16_t pos_word;

    if(((!gEndat1_is_multi_ch || !gEndat1_is_load_share_mode) && gEndatHandle2->has_safety[gEndatHandle2->current_channel]))
    {
        cmd = 9, cmd_supplement.address = gEndat1_2_2_loop_mrs;
    }
    else
    {
        cmd = 8;
    }

    pos_word = _endat1_process_2_2_position_command(cmd, &cmd_supplement, a0);

    if((gEndat1_is_multi_ch) || (!gEndatHandle2->has_safety[gEndatHandle2->current_channel]) || (gEndat1_is_load_share_mode))
    {
        return;
    }

    /* WORD3 in addinfo1 */
    if(gEndat1_2_2_loop_mrs == MRS_POS_VAL2_WORD1)
    {
        gEndat1_2_2_pos_val2[a0] &= 0xFFFF0000FFFFFFFF;
        gEndat1_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 32;
        gEndat1_2_2_loop_mrs = MRS_POS_VAL2_WORD2;
    }
    /* WORD1 in addinfo1 */
    else if(gEndat1_2_2_loop_mrs == MRS_POS_VAL2_WORD2)
    {
        gEndat1_2_2_pos_val2[a0] &= 0xFFFFFFFFFFFF0000;
        gEndat1_2_2_pos_val2[a0] |= (uint64_t)pos_word;
        gEndat1_2_2_loop_mrs = MRS_POS_VAL2_WORD3;
    }
    /* WORD2 in addinfo1 */
    else if(gEndat1_2_2_loop_mrs == MRS_POS_VAL2_WORD3)
    {
        gEndat1_2_2_pos_val2[a0] &= 0xFFFFFFFF0000FFFF;
        gEndat1_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 16;
        gEndat1_2_2_loop_mrs = MRS_POS_VAL2_WORD1;
    }
}

#endif
static int32_t endat_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "endat_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)endat_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\rTask2 creation failed\n");
    }

    return status ;
}

static void endat_loop_timer_create(int32_t us)
{
    TimerP_Params timerParams;

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = CONFIG_TIMER0_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_TIMER0_INPUT_CLK_HZ;
    timerParams.periodInUsec   = us;
    timerParams.oneshotMode    = 0;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);

    return ;
}

static int32_t endat_get_position_loop_chars(Endat_Handle handle,
        int32_t continuous, int32_t is_2_2)
{
    int32_t i;

    if(handle->multi_turn_res[handle->current_channel])
    {
        i = 34;
    }
    else
    {
        i = 16;
    }

    if(!continuous)
    {
        i += 12;
    }

    i += 4;

    if(!continuous && is_2_2)
    {
        i += 4;
    }

    return i;
}

/* Modified version of endat_print_position_loop to handle channel-specific data sources */
static void endat_print_position_loop(Endat_Handle handle, int32_t continuous,
                                      int32_t is_2_2, int32_t ch)
{
    uint64_t max = pow(2, handle->single_turn_res[handle->current_channel]);
    union position position;
    Endat_FormatData *data_source;

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /* Use instance_index to identify which channel data to use */
    if(handle->instance_index == 1) /* Second channel */
    {
        data_source = gEndat1_format_data_mtrctrl;
    }
    else /* First channel */
#endif
    {
        data_source = gEndat_format_data_mtrctrl;
    }

    if(handle->type[handle->current_channel] == rotary)
    {
        position.angle = ((float)
                          data_source[ch].position_addinfo.position.position) /
                         (float)max * (float)360;
    }
    else
    {
        position.length =
            data_source[ch].position_addinfo.position.position * handle->step[handle->current_channel];
    }

    /* max value is 2x48, has 15 digits, so 16 is safe */
    if(handle->multi_turn_res[handle->current_channel])
    {
        sprintf(gUart_buffer, "%16.12f, %16s", position.angle,
                uint64_to_str(data_source[ch].position_addinfo.position.revolution));
    }
    else
    {
        if(handle->type[handle->current_channel] == rotary)
        {
            sprintf(gUart_buffer, "%16.12f", position.angle);
        }
        else
        {
            sprintf(gUart_buffer, "%16s", uint64_to_str(position.length));
        }
    }

    DebugP_log("\r%s", gUart_buffer);

    if(!continuous)
    {
        if(is_2_2)
        {
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(handle->instance_index == 1) /* Second channel */
            {
                DebugP_log(", %10u", gEndat1_2_2_crc_position_err_cnt[ch]);
            }
            else /* First channel */
#endif
            {
                DebugP_log(", %10u", gEndat_2_2_crc_position_err_cnt[ch]);
            }
        }
        else
        {
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            if(handle->instance_index == 1) /* Second channel */
            {
                DebugP_log(", %10u", gEndat1_mtrctrl_crc_err[ch]);
            }
            else /* First channel */
#endif
            {
                DebugP_log(", %10u", gEndat_mtrctrl_crc_err[ch]);
            }
        }
    }

    DebugP_log(",%3u",
                data_source[ch].position_addinfo.position.f1);

    if(!continuous && is_2_2)
    {
        DebugP_log(",%3u",
                    data_source[ch].position_addinfo.position.f2);
    }
}

static void endat_print_position_loop_channel_info(Endat_Handle handle,
        int32_t is_2_2)
{
    int32_t k, j, i = endat_get_position_loop_chars(handle, 0, is_2_2);

    /* add 3 extra to account for spacing b/n channels */
    i += 3;
    /* find mid point */
    i /= 2;
    /* account for "CHANNEL x" display */
    i -= 4;

    for(k = i, j = 0; j < 3; j++, k = i)
        if(gEndat_multi_ch_mask & 1 << j)
        {
            while(k--)
            {
                DebugP_log("%c", ' ');
            }

            DebugP_log("\rCHANNEL %d", j);

            /* 3 - extra to account for spacing b/n channels, 9 - "CHANNEL x" length */
            k = endat_get_position_loop_chars(handle, 0, is_2_2) + 3 - i - 9;

            while(k--)
            {
                DebugP_log("%c", ' ');
            }
        }
}

static void endat_handle_prop_delay(Endat_Handle handle,
                                    uint16_t prop_delay)
{
    /*convert rx_en_cnt into ns */
    float ct = ((handle->rx_en_cnt/ENDAT_DELAY_COUNTER_INCREMENT)*((float)1000000000/handle->pru_cfg.pru_clock))/2; /*one endat clock cycle time = 1/endat frequency = 2*rx_en_cnt*/
    /* if propagation delay is more than half clock cycle time (2/endat frequency) then we have to reduce clock cycles for rx*/
    if(prop_delay > (ct/2))
    {
        uint16_t dis = floor(prop_delay/ct);
        /* convert propagation delay into rx arm counts */
        uint16_t temp = ((uint16_t)(((float)prop_delay * handle->pru_cfg.pru_clock )/1000000000)) * ENDAT_DELAY_COUNTER_INCREMENT;
        endat_config_rx_arm_cnt(handle, temp);
        /* propagation delay/cycle_time */
        endat_config_rx_clock_disable(handle, dis);
    }
    else
    {
        endat_config_rx_arm_cnt(handle, handle->rx_en_cnt);
        endat_config_rx_clock_disable(handle, 0);
    }
}

static void endat_process_host_command(int32_t cmd,
                                       Endat_CmdSupplement *cmd_supplement, Endat_Handle handle)
{
    Endat_ClkCfg_Internal clk_cfg;

    /* clock configuration */
    if(cmd == 100)
    {
        if(endat_calc_clock(handle, cmd_supplement->frequency, &clk_cfg) < 0)
        {
            return;
        }

        endat_config_clock(handle, &clk_cfg);

        handle->rx_en_cnt = clk_cfg.rx_en_cnt;

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        if(handle->instance_index == 0)
        {
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;
                uint16_t d;

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        endat_multi_channel_set_cur(handle, j);
                        endat_handle_prop_delay(handle, gEndat_prop_delay[handle->current_channel]);
                        d = gEndat_prop_delay_max - gEndat_prop_delay[j];
                        endat_config_wire_delay(handle, d);
                    }
                }
            }
            else
            {     

                endat_handle_prop_delay(handle, gEndat_prop_delay[handle->current_channel]);
            }   
        }
        else
        {
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                int32_t j;
                uint16_t d;

                for(j = 0; j < 3; j++)
                {
                    if(gEndat1_multi_ch_mask & 1 << j)
                    {
                        endat_multi_channel_set_cur(handle, j);
                        endat_handle_prop_delay(handle, gEndat1_prop_delay[handle->current_channel]);
                        d = gEndat1_prop_delay_max - gEndat1_prop_delay[j];
                        endat_config_wire_delay(handle, d);
                    }
                }
            }
            else
            {     

                endat_handle_prop_delay(gEndatHandle2, gEndat1_prop_delay[handle->current_channel]);
            }
            
        }
#else
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            uint16_t d;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(handle, j);
                    endat_handle_prop_delay(handle, gEndat_prop_delay[handle->current_channel]);
                    d = gEndat_prop_delay_max - gEndat_prop_delay[j];
                    endat_config_wire_delay(handle, d);
                }
            }
        }
        else
        {     

            endat_handle_prop_delay(handle, gEndat_prop_delay[handle->current_channel]);
        }
#endif
        /* set tST to 2us if frequency > 1MHz, else turn it off */
        if(cmd_supplement->frequency >= 1000000)
        {
            cmd_supplement->frequency = 2000;
        }
        else
        {
            cmd_supplement->frequency = 0;
        }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        if(handle->instance_index == 0)
        {
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;
                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        endat_multi_channel_set_cur(handle, j);
                        endat_process_host_command(103, cmd_supplement, handle);
                    }
               }
            }
            else
            {   
                endat_process_host_command(103, cmd_supplement, handle);
            }   
        }
        else
        {
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                int32_t j;
                for(j = 0; j < 3; j++)
                {
                    if(gEndat1_multi_ch_mask & 1 << j)
                    {
                        endat_multi_channel_set_cur(handle, j);
                        endat_process_host_command(103, cmd_supplement, handle);
                    }
               }
            }
            else
            {   
                endat_process_host_command(103, cmd_supplement, handle);
            }
        }
#else
        /* control loop */
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(handle, j);
                    endat_process_host_command(103, cmd_supplement, handle);
                }
           }
        }
        else
        {
            endat_process_host_command(103, cmd_supplement, handle);
        }
#endif
    }
    else if(cmd == 102)
    {
        handle->raw_data ^= 1;
    }
    else if(cmd == 103)
    {
        uint32_t delay;

        /* convert tst delay from ns to tst counts*/
        cmd_supplement->frequency = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->frequency * handle->pru_cfg.pru_clock)/1000000000));

        delay = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(delay <= (uint16_t)~0)
        {
            endat_config_tst_delay(handle, (uint16_t) delay);
        }
    }
    else if(cmd == 105)
    {
        uint32_t val;
        /* convert rx arm delay from ns to rx arm count*/
        cmd_supplement->frequency = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->frequency * handle->pru_cfg.pru_clock)/1000000000));

        /* reuse tST delay sanity check */
        val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(val <= (uint16_t)~0)
        {
            endat_config_rx_arm_cnt(handle, (uint16_t)val);
        }
    }
    else if(cmd == 106)
    {

        /*convert rx_en_cnt into 1 enadt clock cycle period */
        float ct = ((handle->rx_en_cnt/ENDAT_DELAY_COUNTER_INCREMENT)*((float)1000000000/handle->pru_cfg.pru_clock))/2;
        uint16_t dis = floor(cmd_supplement->frequency / ct);

        endat_config_rx_clock_disable(handle, dis);
    }
    else if(cmd == 108)
    {
        /* reuse tST delay sanity check */
        uint32_t val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(val > (uint16_t)~0)
        {
            return;
        }
        endat_handle_prop_delay(handle, (uint16_t)val);

    }
    else if(cmd == 109)
    {
        /* convert from ns to wire delay count*/
        cmd_supplement->frequency = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->frequency * handle->pru_cfg.pru_clock)/1000000000));
        /* reuse tST delay sanity check */
        uint32_t val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        endat_multi_channel_set_cur(handle, cmd_supplement->address);
        endat_config_wire_delay(handle, val);
    }
    else if(cmd == 110)
    {
        uint32_t recovery_time;
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        if(handle->instance_index == 0)
        {
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;
                for( j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        endat_multi_channel_set_cur(handle, j);
                        DebugP_log("channel: %d",handle->current_channel);
                        DebugP_log("\t");

                        recovery_time = endat_get_recovery_time(handle);
                        DebugP_log("\r Recovery Time: %10u ns \n", recovery_time);
                        DebugP_log("\r Current value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue);
                        DebugP_log("\r Previous value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue);
                        DebugP_log("\r Starting value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue);
                    }
                }
            }
            else
            {
                recovery_time = endat_get_recovery_time(handle);
                DebugP_log("\r Recovery Time: %10u ns \n", recovery_time);
                DebugP_log("\r Current value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue);
                DebugP_log("\r Previous value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue);
                DebugP_log("\r Starting value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue);
            }
        }
        else
        {
            if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
            {
                int32_t j;
                for( j = 0; j < 3; j++)
                {
                    if(gEndat1_multi_ch_mask & 1 << j)
                    {
                        endat_multi_channel_set_cur(handle, j);
                        DebugP_log("channel: %d",handle->current_channel);
                        DebugP_log("\t");
                        recovery_time = endat_get_recovery_time(handle);
                        DebugP_log("\r Recovery Time for 2nd slice: %10u ns \n", recovery_time);
                        DebugP_log("\r Current value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue);
                        DebugP_log("\r Previous value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue);
                        DebugP_log("\r Starting value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue);
                    }
                }
            }
            else
            {
                recovery_time = endat_get_recovery_time(handle);
                DebugP_log("\r Recovery Time for 2nd slice: %10u ns \n", recovery_time);
                DebugP_log("\r Current value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue);
                DebugP_log("\r Previous value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue);
                DebugP_log("\r Starting value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue);
            }
        }
#else
        if(gEndat_is_multi_ch||gEndat_is_load_share_mode)
        {
            int32_t j;
            for( j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(handle, j);
                    DebugP_log("channel: %d",handle->current_channel);
                    DebugP_log("\t");

                    recovery_time = endat_get_recovery_time(handle);
                    DebugP_log("\r Recovery Time: %10u ns \n", recovery_time);
                    DebugP_log("\r Current value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue);
                    DebugP_log("\r Previous value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue);
                    DebugP_log("\r Starting value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue);
                }
            }
        }
        else
        {
            recovery_time = endat_get_recovery_time(handle);
            DebugP_log("\r Recovery Time: %10u ns \n", recovery_time);
            DebugP_log("\r Current value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue);
            DebugP_log("\r Previous value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue);
            DebugP_log("\r Starting value of RT counter: %10u \n", handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue);
        }
#endif
    }
    else if(cmd == 112)
    {
        if(cmd_supplement->frequency == 1)
        {
            endat_enable_rt_measurement(handle);
        }
        else
        {
            endat_disable_rt_measurement(handle);
        }
    }
    else
    {
        DebugP_log("\r| ERROR: non host command being requested to be handled as host command\n|\n|\n");
    }
}
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
static void endat_process_continuous_mode_command(int32_t cmd,
                                      Endat_CmdSupplement *cmd_supplement, Endat_Handle handle, Endat_Handle gEndatHandle2,
                                      Endat_CmdSupplement *cmd_supplement_copy) 
#else
static void endat_process_continuous_mode_command(int32_t cmd,
                                      Endat_CmdSupplement *cmd_supplement, Endat_Handle handle)                                      
#endif                      
{
    static int32_t timer_init;

    /* Common task creation for all continuous mode commands */
    if(endat_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        return;
    }

    /* Set position loop status for all commands */
    endat_position_loop_status = ENDAT_POSITION_LOOP_START;

    /* Command 200: Start periodic continuous mode */
    if(cmd == 200)
    {
        endat_config_periodic_trigger(handle);
        int32_t status;
        int32_t pos_cmd = 1;
        DebugP_assert(endat_command_process(handle, pos_cmd, NULL) >= 0);


#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        endat_config_periodic_trigger(gEndatHandle2);
        DebugP_assert(endat_command_process(gEndatHandle2, pos_cmd, NULL) >= 0);
#endif

        struct endat_periodic_interface endat_periodic_interface;
        if(handle->pru_cfg.iep_instance == 0)
        {
            endat_periodic_interface.pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);
        }
        else
        {
            endat_periodic_interface.pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);
        }
        endat_periodic_interface.pruicss_dmem = handle->pruicss_xchg;
        endat_periodic_interface.load_share = handle->pru_cfg.load_share_enable;
        endat_periodic_interface.ch0_trigger_count = cmd_supplement->ch0_trigger_count;
        endat_periodic_interface.ch1_trigger_count = cmd_supplement->ch1_trigger_count;
        endat_periodic_interface.ch2_trigger_count = cmd_supplement->ch2_trigger_count;
        endat_periodic_interface.cmp0_count = cmd_supplement->iep_reset_count;

        status = endat_config_periodic_mode(&endat_periodic_interface, gPruIcssXHandle, handle);
        DebugP_assert(0 != status);

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        struct endat_periodic_interface endat_periodic_interface1;
        endat_periodic_interface1.pruicss_iep = endat_periodic_interface.pruicss_iep; /*IEP is common for both slice*/
        endat_periodic_interface1.pruicss_dmem = gEndatHandle2->pruicss_xchg;
        endat_periodic_interface1.load_share = gEndatHandle2->pru_cfg.load_share_enable;
        endat_periodic_interface1.ch0_trigger_count = cmd_supplement_copy->ch0_trigger_count;
        endat_periodic_interface1.ch1_trigger_count = cmd_supplement_copy->ch1_trigger_count;
        endat_periodic_interface1.ch2_trigger_count = cmd_supplement_copy->ch2_trigger_count;
        endat_periodic_interface1.cmp0_count = endat_periodic_interface.cmp0_count; /*IEP is common for both slice*/

        status = endat_config_periodic_mode(&endat_periodic_interface1, gPruIcssXHandle, gEndatHandle2);
        DebugP_assert(0 != status);
#endif

        DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n");

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                endat_stop_periodic_continuous_mode(&endat_periodic_interface);
                endat_config_host_trigger(handle);

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                endat_config_host_trigger(gEndatHandle2);
#endif
                return;
            }
            else
            {
                int32_t i = 0;

                /*  Process first channel */
                endat_print_position_header(handle, 1, 0, gEndat_multi_ch_mask);

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;
                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(handle, j);
                            endat_recvd_process(handle, 1, &gEndat_format_data_mtrctrl[j]);
                            i += endat_get_position_loop_chars(handle, 0, 0);
                            DebugP_log("| Ch1-%d: ", j);
                            endat_print_position_loop(handle, 1, 0, j);
                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    endat_recvd_process(handle, 1, &gEndat_format_data_mtrctrl[0]);
                    i = endat_get_position_loop_chars(handle, 1, 0);
                    endat_print_position_loop(handle, 1, 0, 0);
                    DebugP_log("\n| ");
                }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                /* Process second channel */
                DebugP_log("\r|\n\r| ---2nd Channel ---\r\n| ");
                endat_print_position_header(gEndatHandle2, 1, 0, gEndat1_multi_ch_mask);

                if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
                {
                    int32_t j;
                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat1_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(gEndatHandle2, j);
                            endat_recvd_process(gEndatHandle2, 1, &gEndat1_format_data_mtrctrl[j]);
                            i += endat_get_position_loop_chars(gEndatHandle2, 0, 0);
                            DebugP_log("| Ch2-%d: ", j);
                            endat_print_position_loop(gEndatHandle2, 1, 0, j);
                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    endat_recvd_process(gEndatHandle2, 1, &gEndat1_format_data_mtrctrl[0]);
                    i += endat_get_position_loop_chars(gEndatHandle2, 1, 0);
                    endat_print_position_loop(gEndatHandle2, 1, 0, 0);
                    DebugP_log("\n| ");
                }
#endif
                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);
                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }

    /* Command 101: Simulate motor control 2.1 position loop */
    else if(cmd == 101)
    {
        int32_t us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return;
        }

        endat_fn_position_loop = endat_process_position_command;
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        endat1_fn_position_loop = endat1_process_position_command;
#endif

        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

        DebugP_log("\r|\r\n| press enter to stop the position display|\n");

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
                return;
            }
            else
            {
                int32_t i = 0;

                /*  Process first channel */
                endat_print_position_header(handle, 0, 0, gEndat_multi_ch_mask);

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(handle, j);
                            i += endat_get_position_loop_chars(handle, 0, 0);
                            DebugP_log("| Ch1-%d: ", j);
                            endat_print_position_loop(handle, 0, 0, j);
                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i = endat_get_position_loop_chars(handle, 0, 0);
                    endat_print_position_loop(handle, 0, 0, 0);
                    DebugP_log("\n| ");
                }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                /*Process second channel*/
                DebugP_log("\r|\n\r| --- 2nd Channel  ---\r\n| ");
                endat_print_position_header(gEndatHandle2, 0, 0, gEndat1_multi_ch_mask);

                if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat1_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(gEndatHandle2, j);
                            i += endat_get_position_loop_chars(gEndatHandle2, 0, 0);
                            DebugP_log("| Ch2-%d: ", j);
                            endat_print_position_loop(gEndatHandle2, 0, 0, j);
                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i += endat_get_position_loop_chars(gEndatHandle2, 0, 0);
                    endat_print_position_loop(gEndatHandle2, 0, 0, 0);
                    DebugP_log("\n| ");
                }
#endif

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(500);

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }

    /* Command 104: Start continuous mode */
    else if(cmd == 104)
    {
        endat_start_continuous_mode(handle);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        endat_start_continuous_mode(gEndatHandle2);
#endif

        DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n");

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                endat_stop_continuous_mode(handle);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                endat_stop_continuous_mode(gEndatHandle2);
#endif
                return;
            }
            else
            {
                int32_t i = 0;

                /* Process first channel */
                endat_print_position_header(handle, 1, 0, gEndat_multi_ch_mask);

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;
                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(handle, j);
                            endat_recvd_process(handle, 1, &gEndat_format_data_mtrctrl[j]);
                            i += endat_get_position_loop_chars(handle, 0, 0);
                            DebugP_log("| Ch1-%d: ", j);
                            endat_print_position_loop(handle, 1, 0, j);
                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    endat_recvd_process(handle, 1, &gEndat_format_data_mtrctrl[0]);
                    i = endat_get_position_loop_chars(handle, 1, 0);
                    endat_print_position_loop(handle, 1, 0, 0);
                    DebugP_log("\n| ");
                }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                /* Process second channel */
                DebugP_log("\r|\n\r| --- 2nd Channel  ---\r\n| ");
                endat_print_position_header(gEndatHandle2, 1, 0, gEndat1_multi_ch_mask);

                if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
                {
                    int32_t j;
                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat1_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(gEndatHandle2, j);
                            endat_recvd_process(gEndatHandle2, 1, &gEndat1_format_data_mtrctrl[j]);
                            i += endat_get_position_loop_chars(gEndatHandle2, 0, 0);
                            DebugP_log("| Ch2-%d: ", j);
                            endat_print_position_loop(gEndatHandle2, 1, 0, j);
                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    endat_recvd_process(gEndatHandle2, 1, &gEndat1_format_data_mtrctrl[0]);
                    i += endat_get_position_loop_chars(gEndatHandle2, 1, 0);
                    endat_print_position_loop(gEndatHandle2, 1, 0, 0);
                    DebugP_log("\n| ");
                }
#endif
                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);
                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }

    /* Command 107: Simulate motor control 2.2 position loop */
    else if(cmd == 107)
    {
        int32_t us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return;
        }

        endat_fn_position_loop = endat_process_2_2_position_command;
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        endat1_fn_position_loop = endat1_process_2_2_position_command;
#endif

        /* This setup is similar to command 101 */
        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        /* reset additional info's if present */
        endat_command_process(handle, 5, NULL);
        endat_addinfo_track(handle, 5, NULL);

        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD1;

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        endat_command_process(gEndatHandle2, 5, NULL);
        endat_addinfo_track(gEndatHandle2, 5, NULL);

        gEndat1_2_2_loop_mrs = MRS_POS_VAL2_WORD1;
#endif

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        /* so that proper position value 2 is displayed from the begining */
        ClockP_usleep(us * 3);

        if((!gEndat_is_multi_ch && !handle->has_safety[handle->current_channel]) || (!gEndat_is_load_share_mode && !handle->has_safety[handle->current_channel]))
        {
            DebugP_log("\r|\n| encoder does not support safety, position value 2 would not be displayed\n|\n");
        }

        DebugP_log("\r|\n\r| press enter to stop the position display\n\r|\n");

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

                /* reset additional info1 */
                endat_command_process(handle, 5, NULL);
                endat_addinfo_track(handle, 5, NULL);

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                endat_command_process(gEndatHandle2, 5, NULL);
                endat_addinfo_track(gEndatHandle2, 5, NULL);
#endif
                return;
            }
            else
            {
                int32_t i = 0;

                /*Process first channel*/
                endat_print_position_header(handle, 0, 1, gEndat_multi_ch_mask);

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(handle, j);
                            i += endat_get_position_loop_chars(handle, 0, 1);
                            DebugP_log("| Ch1-%d: ", j);
                            endat_print_position_loop(handle, 0, 1, j);

                            if((!gEndat_is_multi_ch && handle->has_safety[handle->current_channel]) || (!gEndat_is_load_share_mode && handle->has_safety[handle->current_channel]))
                            {
                                uint64_t multi_turn, single_turn;
                                union position position2;
                                uint64_t max = pow(2, handle->single_turn_res[handle->current_channel]);

                                multi_turn = ENDAT_GET_POS_MULTI_TURN(gEndat_2_2_pos_val2[j], handle);
                                single_turn = ENDAT_GET_POS_SINGLE_TURN(gEndat_2_2_pos_val2[j], handle);

                                if(handle->type[handle->current_channel] == rotary)
                                {
                                    position2.angle = (float)single_turn / (float)max * (float)360;
                                }
                                else
                                {
                                    position2.length = single_turn * handle->step[handle->current_channel];
                                }

                                DebugP_log(", ");

                                if(handle->multi_turn_res[handle->current_channel])
                                {
                                    sprintf(gUart_buffer, "%16.12f, %16s", position2.angle, uint64_to_str(multi_turn));
                                }
                                else
                                {
                                    if(handle->type[handle->current_channel] == rotary)
                                    {
                                        sprintf(gUart_buffer, "%16.12f", position2.angle);
                                    }
                                    else
                                    {
                                        sprintf(gUart_buffer, "%16s", uint64_to_str(position2.length));
                                    }
                                }

                                DebugP_log("%s", gUart_buffer);
                                DebugP_log(",    %10u", gEndat_2_2_crc_addinfo1_err_cnt[j]);
                            }

                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i = endat_get_position_loop_chars(handle, 0, 1);
                    endat_print_position_loop(handle, 0, 1, 0);

                    if((!gEndat_is_multi_ch && handle->has_safety[handle->current_channel]) || (!gEndat_is_load_share_mode && handle->has_safety[handle->current_channel]))
                    {
                        uint64_t multi_turn, single_turn;
                        union position position2;
                        uint64_t max = pow(2, handle->single_turn_res[handle->current_channel]);

                        multi_turn = ENDAT_GET_POS_MULTI_TURN(gEndat_2_2_pos_val2[0], handle);
                        single_turn = ENDAT_GET_POS_SINGLE_TURN(gEndat_2_2_pos_val2[0], handle);

                        if(handle->type[handle->current_channel] == rotary)
                        {
                            position2.angle = (float)single_turn / (float)max * (float)360;
                        }
                        else
                        {
                            position2.length = single_turn * handle->step[handle->current_channel];
                        }

                        DebugP_log(", ");

                        if(handle->multi_turn_res[handle->current_channel])
                        {
                            sprintf(gUart_buffer, "%16.12f, %16s", position2.angle, uint64_to_str(multi_turn));
                        }
                        else
                        {
                            if(handle->type[handle->current_channel] == rotary)
                            {
                                sprintf(gUart_buffer, "%16.12f", position2.angle);
                            }
                            else
                            {
                                sprintf(gUart_buffer, "%16s", uint64_to_str(position2.length));
                            }
                        }

                        DebugP_log("%s", gUart_buffer);
                        DebugP_log(",    %10u", gEndat_2_2_crc_addinfo1_err_cnt[0]);

                        if(handle->multi_turn_res[handle->current_channel])
                        {
                            i += 2 + 46 + 3;
                        }
                        else
                        {
                            i += 2 + 28 + 3;
                        }
                    }

                    DebugP_log("\n| ");
                }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                /* Process second channel*/
                DebugP_log("\r|\n\r| --- Channel 2 ---\r\n| ");
                endat_print_position_header(gEndatHandle2, 0, 1, gEndat1_multi_ch_mask);

                if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat1_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(gEndatHandle2, j);
                            i += endat_get_position_loop_chars(gEndatHandle2, 0, 1);
                            DebugP_log("| Ch2-%d: ", j);
                            endat_print_position_loop(gEndatHandle2, 0, 1, j);

                            if((!gEndat1_is_multi_ch && gEndatHandle2->has_safety[gEndatHandle2->current_channel]) || (!gEndat1_is_load_share_mode && gEndatHandle2->has_safety[gEndatHandle2->current_channel]))
                            {
                                uint64_t multi_turn, single_turn;
                                union position position2;
                                uint64_t max = pow(2, gEndatHandle2->single_turn_res[gEndatHandle2->current_channel]);

                                multi_turn = ENDAT_GET_POS_MULTI_TURN(gEndat1_2_2_pos_val2[j], gEndatHandle2);
                                single_turn = ENDAT_GET_POS_SINGLE_TURN(gEndat1_2_2_pos_val2[j], gEndatHandle2);

                                if(gEndatHandle2->type[gEndatHandle2->current_channel] == rotary)
                                {
                                    position2.angle = (float)single_turn / (float)max * (float)360;
                                }
                                else
                                {
                                    position2.length = single_turn * gEndatHandle2->step[gEndatHandle2->current_channel];
                                }

                                DebugP_log(", ");

                                if(gEndatHandle2->multi_turn_res[gEndatHandle2->current_channel])
                                {
                                    sprintf(gUart_buffer, "%16.12f, %16s", position2.angle, uint64_to_str(multi_turn));
                                }
                                else
                                {
                                    if(gEndatHandle2->type[gEndatHandle2->current_channel] == rotary)
                                    {
                                        sprintf(gUart_buffer, "%16.12f", position2.angle);
                                    }
                                    else
                                    {
                                        sprintf(gUart_buffer, "%16s", uint64_to_str(position2.length));
                                    }
                                }

                                DebugP_log("%s", gUart_buffer);
                                DebugP_log(",    %10u", gEndat1_2_2_crc_addinfo1_err_cnt[j]);
                            }

                            DebugP_log("\n| ");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i += endat_get_position_loop_chars(gEndatHandle2, 0, 1);
                    endat_print_position_loop(gEndatHandle2, 0, 1, 0);

                    if((!gEndat1_is_multi_ch && gEndatHandle2->has_safety[gEndatHandle2->current_channel]) || (!gEndat1_is_load_share_mode && gEndatHandle2->has_safety[gEndatHandle2->current_channel]))
                    {
                        uint64_t multi_turn, single_turn;
                        union position position2;
                        uint64_t max = pow(2, gEndatHandle2->single_turn_res[gEndatHandle2->current_channel]);

                        multi_turn = ENDAT_GET_POS_MULTI_TURN(gEndat1_2_2_pos_val2[0], gEndatHandle2);
                        single_turn = ENDAT_GET_POS_SINGLE_TURN(gEndat1_2_2_pos_val2[0], gEndatHandle2);

                        if(gEndatHandle2->type[gEndatHandle2->current_channel] == rotary)
                        {
                            position2.angle = (float)single_turn / (float)max * (float)360;
                        }
                        else
                        {
                            position2.length = single_turn * gEndatHandle2->step[gEndatHandle2->current_channel];
                        }

                        DebugP_log(", ");

                        if(gEndatHandle2->multi_turn_res[gEndatHandle2->current_channel])
                        {
                            sprintf(gUart_buffer, "%16.12f, %16s", position2.angle, uint64_to_str(multi_turn));
                        }
                        else
                        {
                            if(gEndatHandle2->type[gEndatHandle2->current_channel] == rotary)
                            {
                                sprintf(gUart_buffer, "%16.12f", position2.angle);
                            }
                            else
                            {
                                sprintf(gUart_buffer, "%16s", uint64_to_str(position2.length));
                            }
                        }

                        DebugP_log("%s", gUart_buffer);
                        DebugP_log(",    %10u", gEndat1_2_2_crc_addinfo1_err_cnt[0]);

                        if(gEndatHandle2->multi_turn_res[gEndatHandle2->current_channel])
                        {
                            i += 2 + 46 + 3;
                        }
                        else
                        {
                            i += 2 + 28 + 3;
                        }
                    }

                    DebugP_log("\n| ");
                }
#endif

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);
                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }

    /* Command 111: Simulate motor control 2.1 position loop for long time */
    else if(cmd == 111)
    {
        DebugP_log("\r|press enter to stop the long time continuous mode\n|");

        uint64_t position_read = 0;
        /*clear CRC error count */
        memset(gEndat_mtrctrl_crc_err, 0, 3);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        memset(gEndat1_mtrctrl_crc_err, 0, 3);
#endif

        while(1)
        {
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                /* Process first channel results */
                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                           DebugP_log("\r -------Channel %u ------\n", j);
                           DebugP_log("\r position command sent = %u \n", position_read);
                           DebugP_log("\r CRC failures encountered = %u \n", gEndat_mtrctrl_crc_err[j]);
                           DebugP_log(" ");
                           DebugP_log("\r");
                        }
                    }
                }
                else
                {
                    DebugP_log("\r position command sent = %u \n", position_read);
                    DebugP_log("\r CRC failures encountered = %u \n", gEndat_mtrctrl_crc_err[0]);
                    DebugP_log(" ");
                    DebugP_log("\r");
                }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                /* Process second channel results */
                if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat1_multi_ch_mask & 1 << j)
                        {
                           DebugP_log("\r -------2nd Channel -%u ------\n", j);
                           DebugP_log("\r position command sent = %u \n", position_read);
                           DebugP_log("\r CRC failures encountered = %u \n", gEndat1_mtrctrl_crc_err[j]);
                           DebugP_log(" ");
                           DebugP_log("\r");
                        }
                    }
                }
                else
                {
                    DebugP_log("\r -------2nd Channel ------\n");
                    DebugP_log("\r position command sent = %u \n", position_read);
                    DebugP_log("\r CRC failures encountered = %u \n", gEndat1_mtrctrl_crc_err[0]);
                    DebugP_log(" ");
                    DebugP_log("\r");
                }
#endif

                return;
            }
            else
            {
                position_read++;

                /* Process position command */
                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(handle, j);
                            endat_process_position_command(j);
                        }
                    }
                }
                else
                {
                    endat_process_position_command(0);
                }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
                /*Process second channel*/
                if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat1_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(gEndatHandle2, j);
                            endat1_process_position_command(j);
                        }
                    }
                }
                else
                {
                    endat1_process_position_command(0);
                }
#endif

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(500);
            }
        }
    }
}

static void endat_handle_rx(Endat_Handle handle, int32_t cmd)
{
    uint32_t crc;
    Endat_FormatData endat_format_data;

    if(handle->raw_data)
    {
        endat_display_raw_data(cmd, handle);
    }

    endat_recvd_process(handle, cmd,  &endat_format_data);
    crc = endat_recvd_validate(handle, cmd, &endat_format_data);
    endat_recvd_print(cmd, handle, &endat_format_data, crc);

    return;
}

static void endat_print_encoder_info(Endat_Handle handle)
{
    DebugP_log("EnDat 2.%d %s encoder\tID: %u %s\tSN: %c %u %c\n\n",
                handle->cmd_set_2_2 ? 2 : 1,
                (handle->type[handle->current_channel] == rotary) ? "rotary" : "linear",
                handle->id.binary, (char *)&handle->id.ascii,
                (char)handle->sn.ascii_msb, handle->sn.binary, (char)handle->sn.ascii_lsb);
    DebugP_log("\rPosition: %d bits ", handle->pos_res);

    if(handle->type[handle->current_channel] == rotary)
    {
        DebugP_log("(singleturn: %d, multiturn: %d) ", handle->single_turn_res[handle->current_channel],
                    handle->multi_turn_res[handle->current_channel]);
    }

    DebugP_log("[resolution: %d %s]", handle->step[handle->current_channel],
                handle->type[handle->current_channel] == rotary ? "M/rev" : "nm");
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(handle->instance_index == 0)
    {
        DebugP_log("\r\n\nPropagation delay: %dns",
                gEndat_prop_delay[handle->current_channel]);
    }
    else
    {
        DebugP_log("\r\n\nPropagation delay: %dns",
                gEndat1_prop_delay[handle->current_channel]);
    }
#else
    DebugP_log("\r\n\nPropagation delay: %dns",
                gEndat_prop_delay[handle->current_channel]);
#endif
    DebugP_log("\n\n\n");
}

void endat_main(void *args)
{
    int32_t i;
    Endat_CmdSupplement cmd_supplement;
    uint64_t icssClk;
    Endat_ClkCfg endat_clk_config;  /* Clock configuration structure */
    memset(&endat_clk_config, 0, sizeof(endat_clk_config));
    Endat_Params endat_params;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

/*C16 pin High for Enabling ch0 in booster pack */
#if(CONFIG_ENDAT0_BOOSTER_PACK && CONFIG_ENDAT0_CHANNEL0)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
/*B17 pin High for Enabling ch2 in booster pack */
#if(CONFIG_ENDAT0_BOOSTER_PACK && CONFIG_ENDAT0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif

    i = endat_get_fw_version();

    DebugP_log("\n\n\n");
    DebugP_log("EnDat firmware \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    gEndat_is_multi_ch = CONFIG_ENDAT0_MODE & 1;
    gEndat_is_load_share_mode = CONFIG_ENDAT0_MODE & 2;
    icssClk = CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ;

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    int32_t dual_ch;
    dual_ch = endat1_get_fw_version();

    DebugP_log("\n\n\n");
    DebugP_log("2nd Slice EnDat firmware \t: %x.%x.%x (%s)\n\n", (dual_ch >> 24) & 0x7F,
                (dual_ch >> 16) & 0xFF, dual_ch & 0xFFFF, dual_ch & (1 << 31) ? "internal" : "release");

    gEndat1_is_multi_ch = CONFIG_ENDAT1_MODE & 1;
    gEndat1_is_load_share_mode = CONFIG_ENDAT1_MODE & 2;
    
#endif

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

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
    {

        gEndat1_multi_ch_mask = (CONFIG_ENDAT1_CHANNEL0<<0|CONFIG_ENDAT1_CHANNEL1<<1|CONFIG_ENDAT1_CHANNEL2<<2);

        DebugP_log("\r\nchannels %s %s %s selected\n",
                    gEndat1_multi_ch_mask & ENDAT_MULTI_CH0 ? "0" : "",
                    gEndat1_multi_ch_mask & ENDAT_MULTI_CH1 ? "1" : "",
                    gEndat1_multi_ch_mask & ENDAT_MULTI_CH2 ? "2" : "");

        if(!gEndat1_multi_ch_mask)
        {
            DebugP_log("\r\nERROR: please select channels to be used in multi channel configuration -\n\n");
            DebugP_log("\rexit %s as no channel selected in multichannel configuration\n",
                          __func__);
            return;
        }
    }
    else
    {   
        if(CONFIG_ENDAT1_CHANNEL2 == 1)
        {
            dual_ch = 2;
        }
        else if (CONFIG_ENDAT1_CHANNEL1 == 1)
        {
            dual_ch = 1;
        }
        else if (CONFIG_ENDAT1_CHANNEL0 == 1)
        {
            dual_ch = 0;
        }
       
        if(dual_ch < 0 || dual_ch > 2)
        {
           DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
           dual_ch = 0;
        }
    }
#endif

    DebugP_log("\r\n\n");

    /*Translate the TCM local view addr to globel view addr */
    uint64_t gEndatChInfoGlobalAddr = CPU0_BTCM_SOCVIEW((uint64_t)&gEndatChInfo);
    endat_params.ch_info_global_addr = gEndatChInfoGlobalAddr;
    endat_params.channel_rx_info = &gEndatChInfo;
    /* Initialize the clock config pointer to point to local structure */
    endat_params.endat_clk_config = &endat_clk_config;
    endat_params.endat_clk_config->rx_clock_source = CONFIG_ENDAT0_TX_RX_FIFO_CLOCK_SOURCE;
    endat_params.endat_clk_config->tx_clock_source = CONFIG_ENDAT0_TX_RX_FIFO_CLOCK_SOURCE;
    endat_params.endat_clk_config->rx_os_rate = ENDAT_RX_OVERSAMPLING_RATE - 1;
    endat_params.pru_cfg.pruicss_handle = gPruIcssXHandle;
    endat_params.pru_cfg.iep_clock = CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ;
    endat_params.pru_cfg.pru_clock = CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ;
    endat_params.pru_cfg.uart_clock = CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ;
    endat_params.pru_cfg.iep_instance =  ENDAT_PERIODIC_MODE_IEP_INSTANCE;
    endat_params.pru_cfg.pru_slice = ENDAT0_PRUICSS_SLICEx;
    endat_params.pru_cfg.load_share_enable = gEndat_is_load_share_mode;
    
    gEndatHandle1 = endat_init(CONFIG_ENDAT0, endat_params);
   
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /*Translate the TCM local view addr to globel view addr */
    uint64_t gEndat1ChInfoGlobalAddr = CPU0_BTCM_SOCVIEW((uint64_t)&gEndat1ChInfo);
    Endat_ClkCfg endat1_clk_config;  /* Clock configuration structure for second slice */
    Endat_Params endat1_params;
    endat1_params.ch_info_global_addr = gEndat1ChInfoGlobalAddr;
    endat1_params.channel_rx_info = &gEndat1ChInfo;
    /* Initialize the clock config pointer to point to local structure */
    endat1_params.endat_clk_config = &endat1_clk_config;
    endat1_params.endat_clk_config->rx_clock_source = CONFIG_ENDAT1_TX_RX_FIFO_CLOCK_SOURCE;
    endat1_params.endat_clk_config->tx_clock_source = CONFIG_ENDAT1_TX_RX_FIFO_CLOCK_SOURCE;
    endat1_params.endat_clk_config->rx_os_rate = ENDAT_RX_OVERSAMPLING_RATE - 1;
    endat1_params.pru_cfg.pruicss_handle = gPruIcssXHandle;
    endat1_params.pru_cfg.iep_clock = CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ;
    endat1_params.pru_cfg.pru_clock = CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ;
    endat1_params.pru_cfg.uart_clock = CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ;
    endat1_params.pru_cfg.iep_instance =  ENDAT_PERIODIC_MODE_IEP_INSTANCE;
    endat1_params.pru_cfg.pru_slice = ENDAT1_PRUICSS_PRUx;
    endat1_params.pru_cfg.load_share_enable = gEndat1_is_load_share_mode;
    
    gEndatHandle2 = endat_init(CONFIG_ENDAT1, endat1_params);

#endif

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        endat_config_multi_channel_mask(gEndatHandle1, gEndat_multi_ch_mask, gEndat_is_load_share_mode);
    }
    else
    {
        endat_config_channel(gEndatHandle1, i);
    }
    endat_config_host_trigger(gEndatHandle1);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
    {
        endat_config_multi_channel_mask(gEndatHandle2, gEndat1_multi_ch_mask, gEndat1_is_load_share_mode);
    }
    else
    {
        endat_config_channel(gEndatHandle2, dual_ch);
    }
    endat_config_host_trigger(gEndatHandle2);
#endif

    i = endat_pruicss_load_run_fw(gEndatHandle1);

    if(i < 0)
    {
        DebugP_log("\rERROR: EnDat initialization failed -\n\n");

        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            uint8_t tmp;

            tmp = endat_multi_channel_detected(gEndatHandle1) & gEndat_multi_ch_mask;
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

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    
    dual_ch = endat1_pruicss_load_run_fw(gEndatHandle2);

    if(dual_ch < 0)
    {
        DebugP_log("\rERROR: EnDat initialization failed -\n\n");

        if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
        {
            uint8_t tmp1;

            tmp1 = endat_multi_channel_detected(gEndatHandle2) & gEndat1_multi_ch_mask;
            tmp1 ^= gEndat1_multi_ch_mask;
            DebugP_log("\r\tunable to detect encoder in channel %s %s %s\n",
                        tmp1 & ENDAT_MULTI_CH0 ? "0" : "",
                        tmp1 & ENDAT_MULTI_CH1 ? "1" : "",
                        tmp1 & ENDAT_MULTI_CH2 ? "2" : "");
        }
        else
        {
            DebugP_log("\r\tcheck whether encoder is connected and ensure proper connections\n");
        }

        DebugP_log("\rexit %s due to failed firmware initialization\n", __func__);
        return;
    }
#endif

    /* read encoder info at low frequency so that cable length won't affect */
    cmd_supplement.frequency = 200 * 1000;
    endat_process_host_command(100, &cmd_supplement, gEndatHandle1);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    cmd_supplement.frequency = 200 * 1000;
    endat_process_host_command(100, &cmd_supplement, gEndatHandle2);
#endif

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        int32_t j;

        for(j = 0; j < 3; j++)
        {
            if(gEndat_multi_ch_mask & 1 << j)
            {
                endat_multi_channel_set_cur(gEndatHandle1, j);
                /*Initialization of RT parameters*/
                endat_init_rt_measurement(gEndatHandle1);
                if(endat_get_encoder_info(gEndatHandle1) < 0)
                {
                    DebugP_log("\rEnDat initialization channel %d failed\n", j);
                    DebugP_log("\rexit %s due to failed initialization\n", __func__);
                    return;
                }
                /*convert cnt to time in ns ((cnt*1000000000)/icssClk) before use*/
                gEndat_prop_delay[gEndatHandle1->current_channel] = endat_get_prop_delay(gEndatHandle1)*((float)(1000000000)/icssClk);
                DebugP_log("\n\t\t\t\tCHANNEL %d\n\n", j);
                endat_print_encoder_info(gEndatHandle1);
            }
        }

        gEndat_prop_delay_max = gEndat_prop_delay[0] > gEndat_prop_delay[1] ?
                               gEndat_prop_delay[0] : gEndat_prop_delay[1];
        gEndat_prop_delay_max = gEndat_prop_delay_max > gEndat_prop_delay[2] ?
                               gEndat_prop_delay_max : gEndat_prop_delay[2];

    }
    else
    {
        /*Initialization of RT parameters*/
        endat_init_rt_measurement(gEndatHandle1);
        if(endat_get_encoder_info(gEndatHandle1) < 0)
        {
            DebugP_log("\rEnDat initialization failed\n");
            DebugP_log("\rexit %s due to failed initialization\n", __func__);
            return;
        }
        /*convert cnt to time in ns ((cnt*1000000000)/icssClk) before use*/
        gEndat_prop_delay[gEndatHandle1->current_channel] = endat_get_prop_delay(gEndatHandle1)*((float)(1000000000)/icssClk);

        endat_print_encoder_info(gEndatHandle1);
    }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
    {
        int32_t j;

        for(j = 0; j < 3; j++)
        {
            if(gEndat1_multi_ch_mask & 1 << j)
            {
                endat_multi_channel_set_cur(gEndatHandle2, j);
                /*Initialization of RT parameters*/
                endat_init_rt_measurement(gEndatHandle2);
                if(endat_get_encoder_info(gEndatHandle2) < 0)
                {
                    DebugP_log("\rEnDat initialization channel %d failed\n", j);
                    DebugP_log("\rexit %s due to failed initialization\n", __func__);
                    return;
                }
                /*convert cnt to time in ns ((cnt*1000000000)/icssClk) before use*/
                gEndat1_prop_delay[gEndatHandle2->current_channel] = endat_get_prop_delay(gEndatHandle2)*((float)(1000000000)/icssClk);
                DebugP_log("\n\t\t\t\tCHANNEL %d\n\n", j);
                endat_print_encoder_info(gEndatHandle2);
            }
        }

        gEndat1_prop_delay_max = gEndat1_prop_delay[0] > gEndat1_prop_delay[1] ?
                               gEndat1_prop_delay[0] : gEndat1_prop_delay[1];
        gEndat1_prop_delay_max = gEndat1_prop_delay_max > gEndat1_prop_delay[2] ?
                               gEndat1_prop_delay_max : gEndat1_prop_delay[2];

    }
    else
    {
        /*Initialization of RT parameters*/
        endat_init_rt_measurement(gEndatHandle2);
        if(endat_get_encoder_info(gEndatHandle2) < 0)
        {
            DebugP_log("\rEnDat initialization failed\n");
            DebugP_log("\rexit %s due to failed initialization\n", __func__);
            return;
        }
        /*convert cnt to time in ns ((cnt*1000000000)/icssClk) before use*/
        gEndat1_prop_delay[gEndatHandle2->current_channel] = endat_get_prop_delay(gEndatHandle2)*((float)(1000000000)/icssClk);

        endat_print_encoder_info(gEndatHandle2);
    }
#endif
    

    /* default frequency - 8MHz for 2.2 encoders, 1MHz for 2.1 encoders */
    if(gEndatHandle1->cmd_set_2_2)
    {
    #if (ENDAT_INPUT_CLOCK_UART_FREQUENCY == 160000000) || (ENDAT_INPUT_CLOCK_FREQUENCY == 200000000)
        cmd_supplement.frequency = 5 * 1000 * 1000;
    #else
        cmd_supplement.frequency = 8 * 1000 * 1000;
    #endif

    }
    else
    {
        cmd_supplement.frequency = 1 * 1000 * 1000;
    }

    endat_process_host_command(100, &cmd_supplement, gEndatHandle1);

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /* default frequency - 8MHz for 2.2 encoders, 1MHz for 2.1 encoders */
    if(gEndatHandle2->cmd_set_2_2)
    {
    #if (ENDAT_INPUT_CLOCK_UART_FREQUENCY == 160000000) || (ENDAT_INPUT_CLOCK_FREQUENCY == 200000000)
        cmd_supplement.frequency = 5 * 1000 * 1000;
    #else
        cmd_supplement.frequency = 8 * 1000 * 1000;
    #endif
    }
    else
    {
        cmd_supplement.frequency = 1 * 1000 * 1000;
    }
    endat_process_host_command(100, &cmd_supplement, gEndatHandle2);
#endif

    while(1)
    {
        int32_t cmd;

        cmd = endat_handle_user(&cmd_supplement);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        /* By default, all user parameters are the same for both slice channels. For the second slice, if the user wants to change the parameters, then the variable of `cmd_supplement_copy` can be configured manually. */
        Endat_CmdSupplement cmd_supplement_copy = cmd_supplement;
#endif

        if(cmd < 0)
        {
            continue;
        }

        if(VALID_HOST_CMD(cmd))
        {
            endat_process_host_command(cmd, &cmd_supplement, gEndatHandle1);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            endat_process_host_command(cmd, &cmd_supplement_copy, gEndatHandle2);
#endif
            DebugP_log("\r|\n\r|\n");
            continue;
        }

        if(VALID_CONT_MODE_CMD(cmd))
        {
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
            endat_process_continuous_mode_command(cmd, &cmd_supplement, gEndatHandle1, gEndatHandle2, &cmd_supplement_copy);
#else
            endat_process_continuous_mode_command(cmd, &cmd_supplement, gEndatHandle1);
#endif
            DebugP_log("\r|\n\r|\n");
            continue;
        }

        if(endat_command_process(gEndatHandle1, cmd, &cmd_supplement) < 0)
        {
            continue;
        }

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        if(endat_command_process(gEndatHandle2, cmd, &cmd_supplement_copy) < 0)
        {
            continue;
        }
#endif

        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            int8_t rt_error;

            DebugP_log("\r|\n");

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(gEndatHandle1, j);
                    DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", j);
                    endat_handle_rx(gEndatHandle1, cmd);
                    /* Recovery Time validation */
                    if(endat_status_rt_measurement(gEndatHandle1) == 1)
                    {
                        rt_error =  endat_check_rt_error(gEndatHandle1);
                        if(rt_error != RT_NO_ERROR)
                        {
                            DebugP_log("\r Error: Channel %d - Recovery time out of expected range. \n", gEndatHandle1->current_channel);
                            if(rt_error == RT_COUNTER_STUCK_ERROR)
                            {
                                DebugP_log("\r Error: Counter for Channel %d is stuck.\n", gEndatHandle1->current_channel);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            int8_t rt_error;
            endat_handle_rx(gEndatHandle1, cmd);
            /* Recovery Time validation */
            if(endat_status_rt_measurement(gEndatHandle1) == 1)
            {
                rt_error =  endat_check_rt_error(gEndatHandle1);
                if(rt_error == RT_COUNTER_STUCK_ERROR)
                {
                    DebugP_log("\r Error: Channel %d - Recovery time out of expected range. \n", gEndatHandle1->current_channel);
                    if(gEndatHandle1->channel_rx_info->ch[gEndatHandle1->current_channel].recoveryTimeParms.isCounterStuck == 1)
                    {
                        DebugP_log("\r Error: Counter for Channel %d is stuck. \n", gEndatHandle1->current_channel);
                    }
                }
            }
        }
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        if(gEndat1_is_multi_ch || gEndat1_is_load_share_mode)
        {
            int32_t j;
            int8_t rt_error;

            DebugP_log("\r|\n");

            for(j = 0; j < 3; j++)
            {
                if(gEndat1_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(gEndatHandle2, j);
                    DebugP_log("\r|\n|\t\t\t\t2nd SLICE CHANNEL %d\n", j);
                    endat_handle_rx(gEndatHandle2, cmd);
                    /* Recovery Time validation */
                    if(endat_status_rt_measurement(gEndatHandle2) == 1)
                    {
                        rt_error =  endat_check_rt_error(gEndatHandle2);
                        if(rt_error != RT_NO_ERROR)
                        {
                            DebugP_log("\r Error: Channel %d - Recovery time out of expected range. \n", gEndatHandle2->current_channel);
                            if(rt_error == RT_COUNTER_STUCK_ERROR)
                            {
                                DebugP_log("\r Error: Counter for Channel %d is stuck.\n", gEndatHandle2->current_channel);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            int8_t rt_error;
            endat_handle_rx(gEndatHandle2, cmd);
            /* Recovery Time validation */
            if(endat_status_rt_measurement(gEndatHandle2) == 1)
            {
                rt_error =  endat_check_rt_error(gEndatHandle2);
                if(rt_error == RT_COUNTER_STUCK_ERROR)
                {
                    DebugP_log("\r Error: Channel %d - Recovery time out of expected range. \n", gEndatHandle2->current_channel);
                    if(gEndatHandle2->channel_rx_info->ch[gEndatHandle2->current_channel].recoveryTimeParms.isCounterStuck == 1)
                    {
                        DebugP_log("\r Error: Counter for Channel %d is stuck. \n", gEndatHandle2->current_channel);
                    }
                }
            }
        }
#endif

        /* this cannot be done except as last in loop; additional info becomes applicable from next command onwards only */
        endat_addinfo_track(gEndatHandle1, cmd, &cmd_supplement);
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
        endat_addinfo_track(gEndatHandle2, cmd, &cmd_supplement);
#endif
    }

    Board_driversClose();
    Drivers_close();
}

