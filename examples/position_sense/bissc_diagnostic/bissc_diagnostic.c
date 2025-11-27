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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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
#include <position_sense/bissc/include/bissc_drv.h>
#include <position_sense/bissc/include/bissc_api.h>
#include "bissc_periodic_trigger.h"

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
/* Multi-channel single PRU mode */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#include  <bissc_receiver_multi_pru1_bin.h>
#else
#include  <bissc_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
/* Multi-channel multi-PRU mode (load share mode) */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#include <bissc_receiver_multi_rtu_pru1_bin.h>
#include <bissc_receiver_multi_pru1_bin.h>
#include <bissc_receiver_multi_tx_pru1_bin.h>
#else
#include <bissc_receiver_multi_rtu_pru0_bin.h>
#include <bissc_receiver_multi_pru0_bin.h>
#include <bissc_receiver_multi_tx_pru0_bin.h>
#endif
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
/* Single channel single PRU mode */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#include  <bissc_receiver_pru1_bin.h>
#else
#include  <bissc_receiver_pru0_bin.h>
#endif
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define WAIT_5_SECOND                       (5000)
#define WAIT_2_SECOND                       (2000)

#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)

#define BISSC_CMD_EXIT_APP                  (0)
#define BISSC_CMD_ENC_LEN_UPDATE            (1)
#define BISSC_CMD_ENC_FREQ_UPDATE           (2)
#define BISSC_CMD_ENC_SEND_POS              (3)
#define BISSC_CMD_ENC_CTRL_CMD              (4)
#define BISSC_CMD_ENC_LOOP_OVER_CYC         (5)
#define BISSC_CMD_PERIODIC_TRIGGER          (6)
#define BISSC_ENABLE_SAFETY                 (7)

#define BISSC_POSITION_LOOP_STOP            0
#define BISSC_POSITION_LOOP_START           1

#define BISSC_CLOCK_CONFIG_DELAY_SEC        (2U)
#define BISSC_CTRL_REG_ADDR_MASK            (0x7FU)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* BiSS-C Driver Handle */
bissc_handle gBisscHandle0 = NULL;

/* BiSS-C Periodic Interface Struct Instance */
bissc_periodic_interface gBisscPeriodicInterface = {NULL, 0, 0, 0, 0};

/* Global variable to track position loop status */
volatile int32_t gBisscPositionLoopStatus;

/* Task related global variables */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void bissc_pruicss_init(void);
static int32_t bissc_pruicss_load_run_fw(bissc_handle handle, uint8_t channel_mask);
static uint64_t bissc_get_fw_version(void);
static void bissc_display_menu(void);
static void bissc_get_enc_data_len(bissc_handle handle);
static void bissc_print_res(bissc_handle handle);
static int32_t bissc_get_command(void);
static void bissc_position_loop_decide_termination(void *args);
static int32_t bissc_loop_task_create(void);
static void bissc_process_periodic_command(bissc_handle handle, int64_t ch0_trigger_count, int64_t ch1_trigger_count, int64_t ch2_trigger_count, int64_t iep_reset_count);
void bissc_main(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void bissc_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    int32_t size;

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);

#ifdef CONFIG_BISSC0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Clear PRU-ICSS DATA RAM 0/1 based on slice */
    size = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_BISSC0_PRUICSS_SLICE));
    DebugP_assert(size);

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* Multi-channel multi-PRU mode (load share mode) - disable all cores */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTU_PRU1);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRU1);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TX_PRU1);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTU_PRU0);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRU0);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TX_PRU0);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#endif
#else
    /* Single channel or multi-channel single PRU mode - disable only PRU */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRU1);
    DebugP_assert(SystemP_SUCCESS == status);
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRU0);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#endif
}

static int32_t bissc_pruicss_load_run_fw(bissc_handle handle, uint8_t channel_mask)
{
    int32_t status = SystemP_FAILURE, size;
    const bissc_attrs *attrs = bissc_get_attrs(handle);

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    /* Multi-channel multi-PRU mode (load share mode) */
    if(attrs->channel0_enabled)
    {
        status = PRUICSS_disableCore(gPruIcssXHandle, attrs->rtu_pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
        size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(CONFIG_BISSC0_PRUICSS_SLICE),
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
                                                            0, (uint32_t *) BiSSFirmwareMultiMakeRtuPru1_0,
                                                            sizeof(BiSSFirmwareMultiMakeRtuPru1_0));
#else
                                                            0, (uint32_t *) BiSSFirmwareMultiMakeRtuPru0_0,
                                                            sizeof(BiSSFirmwareMultiMakeRtuPru0_0));
#endif
        DebugP_assert(size);
        status = PRUICSS_resetCore(gPruIcssXHandle, attrs->rtu_pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
        status = PRUICSS_enableCore(gPruIcssXHandle, attrs->rtu_pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
    }
    if(attrs->channel1_enabled)
    {
        status=PRUICSS_disableCore(gPruIcssXHandle, attrs->pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
        size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_BISSC0_PRUICSS_SLICE),
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
                                                          0, (uint32_t *) BiSSFirmwareMultiMakePru1_0,
                                                          sizeof(BiSSFirmwareMultiMakePru1_0));
#else
                                                          0, (uint32_t *) BiSSFirmwareMultiMakePru0_0,
                                                          sizeof(BiSSFirmwareMultiMakePru0_0));
#endif
        DebugP_assert(size);
        status = PRUICSS_resetCore(gPruIcssXHandle, attrs->pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
        status = PRUICSS_enableCore(gPruIcssXHandle, attrs->pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
    }
    if(attrs->channel2_enabled)
    {
        status = PRUICSS_disableCore(gPruIcssXHandle, attrs->tx_pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
        size = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(CONFIG_BISSC0_PRUICSS_SLICE),
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
                                                            0, (uint32_t *) BiSSFirmwareMultiMakeTxPru1_0,
                                                            sizeof(BiSSFirmwareMultiMakeTxPru1_0));
#else
                                                            0, (uint32_t *) BiSSFirmwareMultiMakeTxPru0_0,
                                                            sizeof(BiSSFirmwareMultiMakeTxPru0_0));
#endif
        DebugP_assert(size);
        status = PRUICSS_resetCore(gPruIcssXHandle, attrs->tx_pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
        status = PRUICSS_enableCore(gPruIcssXHandle, attrs->tx_pru_id);
        DebugP_assert(SystemP_SUCCESS == status);
    }
#else
    /* Single channel or multi-channel single PRU mode */
    status = PRUICSS_disableCore(gPruIcssXHandle, attrs->pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(attrs->pru_id),
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
                                0, (uint32_t *) BiSSFirmwareMultiPru1_0,
                                sizeof(BiSSFirmwareMultiPru1_0));
#else
                                0, (uint32_t *) BiSSFirmwareMultiPru0_0,
                                sizeof(BiSSFirmwareMultiPru0_0));
#endif
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(attrs->pru_id),
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
                                0, (uint32_t *) BiSSFirmwarePru1_0,
                                sizeof(BiSSFirmwarePru1_0));
#else
                                0, (uint32_t *) BiSSFirmwarePru0_0,
                                sizeof(BiSSFirmwarePru0_0));
#endif
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, attrs->pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, attrs->pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* check initialization ack from firmware, with a timeout of 5 second */
    status = bissc_wait_for_fw_initialization(handle, WAIT_5_SECOND, channel_mask);
    return status;
}

static uint64_t bissc_get_fw_version(void)
{
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    return *((unsigned long *)BiSSFirmwareMultiPru1_0 + 2);
#else
    return *((unsigned long *)BiSSFirmwareMultiPru0_0 + 2);
#endif
#elif (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    return *((unsigned long *)BiSSFirmwareMultiMakeRtuPru1_0 + 2);
#else
    return *((unsigned long *)BiSSFirmwareMultiMakeRtuPru0_0 + 2);
#endif
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    return *((unsigned long *)BiSSFirmwareMultiMakePru1_0 + 2);
#else
    return *((unsigned long *)BiSSFirmwareMultiMakePru0_0 + 2);
#endif
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    return *((unsigned long *)BiSSFirmwareMultiMakeTxPru1_0 + 2);
#else
    return *((unsigned long *)BiSSFirmwareMultiMakeTxPru0_0 + 2);
#endif
#endif
#elif (CONFIG_BISSC0_MODE == BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    return *((unsigned long *)BiSSFirmwarePru1_0 + 2);
#else
    return *((unsigned long *)BiSSFirmwarePru0_0 + 2);
#endif
#endif
}

static void bissc_display_menu(void)
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select input parametes                           |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 1 : Number of position data bits                                             |");
    DebugP_log("\r\n| 2 : Select clock frequency in MHz(1/2/5/8/10)                                |");
    DebugP_log("\r\n| 3 : Encoder send position values                                             |");
    DebugP_log("\r\n| 4 : Control Communication - Register Read/Write                              |");
    DebugP_log("\r\n| 5 : Loop over BiSS-C cycles                                                  |");
    DebugP_log("\r\n| 6 : Start continuous mode                                                    |");
    DebugP_log("\r\n| 7 : Enable safety mode                                                       |");
    DebugP_log("\r\n| 0 : Exit the application                                                     |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| enter value:\r\n");
}

static void bissc_get_enc_data_len(bissc_handle handle)
{
    int32_t ret;
    uint32_t single_turn_len[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX], multi_turn_len[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX], enc_num = 0, total_channels, ch_num;
    const bissc_attrs *attrs = bissc_get_attrs(handle);

    ret = bissc_clear_data_len(handle);
    if(ret != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: Failed to clear data length\n");
        return;
    }

    if(attrs->load_share_enabled)
        total_channels = bissc_get_total_channels(handle);
    else
        total_channels = 1;

    for(ch_num = 0; ch_num < total_channels; ch_num++)
    {
        for(enc_num = 0; enc_num < BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX; enc_num++)
        {
            single_turn_len[enc_num] = 0;
            multi_turn_len[enc_num] = 0;
        }

        /* BiSS-C Frame Size Constraints:
         * BiSS-C protocol uses 64-bit data frames. Total frame size must not exceed 64 bits.
         *
         * Frame structure WITHOUT Safety mode:
         * Position Data + E/W(2) + CRC(6) <= 64 bits
         *
         *   Therefore: single_turn + multi_turn <= 56 bits
         *
         * Frame structure WITH Safety mode:
         *   Position Data + E/W(2) + sign-of-life(6) + safety CRC(16) <= 64 bits
         *   Therefore: single_turn + multi_turn <= 40 bits
         *
         * Examples:
         *   - Without Safety: 32-bit single-turn + 12-bit multi-turn = 44 bits (valid, <= 56)
         *   - With Safety: 32-bit single-turn + 8-bit multi-turn = 40 bits (valid, <= 40)
         *   - Without Safety: 40-bit single-turn + 20-bit multi-turn = 60 bits (invalid, > 56)
         */

        DebugP_log("\r\n=======================================================================\n");
        DebugP_log("BiSS-C Frame Size Constraints:\n");
        DebugP_log("  - Without Safety: single_turn + multi_turn <= 56 bits\n");
        DebugP_log("  - With Safety: single_turn + multi_turn <= 40 bits\n");
        DebugP_log("=======================================================================\n");
        DebugP_log("\r\nPlease enter encoder lengths connected to Channel %u:\n", handle->priv->channel[ch_num]);
        DebugP_log("\r\nPlease enter 1st encoder single turn length\n");
        DebugP_scanf("%u\n", &single_turn_len[0]);
        if(single_turn_len[0])
        {
            DebugP_log("\r\nPlease enter 1st encoder multiturn length, 0 if not multiturn\n");
            DebugP_scanf("%u\n", &multi_turn_len[0]);
        }
        DebugP_log("\r\nPlease enter 0 as data length if daisy chain is not used\n");
        DebugP_log("\r\nPlease enter 2nd encoder single turn length\n");
        DebugP_scanf("%u\n", &single_turn_len[1]);
        if(single_turn_len[1])
        {
            DebugP_log("\r\nPlease enter 2nd encoder multiturn length, 0 if not multiturn\n");
            DebugP_scanf("%u\n", &multi_turn_len[1]);
        }
        if(single_turn_len[1])
        {
            DebugP_log("\r\nPlease enter 3rd encoder single turn length\n");
            DebugP_scanf("%u\n", &single_turn_len[2]);
            if(single_turn_len[2])
            {
                DebugP_log("\r\nPlease enter 3rd encoder multiturn length, 0 if not multiturn\n");
                DebugP_scanf("%u\n", &multi_turn_len[2]);
            }
        }
        ret = bissc_update_data_len(handle, single_turn_len, multi_turn_len, ch_num);
        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r\n[BiSS-C] ERROR: Invalid encoder configuration for channel %u\n", ch_num);
            return;
        }
    }
}

static void bissc_print_res(bissc_handle handle)
{
    const bissc_attrs *attrs = bissc_get_attrs(handle);
    uint32_t ch_num, ch, ls_ch;

    for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
    {
        ch = bissc_get_current_channel(handle, ch_num);
        if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
            ls_ch = ch;
        else
            ls_ch = 0;
        if((attrs->mode == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU) || (attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU))
            DebugP_log("%s", (ch_num != (attrs->total_channels-1))?"\r":" & ");
        else
            DebugP_log("\r");
        if(handle->priv->data_len[ls_ch][1])
        {
            if(handle->priv->data_len[ls_ch][2])
            {
                if(handle->priv->multi_turn_len[ls_ch][2])
                {
                    DebugP_log("Channel:%u - Enc3: MT rev:%u, Angle:%.12f, Enc2: MT rev:%u, Angle:%.12f, Enc1: MT rev:%u, Angle:%.12f, crc error count enc3:%u, crc error count enc2:%u, crc error count enc_1:%u ", ch, handle->priv->enc_pos_data[ch].num_of_turns[2], handle->priv->enc_pos_data[ch].angle[2],handle->priv->enc_pos_data[ch].num_of_turns[1], handle->priv->enc_pos_data[ch].angle[1],
                    handle->priv->enc_pos_data[ch].num_of_turns[0], handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][2], handle->priv->pd_crc_err_cnt[ch][1], handle->priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("Channel:%u - Enc3: Angle:%.12f, Enc2: Angle:%.12f, Enc1: Angle:%.12f, crc error count enc3:%u, crc error count enc2:%u, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].angle[2], handle->priv->enc_pos_data[ch].angle[1],
                    handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][2], handle->priv->pd_crc_err_cnt[ch][1], handle->priv->pd_crc_err_cnt[ch][0]);
                }
            }
            else
            {
                if(handle->priv->multi_turn_len[ls_ch][1])
                {
                    DebugP_log("Channel:%u - Enc2: MT rev:%u, Angle:%.12f, Enc1: MT rev:%u, Angle:%.12f, crc error count enc2:%u, crc error count enc1:%u ", ch,handle->priv->enc_pos_data[ch].num_of_turns[1], handle->priv->enc_pos_data[ch].angle[1],
                    handle->priv->enc_pos_data[ch].num_of_turns[0], handle->priv->enc_pos_data[ch].angle[0],handle->priv->pd_crc_err_cnt[ch][1], handle->priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("Channel:%u - Enc2: Angle:%.12f, Enc1: Angle:%.12f, crc error count enc2:%u, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].angle[1], handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][1],
                    handle->priv->pd_crc_err_cnt[ch][0]);
                }
            }
        }
        else
        {
            if(handle->priv->multi_turn_len[ls_ch][0])
            {
                DebugP_log("Channel:%u - Enc1: MT rev:%u, Angle:%.12f, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].num_of_turns[0], handle->priv->enc_pos_data[ch].angle[0],
                handle->priv->pd_crc_err_cnt[ch][0]);
            }
            else
            {
                DebugP_log("Channel:%u - Enc1: Angle:%.12f, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].angle[0],
                handle->priv->pd_crc_err_cnt[ch][0]);
            }
        }
    }
}

static int32_t bissc_get_command()
{
    int32_t cmd;
    DebugP_scanf("%d\n", &cmd);
    /* Check to make sure that the command issued is correct */
    if( cmd < BISSC_CMD_EXIT_APP || cmd > BISSC_ENABLE_SAFETY )
    {
        DebugP_log("\r\n| WARNING: invalid option try again\n");
        return SystemP_FAILURE;
    }
    return cmd;
}

static void bissc_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        gBisscPositionLoopStatus = BISSC_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static int32_t bissc_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params task_params;

    TaskP_Params_init(&task_params);
    task_params.name = "bissc_position_loop_decide_termination";
    task_params.stackSize = TASK_STACK_SIZE;
    task_params.stack = (uint8_t *)gTaskFxnStack;
    task_params.priority = TASK_PRIORITY;
    task_params.taskMain = (TaskP_FxnMain)bissc_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &task_params);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\rbissc_position_loop_decide_termination creation failed\n");
    }

    return status ;
}

static void bissc_process_periodic_command(bissc_handle handle, int64_t ch0_trigger_count, int64_t ch1_trigger_count, int64_t ch2_trigger_count, int64_t iep_reset_count)
{
    int32_t status, ret;
    uint32_t pos_fail_cnt = 0, pos_total_cnt = 0;

    ret = bissc_config_periodic_trigger(handle);
    if(ret != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: Failed to configure periodic trigger\n");
        return;
    }

    if(bissc_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");

        DebugP_log("\r| Revert to host trigger\n");

        ret = bissc_config_host_trigger(handle);
        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: Failed to revert to host trigger\n");
        }

        return;
    }

    bissc_periodic_interface_init(handle, &gBisscPeriodicInterface, ch0_trigger_count, ch1_trigger_count, ch2_trigger_count, iep_reset_count);
    status = bissc_config_periodic_mode(&gBisscPeriodicInterface);
    DebugP_assert(SystemP_SUCCESS == status);

    gBisscPositionLoopStatus = BISSC_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|");

    while(1)
    {
        pos_total_cnt++;
        if(gBisscPositionLoopStatus == BISSC_POSITION_LOOP_STOP)
        {
            bissc_stop_periodic_mode(&gBisscPeriodicInterface);
            ret = bissc_config_host_trigger(handle);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Failed to configure host trigger\n");
            }
            DebugP_log("\r\n Failed %u out of %u times\n", pos_fail_cnt, pos_total_cnt);
            return;
        }
        else
        {
            ret = bissc_get_pos(handle);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Position data measurement failed \n");
                pos_fail_cnt++;
                continue;
            }
            bissc_print_res(handle);
        }
    }
}

/**
 * \brief   BiSS-C diagnostic application main function
 *
 * \details This function implements the main diagnostic application flow for BiSS-C
 *          encoder interface. It initializes the BiSS-C driver, loads and starts PRU
 *          firmware, and provides an interactive menu-driven interface for various
 *          encoder operations including:
 *          - Position data acquisition (single-shot and continuous)
 *          - Control communication (register read/write)
 *          - Frequency configuration
 *          - Safety mode operations
 *          - Periodic trigger mode configuration
 *
 *          Flow:
 *          1. Initialize SoC drivers and board drivers
 *          2. Initialize BiSS-C driver with configuration from SysConfig
 *          3. Get encoder resolution parameters from user
 *          4. Load and run PRU firmware(s)
 *          5. Validate encoder processing delays (multi-channel mode)
 *          6. Enter interactive menu loop for encoder operations
 *          7. De-initialize on exit
 *
 *          NOTE on driver APIs:
 *          BiSS-C driver APIs use a simplified validation approach for optimal performance:
 *          - **Handle validation**: All public APIs validate the handle parameter for NULL
 *          - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *          - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *            are validated once during bissc_init() and assumed valid in subsequent API calls
 *          - This strategy reduces overhead in time-critical data path functions while maintaining safety
 *
 * \param[in]   args    Unused
 */

void bissc_main(void *args)
{
    int32_t i, ret;
    uint32_t ch = 0, ls_ch = 0, ch_num, enc_num = 0, total_channels;
    const bissc_attrs *attrs = NULL;

    /* ========================================================================== */
    /* STEP 1: Initialize SoC drivers and board drivers                           */
    /* ========================================================================== */
    Drivers_open();          /* Open SoC drivers */
    Board_driversOpen();     /* Open board-specific drivers */
    /*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_BISSC0_BOOSTER_PACK)
#if (CONFIG_BISSC0_CHANNEL0)
    GPIO_setDirMode(ENC0_EN_BASE_ADDR, ENC0_EN_PIN, ENC0_EN_DIR);
    GPIO_pinWriteHigh(ENC0_EN_BASE_ADDR, ENC0_EN_PIN);
#endif
#if (CONFIG_BISSC0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif
#endif

    /* ========================================================================== */
    /* STEP 2: Initialize PRU-ICSS and BiSS-C driver                              */
    /* ========================================================================== */

    /* Get and display BiSS-C firmware version from PRU firmware image */
    i = bissc_get_fw_version();

    DebugP_log("\n\n");
    DebugP_log("BiSS-C firmware \t: %x.%x.%x (%s)\n", (i >> 24) & 0x7F, (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    /* Initialize PRU-ICSS instance, initialize DRAM, and disable PRU cores */
    bissc_pruicss_init();

    /* Initialize BiSS-C parameters with defaults and set PRU-ICSS handle */
    bissc_params bissc_params_instance;
    bissc_params_init(&bissc_params_instance);
    /* Default delay values are used:
     *   - cmd_process_delay_us = 1000us (delay for command processing polling loop)
     *   - fw_wait_delay_us = 1000us (delay for firmware status checks)
     *   - max_cycle_timeout_ms = 5ms (maximum BiSS-C cycle timeout)
     * If needed, these can be modified before calling bissc_init():
     *   bissc_params_instance.cmd_process_delay_us = <custom_value>;
     *   bissc_params_instance.fw_wait_delay_us = <custom_value>;
     *   bissc_params_instance.max_cycle_timeout_ms = <custom_value>;
     */
    bissc_params_instance.pruicss_handle = gPruIcssXHandle;

    /* Initialize BiSS-C driver instance
     * This calls: bissc_hw_init(), bissc_config_channel(), bissc_config_load_share() and bissc_set_default_initialization() */
    gBisscHandle0 = bissc_init(CONFIG_BISSC0, &bissc_params_instance);

    if(gBisscHandle0 == NULL)
    {
        DebugP_log("\r\nERROR: BiSS-C initialization failed\n");
        return;
    }

    /* Get pointer to BiSS-C attributes (configuration data from SysConfig) */
    attrs = bissc_get_attrs(gBisscHandle0);

    /* Display operation mode and enabled channels */
    if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        /* Multi-channel multi-PRU (load share) mode: Each channel uses a separate PRU core */
        DebugP_log("\r\nBiSS-C Load Share Demo application is running......\n");
        for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
        {
            DebugP_log("\r\nChannel %u is enabled\n", gBisscHandle0->priv->channel[ch_num]);
        }
    }
    else if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        /* Multi-channel single PRU mode: Multiple channels handled by one PRU core */
        DebugP_log("\r\nBiSS-C Multi channel, Single PRU Demo application is running......\n");
        for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
        {
            DebugP_log("\r\nChannel %u is enabled\n", gBisscHandle0->priv->channel[ch_num]);
        }
    }
    else
    {
        /* Single channel single PRU mode: One channel on one PRU core */
        DebugP_log("\r\nBiSS-C Single channel, Single PRU Demo application is running......\n");
        DebugP_log("\r\nChannel %u is enabled\n", gBisscHandle0->priv->channel[0]);
    }

    /* ========================================================================== */
    /* STEP 3: Get encoder resolution parameters from user                        */
    /* ========================================================================== */
    /* Prompt user for encoder resolution (single-turn and multi-turn bit lengths)
     * Calls: bissc_clear_data_len() once and bissc_update_data_len() per channel */
    bissc_get_enc_data_len(gBisscHandle0);

    /* ========================================================================== */
    /* STEP 4: Load and run PRU firmware                                          */
    /* ========================================================================== */
    /* Load PRU firmware image, set default initialization parameters, wait for
     * firmware init, and measure encoder processing delays */
    ret = bissc_pruicss_load_run_fw(gBisscHandle0, attrs->channel_mask);
    if(ret != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: BiSS-C initialization failed \n");
        DebugP_log("\r\ncheck whether encoder is connected and ensure proper connections\n");
        DebugP_log("\r\nexit %s due to failed firmware initialization\n", __func__);
        goto deinit;
    }

    /* =============================================================================== */
    /* STEP 5: Get and validate encoder processing delays                              */
    /* =============================================================================== */
    /* In multi-channel single PRU mode, all encoders must have the same processing
     * delay for proper synchronization. If delays differ, operation is not supported. */

    /* Copy measured processing delays from PRU firmware to private driver structure */
    ret = bissc_get_enc_proc_delay(gBisscHandle0);
    if(ret != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: Failed to get encoder processing delay\n");
        goto deinit;
    }

    if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        if(attrs->total_channels > 1)
        {
            if(gBisscHandle0->priv->proc_delay[gBisscHandle0->priv->channel[0]] == gBisscHandle0->priv->proc_delay[gBisscHandle0->priv->channel[1]])
            {
                if(attrs->total_channels > 2)
                {
                    if(gBisscHandle0->priv->proc_delay[gBisscHandle0->priv->channel[1]] !=  gBisscHandle0->priv->proc_delay[gBisscHandle0->priv->channel[2]])
                    {
                        DebugP_log("\r\n Encoders connected accross channels have different processing delays, multi channel configuration is not supported\n");
                        goto deinit;
                    }
                }
            }
            else
            {
                DebugP_log("\r\n Encoders connected accross channels have different processing delays, multi channel configuration is not supported\n");
                goto deinit;
            }
        }
    }

    /* Display encoder detection status and processing delays */
    DebugP_log("\r\nBiSS-C encoder/encoders detected and running at frequency %u MHz\n", attrs->baud_rate);
    for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
    {
        DebugP_log("\r\nProcessing Delay in clock cycles for Channel %u : %u\n",gBisscHandle0->priv->channel[ch_num], gBisscHandle0->priv->proc_delay[gBisscHandle0->priv->channel[ch_num]]);
    }

    /* ========================================================================== */
    /* STEP 6: Interactive menu loop for encoder operations                       */
    /* ========================================================================== */
    while(1)
    {
        int32_t cmd, ret;
        int64_t ch0_trigger_count=0, ch1_trigger_count=0, ch2_trigger_count=0, iep_reset_count=0;
        uint32_t freq, ctrl_cmd[BISSC_NUM_CH_PER_SLICE_MAX]={0};
        uint32_t loop_cnt;
        uint32_t safety = 0;
        uint32_t ctrl_write_status, ctrl_reg_address, ctrl_reg_data = 0, ctrl_enc_id = 0;
        /* Display menu and get user command */
        bissc_display_menu();
        cmd = bissc_get_command();

        if(cmd < BISSC_CMD_EXIT_APP)
        {
            /* Invalid command, show menu again */
            continue;
        }
        else if(cmd == BISSC_CMD_EXIT_APP)
        {
            /* Exit application */
            DebugP_log("\r\tGood bye!\n");
            break;
        }
        else if(cmd == BISSC_CMD_ENC_LEN_UPDATE)
        {
           /* Update encoder resolution parameters
            * Calls: bissc_clear_data_len() once and bissc_update_data_len() per channel */
           bissc_get_enc_data_len(gBisscHandle0);
        }
        else if(cmd == BISSC_CMD_ENC_FREQ_UPDATE)
        {
            /* Change BiSS-C communication frequency (1/2/5/8/10 MHz)
             * Calls: bissc_clock_config() which internally calls multiple functions */
            DebugP_log("\r\nPlease enter frequency in MHz:\n");
            DebugP_scanf("%u\n", &freq);

            /* Validate frequency (only 1, 2, 5, 8, 10 MHz supported) */
            if(!((freq == BISSC_FREQ_1MHZ) || (freq == BISSC_FREQ_2MHZ) || (freq == BISSC_FREQ_5MHZ) || (freq == BISSC_FREQ_8MHZ) || (freq == BISSC_FREQ_10MHZ)))
            {
                DebugP_log("\r\n CLK divisors will not be possible. Please provide valid freq: 1/2/5/8/10 \n");
                continue;
            }

            /* Reconfigure clock frequency and wait for processing delay measurement */
            ret = bissc_clock_config(gBisscHandle0, freq, WAIT_5_SECOND);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Processing time measurement failed \n");
                DebugP_log("\r\n check whether encoder is connected and ensure proper connections \n");
                DebugP_log("\r\n Good bye!\n");
                break;
            }
            ClockP_sleep(BISSC_CLOCK_CONFIG_DELAY_SEC);
        }
        else if(cmd == BISSC_CMD_ENC_SEND_POS)
        {
            /* Get single-shot position data from encoder(s)
             * Calls: bissc_command_process() which internally calls
             *        bissc_command_send() and bissc_command_wait() */
            ret = bissc_get_pos(gBisscHandle0);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Position data measurement failed \n");
            }
            for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
            {
                ch = bissc_get_current_channel(gBisscHandle0, ch_num);
                if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
                    ls_ch = ch;
                else
                    ls_ch = 0;
                DebugP_log("\r\n Channel %u:\n", ch);
                if(gBisscHandle0->priv->multi_turn_len[ls_ch][0])
                {
                    if(gBisscHandle0->priv->has_safety[ls_ch][0])
                    {
                        DebugP_log("\r\n Encoder-1 Multiturn rev: %u, Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n", gBisscHandle0->priv->enc_pos_data[ch].num_of_turns[0],
                        gBisscHandle0->priv->enc_pos_data[ch].angle[0], gBisscHandle0->priv->rcv_safety_crc[ch][0], gBisscHandle0->priv->calc_safety_crc[ch][0], gBisscHandle0->priv->enc_pos_data[ch].ew[0], gBisscHandle0->priv->sign_of_life_cnt[ch][0]);
                    }
                    else
                    {
                        DebugP_log("\r\n Encoder-1 Multiturn rev: %u, Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gBisscHandle0->priv->enc_pos_data[ch].num_of_turns[0],
                        gBisscHandle0->priv->enc_pos_data[ch].angle[0], gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[0], gBisscHandle0->priv->enc_pos_data[ch].otf_crc[0], gBisscHandle0->priv->enc_pos_data[ch].ew[0]);
                    }
                }
                else
                {
                    if(gBisscHandle0->priv->has_safety[ls_ch][0])
                    {
                        DebugP_log("\r\n Encoder-1 Singleturn Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n",
                        gBisscHandle0->priv->enc_pos_data[ch].angle[0], gBisscHandle0->priv->rcv_safety_crc[ch][0], gBisscHandle0->priv->calc_safety_crc[ch][0], gBisscHandle0->priv->enc_pos_data[ch].ew[0], gBisscHandle0->priv->sign_of_life_cnt[ch][0]);
                    }
                    else
                    {
                        DebugP_log("\r\n Encoder-1 Singleturn Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gBisscHandle0->priv->enc_pos_data[ch].angle[0], gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[0],
                        gBisscHandle0->priv->enc_pos_data[ch].otf_crc[0], gBisscHandle0->priv->enc_pos_data[ch].ew[0]);
                    }
                }
                if(gBisscHandle0->priv->has_safety[ls_ch][0])
                {
                    DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gBisscHandle0->priv->rcv_safety_crc[ch][0] == gBisscHandle0->priv->calc_safety_crc[ch][0]) ? "success" : "failure",gBisscHandle0->priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[0] == gBisscHandle0->priv->enc_pos_data[ch].otf_crc[0]) ? "success" : "failure" ,
                    gBisscHandle0->priv->pd_crc_err_cnt[ch][0]);
                }
                if(gBisscHandle0->priv->data_len[ls_ch][1])
                {
                    if(gBisscHandle0->priv->multi_turn_len[ls_ch][1])
                    {
                        if(gBisscHandle0->priv->has_safety[ls_ch][1])
                        {
                            DebugP_log("\r\n Encoder-2 Multiturn rev: %u, Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n", gBisscHandle0->priv->enc_pos_data[ch].num_of_turns[1],
                            gBisscHandle0->priv->enc_pos_data[ch].angle[1], gBisscHandle0->priv->rcv_safety_crc[ch][1], gBisscHandle0->priv->calc_safety_crc[ch][1], gBisscHandle0->priv->enc_pos_data[ch].ew[1], gBisscHandle0->priv->sign_of_life_cnt[ch][1]);
                        }
                        else
                        {
                            DebugP_log("\r\n Encoder-2 Multiturn rev: %u, Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gBisscHandle0->priv->enc_pos_data[ch].num_of_turns[1],
                            gBisscHandle0->priv->enc_pos_data[ch].angle[1], gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[1], gBisscHandle0->priv->enc_pos_data[ch].otf_crc[1], gBisscHandle0->priv->enc_pos_data[ch].ew[1]);
                        }
                    }
                    else
                    {
                        if(gBisscHandle0->priv->has_safety[ls_ch][1])
                        {
                            DebugP_log("\r\n Encoder-2 Singleturn Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n",
                            gBisscHandle0->priv->enc_pos_data[ch].angle[1], gBisscHandle0->priv->rcv_safety_crc[ch][1], gBisscHandle0->priv->calc_safety_crc[ch][1], gBisscHandle0->priv->enc_pos_data[ch].ew[1], gBisscHandle0->priv->sign_of_life_cnt[ch][1]);
                        }
                        else
                        {
                            DebugP_log("\r\n Encoder-2 Singleturn Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gBisscHandle0->priv->enc_pos_data[ch].angle[1], gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[1],
                            gBisscHandle0->priv->enc_pos_data[ch].otf_crc[1], gBisscHandle0->priv->enc_pos_data[ch].ew[1]);
                        }
                    }
                    if(gBisscHandle0->priv->has_safety[ls_ch][1])
                    {
                        DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gBisscHandle0->priv->rcv_safety_crc[ch][1] == gBisscHandle0->priv->calc_safety_crc[ch][1]) ? "success" : "failure",gBisscHandle0->priv->pd_crc_err_cnt[ch][1]);
                    }
                    else
                    {
                        DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[1] == gBisscHandle0->priv->enc_pos_data[ch].otf_crc[1]) ? "success" : "failure" ,
                        gBisscHandle0->priv->pd_crc_err_cnt[ch][1]);
                    }
                    if(gBisscHandle0->priv->data_len[ls_ch][2])
                    {
                        if(gBisscHandle0->priv->multi_turn_len[ls_ch][2])
                        {
                            if(gBisscHandle0->priv->has_safety[ls_ch][2])
                            {
                                DebugP_log("\r\n Encoder-3 Multiturn rev: %u, Angle:  %.12f, received safety crc: 0x%x, calculated safety crc: 0x%x, e_w: 0x%x, sign-of-life counter: %d\n", gBisscHandle0->priv->enc_pos_data[ch].num_of_turns[2],
                                gBisscHandle0->priv->enc_pos_data[ch].angle[2], gBisscHandle0->priv->rcv_safety_crc[ch][2], gBisscHandle0->priv->calc_safety_crc[ch][2], gBisscHandle0->priv->enc_pos_data[ch].ew[2], gBisscHandle0->priv->sign_of_life_cnt[ch][2]);
                            }
                            else
                            {
                                DebugP_log("\r\n Encoder-3 Multiturn rev: %u, Angle:  %.12f, crc: 0x%x, otf crc: 0x%x, e_w: 0x%x\n", gBisscHandle0->priv->enc_pos_data[ch].num_of_turns[2],
                                gBisscHandle0->priv->enc_pos_data[ch].angle[2], gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[2], gBisscHandle0->priv->enc_pos_data[ch].otf_crc[2], gBisscHandle0->priv->enc_pos_data[ch].ew[2]);
                            }
                        }
                        else
                        {
                            if(gBisscHandle0->priv->has_safety[ls_ch][2])
                            {
                                DebugP_log("\r\n Encoder-3 Singleturn Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n",
                                gBisscHandle0->priv->enc_pos_data[ch].angle[2], gBisscHandle0->priv->rcv_safety_crc[ch][2], gBisscHandle0->priv->calc_safety_crc[ch][2], gBisscHandle0->priv->enc_pos_data[ch].ew[2], gBisscHandle0->priv->sign_of_life_cnt[ch][2]);
                            }
                            else
                            {
                                DebugP_log("\r\n Encoder-3 Singleturn Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gBisscHandle0->priv->enc_pos_data[ch].angle[2], gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[2],
                                gBisscHandle0->priv->enc_pos_data[ch].otf_crc[2], gBisscHandle0->priv->enc_pos_data[ch].ew[2]);
                            }
                        }
                        if(gBisscHandle0->priv->has_safety[ls_ch][2])
                        {
                            DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gBisscHandle0->priv->rcv_safety_crc[ch][2] == gBisscHandle0->priv->calc_safety_crc[ch][2]) ? "success" : "failure",gBisscHandle0->priv->pd_crc_err_cnt[ch][2]);
                        }
                        else
                        {
                            DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gBisscHandle0->priv->enc_pos_data[ch].rcv_crc[2] == gBisscHandle0->priv->enc_pos_data[ch].otf_crc[2]) ? "success" : "failure" ,
                            gBisscHandle0->priv->pd_crc_err_cnt[ch][2]);
                        }
                    }
                }
            }
        }
        else if(cmd == BISSC_CMD_ENC_CTRL_CMD)
        {
            DebugP_log("\r\nPlease enter control communication details:\n");
            total_channels = bissc_get_total_channels(gBisscHandle0);
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ctrl_reg_data = 0;
                ctrl_enc_id = 0;
                if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
                {
                    ls_ch = bissc_get_current_channel(gBisscHandle0, ch_num);
                    DebugP_log("\r\n Channel %u: ",ls_ch);
                }
                else
                {
                    ls_ch = 0;
                    total_channels = 1;
                }
                DebugP_log("\r\n Enter type of access(0: Read & 1: Write): ");
                DebugP_scanf("%x\n", &ctrl_write_status);
                while(1)
                {
                    DebugP_log("\r\n Enter Register Address(in hex): ");
                    DebugP_scanf("%x\n", &ctrl_reg_address);
                    if(ctrl_reg_address > BISSC_CTRL_REG_ADDR_MASK)
                        DebugP_log("\r\n Please enter a 7-bit address\n");
                    else
                        break;
                }
                if(ctrl_write_status == 1)
                {
                    DebugP_log("\r\n Enter Data(in hex) to write at 0x%x Register\n", ctrl_reg_address);
                    DebugP_scanf("%x\n", &ctrl_reg_data);
                }
                DebugP_log("\r\n Enter Encoder ID(0: if daisy chain is not in use)\n");
                DebugP_scanf("%x\n", &ctrl_enc_id);
                ctrl_cmd[ls_ch] = bissc_generate_ctrl_cmd(gBisscHandle0, ls_ch, ctrl_write_status, ctrl_reg_address, ctrl_reg_data, ctrl_enc_id);
            }
            ret = bissc_set_ctrl_cmd_and_process(gBisscHandle0, ctrl_cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Control communication failed \n");
            }
            for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
            {
                ch = bissc_get_current_channel(gBisscHandle0, ch_num);
                DebugP_log("\r\n Channel %u:\n", ch);
                DebugP_log("\r\n Control communication result: 0x%x, crc: 0x%x, otf crc: 0x%x, status: %s\n",gBisscHandle0->priv->enc_ctrl_data[ch].cmd_result,
                gBisscHandle0->priv->enc_ctrl_data[ch].cmd_rcv_crc, gBisscHandle0->priv->enc_ctrl_data[ch].cmd_otf_crc,
                (gBisscHandle0->priv->enc_ctrl_data[ch].cmd_rcv_crc == gBisscHandle0->priv->enc_ctrl_data[ch].cmd_otf_crc) ? "success" : "failure");

                DebugP_log("\r\n CTRL CRC error count: %u\n", gBisscHandle0->priv->ctrl_crc_err_cnt[ch]);
            }
        }
        else if(cmd == BISSC_CMD_ENC_LOOP_OVER_CYC)
        {
            DebugP_log("\r\n Enter number of BiSS-C cycles:\n");
            DebugP_scanf("%u\n", &loop_cnt);
            if(loop_cnt)
            {
                do
                {
                    ret = bissc_get_pos(gBisscHandle0);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n ERROR: Position data measurement for first encoder failed \n");
                    }
                    bissc_print_res(gBisscHandle0);
                    loop_cnt--;
                }while(loop_cnt);
            }
            else
            {
                DebugP_log("Please enter non-zero value\n");
            }
        }
        else if(cmd == BISSC_CMD_PERIODIC_TRIGGER)
        {
            DebugP_log("\r| Enter IEP cycle count(must be greater than BiSS cycle time including timeout period, in IEP cycles): ");
            DebugP_scanf("%lld\n", &iep_reset_count);
            if(iep_reset_count <= IEP_DEFAULT_INC)
            {
                DebugP_log("\r\n| WARNING: invalid value entered\n");
                continue;
            }
            if(attrs->load_share_enabled)
            {

                if(attrs->channel0_enabled)  /* Channel 0 */
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel0: \n");
                    DebugP_scanf("%lld\n", &ch0_trigger_count);
                    if((ch0_trigger_count > iep_reset_count) || (ch0_trigger_count <= IEP_DEFAULT_INC))
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        continue;
                    }
                }

                if(attrs->channel1_enabled)  /* Channel 1 */
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel1: \n");
                    DebugP_scanf("%lld\n", &ch1_trigger_count);
                    if((ch1_trigger_count > iep_reset_count) || (ch1_trigger_count <= IEP_DEFAULT_INC))
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        continue;
                    }
                }
                if(attrs->channel2_enabled)  /* Channel 2 */
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel2: \n");
                    DebugP_scanf("%lld\n", &ch2_trigger_count);
                    if((ch2_trigger_count > iep_reset_count) || (ch2_trigger_count <= IEP_DEFAULT_INC))
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        continue;
                    }
                }

            }
            else
            {
                DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
                DebugP_scanf("%lld\n", &ch0_trigger_count);
                if((ch0_trigger_count > iep_reset_count) || (ch0_trigger_count <= IEP_DEFAULT_INC))
                {
                    DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                    continue;
                }
            }

            bissc_process_periodic_command(gBisscHandle0, ch0_trigger_count, ch1_trigger_count, ch2_trigger_count, iep_reset_count);
        }
        else if(cmd == BISSC_ENABLE_SAFETY)
        {
            /* Clear all previously enabled safety flags */
            ret = bissc_disable_safety(gBisscHandle0);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Failed to disable safety\n");
                continue;
            }
            total_channels = bissc_get_total_channels(gBisscHandle0);
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                if(attrs->load_share_enabled)
                {
                    ls_ch = bissc_get_current_channel(gBisscHandle0, ch_num);
                }
                else
                {
                    total_channels = 1;
                    ls_ch = 0;
                }
                DebugP_log("\r\n Enter 0 to Disable Safety or 1 to Enable Safety for Channel %u:\n", gBisscHandle0->priv->channel[ch_num]);
                for(enc_num = 0; enc_num < gBisscHandle0->priv->num_encoders[ls_ch]; enc_num++)
                {
                    DebugP_log("\r\nPlease enter encoder %d safety status\n", enc_num);
                    DebugP_scanf("%d", &safety);
                    if(safety == 1)
                    {
                        ret = bissc_enable_safety(gBisscHandle0, enc_num, ls_ch);
                        if(ret != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: Failed to enable safety for encoder %d, channel %u\n", enc_num, ls_ch);
                        }
                    }
                }
            }
        }
    }
deinit:

    Board_driversClose();
    Drivers_close();
    return;
}
